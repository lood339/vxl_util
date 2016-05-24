//
//  vil_weak_line_tracking.cpp
//  QuadCopter
//
//  Created by jimmy on 7/9/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_weak_line_tracking.h"
#include "vxl_plus.h"
#include "vnl_plus.h"
#include "vil_algo_plus.h"
#include <vgl/vgl_closest_point.h>
#include "vil_plus.h"
#include "vxl_hough_line.h"
#include <vgl/vgl_distance.h>
#include "vgl_plus.h"
#include "vil_line_tracking.h"



bool VilWeakLineTracking::scan_on_one_direction(const vil_image_view<double> & magnitude,
                                                const vcl_vector<vgl_point_2d<double> > & scanLinePts,
                                                ScanLineParameter & para, vcl_vector<vgl_point_2d<double> > & edgePixels)
{
    const int dx = para.scan_dir_.x();
    const int dy = para.scan_dir_.y();
    const int scan_distance = para.scan_distance_;
    const double lambda = para.lambda_threshold_;
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    const double magnitude_threshold = para.magnitude_threshold_;
    
    // parameter adjustment
    vcl_vector<double> magnitudes;
    vcl_vector<double> lambdas;
    
    // return values
    edgePixels.clear();
    for (int i =0; i<scanLinePts.size(); i++)
    {
        int x = scanLinePts[i].x();
        int y = scanLinePts[i].y();
        double sumMag = 0.0;
        int num = 0;
        for (int nPixel = 0; nPixel<scan_distance; nPixel++)
        {
            if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h))
            {
                continue;
            }
            if (num >= 1)
            {
                if (magnitude(x, y) > magnitude_threshold &&
                    magnitude(x, y) >= lambda * sumMag/num) {
                    edgePixels.push_back(vgl_point_2d<double>(x, y));
                    
                    // save for adjust parameter
                    magnitudes.push_back(magnitude(x, y));
                    lambdas.push_back(magnitude(x, y)/(sumMag/num));
                    break;
                }
            }
            sumMag += magnitude(x, y);
            num++;
            x += dx;
            y += dy;
        }
    }
    if (edgePixels.size() < 1) {
        return false;
    }
    
    double mean = 0.0;
    double sigma = 0.0;
    VnlPlus::mean_std(&magnitudes[0], (unsigned int)magnitudes.size(), mean, sigma);
    para.mag_mean_std_ = vnl_vector_fixed<double, 2>(mean, sigma);
    
    VnlPlus::mean_std(&lambdas[0], (unsigned int)lambdas.size(), mean, sigma);
    para.lambda_mean_std_ = vnl_vector_fixed<double, 2>(mean, sigma);
    
    return true;
}

bool VilWeakLineTracking::multiple_scan_one_direction(const vil_image_view<double> & magnitude,
                                                      const vcl_vector<vgl_point_2d<double> > & scanLinePts,
                                                      const ScanLineParameter & para, vcl_vector<vgl_point_2d<double> > & edgePixels)
{
    // get border pixels and magnitude, lambda parameters from initial parameters
    ScanLineParameter initialPara = para;
    vcl_vector<vgl_point_2d<double> > initialEdgePixels;
    bool isScanOk = VilWeakLineTracking::scan_on_one_direction(magnitude, scanLinePts, initialPara, initialEdgePixels);
    // vcl_cout<<"magnitude, lambda is "<<initialPara.mag_mean_std_<<vcl_endl;
    // vcl_cout<<"magnitude, lambda is "<<initialPara.lambda_mean_std_<<vcl_endl<<vcl_endl;
    if (!isScanOk) {
        printf("scan failed.\n");
        printf("node, scan distance is %d %d.\n\n", para.test_node_, para.scan_distance_);
       // vcl_cout<<"magnitude, is "<<initialPara.mag_mean_std_<<vcl_endl;
       //vcl_cout<<"lambda, is "<<initialPara.lambda_mean_std_<<vcl_endl<<vcl_endl;
        return false;
    }
    
   // vcl_cout<<"magnitude, is "<<initialPara.mag_mean_std_<<vcl_endl;
   // vcl_cout<<"lambda, is "<<initialPara.lambda_mean_std_<<vcl_endl<<vcl_endl;
    
    vcl_vector<double> magnitude_candidates;
    vcl_vector<double> lambda_candidates;
    magnitude_candidates.push_back(initialPara.mag_mean_std_[0] - initialPara.mag_mean_std_[1]);
    magnitude_candidates.push_back(initialPara.mag_mean_std_[0] );
    magnitude_candidates.push_back(initialPara.mag_mean_std_[0] + initialPara.mag_mean_std_[1]);
    lambda_candidates.push_back(initialPara.lambda_mean_std_[0] - initialPara.lambda_mean_std_[1]);
    lambda_candidates.push_back(initialPara.lambda_mean_std_[0]);
    lambda_candidates.push_back(initialPara.lambda_mean_std_[0] + initialPara.lambda_mean_std_[1]);
    
    vcl_vector<vgl_point_2d<double> > total_edge_pts;
    for (int i = 0; i<magnitude_candidates.size(); i++) {
        for (int j = 0; j<lambda_candidates.size(); j++) {
            ScanLineParameter aScanlinePara = para; // only change the lambda and magnitude
            aScanlinePara.magnitude_threshold_ = magnitude_candidates[i];
            aScanlinePara.lambda_threshold_ = lambda_candidates[j];
            
            vcl_vector<vgl_point_2d<double> > curPts;
            VilWeakLineTracking::scan_on_one_direction(magnitude, scanLinePts, aScanlinePara, curPts);
            total_edge_pts.insert(total_edge_pts.begin(), curPts.begin(), curPts.end());
        }
    }
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    vcl_vector<vgl_line_2d<double> > borderLines;
    vil_image_view<vxl_byte> edgeMask(w, h, 1);
    edgeMask.fill(0);
    for (int i = 0; i<total_edge_pts.size(); i++) {
        int x = total_edge_pts[i].x();
        int y = total_edge_pts[i].y();
        edgeMask(x, y) += 1;
    }
    
    for (int j = 0; j<edgeMask.nj(); j++) {
        for (int i = 0; i<edgeMask.ni(); i++) {
            if (edgeMask(i, j) >= 3) {  // the edge pixel appears multiple times
                edgePixels.push_back(vgl_point_2d<double>(i, j));
            }
        }
    }
    return true;
}

bool VilWeakLineTracking::refineLineSegmentByMultipleParameter(const vil_image_view<vxl_byte> & image, // for test only
                                                               const vil_image_view<double> & magnitude, const vgl_line_segment_2d<double> & initLineSeg,
                                                               const WeakLineTrackingParameter & para, vgl_line_segment_2d<double> & refinedLineSeg)
{
    assert(magnitude.nplanes() == 1);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    // sample line pixels
    
    vgl_point_2d<double> p1 = initLineSeg.point1();
    vgl_point_2d<double> p2 = initLineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linetPts;
    VilAlgoPlus::linePixels(p1, p2, linetPts, w, h);
    if (linetPts.size() <= 10) {
        // line segment too short
        printf("init line pixel number is %lu. Too short.\n", linetPts.size());
        return false;
    }
    
    // dx, dy from norm direction
    int octIndex = VilAlgoPlus::octaveIndex(initLineSeg.normal().x(), initLineSeg.normal().y());
    vgl_vector_2d<int> scanDir =  VilAlgoPlus::scanDirectionFromOctaveIndex(octIndex);
    
    vcl_vector<vgl_point_2d<double> > edge1Pixels;
    vcl_vector<vgl_point_2d<double> > edge2Pixels;
    
    const int nodeId = para.test_node_;
    ScanLineParameter scanPara;
    scanPara.lambda_threshold_ = para.lambda_;
    scanPara.magnitude_threshold_ = para.mag_threshold_1_;
    scanPara.scan_distance_ = para.line_width_;
    scanPara.scan_dir_ = scanDir;
    scanPara.test_node_ = para.test_node_;
    
    // positive direction
    VilWeakLineTracking::multiple_scan_one_direction(magnitude, linetPts, scanPara, edge1Pixels);
    scanPara.scan_dir_ =  -scanPara.scan_dir_; // oppposite direction
    // negative direction
    VilWeakLineTracking::multiple_scan_one_direction(magnitude, linetPts, scanPara, edge2Pixels);    
    
    
    // fit lines from edge pixels
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = vgl_distance(p1, p2) * para.pixel_num_ratio_;
    vgl_line_2d<double> line1;
    vgl_line_2d<double> line2;
    vcl_vector<vgl_point_2d<double> > inlier_pts1;
    vcl_vector<vgl_point_2d<double> > inlier_pts2;
    bool isLine1 = false;
    if (edge1Pixels.size() >= houghPara.minPixeNum_) {
        isLine1 = VxlHoughLine::detectOneLine(edge1Pixels, w, h, houghPara, line1, inlier_pts1);
    }
    bool isLine2 = false;
    if (edge2Pixels.size() >= houghPara.minPixeNum_) {
        isLine2 = VxlHoughLine::detectOneLine(edge2Pixels, w, h, houghPara, line2, inlier_pts2);
    }
    
    if (!isLine1 && !isLine2) {
        printf("node %d can not find the line on both side of inital line segment. Line pixel number should be larger than %d\n", nodeId, houghPara.minPixeNum_);
        return false;
    }
    // pick one of the lines that has more inliers
    vgl_line_2d<double> line = line1;
    if (inlier_pts2.size() > inlier_pts1.size()) {
        line = line2;
    }
    
    // check direction should be parallel
    double cosAngle = fabs(cos_angle(initLineSeg.direction(), line.direction()));
    if (cosAngle < para.parallel_line_direction_threshold_) {
        // parallel_line_direction_threshold_  cosAngle < para.pixel_num_ratio
        printf("Line refinement failed: refined line and initial line are not parallel.\n");
        return false;
    }
    
    // project initial line segment end point to tracked line
    vgl_point_2d<double> p3 = vgl_closest_point(line, p1);
    vgl_point_2d<double> p4 = vgl_closest_point(line, p2);
    refinedLineSeg = vgl_line_segment_2d<double>(p3, p4);
    
    
    
        
    if(0)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, edge1Pixels, 3, VilPlus::blue());
        VilPlus::draw_cross(showImage, edge2Pixels, 3, VilPlus::red());
        VilPlus::draw_segment(showImage, initLineSeg.point1(), initLineSeg.point2(), VilPlus::white());
        VilPlus::draw_segment(showImage, p3, p4, VilPlus::green());
        char buf[1024] = {NULL};
        sprintf(buf, "double_edge_initial_%d.jpg", rand());
        VilPlus::vil_save(showImage, buf);
    }    
    
    return true;
}

bool VilWeakLineTracking::trackingLineFromSegmentByMultipleParameter(const vil_image_view<vxl_byte> & image, // for test only
                                                                     const vil_image_view<double> & magnitude,
                                                                     const vil_image_view<double> & grad_i,
                                                                     const vil_image_view<double> & grad_j,
                                                                     const vgl_line_segment_2d<double> & lineSeg,
                                                                     const WeakLineTrackingParameter & para,
                                                                     vgl_line_2d<double> & line,
                                                                     vgl_line_segment_2d<double> & refinedLineSeg)
{
    assert(magnitude.nplanes() == 1);
    assert(grad_i.ni() == magnitude.ni());
    assert(grad_i.nj() == magnitude.nj());
    assert(grad_j.ni() == magnitude.ni());
    assert(grad_j.nj() == magnitude.nj());
    
    double minEdgePixelRatio = para.pixel_num_ratio_;
    double minMagnitude = para.mag_threshold_1_;
    
    // verify the initial line is on th edge   
    double onEdgeRatio = VilLineTracking::onEdgePixelRatio(magnitude, grad_i, grad_j, lineSeg, minMagnitude, 5);
    if (onEdgeRatio < minEdgePixelRatio) {
        printf("Line tracking failed: initial line segment is not along an edge. The on line edge pixel ratio is %f.\n", onEdgeRatio);
        return false;
    }
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    // parallel moving
    vgl_line_segment_2d<double> seg1;
    vgl_line_segment_2d<double> seg2;
    VglPlus::parallelMove(lineSeg, para.line_width_, seg1, seg2);
    
    // pointx in these lines
    vcl_vector<vgl_point_2d<double> > scanPixels1;
    vcl_vector<vgl_point_2d<double> > scanPixels2;
    VilAlgoPlus::linePixels(seg1.point1(), seg1.point2(), scanPixels1, w, h);
    VilAlgoPlus::linePixels(seg2.point1(), seg2.point2(), scanPixels2, w, h);
    
    // detect two edge line
    // fit two lines
    vcl_vector<vgl_point_2d<double> > edgePixel1;  // inline pixels
    vcl_vector<vgl_point_2d<double> > edgePixel2;
    ScanLineParameter scanLinePara;
    scanLinePara.scan_dir_ = VilAlgoPlus::scanDirection(seg1.point_t(0.5), lineSeg.point_t(0.5)); // one center to another center
    scanLinePara.magnitude_threshold_ = para.mag_threshold_1_;
    scanLinePara.lambda_threshold_ = para.lambda_;
    scanLinePara.scan_distance_ = para.line_width_;
    scanLinePara.test_node_ = para.test_node_;
    
    bool isFirstTracked  = VilWeakLineTracking::multiple_scan_one_direction(magnitude, scanPixels1, scanLinePara, edgePixel1);
    scanLinePara.scan_dir_ = - scanLinePara.scan_dir_; // opposite direction
    bool isSecondTracked = VilWeakLineTracking::multiple_scan_one_direction(magnitude, scanPixels2, scanLinePara, edgePixel2);
    
    if (!isFirstTracked && !isSecondTracked) {
        printf("scan half edge failed.\n");
        return false;
    }
    // find inlier pixels
    // fit lines from edge pixels
    vcl_vector<vgl_point_2d<double> > inlier_edge_pixels1;
    vcl_vector<vgl_point_2d<double> > inlier_edge_pixels2;
    
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = 15;
    houghPara.inlierDistance_ = 2.0;
    vgl_line_2d<double> line1;
    vgl_line_2d<double> line2;
    bool isLine1 = VxlHoughLine::detectOneLine(edgePixel1, w, h, houghPara, line1, inlier_edge_pixels1);
    bool isLine2 = VxlHoughLine::detectOneLine(edgePixel2, w, h, houghPara, line2, inlier_edge_pixels2);
    if (!isLine1 && !isLine2) {
        printf("hald edge line fitting failed.\n");
        return false;
    }
    
    
    if (0 && para.test_node_ == 26) {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        
        VilPlus::draw_cross(showImage, inlier_edge_pixels1, 3, VilPlus::green());
        VilPlus::draw_cross(showImage, inlier_edge_pixels2, 3, VilPlus::red());
        VilPlus::vil_save(showImage, "node_16_.jpg", showImage);
    }
        
    // pick one that has large inlier pixels
    if (inlier_edge_pixels1.size() > inlier_edge_pixels2.size()) {
        line = line1;
        vgl_point_2d<double> p3 = vgl_closest_point(line, lineSeg.point1());
        vgl_point_2d<double> p4 = vgl_closest_point(line, lineSeg.point2());
        refinedLineSeg = vgl_line_segment_2d<double>(p3, p4);
        return true;
    }
    else
    {
        line = line2;
        vgl_point_2d<double> p3 = vgl_closest_point(line, lineSeg.point1());
        vgl_point_2d<double> p4 = vgl_closest_point(line, lineSeg.point2());
        refinedLineSeg = vgl_line_segment_2d<double>(p3, p4);
        return true;
    }    
    
    return false;
}

