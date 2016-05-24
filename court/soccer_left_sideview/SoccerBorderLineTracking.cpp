//
//  SoccerBorderLineTracking.cpp
//  QuadCopter
//
//  Created by jimmy on 7/7/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "SoccerBorderLineTracking.h"
#include "vgl_plus.h"
#include "vil_line_tracking.h"
#include "vil_plus.h"
#include "vil_line_tracking.h"
#include "vxl_plus.h"
#include "vxl_hough_line.h"
#include <vgl/vgl_distance.h>
#include "vnl_plus.h"
#include "vil_algo_plus.h"
#include <vcl_algorithm.h>



vcl_vector<vgl_point_2d<double> > SoccerBorderLineTracking::scan_line(const vil_image_view<double> & magnitude,
                                                                      const vcl_vector<vgl_point_2d<double> > & scanLinePts,
                                                                      ScanLineParameter & para)
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
    vcl_vector<vgl_point_2d<double> > borderPixels;
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
                    borderPixels.push_back(vgl_point_2d<double>(x, y));
                    
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
    if (borderPixels.size() < 1) {
        return borderPixels;
    }
    
    double mean = 0.0;
    double sigma = 0.0;
    VnlPlus::mean_std(&magnitudes[0], (unsigned int)magnitudes.size(), mean, sigma);
    para.mag_mean_std_ = vnl_vector_fixed<double, 2>(mean, sigma);
    
    VnlPlus::mean_std(&lambdas[0], (unsigned int)lambdas.size(), mean, sigma);
    para.lambda_mean_std_ = vnl_vector_fixed<double, 2>(mean, sigma);
    
    return borderPixels;
}

bool SoccerBorderLineTracking::borderLineDetection(const VilGMM & green_gmm,
                                                   const vil_image_view<vxl_byte> & image,
                                                   const vil_image_view<double> & magnitude,
                                                   const vgl_line_segment_2d<double> & initLineSegment,
                                                   const LeftsideViewBorderLineParameter & para,
                                                   vgl_line_2d<double> & line)
{
    double distance = para.shift_distance_;
    const int w = image.ni();
    const int h = image.nj();
    double greenGMMProbThreshold = para.greenGMMProbThreshold_;
    
    // shift the line segment and decide which side is in the grass land
    vgl_line_segment_2d<double> seg1;
    vgl_line_segment_2d<double> seg2;
    VglPlus::parallelMove(initLineSegment, distance, seg1, seg2);
    
    int seg1_green_pixel_num = 0;
    int seg2_green_pixel_num = 0;
    double greenColorLogProb = green_gmm.gmm_->log_prob_thresh(greenGMMProbThreshold);
    {
        vgl_point_2d<double> p1 = seg1.point1();
        vgl_point_2d<double> p2 = seg1.point2();
        vcl_vector<vgl_point_2d<double> > linetPts;
        VilPlus::draw_line(p1, p2, linetPts, w, h);
        vnl_vector<double> color(3);
        for (int i = 0; i<linetPts.size(); i++) {
            int x = linetPts[i].x();
            int y = linetPts[i].y();
            color[0] = image(x, y, 0);
            color[1] = image(x, y, 1);
            color[2] = image(x, y, 2);
            if (green_gmm.gmm_->log_p(color) > greenColorLogProb) {
                seg1_green_pixel_num++;
            }
        }
    }
    {
        vgl_point_2d<double> p1 = seg2.point1();
        vgl_point_2d<double> p2 = seg2.point2();
        vcl_vector<vgl_point_2d<double> > linetPts;
        VilPlus::draw_line(p1, p2, linetPts, w, h);
        vnl_vector<double> color(3);
        for (int i = 0; i<linetPts.size(); i++) {
            int x = linetPts[i].x();
            int y = linetPts[i].y();
            color[0] = image(x, y, 0);
            color[1] = image(x, y, 1);
            color[2] = image(x, y, 2);
            if (green_gmm.gmm_->log_p(color) > greenColorLogProb) {
                seg2_green_pixel_num++;
            }
        }
    }
    
    vgl_line_segment_2d<double> scanSeg;
    if (seg1_green_pixel_num > seg2_green_pixel_num) {
        scanSeg = seg1;
    }
    else
    {
        scanSeg = seg2;
    }
    
    if(0)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_segment(showImage, scanSeg.point1(), scanSeg.point2(), VilPlus::green());
        VilPlus::vil_save(showImage, "scan.jpg");
    }
    
    // decide san direction
    vgl_point_2d<double> org_center = initLineSegment.point_t(0.5);
    vgl_point_2d<double> scan_center = scanSeg.point_t(0.5);
    
    int scan_line_canny_direction = VglPlus::octaveDirection(org_center.x() - scan_center.x(), org_center.y() - scan_center.y());
    
    // direction is perpandicular to the canny direction
    //   3 2 1
    //   4 * 0
    //   5 6 7
    
    // (dx, dy)
    int scanDirection[][2] = { 1, 0,
        1, -1, 0, -1, -1, -1,
        -1, 0,
        -1, 1, 0, 1, 1, 1};
    assert(sizeof(scanDirection)/sizeof(scanDirection[0][0]) == 8 * 2);
    
    // calculate scan direction
    int dirInd = scan_line_canny_direction;
    assert(dirInd >= 0 && dirInd <= 7);
    const int dx = scanDirection[dirInd][0];
    const int dy = scanDirection[dirInd][1];
  
    vcl_vector<vgl_point_2d<double> > scanLinePts;
    VilPlus::draw_line(scanSeg.point1(), scanSeg.point2(), scanLinePts, w, h);
    
    // get border pixels and magnitude, lambda parameters from initial parameters
    ScanLineParameter aScanlinePara;
    aScanlinePara.magnitude_threshold_ = para.mag_threshold_;
   // aScanlinePara.imageW_ = w;
   // aScanlinePara.imageH_ = h;
    aScanlinePara.lambda_threshold_ = para.lambda_;
    aScanlinePara.scan_distance_ = para.lineWidth_;
    aScanlinePara.scan_dir_ = vgl_vector_2d<int>(dx, dy);
    vcl_vector<vgl_point_2d<double> > borderPixels = SoccerBorderLineTracking::scan_line(magnitude, scanLinePts, aScanlinePara);
    
    
    vcl_vector<double> magnitude_candidates;
    vcl_vector<double> lambda_candidates;
    magnitude_candidates.push_back(aScanlinePara.mag_mean_std_[0] - aScanlinePara.mag_mean_std_[1]);
    magnitude_candidates.push_back(aScanlinePara.mag_mean_std_[0] );
    magnitude_candidates.push_back(aScanlinePara.mag_mean_std_[0] + aScanlinePara.mag_mean_std_[1]);
    lambda_candidates.push_back(aScanlinePara.lambda_mean_std_[0] - aScanlinePara.lambda_mean_std_[1]);
    lambda_candidates.push_back(aScanlinePara.lambda_mean_std_[0]);
    lambda_candidates.push_back(aScanlinePara.lambda_mean_std_[0] + aScanlinePara.lambda_mean_std_[1]);
    
    vcl_vector<vgl_point_2d<double> > total_borderLine_pts;
    for (int i = 0; i<magnitude_candidates.size(); i++) {
        for (int j = 0; j<lambda_candidates.size(); j++) {
            ScanLineParameter aScanlinePara;
            aScanlinePara.magnitude_threshold_ = magnitude_candidates[i];
       //     aScanlinePara.imageW_ = w;
       //     aScanlinePara.imageH_ = h;
            aScanlinePara.lambda_threshold_ = lambda_candidates[j];
            aScanlinePara.scan_distance_ = para.lineWidth_;
            aScanlinePara.scan_dir_ = vgl_vector_2d<int>(dx, dy);
            vcl_vector<vgl_point_2d<double> > curPts = SoccerBorderLineTracking::scan_line(magnitude, scanLinePts, aScanlinePara);
            
            total_borderLine_pts.insert(total_borderLine_pts.begin(), curPts.begin(), curPts.end());
        }
    }
    
    
    vcl_vector<vgl_line_2d<double> > borderLines;
    vil_image_view<vxl_byte> edgeMask(w, h, 1);
    edgeMask.fill(0);
    for (int i = 0; i<total_borderLine_pts.size(); i++) {
        int x = total_borderLine_pts[i].x();
        int y = total_borderLine_pts[i].y();
        edgeMask(x, y) += 1;
    }
    
    vcl_vector<vgl_point_2d<double> > finalBorderPixels;
    for (int j = 0; j<edgeMask.nj(); j++) {
        for (int i = 0; i<edgeMask.ni(); i++) {
            if (edgeMask(i, j) >= 3) {  // the edge pixel appears multiple times
                edgeMask(i, j) = 255;
                finalBorderPixels.push_back(vgl_point_2d<double>(i, j));
            }
            else
            {
                edgeMask(i, j) = 0;
            }
        }
    }
    
    if(1)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        
        char buf[1024] = {NULL};
        sprintf(buf, "border_pixel_%d.jpg", rand());
        VilPlus::draw_dot(showImage, finalBorderPixels, VilPlus::green());
        VilPlus::vil_save(showImage, buf);
    }

    
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
  
    VxlHoughLine::oneByOneLineDetection(edgeMask, houghPara, borderLines);
    if (borderLines.size() != 1) {
        printf("Warning: can not find boardline\n");
        return false;
    }
    line = borderLines[0];
    return true;
}


bool SoccerBorderLineTracking::commercialBoardDetectionByColorClassifer(const vil_image_view<vxl_byte> & image,
                                                                        const vil_image_view<vxl_byte> & pixelTypes,
                                                                        const vgl_line_segment_2d<double> & initLineSegment,
                                                                        const FarCommerticalBoarderLineParameter & para,
                                                                        vgl_line_2d<double> & line)
{
    assert(image.nplanes() == 3);
    assert(image.ni() == pixelTypes.ni());
    
    const int w = image.ni();
    const int h = image.nj();
    const int grass_mask = para.grass_mask;
    const int cb_mask = para.cb_mask;
    const int border_scan_distance = 100;  // hard code
    // shift the line segment and decide which side is in the grass land
    vgl_line_segment_2d<double> seg1;
    vgl_line_segment_2d<double> seg2;
    VglPlus::parallelMove(initLineSegment, para.lineWidth_, seg1, seg2);
    
    double nGreenPixel1 = 0;
    double nGreenPixel2 = 0;
    {
        vgl_point_2d<double> p1 = seg1.point1();
        vgl_point_2d<double> p2 = seg1.point2();
        vcl_vector<vgl_point_2d<double> > linePts;
        VilAlgoPlus::linePixels(p1, p2, linePts, w, h);
        vnl_vector<double> color(3);
        for (int i = 0; i<linePts.size(); i++) {
            int x = linePts[i].x();
            int y = linePts[i].y();
            if (pixelTypes(x, y) == grass_mask) {
                nGreenPixel1 += 1.0;
            }
        }
    }
    {
        vgl_point_2d<double> p1 = seg2.point1();
        vgl_point_2d<double> p2 = seg2.point2();
        vcl_vector<vgl_point_2d<double> > linePts;
        VilAlgoPlus::linePixels(p1, p2, linePts, w, h);
        for (int i = 0; i<linePts.size(); i++) {
            int x = linePts[i].x();
            int y = linePts[i].y();
            if (pixelTypes(x, y) == grass_mask) {
                nGreenPixel2 += 1.0;
            }
        }
    }
    if (fabs(nGreenPixel1 - nGreenPixel2)/vcl_max(nGreenPixel1,  nGreenPixel2) <= 0.5) {
        printf("commercialBoardDetectionByColorClassifer Error 1: commercial border detection failed. two line segment main both loated on grass field.\n");
        return false;
    }
    
    vgl_line_segment_2d<double> scanSeg;
    if (nGreenPixel1 > nGreenPixel2) {
        scanSeg = seg1;
    }
    else
    {
        scanSeg = seg2;
    }
    
    // san direction
    vgl_vector_2d<int> scan_dir = VilAlgoPlus::scanDirection(scanSeg.point_t(0.5),initLineSegment.point_t(0.5));
    vcl_vector<vgl_point_2d<double> > scanStartPts;
    VilAlgoPlus::linePixels(scanSeg.point1(), scanSeg.point2(), scanStartPts, w, h);
    
    // scan and find the first commercial board  pixel
    vcl_vector<vgl_point_2d<double> > cb_pixels;
    for (int i = 0; i<scanStartPts.size(); i++) {
        int x = scanStartPts[i].x();
        int y = scanStartPts[i].y();
        for (int j = 0; j<border_scan_distance; j++) {
            x += scan_dir.x();
            y += scan_dir.y();
            if (!image.in_range(x, y)) {
                break;
            }
            // first commercial board pixel
            if (pixelTypes(x, y) == cb_mask) {
                cb_pixels.push_back(vgl_point_2d<double>(x, y));
                break;
            }
        }
    }
    if(0)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
     //   VilPlus::draw_cross(showImage, cb_pixels, 5, VilPlus::red());
        VilPlus::draw_dot(showImage, cb_pixels, VilPlus::red());
        VilPlus::vil_save(showImage, "cb_pixels.jpg");
    }
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = vgl_distance(initLineSegment.point1(), initLineSegment.point2()) * 0.1;    
    vcl_vector<vgl_point_2d<double> > inlier_pixels;
    bool isLine = VxlHoughLine::detectOneLine(cb_pixels, w, h, houghPara, line, inlier_pixels);
    if (!isLine) {
        printf("commercialBoardDetectionByColorClassifer Error 2: hough line detection failed.\n");
        return false;
    }
    return true;
}

bool SoccerBorderLineTracking::rightBorderDetectionByColorClassifer(const vil_image_view<vxl_byte> & image,  // for test only
                                                                    const vil_image_view<vxl_byte> & pixelTypes,
                                                                    const vgl_line_segment_2d<double> & initLineSegment,
                                                                    const RightBorderLine & para,
                                                                    vgl_line_2d<double> & line)
{
    assert(image.nplanes() == 3);
    assert(image.ni() == pixelTypes.ni());
    
    const int w = image.ni();
    const int h = image.nj();
    const int grass_mask = para.grass_mask;
    const int cb_mask = para.cb_mask;
    const int border_scan_distance = 100;  // hard code
    // shift the line segment and decide which side is in the grass land
    vgl_line_segment_2d<double> seg1;
    vgl_line_segment_2d<double> seg2;
    VglPlus::parallelMove(initLineSegment, para.lineWidth_, seg1, seg2);
    
    double nGreenPixel1 = 0;
    double nGreenPixel2 = 0;
    {
        vgl_point_2d<double> p1 = seg1.point1();
        vgl_point_2d<double> p2 = seg1.point2();
        vcl_vector<vgl_point_2d<double> > linePts;
        VilAlgoPlus::linePixels(p1, p2, linePts, w, h);
        vnl_vector<double> color(3);
        for (int i = 0; i<linePts.size(); i++) {
            int x = linePts[i].x();
            int y = linePts[i].y();
            if (pixelTypes(x, y) == grass_mask) {
                nGreenPixel1 += 1.0;
            }
        }
    }
    {
        vgl_point_2d<double> p1 = seg2.point1();
        vgl_point_2d<double> p2 = seg2.point2();
        vcl_vector<vgl_point_2d<double> > linePts;
        VilAlgoPlus::linePixels(p1, p2, linePts, w, h);
        for (int i = 0; i<linePts.size(); i++) {
            int x = linePts[i].x();
            int y = linePts[i].y();
            if (pixelTypes(x, y) == grass_mask) {
                nGreenPixel2 += 1.0;
            }
        }
    }
    if (fabs(nGreenPixel1 - nGreenPixel2)/vcl_max(nGreenPixel1,  nGreenPixel2) <= 0.5) {
        printf("SoccerBorderLineTracking Error 1: commercial border detection failed. two line segment main both loated on grass field.\n");
        return false;
    }
    
    vgl_line_segment_2d<double> scanSeg;
    if (nGreenPixel1 > nGreenPixel2) {
        scanSeg = seg1;
    }
    else
    {
        scanSeg = seg2;
    }
    
    // san direction
    vgl_vector_2d<int> scan_dir = VilAlgoPlus::scanDirection(scanSeg.point_t(0.5),initLineSegment.point_t(0.5));
    vcl_vector<vgl_point_2d<double> > scanStartPts;
    VilAlgoPlus::linePixels(scanSeg.point1(), scanSeg.point2(), scanStartPts, w, h);
    
    // scan and find the first commercial board  pixel
    vcl_vector<vgl_point_2d<double> > cb_pixels;
    for (int i = 0; i<scanStartPts.size(); i++) {
        int x = scanStartPts[i].x();
        int y = scanStartPts[i].y();
        for (int j = 0; j<border_scan_distance; j++) {
            x += scan_dir.x();
            y += scan_dir.y();
            if (!image.in_range(x, y)) {
                break;
            }
            // first commercial board pixel
            if (pixelTypes(x, y) == cb_mask) {
                cb_pixels.push_back(vgl_point_2d<double>(x, y));
                break;
            }
        }
    }
    if(0)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        //   VilPlus::draw_cross(showImage, cb_pixels, 5, VilPlus::red());
        VilPlus::draw_dot(showImage, cb_pixels, VilPlus::red());
        VilPlus::vil_save(showImage, "right_border_pixels.jpg");
    }
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = vgl_distance(initLineSegment.point1(), initLineSegment.point2()) * 0.1;
    vcl_vector<vgl_point_2d<double> > inlier_pixels;
    bool isLine = VxlHoughLine::detectOneLine(cb_pixels, w, h, houghPara, line, inlier_pixels);
    if (!isLine) {
        printf("SoccerBorderLineTracking Error 2: hough line detection failed.\n");
        return false;
    }    
    return true;
}

