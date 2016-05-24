//
//  vil_line_tracking.cpp
//  OnlineStereo
//
//  Created by jimmy on 3/6/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_line_tracking.h"
#include "vil_plus.h"
#include "vnl_plus.h"
#include "vxl_plus.h"
#include "vxl_hough_line.h"
#include <vgl/vgl_distance.h>
#include <vgl/vgl_closest_point.h>
#include <vcl_algorithm.h>
#include <vgl/vgl_fit_line_2d.h>
#include "vgl_plus.h"

bool VilLineTracking::isLineSegmentOnEdge(const vil_image_view<vxl_byte> & image, // for test only
                                          const vil_image_view<double> & magnitude,
                                          const vil_image_view<double> & grad_i,
                                          const vil_image_view<double> & grad_j,
                                          const vgl_line_segment_2d<double> & lineSeg,
                                          const LinesegmntTrackingParameter & para,
                                          double edgePixelRatioThreshold)
{
    assert(magnitude.nplanes() == 1);
    assert(grad_i.ni() == magnitude.ni());
    assert(grad_i.nj() == magnitude.nj());
    assert(grad_j.ni() == magnitude.ni());
    assert(grad_j.nj() == magnitude.nj());
    
    int sz = para.edge_neighbor_size_;
    assert(sz >= 3 && sz <= 5);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    
    vgl_point_2d<double> p1 = lineSeg.point1();
    vgl_point_2d<double> p2 = lineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linePts;
    VilPlus::draw_line(p1, p2, linePts, w, h);
    if (linePts.size() < 10) {
        return false;
    }
    
    int cannyDir = VilLineTracking::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    switch (cannyDir) {
        case 0:
            cannyDir = 2;
            break;
        case 1:
            cannyDir = 3;
            break;
        case 2:
            cannyDir = 0;
            break;
        case 3:
            cannyDir = 1;
            break;
        default:
            break;
    }
    
    int num = 0;
    for (int i = 0; i<linePts.size(); i++) {
        int startX = linePts[i].x();
        int startY = linePts[i].y();
        bool isFound = false;
        
       // printf("magnitude is %f\n", magnitude(x, y));
        for (int j = -sz/2; j <= sz/2; j++) {
            for (int k = -sz/2; k <= sz/2; k++) {
                int xx = startX + k;
                int yy = startY + j;
                if (xx >= 0 && xx<w &&
                    yy >= 0 && yy<h) {
                    if (magnitude(xx, yy) > para.mag_threshold_1_ &&
                        cannyDir == VilLineTracking::cannyDirection(grad_i(xx, yy), grad_j(xx, yy))) {
                        isFound = true;
                        break;
                    }
                }
            }
            if (isFound) {
                break;
            }
        }
        if (isFound) {
            num++;
        }
    }
    
    double ratio = 1.0 * num /linePts.size();
   // printf("near edge area ratio is %f\n", ratio);
    
    return ratio > edgePixelRatioThreshold;
}

double VilLineTracking::onEdgePixelRatio(const vil_image_view<double> & magnitude,
                                         const vil_image_view<double> & grad_i,
                                         const vil_image_view<double> & grad_j,
                                         const vgl_line_segment_2d<double> & lineSeg,
                                         const double min_magnitude,
                                         const int edge_neighbor_size)
{
    assert(magnitude.nplanes() == 1);
    assert(grad_i.ni() == magnitude.ni());
    assert(grad_i.nj() == magnitude.nj());
    assert(grad_j.ni() == magnitude.ni());
    assert(grad_j.nj() == magnitude.nj());
    
    int sz = edge_neighbor_size;
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    
    vgl_point_2d<double> p1 = lineSeg.point1();
    vgl_point_2d<double> p2 = lineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linePts;
    VilPlus::draw_line(p1, p2, linePts, w, h);
    if (linePts.size() < 10) {
        return false;
    }
    
    int cannyDir = VilLineTracking::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    switch (cannyDir) {
        case 0:
            cannyDir = 2;
            break;
        case 1:
            cannyDir = 3;
            break;
        case 2:
            cannyDir = 0;
            break;
        case 3:
            cannyDir = 1;
            break;
        default:
            break;
    }
    
    int num = 0;
    for (int i = 0; i<linePts.size(); i++) {
        int startX = linePts[i].x();
        int startY = linePts[i].y();
        bool isFound = false;
        
        for (int j = -sz/2; j <= sz/2; j++) {
            for (int k = -sz/2; k <= sz/2; k++) {
                int xx = startX + k;
                int yy = startY + j;
                if (xx >= 0 && xx<w &&
                    yy >= 0 && yy<h) {
                    if (magnitude(xx, yy) > min_magnitude &&
                        cannyDir == VilLineTracking::cannyDirection(grad_i(xx, yy), grad_j(xx, yy)))
                    {
                        isFound = true;
                        break;
                    }
                }
            }
            if (isFound) {
                break;
            }
        }
        if (isFound) {
            num++;
        }
    }
    
    double ratio = 1.0 * num /linePts.size();
    return ratio;
}

bool VilLineTracking::refineLineSegment(const vil_image_view<vxl_byte> & image, // for test only
                                        const vil_image_view<double> & magnitude, const vgl_line_segment_2d<double> & initLineSeg,
                                        const LinesegmntTrackingParameter & para, vgl_line_segment_2d<double> & refinedLineSeg)
{
    assert(magnitude.nplanes() == 1);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    
    vgl_point_2d<double> p1 = initLineSeg.point1();
    vgl_point_2d<double> p2 = initLineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linetPts;
    VilPlus::draw_line(p1, p2, linetPts, w, h);
    if (linetPts.size() <= 10) {
        // line segment too short
        printf("init line pixel number is %lu. Too short.\n", linetPts.size());
        return false;
    }
    
    // direction is perpandicular to the canny direction
    int scanDirection[][2] = {0, 1, 1, 0,
                              1, 1, -1, -1};
    
    // calculate scan direction
    int dirInd = VilLineTracking::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    assert(dirInd >= 0 && dirInd < 4);
    const int dx = scanDirection[dirInd][0];
    const int dy = scanDirection[dirInd][1];
    
  //  printf("dirction index, sample number is %d %lu,\n", dirInd, linetPts.size());
    
    vcl_vector<vgl_point_2d<double> > edge1Pixels;
    vcl_vector<vgl_point_2d<double> > edge2Pixels;
    
    // positive direction
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();
        int y = linetPts[i].y();
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j < para.line_width_; j++) {
            if (!VglPlus::vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                continue;
            }
            
            if (num >= 1) {
                if (magnitude(x, y) > para.mag_threshold_1_ &&
                    magnitude(x, y) < para.mag_threshold_2_ &&
                    magnitude(x, y) >= para.lambda_ * sumMag/num) {
                    edge1Pixels.push_back(vgl_point_2d<double>(x, y));
                    break;
                }
            }
            sumMag += magnitude(x, y);
            num++;
            
            x += dx;
            y += dy;
        }
    }
    
    // negative direction
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();
        int y = linetPts[i].y();
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j < para.line_width_; j++) {
            if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                break;
            }
            if (num >= 1) {
                if (magnitude(x, y) > para.mag_threshold_1_ &&
                    magnitude(x, y) < para.mag_threshold_2_ &&
                    magnitude(x, y) >= para.lambda_ * sumMag/num) {
                    edge2Pixels.push_back(vgl_point_2d<double>(x, y));
                    break;
                }
            }
            sumMag += magnitude(x, y);
            num++;
            
            x -= dx;
            y -= dy;
        }
    }
    
    printf("find %lu %lu edge pixels\n", edge1Pixels.size(), edge2Pixels.size());
    
    if(0)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);        
        VilPlus::draw_cross(showImage, edge1Pixels, 3, VilPlus::blue());
        VilPlus::draw_cross(showImage, edge2Pixels, 3, VilPlus::red());
        VilPlus::draw_segment(showImage, initLineSeg.point1(), initLineSeg.point2(), VilPlus::white());
        VilPlus::vil_save(showImage, "double_edge_initial.jpg");
    }

    
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
 //       printf("node %d, can not find the line on both side of inital line segment. Line pixel number should be larger than %d\n", houghPara.minPixeNum_);
        return false;
    }
    // pick one of the lines that has more inliers
    vgl_line_2d<double> line = line1;
    if (inlier_pts2.size() > inlier_pts1.size()) {
        line = line2;
    }
    
    // check direction should be parallel
    double cosAngle = fabs(cos_angle(initLineSeg.direction(), line.direction()));
    if (cosAngle < para.parallel_line_direction_threshold_) { // @todo this is a bug, but the program works
        // parallel_line_direction_threshold_  cosAngle < para.pixel_num_ratio
        printf("Line refinement failed: refined line and initial line are not parallel.\n");
        return false;
    }
    
    // project initial line segment end point to tracked line
    vgl_point_2d<double> p3 = vgl_closest_point(line, p1);
    vgl_point_2d<double> p4 = vgl_closest_point(line, p2);
    
 //   double d1 = vgl_distance(p1, p3);
 //   double d2 = vgl_distance(p2, p4);
  //  printf("initial line and tracked line average distance is %f\n",(d1 + d2)/2.0);
    
    refinedLineSeg = vgl_line_segment_2d<double>(p3, p4);
    return true;    
}

bool VilLineTracking::refineLineSegment(const vil_image_view<vxl_byte> & image,
                                        const vil_image_view<double> & magnitude,
                                        const vil_image_view<double> & grad_i,
                                        const vil_image_view<double> & grad_j,
                                        const vgl_line_segment_2d<double> & initLineSeg,
                                        const LinesegmntTrackingParameter & para,
                                        vgl_line_segment_2d<double> & refinedLineSeg)
{
    assert(magnitude.nplanes() == 1);
    assert(grad_i.nplanes() == 1);
    assert(grad_j.nplanes() == 1);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    
    vgl_point_2d<double> p1 = initLineSeg.point1();
    vgl_point_2d<double> p2 = initLineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linetPts;
    VilPlus::draw_line(p1, p2, linetPts, w, h);
    if (linetPts.size() <= 10) {
        // line segment too short
        printf("init line pixel number is %lu. Too short.\n", linetPts.size());
        return false;
    }
    
    // direction is perpandicular to the canny direction
    int scanDirection[][2] = {0, 1, 1, 0,
                              1, 1, -1, -1};
    
    // calculate scan direction
    int dirInd = VilLineTracking::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    assert(dirInd >= 0 && dirInd < 4);
    const int dx = scanDirection[dirInd][0];
    const int dy = scanDirection[dirInd][1];
    
    
    vcl_vector<vgl_point_2d<double> > edge1Pixels;
    vcl_vector<vgl_point_2d<double> > edge2Pixels;
    
    // positive direction
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();
        int y = linetPts[i].y();
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j < para.line_width_; j++) {
            if (!VglPlus::vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                continue;
            }
            
            if (num >= 1) {
                if (magnitude(x, y) > para.mag_threshold_1_ &&
                    magnitude(x, y) < para.mag_threshold_2_ &&
                    magnitude(x, y) >= para.lambda_ * sumMag/num) {
                    
                    edge1Pixels.push_back(vgl_point_2d<double>(x, y));
                    break;
                }
            }
            sumMag += magnitude(x, y);
            num++;
            
            x += dx;
            y += dy;
        }
    }
    
    // negative direction
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();
        int y = linetPts[i].y();
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j < para.line_width_; j++) {
            if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                break;
            }
            if (num >= 1) {
                if (magnitude(x, y) > para.mag_threshold_1_ &&
                    magnitude(x, y) < para.mag_threshold_2_ &&
                    magnitude(x, y) >= para.lambda_ * sumMag/num)
                {                    
                  //  vgl_vector_2d<double> dxy(grad_i(x,y), grad_j(x, y));
                  //  dxy = normalize(dxy);
                    // check angle
                  //  double cosAngle = fabs(cos_angle(initLineSeg.direction(), dxy));
                  //  if (cosAngle > para.parallel_line_direction_threshold_) {
                  //  }
                    edge2Pixels.push_back(vgl_point_2d<double>(x, y));
                    break;
                }
            }
            sumMag += magnitude(x, y);
            num++;
            
            x -= dx;
            y -= dy;
        }
    }
    
    printf("find %lu %lu edge pixels\n", edge1Pixels.size(), edge2Pixels.size());
    
    if(1)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, edge1Pixels, 3, VilPlus::blue());
        VilPlus::draw_cross(showImage, edge2Pixels, 3, VilPlus::red());
        VilPlus::draw_segment(showImage, initLineSeg.point1(), initLineSeg.point2(), VilPlus::white());
        VilPlus::vil_save(showImage, "double_edge_initial.jpg");
    }
    
    
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
    //    printf("node %d, can not find the line on both side of inital line segment. Line pixel number should be larger than %d\n", houghPara.minPixeNum_);
        return false;
    }
    // pick one of the lines that has more inliers
    vgl_line_2d<double> line = line1;
    if (inlier_pts2.size() > inlier_pts1.size()) {
        line = line2;
    }
    
    // check direction should be parallel
    double cosAngle = fabs(cos_angle(initLineSeg.direction(), line.direction()));
    if (cosAngle < para.parallel_line_direction_threshold_) { // @todo this is a bug, but the program works
        // parallel_line_direction_threshold_  cosAngle < para.pixel_num_ratio
    //    printf("Line refinement failed: refined line and initial line are not parallel.\n");
        return false;
    }
    
    // project initial line segment end point to tracked line
    vgl_point_2d<double> p3 = vgl_closest_point(line, p1);
    vgl_point_2d<double> p4 = vgl_closest_point(line, p2);
    
    refinedLineSeg = vgl_line_segment_2d<double>(p3, p4);
    return true;
}

bool VilLineTracking::refineLineSegmentByMagnitudePeak(const vil_image_view<vxl_byte> & image, // for test only
                                                       const vil_image_view<double> & magnitude,
                                                       const vil_image_view<double> & grad_i,
                                                       const vil_image_view<double> & grad_j,
                                                       const vgl_line_segment_2d<double> & initLineSeg,
                                                       const LinesegmntTrackingParameter & para,
                                                       vgl_line_segment_2d<double> & refinedLineSeg,
                                                       vgl_line_2d<double> & fefinedLine)
{
    assert(magnitude.nplanes() == 1);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    // sample line pixels
    
    vgl_point_2d<double> p1 = initLineSeg.point1();
    vgl_point_2d<double> p2 = initLineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linetPts;
    VilPlus::draw_line(p1, p2, linetPts, w, h);
    if (linetPts.size() <= 10) {
        // line segment too short
        printf("init line pixel number is %lu. Too short.\n", linetPts.size());
        return false;
    }
    
    // direction is perpandicular to the canny direction
    int scanDirection[][2] = {0, 1, 1, 0,
                              1, 1, -1, -1};
    
    // calculate scan direction
    const int dirInd = VilLineTracking::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    assert(dirInd >= 0 && dirInd < 4);
    const int dx = scanDirection[dirInd][0];
    const int dy = scanDirection[dirInd][1];
    
    printf("dirction index, sample number is %d %lu,\n", dirInd, linetPts.size());
    
    vcl_vector<vgl_point_2d<double> > edgePixels;
    // scan in two directions
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();  // scan line start point
        int y = linetPts[i].y();
        
        // record pixels with maximum magnitude
        double max_mag = 0.0;
        bool isFind = false;
        vgl_point_2d<double> ep;  // edge point
        // scan in positive direction
        {
            for (int j = 0; j < para.line_width_; j++) {
                if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                    continue;
                }
                if (magnitude(x, y) > para.mag_threshold_1_) {
                    int mag_dir = VilLineTracking::cannyDirection(grad_i(x, y), grad_j(x, y));
                    if (mag_dir == dirInd) { // magnitude direction is perpendicular to to estimated line direction
                        if (magnitude(x, y) > max_mag) {
                            max_mag = magnitude(x, y);
                            ep.set(x, y);
                            isFind = true;
                        }
                    }
                }
                x += dx;
                y += dy;
            }
        }
        
        x = linetPts[i].x();
        y = linetPts[i].y();
        // scan in negative direction
        {
            for (int j = 0; j < para.line_width_; j++) {
                if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                    continue;
                }
                if (magnitude(x, y) > para.mag_threshold_1_) {
                    int mag_dir = VilLineTracking::cannyDirection(grad_i(x, y), grad_j(x, y));
                    if (mag_dir == dirInd)
                    { // magnitude direction is perpendicular to to estimated line direction
                        if (magnitude(x, y) > max_mag) {
                            max_mag = magnitude(x, y);
                            ep.set(x, y);
                            isFind = true;
                        }
                    }
                }
                x -= dx;
                y -= dy;
            }
        }
        
        if (isFind) {
            edgePixels.push_back(ep);
        }
    }
    
    printf("find %lu edge pixels by finding peak magnitude\n", edgePixels.size());
    static int std_idx = 0;
    if(0)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        
        char buf[1024] = {NULL};
        sprintf(buf, "peak_edge_initial_%d.jpg", std_idx++);
        VilPlus::draw_cross(showImage, edgePixels, 3, VilPlus::blue());
        VilPlus::draw_segment(showImage, initLineSeg.point1(), initLineSeg.point2(), VilPlus::white());
        VilPlus::vil_save(showImage, buf);
    }
    
    // fit lines from edge pixels
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = vgl_distance(p1, p2) * para.pixel_num_ratio_;
    vgl_line_2d<double> line;
    vcl_vector<vgl_point_2d<double> > inlier_pts;
    
    bool isLine = VxlHoughLine::detectOneLine(edgePixels, w, h, houghPara, line, inlier_pts);
    
    if (!isLine) {
        printf("can not find the line. Line pixel number should be larger than %d\n", houghPara.minPixeNum_);
        return false;
    }    
    
    // check direction should be parallel
    double cosAngle = fabs(cos_angle(initLineSeg.direction(), line.direction()));
    if (cosAngle < para.pixel_num_ratio_) {
        printf("Line traking failed: tracked line and initial line are not parallel.\n");
        return false;
    }
    
    // project initial line segment end point to tracked line
    vgl_point_2d<double> p3 = vgl_closest_point(line, p1);
    vgl_point_2d<double> p4 = vgl_closest_point(line, p2);
    
    double d1 = vgl_distance(p1, p3);
    double d2 = vgl_distance(p2, p4);
    printf("initial line and tracked line average distance is %f\n",(d1 + d2)/2.0);
    
    refinedLineSeg = vgl_line_segment_2d<double>(p3, p4);
    
    fefinedLine = line;
    return true;
}

bool VilLineTracking::refineLineSegmentByLineFilter(const vil_image_view<vxl_byte> & image, // for test only
                                                    const vil_image_view<double> & magnitude,
                                                    const vil_image_view<double> & grad_i,
                                                    const vil_image_view<double> & grad_j,
                                                    
                                                    const vil_image_view<double> & blueComponent,
                                                    const vil_image_view<bool> & isGrassMask,
                                   
                                                    const LinesegmntTrackingParameter & lineTrackingPara,
                                                    const LineFilterParameter & filterOutputPara,
                                                    const vgl_line_segment_2d<double> & initLineSeg,
                                   
                                                    vgl_line_segment_2d<double> & refinedLineSeg,
                                                    vgl_line_2d<double> & fefinedLine)
{
    assert(magnitude.nplanes() == 1);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    // sample line pixels
    
    vgl_point_2d<double> p1 = initLineSeg.point1();
    vgl_point_2d<double> p2 = initLineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linetPts;
    VilPlus::draw_line(p1, p2, linetPts, w, h);
    if (linetPts.size() <= 10) {
        // line segment too short
        printf("init line pixel number is %lu. Too short.\n", linetPts.size());
        return false;
    }
    
    // direction is perpandicular to the canny direction
    int scanDirection[][2] = {0, 1, 1, 0,
                              1, 1, -1, -1};
    
    // filter step in horizontal and vertical
    unsigned int filterStep[][2] = {filterOutputPara.edge_width_, 0,
                                    0, filterOutputPara.edge_width_};
        
    
    // calculate scan direction
    const int dirInd = VilLineTracking::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    assert(dirInd >= 0 && dirInd < 4);
    const int dx = scanDirection[dirInd][0];
    const int dy = scanDirection[dirInd][1];
    // filter direction
    int filterDx = filterStep[0][0]; // assume is horizontal
    int filterDy = filterStep[0][1];
    if (dirInd == 0) {
        filterDx = filterStep[1][0];
        filterDy = filterStep[1][1];
    }
    
    printf("dirction index, sample number is %d %lu,\n", dirInd, linetPts.size());
    
    vcl_vector<vgl_point_2d<double> > edgePixels;
    // scan in two directions
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();  // scan line start point
        int y = linetPts[i].y();
        
        
        // record pixels with maximum magnitude
        double max_response = 0.0;
        bool isFind = false;
        vgl_point_2d<double> ep;  // edge point
        // scan in positive direction
        {
            for (int j = 0; j < lineTrackingPara.line_width_; j++) {
                //filter window should be inside image
                if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h, - filterOutputPara.edge_width_*2)) {
                    continue;
                }
                if (magnitude(x, y) > lineTrackingPara.mag_threshold_1_) {
                    int mag_dir = VilLineTracking::cannyDirection(grad_i(x, y), grad_j(x, y));
                    if (mag_dir == dirInd) { // magnitude direction is perpendicular to to estimated line direction
                        double a = blueComponent(x - filterDx, y - filterDy);
                        double b = blueComponent(x, y);
                        double c = blueComponent(x + filterDx, y + filterDy);
                        double filter_response = fabs(vcl_min(b-a, b-c)); // filter response
                        if (filter_response > max_response) {
                            max_response = filter_response;
                            ep.set(x, y);
                            isFind = true;
                        }
                    }
                }
                x += dx;
                y += dy;
            }
        }
        
        x = linetPts[i].x();  // scan line start point
        y = linetPts[i].y();
        // scan in negative direction
        {
            for (int j = 0; j < lineTrackingPara.line_width_; j++) {
                //filter window should be inside image
                if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h, - filterOutputPara.edge_width_*2)) {
                    continue;
                }
                if (magnitude(x, y) > lineTrackingPara.mag_threshold_1_) {
                    int mag_dir = VilLineTracking::cannyDirection(grad_i(x, y), grad_j(x, y));
                    if (mag_dir == dirInd) { // magnitude direction is perpendicular to to estimated line direction
                        double a = blueComponent(x - filterDx, y - filterDy);
                        double b = blueComponent(x, y);
                        double c = blueComponent(x + filterDx, y + filterDy);
                        double filter_response = fabs(vcl_min(b-a, b-c)); // filter response
                        if (filter_response > max_response) {
                            max_response = filter_response;
                            ep.set(x, y);
                            isFind = true;
                        }
                    }
                }
                x -= dx;
                y -= dy;
            }
        }
        
        if (isFind) {
            edgePixels.push_back(ep);
        }
    }
    
    printf("find %lu edge pixels by finding peak magnitude\n", edgePixels.size());
    static int std_idx = 0;
    if(0)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        
        char buf[1024] = {NULL};
        sprintf(buf, "peak_edge_initial_%d.jpg", std_idx++);
        VilPlus::draw_cross(showImage, edgePixels, 3, VilPlus::blue());
        VilPlus::draw_segment(showImage, initLineSeg.point1(), initLineSeg.point2(), VilPlus::white());
        VilPlus::vil_save(showImage, buf);
    }
    
    // fit lines from edge pixels
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = vgl_distance(p1, p2) * lineTrackingPara.pixel_num_ratio_;
    vgl_line_2d<double> line;
    vcl_vector<vgl_point_2d<double> > inlier_pts;
    
    bool isLine = VxlHoughLine::detectOneLine(edgePixels, w, h, houghPara, line, inlier_pts);
    
    if (!isLine) {
        printf("can not find the line. Line pixel number should be larger than %d\n", houghPara.minPixeNum_);
        return false;
    }
    
    // check direction should be parallel
    double cosAngle = fabs(cos_angle(initLineSeg.direction(), line.direction()));
    if (cosAngle < lineTrackingPara.pixel_num_ratio_) {
        printf("Line traking failed: tracked line and initial line are not parallel.\n");
        return false;
    }
    
    // project initial line segment end point to tracked line
    vgl_point_2d<double> p3 = vgl_closest_point(line, p1);
    vgl_point_2d<double> p4 = vgl_closest_point(line, p2);
    
    double d1 = vgl_distance(p1, p3);
    double d2 = vgl_distance(p2, p4);
    printf("initial line and tracked line average distance is %f\n",(d1 + d2)/2.0);
    
    refinedLineSeg = vgl_line_segment_2d<double>(p3, p4);
    
    fefinedLine = line;
    
    return true;
}

/*
bool VilLineTracking::trackingLineFromSegment(const vil_image_view<vxl_byte> & image,
                                              const vil_image_view<double> & magnitude,
                                              const vil_image_view<double> & grad_i,
                                              const vil_image_view<double> & grad_j,
                                              const vgl_line_segment_2d<double> & lineSeg,
                                              const LinesegmntTrackingParameter & para, vgl_line_2d<double> & line,
                                              vgl_line_segment_2d<double> & cenerLineSeg)
{
    assert(magnitude.nplanes() == 1);
    assert(grad_i.ni() == magnitude.ni());
    assert(grad_i.nj() == magnitude.nj());
    assert(grad_j.ni() == magnitude.ni());
    assert(grad_j.nj() == magnitude.nj());
    
    double minEdgePixelRatio = 0.10;
    // verify lineSeg
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    const vgl_point_2d<double> p1 = lineSeg.point1();
    const vgl_point_2d<double> p2 = lineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linePts;
    VilPlus::draw_line(p1, p2, linePts, w, h);
    if (linePts.size() < 10) {
        return false;
    }
    
    int cannyDir = VilLineTracking::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    switch (cannyDir) {
        case 0:
            cannyDir = 2;
            break;
        case 1:
            cannyDir = 3;
            break;
        case 2:
            cannyDir = 0;
            break;
        case 3:
            cannyDir = 1;
            break;
        default:
            break;
    }
    int num = 0;
    for (int i = 0; i<linePts.size(); i++) {
        int x = linePts[i].x();
        int y = linePts[i].y();
        double dx = grad_i(x, y);
        double dy = grad_j(x, y);
        
        int dir = VilLineTracking::cannyDirection(dx, dy); //
        if (dir == cannyDir) {
            num++;
        }
    }
    double edgePixelRatio = 1.0 * num/linePts.size();
    if (edgePixelRatio < minEdgePixelRatio) {
        printf("Line tracking failed: initial line segment is not along an edge ratio is %f.\n", edgePixelRatio);
        return false;
    }
    
    // parallel moving
    vgl_line_segment_2d<double> seg1;
    vgl_line_segment_2d<double> seg2;
    VglPlus::parallelMove(lineSeg, para.line_width_/2.0, seg1, seg2);
    
    // detect two edge line
    // fit two lines
    vcl_vector<vgl_point_2d<double> > edgePixel1;  // inline pixels
    vcl_vector<vgl_point_2d<double> > edgePixel2;
    LinesegmntTrackingParameter para1 = para;
    para1.scan_line_canny_direction_ = VglPlus::octaveDirection(lineSeg.point1().x() - seg1.point1().x(), lineSeg.point1().y() - seg1.point1().y());
    LinesegmntTrackingParameter para2 = para;   
    para2.scan_line_canny_direction_ = VglPlus::octaveDirection(lineSeg.point1().x() - seg2.point1().x(), lineSeg.point1().y() - seg2.point1().y());
    
    // trackingLinePixelInOneDirectionByPeakRMCAM
    // trackingLinePixelInOneDirection
    bool isFirstTracked  = VilLineTracking::trackingLinePixelInOneDirection(magnitude, seg1, para1, edgePixel1);
    bool isSecondTracked = VilLineTracking::trackingLinePixelInOneDirection(magnitude, seg2, para2, edgePixel2);
    if (!isFirstTracked && !isSecondTracked) {
        printf("Line tracking failed: double edge detection failed to find two edges.\n");
        return false;
    }
    if(0)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        
      //  VilPlus::draw_cross(showImage, edgePixel1, 3, VilPlus::blue());
      //  VilPlus::draw_cross(showImage, edgePixel2, 3, VilPlus::red());
      //  VilPlus::draw_segment(showImage, seg1.point1(), seg1.point2(), VilPlus::white());
      //  VilPlus::draw_segment(showImage, seg2.point1(), seg2.point2(), VilPlus::blue());
      //  VilPlus::draw_segment(showImage, lineSeg.point1(), lineSeg.point2(), VilPlus::red());
        VilPlus::draw_dot(showImage, edgePixel1, VilPlus::green());
        VilPlus::draw_dot(showImage, edgePixel2, VilPlus::red());
        VilPlus::vil_save(showImage, "half_edge.png");
    }
    
    // find middle line
    vgl_line_2d<double> centerLine;
    vcl_vector<vgl_point_2d<double> > center_pts;
    bool isFindCenterLine = false;
    if (edgePixel1.size() > edgePixel2.size()) {
        isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(edgePixel1, edgePixel2, center_pts, centerLine);
    }
    else
    {
        isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(edgePixel2, edgePixel1, center_pts, centerLine);
    }
    
    if (!isFindCenterLine) {
//        printf("Line tracking failed: can not find center line.\n");
        return false;
    }
    
    if (0) {
        // draw center line
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        
        VilPlus::draw_line(showImage, centerLine, VilPlus::blue(), 1);
        VilPlus::vil_save(showImage, "center_line.png");        
    }
    
    
    // center line should close to major pixels
    vgl_point_2d<double> p3 = vgl_closest_point(centerLine, p1);
    vgl_point_2d<double> p4 = vgl_closest_point(centerLine, p2);
    
    // verify detected line
    double d1 = vgl_distance(p1, centerLine);
    double d2 = vgl_distance(p2, centerLine);
    double avg_dis = (d1 + d2)/2.0;
    if (avg_dis > para.avg_end_point_distance_) {
        printf("Line tracking failed: average distance is %f\n", avg_dis);
        return false;
    }
    
    line = centerLine;    
    cenerLineSeg = vgl_line_segment_2d<double>(p3, p4);    
    return true;
}
 */

bool VilLineTracking::trackingLinePixelInOneDirection(const vil_image_view<double> & magnitude, const vgl_line_segment_2d<double> & initLineSeg,
                                                      const LinesegmntTrackingParameter & para, vcl_vector<vgl_point_2d<double> > & edgePixels)
{
    assert(magnitude.nplanes() == 1);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    // sample line pixels
    
    vgl_point_2d<double> p1 = initLineSeg.point1();
    vgl_point_2d<double> p2 = initLineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linetPts;
    VilPlus::draw_line(p1, p2, linetPts, w, h);
    if (linetPts.size() <= 10) {
        // line segment too short
        printf("init line pixel number is %lu. Too short.\n", linetPts.size());
        return false;
    }
    
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
    int dirInd = para.scan_line_canny_direction_;
    assert(dirInd >= 0 && dirInd <= 7);
    const int dx = scanDirection[dirInd][0];
    const int dy = scanDirection[dirInd][1];
    
   // printf("dirction index is %d\n", dirInd);
    
    // positive direction
    vcl_vector<vgl_point_2d<double> > edge1Pixels;
    
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();
        int y = linetPts[i].y();
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j < para.line_width_; j++) {
            if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                continue;
            }
            if (num >= 1) {
                if (magnitude(x, y) > para.mag_threshold_1_ &&
                    magnitude(x, y) < para.mag_threshold_2_ &&
                    magnitude(x, y) >= para.lambda_ * sumMag/num) {
                    edge1Pixels.push_back(vgl_point_2d<double>(x, y));
                    break;
                }
            }
            sumMag += magnitude(x, y);
            num++;
            
            x += dx;
            y += dy;
        }
    }
    
    // fit lines from edge pixels
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = 15;
    vgl_line_2d<double> line1;
    bool isLine1 = VxlHoughLine::detectOneLine(edge1Pixels, w, h, houghPara, line1, edgePixels);
    
    if (!isLine1) {
     //   printf("can not find the line: pixel number, min pixel number, inliner number is %lu %d %lu\n", edge1Pixels.size(), houghPara.minPixeNum_, edgePixels.size());
        return false;
    }
    
    return true;
}

bool VilLineTracking::trackingLinePixelInOneDirectionByPeakRMCAM(const vil_image_view<double> & magnitude, const vgl_line_segment_2d<double> & initLineSeg,
                                                                 const LinesegmntTrackingParameter & para, vcl_vector<vgl_point_2d<double> > & edgePixels)
{
    assert(magnitude.nplanes() == 1);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    // sample line pixels
    
    vgl_point_2d<double> p1 = initLineSeg.point1();
    vgl_point_2d<double> p2 = initLineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linetPts;
    VilPlus::draw_line(p1, p2, linetPts, w, h);
    if (linetPts.size() <= 10) {
        // line segment too short
        printf("init line pixel number is %lu. Too short.\n", linetPts.size());
        return false;
    }
    
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
    int dirInd = para.scan_line_canny_direction_;
    assert(dirInd >= 0 && dirInd <= 7);
    const int dx = scanDirection[dirInd][0];
    const int dy = scanDirection[dirInd][1];
    
  //  printf("dirction index is %d\n", dirInd);
    
    // positive direction
    vcl_vector<vgl_point_2d<double> > edge1Pixels;
    
    // int VnlPlus::maxinumRMCAM(const vcl_vector<double> & data)
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();
        int y = linetPts[i].y();
        
        vcl_vector<double> scan_line_mag;
        for (int j = 0; j < para.line_width_; j++) {
            if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                break;
            }
            scan_line_mag.push_back(magnitude(x, y));
            x += dx;
            y += dy;
        }
        double max_rmcam = 0.0;
        if (!scan_line_mag.empty()) {
            int idx = VnlPlus::maxinumRMCAM(scan_line_mag, max_rmcam);
            x = linetPts[i].x();
            y = linetPts[i].y();
            x += idx * dx;
            y += idx * dy;
            vgl_point_2d<double> p(x, y);
            assert(vgl_inside_image(p, w, h));
            if (magnitude(x, y) > para.mag_threshold_1_ &&
                magnitude(x, y) < para.mag_threshold_2_ &&
                max_rmcam > para.lambda_) {
                edge1Pixels.push_back(p);
            }
        }
    }
    
    // fit lines from edge pixels
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = 15;
    vgl_line_2d<double> line1;
    vcl_vector<vgl_point_2d<double> > inlier_pts1;
    bool isLine1 = VxlHoughLine::detectOneLine(edge1Pixels, w, h, houghPara, line1, edgePixels);
    
    if (!isLine1) {
    //    printf("can not find the line: pixel number, min pixel number, inliner number is %lu %d %lu\n", edge1Pixels.size(), houghPara.minPixeNum_, edgePixels.size());
        return false;
    }
    
    return true;
}

bool VilLineTracking::trackingLinePixel(const vil_image_view<double> & magnitude, const vgl_line_segment_2d<double> & initLineSeg,
                                        const LinesegmntTrackingParameter & para, vcl_vector<vgl_point_2d<double> > & edge1Pixels,
                                        vcl_vector<vgl_point_2d<double> > & edge2Pixels)
{
    assert(magnitude.nplanes() == 1);
    
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    // sample line pixels
    
    vgl_point_2d<double> p1 = initLineSeg.point1();
    vgl_point_2d<double> p2 = initLineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linetPts;
    VilPlus::draw_line(p1, p2, linetPts, w, h);
    if (linetPts.size() <= 10) {
        // line segment too short
        printf("init line pixel number is %lu. Too short.\n", linetPts.size());
        return false;
    }
    
    // direction is perpandicular to the canny direction
    int scanDirection[][2] = {0, 1, 1, 0,
                              1, 1, -1, -1};
    
    // calculate scan direction
    int dirInd = VilLineTracking::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    assert(dirInd >= 0 && dirInd < 4);
    const int dx = scanDirection[dirInd][0];
    const int dy = scanDirection[dirInd][1];
    
  //  printf("dirction index is %d\n", dirInd);
    
    // positive direction
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();
        int y = linetPts[i].y();
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j < para.line_width_; j++) {
            if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                continue;
            }
            
            if (num >= 1) {
                if (magnitude(x, y) > para.mag_threshold_1_ &&
                    magnitude(x, y) < para.mag_threshold_2_ &&
                    magnitude(x, y) >= para.lambda_ * sumMag/num) {
                    edge1Pixels.push_back(vgl_point_2d<double>(x, y));
                    break;
                }
            }
            sumMag += magnitude(x, y);
            num++;
            
            x += dx;
            y += dy;
        }
    }
    
    // negative direction
    for (int i = 0; i<linetPts.size(); i++) {
        int x = linetPts[i].x();
        int y = linetPts[i].y();
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j < para.line_width_; j++) {
            if (!vgl_inside_image(vgl_point_2d<double>(x, y), w, h)) {
                break;
            }
            if (num >= 1) {
                if (magnitude(x, y) > para.mag_threshold_1_ &&
                    magnitude(x, y) < para.mag_threshold_2_ &&
                    magnitude(x, y) >= para.lambda_ * sumMag/num) {
                    edge2Pixels.push_back(vgl_point_2d<double>(x, y));
                    break;
                }
            }
            sumMag += magnitude(x, y);
            num++;
            
            x -= dx;
            y -= dy;
        }
    }
    
    printf("find %lu %lu edge pixels\n", edge1Pixels.size(), edge2Pixels.size());
    
    // fit lines from edge pixels
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = vgl_distance(p1, p2) * para.pixel_num_ratio_;
    vgl_line_2d<double> line1;
    vgl_line_2d<double> line2;
    vcl_vector<vgl_point_2d<double> > inlier_pts1;
    vcl_vector<vgl_point_2d<double> > inlier_pts2;
    bool isLine1 = VxlHoughLine::detectOneLine(edge1Pixels, w, h, houghPara, line1, inlier_pts1);
    bool isLine2 = VxlHoughLine::detectOneLine(edge2Pixels, w, h, houghPara, line2, inlier_pts2);
    
    if (!isLine1 && !isLine2) {
        printf("can not find the line. min pixel number is %d\n", houghPara.minPixeNum_);
        return false;
    }
    // pick one of the lines that has more inliers
    vgl_line_2d<double> line = line1;
    if (inlier_pts2.size() > inlier_pts1.size()) {
        line = line2;
    }
    
    // check direction should be parallel
    double cosAngle = fabs(cos_angle(initLineSeg.direction(), line.direction()));
    if (cosAngle < para.pixel_num_ratio_) {
        printf("Line traking failed: tracked line and initial line are not parallel.\n");
        return false;
    }
    
    // project initial line segment end point to tracked line
    vgl_point_2d<double> p3 = vgl_closest_point(line, p1);
    vgl_point_2d<double> p4 = vgl_closest_point(line, p2);
    
    double d1 = vgl_distance(p1, p3);
    double d2 = vgl_distance(p2, p4);
    printf("initial line and tracked line average distance is %f\n",(d1 + d2)/2.0);
    
    
    return true;
}

int VilLineTracking::cannyDirection(double dx, double dy)
{
    int dir = 0;
    if(VnlPlus::isEqualZero(dx))
    {
        if(VnlPlus::isEqualZero(dy))
        {
            dir = 0;
        }
        else
        {
            dir = 2;
        }
    }
    else
    {
        // 0.4142 --> 22.5^o, 2.4142--> 657.5^o
        double tanV = dy/dx;
        if(tanV > -0.4142f && tanV <= 0.4142f)
        {
            dir = 0;
        }
        else if(tanV > 0.4142f && tanV < 2.4142f)
        {
            dir = 1;
        }
        else if(abs(tanV) >= 2.4142f)
        {
            dir = 2;
        }
        else if(tanV > -2.4142f && tanV <= -0.4142f)
        {
            dir = 3;
        }
    }
    return dir;
}


