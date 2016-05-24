//
//  vxl_half_edge_line_tracking.cpp
//  OpenCVCalib
//
//  Created by jimmy on 9/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vxl_half_edge_line_tracking.h"
#include "vxl_ann.h"
#include <vgl/vgl_fit_line_2d.h>
#include <vil/vil_image_view.h>
#include "vil_line_tracking.h"
#include "vil_plus.h"
#include "vil_algo_plus.h"
#include "vgl_plus.h"
#include <vgl/vgl_closest_point.h>
#include <vgl/vgl_distance.h>
#include <vnl/vnl_matlab_filewrite.h>



bool vxl_half_edge_line_tracking::trackingLineFromSegment(const vil_image_view<vxl_byte> & image,
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
    
    int cannyDir = VilAlgoPlus::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
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
        
        int dir = VilAlgoPlus::cannyDirection(dx, dy); //
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



bool vxl_half_edge_line_tracking::estimateCenterLine(const vcl_vector<vgl_point_2d<double> > & line1Pts, const vcl_vector<vgl_point_2d<double> > &      line2Pts,                        vcl_vector<vgl_point_2d<double> > & centerPts, vgl_line_2d<double> & centerLine)
{
    assert(line1Pts.size() >= line2Pts.size());
    // VxlANNQuadTree(int k, int dim);
    VxlANNQuadTree quadTree(1, 2);
    quadTree.set_tree(line1Pts);
    
    for (int i = 0; i<line2Pts.size(); i++) {
        vgl_point_2d<double> p = line2Pts[i];
        vgl_point_2d<double> q = quadTree.nearest(p);
        double x = (q.x() + p.x())/2.0;
        double y = (q.y() + p.y())/2.0;
        centerPts.push_back(vgl_point_2d<double>(x, y));
    }
    
    if (centerPts.size() < 5) {
        return false;
    }
    
    centerLine = vgl_fit_line_2d(centerPts);
    return true;
}

bool vxl_half_edge_line_tracking::detectLinePixelByAverageGradient(const vil_image_view<double> & mag, const vil_image_view<vxl_byte> & maskImage,
                                                  const AverageMagnitudeParameter & para, vcl_vector<vgl_point_2d<double> > & edge1Pixels,
                                                  vcl_vector<vgl_point_2d<double> > & edge2Pixels)
{
    assert(mag.ni() == maskImage.ni());
    assert(mag.nj() == maskImage.nj());
    assert(mag.nplanes() == 1);
    
    assert(para.direction_ >= 0 && para.direction_ <= 3);
    
    // int direction_;        // 0 horizontal, 1 vertical, 2, left_top to right_buttom diagonal, 3, left_buttom to right_top diagonal
    const int w = mag.ni();
    const int h = mag.nj();
    if (para.direction_ == 0) {
        
        // for test
        vnl_matrix<double> avgGradientMat(h, w, 0);
        vnl_matrix<double> curGradientMat(h, w, 0);
        // from left to right
        for (int j = 0; j<h; j++) {
            double sumMag = 0.0;
            int num = 0;
            for (int i = 0; i<w; i++) {
                if (maskImage(i, j) == 255) {
                    if (num >= 1) {
                        if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                            edge1Pixels.push_back(vgl_point_2d<double>(i, j));
                            break;
                        }
                    }
                    sumMag += mag(i, j);
                    num++;
                    if (num >= 1) {
                        avgGradientMat(j, i) = sumMag/num;
                        curGradientMat(j, j) = mag(i, j);
                    }
                }
            }
        }
        
        // for test
        if(0)
        {
            vnl_matlab_filewrite writer("avgGradient.mat");
            writer.write(avgGradientMat, "avg_gradient");
            writer.write(curGradientMat, "cur_gradient");
            printf("save to %s\n", "avgGradient.mat");
        }
        
        // from right to left
        for (int j = 0; j<h; j++) {
            double sumMag = 0.0;
            int num = 0;
            for (int i = w-1; i >= 0; i--) {
                if (maskImage(i, j) == 255) {
                    if (num >= 1) {
                        if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                            edge2Pixels.push_back(vgl_point_2d<double>(i, j));
                            break;
                        }
                    }
                    sumMag += mag(i, j);
                    num++;
                }
            }
        }
    }
    else if(para.direction_ == 1)
    {
        // up to down
        for (int i = 0; i<w; i++ ) {
            double sumMag = 0.0;
            int num = 0;
            for (int j = 0; j<h; j++) {
                if (maskImage(i, j) == 255) {
                    if (num >= 1) {
                        if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                            edge1Pixels.push_back(vgl_point_2d<double>(i, j));
                            break;
                        }
                    }
                    sumMag += mag(i, j);
                    num++;
                }
            }
        }
        
        // down to up
        for (int i = 0; i<w; i++) {
            double sumMag = 0.0;
            int num = 0;
            for (int j = h-1; j >= 0; j--) {
                if (maskImage(i, j) == 255) {
                    if (num >= 1) {
                        if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                            edge2Pixels.push_back(vgl_point_2d<double>(i, j));
                            break;
                        }
                    }
                    sumMag += mag(i, j);
                    num++;
                }
            }
        }
    }
    else if(para.direction_ == 2)
    {
        // from left to right
        for (int j = 0; j<h; j++) {
            bool isFirst = false;
            for (int i = 0; i<w; i++) {
                // find first mask position
                if (!isFirst && maskImage(i, j) == 255) {
                    isFirst = true;
                    double sumMag = 0.0;
                    int num = 0;
                    int x = i;
                    int y = j;
                    while (x < w && y >= 0) {
                        if (maskImage(x, y) == 255) {
                            if (num >= 1) {
                                if (mag(x, y) > para.mag_threshold_ && mag(x, y) >= para.lambda_ * sumMag/num) {
                                    edge1Pixels.push_back(vgl_point_2d<double>(x, y));
                                    break;
                                }
                            }
                            sumMag += mag(i, j);
                            num++;
                        }
                        x++;
                        y--;
                    }
                }
                if (isFirst) {
                    break;
                }
            }
        }
        
        // from right to left
        for (int j = 0; j<h; j++) {
            bool isFirst = false;
            for (int i = w-1; i >= 0; i--) {
                // find first mask position
                if (!isFirst && maskImage(i, j) == 255) {
                    isFirst = true;
                    double sumMag = 0.0;
                    int num = 0;
                    int x = i;
                    int y = j;
                    while (x >= 0 && y < h) {
                        if (maskImage(x, y) == 255) {
                            if (num >= 1) {
                                if (mag(x, y) > para.mag_threshold_ && mag(x, y) >= para.lambda_ * sumMag/num) {
                                    edge2Pixels.push_back(vgl_point_2d<double>(x, y));
                                    break;
                                }
                            }
                            sumMag += mag(i, j);
                            num++;
                        }
                        x--;
                        y++;
                    }
                }
                if (isFirst) {
                    break;
                }
            }
        }
    }
    else if(para.direction_ == 3)
    {
        // from left to right
        for (int j = 0; j<h; j++) {
            bool isFirst = false;
            for (int i = 0; i<w; i++) {
                // find first mask position
                if (!isFirst && maskImage(i, j) == 255) {
                    isFirst = true;
                    double sumMag = 0.0;
                    int num = 0;
                    int x = i;
                    int y = j;
                    while (x < w && y < h) {
                        if (maskImage(x, y) == 255) {
                            if (num >= 1) {
                                if (mag(x, y) > para.mag_threshold_ && mag(x, y) >= para.lambda_ * sumMag/num) {
                                    edge1Pixels.push_back(vgl_point_2d<double>(x, y));
                                    break;
                                }
                            }
                            sumMag += mag(i, j);
                            num++;
                        }
                        x++;
                        y++;
                    }
                }
                if (isFirst) {
                    break;
                }
            }
        }
        
        // from right to left
        for (int j = 0; j<h; j++) {
            bool isFirst = false;
            for (int i = w-1; i >= 0; i--) {
                // find first mask position
                if (!isFirst && maskImage(i, j) == 255) {
                    isFirst = true;
                    double sumMag = 0.0;
                    int num = 0;
                    int x = i;
                    int y = j;
                    while (x >= 0 && y >= 0) {
                        if (maskImage(x, y) == 255) {
                            if (num >= 1) {
                                if (mag(x, y) > para.mag_threshold_ && mag(x, y) >= para.lambda_ * sumMag/num) {
                                    edge2Pixels.push_back(vgl_point_2d<double>(x, y));
                                    break;
                                }
                            }
                            sumMag += mag(i, j);
                            num++;
                        }
                        x--;
                        y--;
                    }
                }
                if (isFirst) {
                    break;
                }
            }
        }
    }
    return true;
}

bool vxl_half_edge_line_tracking::detectEllipsePixelByAverageGradient(const vil_image_view<double> & mag,
                                                     const vil_image_view<vxl_byte> & mask,
                                                     const vil_image_view<vxl_byte> & nonEllipseMask,
                                                     const AverageMagnitudeParameter & para,
                                                     vcl_vector<vgl_point_2d<double> > & outsideUpPixels,
                                                     vcl_vector<vgl_point_2d<double> > & outsideDownPixels)
{
    const int w = mag.ni();
    const int h = mag.nj();
    assert(w == mask.ni() && h == mask.nj());
    assert(w == nonEllipseMask.ni() && h == nonEllipseMask.nj());
    assert(para.direction_ == 1);
    
    // up to down
    for (int i = 0; i<w; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j<h; j++) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideUpPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    // down to up
    for (int i = 0 ; i<w; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = h-1; j >= 0; j--) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideDownPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    
    return true;
}

bool vxl_half_edge_line_tracking::detectLeftPenaltyEllipseByAverageGradient(const vil_image_view<double> & mag,
                                                           const vil_image_view<vxl_byte> & mask,
                                                           const vil_image_view<vxl_byte> & nonEllipseMask,
                                                           const AverageMagnitudeParameter & para,
                                                           vcl_vector<vgl_point_2d<double> > & outsideHorizontalPixels,
                                                           vcl_vector<vgl_point_2d<double> > & outsideVerticalPixels)
{
    const int w = mag.ni();
    const int h = mag.nj();
    assert(w == mask.ni() && h == mask.nj());
    assert(w == nonEllipseMask.ni() && h == nonEllipseMask.nj());
    
    // right to left
    for (int j = 0; j<h; j++) {
        double sumMag = 0.0;
        int num = 0;
        for (int i = w-1; i >= 0; i-- ) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideHorizontalPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    
    // down to up
    for (int i = 0; i<w; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = h-1; j >= 0; j--) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideVerticalPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    return true;
}

bool vxl_half_edge_line_tracking::detectRightPenaltyEllipseByAverageGradient(const vil_image_view<double> & mag,
                                                            const vil_image_view<vxl_byte> & mask,
                                                            const vil_image_view<vxl_byte> & nonEllipseMask,
                                                            const AverageMagnitudeParameter & para,
                                                            vcl_vector<vgl_point_2d<double> > & outsideHorizontalPixels,
                                                            vcl_vector<vgl_point_2d<double> > & outsideVerticalPixels)
{
    const int w = mag.ni();
    const int h = mag.nj();
    assert(w == mask.ni() && h == mask.nj());
    assert(w == nonEllipseMask.ni() && h == nonEllipseMask.nj());
    
    // left to right
    for (int j = 0; j<h; j++) {
        double sumMag = 0.0;
        int num = 0;
        for (int i = 0; i < w; i++ ) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideHorizontalPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    
    // down to up
    for (int i = 0; i<w; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = h-1; j >= 0; j--) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideVerticalPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    return true;
}





