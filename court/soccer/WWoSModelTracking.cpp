//
//  WWoSModelTracking.cpp
//  OnlineStereo
//
//  Created by jimmy on 9/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "WWoSModelTracking.h"
#include "wwosSoccerCourt.h"
#include "SoccerShortLineModel.h"
#include "vil_plus.h"
#include "vil_line_tracking.h"
#include "vxl_half_edge_line_tracking.h"
#include "vgl_plus.h"
#include "vil_ellipse_tracking.h"
#include "vxl_ELSD.h"
#include "cvx_LSD.h"
#include "vpgl_ptz_estimation.h"

bool WWoSModelTracking::modelTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                         const LineTrackingParameter & para, SoccerModelTrackingResult & model, bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    WWoSSoccerCourt wwosCourt;
    // 1. predict line segment in the image
    SoccerShortLineModel sslModel;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sslModel.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    // 2. tracking lines in the image space
    // edge detection data
    vil_image_view<double> maganitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, maganitude, grad_i, grad_j, false);
    
    vcl_vector<SGCCalibLine> detectedCalibLines;  // verified line segment
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        const int nodeId = calibLine.node();
        
        if (nodeId == 0) {
            int test = 1;
        }
        
        LinesegmntTrackingParameter para;
        para.lambda_ = sslModel.nodeLambda(nodeId);
        para.mag_threshold_1_ = sslModel.nodeMagnitude(nodeId);
        para.line_width_      = sslModel.nodeWidth(nodeId);
        para.online_pixel_ratio_ = sslModel.nodeOnlineRatio(nodeId);
        
        vgl_line_segment_2d<double> initLineSeg;
        vgl_line_segment_2d<double> refinedLineSeg;
        vgl_line_segment_2d<double> finalLineSeg;
        vgl_line_2d<double> finaline;
        bool isFindFinal = false;
        
        bool isSegment = sslModel.projectNode(camera, nodeId, initLineSeg);
        if (!isSegment) {
            continue;
        }
        // initial
        bool isInitGoodEnough = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, initLineSeg, para, para.online_pixel_ratio_);
        if (!isInitGoodEnough) {
            // refine the initial line segment first
            bool isRefined = VilLineTracking::refineLineSegment(image, maganitude, grad_i, grad_j, initLineSeg, para, refinedLineSeg);
            if (!isRefined) {
                if (!silent) {
                    printf("Warning: refine segment failed\n");
                }
                continue;
            }
        }
        else
        {
            refinedLineSeg = initLineSeg;
        }
        
        // double edge refinement
        if (sslModel.hasDoubleEdge(nodeId)) {
            para.line_width_ /= 2.0;
            isFindFinal = vxl_half_edge_line_tracking::trackingLineFromSegment(image, maganitude, grad_i, grad_j, refinedLineSeg, para, finaline, finalLineSeg);
        }
        else
        {
            finaline = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
            finalLineSeg = refinedLineSeg;
            isFindFinal = true;
        }
        
        if (isFindFinal) {
            // check if it is a good line segment
            bool isAnEdge = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, finalLineSeg, para, para.online_pixel_ratio_);
            if (isAnEdge) {
                vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
                vgl_vector_2d<double> dir2 = finaline.direction();
                double cosAngle = fabs(cos_angle(dir1, dir2));
                
                if (cosAngle > lineDirectionThreshold) {
                    calibLine.setDetectedLine(finaline);
                    calibLine.detected_image_line_segment_ = finalLineSeg;
                    detectedCalibLines.push_back(calibLine);
                    
                    if(!silent)
                    {
                        // draw detected line
                        vil_image_view<vxl_byte> showImage;
                        showImage.deep_copy(image);
                        VilPlus::draw_segment(showImage, initLineSeg, VilPlus::red());
                        VilPlus::draw_line(showImage, finaline, VilPlus::green());
                        char buf[1024] = {NULL};
                        sprintf(buf, "center_line_%d.jpg", calibLine.node());
                        VilPlus::vil_save(showImage, buf);
                    }
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        if (!silent) {
            printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        }
        return false;
    }
    printf("detected %lu lines\n", detectedCalibLines.size());
    
    if (!silent) {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            VilPlus::draw_segment(showImage, detectedCalibLines[i].detected_image_line_segment_, VilPlus::green());
        }
        VilPlus::vil_save(showImage, "detected_linesegment.jpg");
    }
    
    // calibration correspondence
    vcl_vector<vgl_point_3d<double> > wld_pts;  // line intersection
    vcl_vector<vgl_point_2d<double> > img_pts;
    
    
    // 3. tracking center circle in the image
    // check ellipse inside image
    bool hasCenterCircle = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    bool isCenterCircleTracked = false;
    vgl_ellipse_2d<double> centerEllipse;
    if (hasCenterCircle) {
        // tracking center circle in image
        ElliseTrackingParameter centerCircleTrackingPara;
        
        wwosCourt.centerCircleBoundingBox(camera, w, h, centerCircleTrackingPara.bounding_box_, 5, 5);
        isCenterCircleTracked = VilEllipseTracking::trackingInBoundingBox(image, maganitude, centerCircleTrackingPara, centerEllipse);
        if (isCenterCircleTracked) {
            // line and center circle intersection
            bool isCenterLine = false;
            SGCCalibLine centerLine;
            for (int i = 0; i<detectedCalibLines.size(); i++) {
                if (detectedCalibLines[i].node() == 0) {
                    isCenterLine = true;
                    centerLine = detectedCalibLines[i];
                    break;
                }
            }
            isCenterCircleTracked = true;
            if (isCenterLine) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                bool isIntersect = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), centerEllipse, pt1, pt2, true);
                if (isIntersect) {
                    SoccerGraphCutUtil sgcUtil;
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    for (int i = 0; i<pts_paris.size(); i++) {
                        wld_pts.push_back(vgl_point_3d<double>(pts_paris[i].first.x(), pts_paris[i].first.y(), 0.0));
                        img_pts.push_back(pts_paris[i].second);
                    }
                    
                    if (!silent) {
                        printf("find circle line intersection\n");
                    }
                }
            }
        }
    }
    
    // left  ellipse
    bool has_left_circle = WWoSSoccerCourt::isPenaltyEllipseInsideImage(camera, w, h, true);
    bool is_left_circle_tracked = false;
    vgl_ellipse_2d<double> left_ellipse;
    vcl_vector<vgl_point_2d<double> > left_ellipse_points;
    if (has_left_circle) {
        vgl_box_2d<int> box;
        wwosCourt.penaltyEllipseBoundingBox(camera, w, h, box, true, 5, 5);
        vil_image_view<vxl_byte> sub_image = vil_crop(image, box.min_x(), box.width(), box.min_y(), box.height());
        is_left_circle_tracked = VxlELSD::detect_largest_ellipse(sub_image, left_ellipse, left_ellipse_points);
        // shift position
        for (int i = 0; i<left_ellipse_points.size(); i++) {
            left_ellipse_points[i] = vgl_point_2d<double>(left_ellipse_points[i].x() + box.min_x(), left_ellipse_points[i].y() + box.min_y());
        }
    }
    
    
    // right ellipse
    
    vcl_vector<int> detected_nodes;
    // 4. find correspondence between line intersections
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sslModel.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (VglPlus::vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(vgl_point_3d<double>(wld_pt.x(), wld_pt.y(), 0.0));
                        img_pts.push_back(img_pt);
                        detected_nodes.push_back(node1);
                        detected_nodes.push_back(node2);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    vcl_sort(detected_nodes.begin(), detected_nodes.end());
    vcl_vector<int>::iterator ite = vcl_unique(detected_nodes.begin(), detected_nodes.end());
    detected_nodes.erase(ite, detected_nodes.end());
    if (!silent) {
        printf("find %lu interextion nodes\n", detected_nodes.size());
        printf("find %lu linesegments in image\n", detectedCalibLines.size());
    }
    
    
    
    if (wld_pts.size() < 2) {
        if (!silent) {
            printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        }
        return false;
    }
    printf("find %lu intersections\n", wld_pts.size());
    
    if(!silent)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, img_pts, 3, VilPlus::green());
        if (isCenterCircleTracked) {
            VilPlus::draw_ellipse(showImage, centerEllipse, VilPlus::green());
            vcl_vector<vgl_point_2d<double> > ellipse_pts;
            VglPlus::sampleElliseInImage(centerEllipse, 10, w, h, ellipse_pts);
            VilPlus::draw_cross(showImage, ellipse_pts, 5, VilPlus::red());
        }
        char buf[1024] = {NULL};
        sprintf(buf, "geometry_intersection.jpg");
        VilPlus::vil_save(showImage, buf);
    }
    
    model.detected_calib_Lines_ = detectedCalibLines;
    model.wld_pts_ = wld_pts;
    model.img_pts_ = img_pts;
    model.detected_node_id_ = detected_nodes;
    model.is_centercircle_tracked_ = isCenterCircleTracked;
    if (isCenterCircleTracked) {
        model.center_ellipse_ = centerEllipse;
    }
    
    model.is_left_ellipe_detected_ = is_left_circle_tracked;
    if (is_left_circle_tracked) {
        model.left_ellipse_ = left_ellipse;
        model.left_ellipse_points_ = left_ellipse_points;
    }
    return true;
}


bool WWoSModelTracking::refineCameraByShortLineAndEllipseOptimizeByDetectPointOnline(const vil_image_view<vxl_byte> & image,
                                                                                        const vpgl_perspective_camera<double> & initCamera,
                                                                                        const LineTrackingParameter & para,
                                                                                        vpgl_perspective_camera<double> & finalCamera, bool silent)
{
    SoccerModelTrackingResult model;
    bool isTracked = WWoSModelTracking::modelTracking(image, initCamera, para, model, silent);
    if (!isTracked) {
        return false;
    }
    
    vcl_vector<SGCCalibLine> detectedCalibLines = model.detected_calib_Lines_;
    vcl_vector<vgl_point_3d<double> > wld_pts = model.wld_pts_;
    vcl_vector<vgl_point_2d<double> > img_pts = model.img_pts_;
    bool isCenterCircleTracked = model.is_centercircle_tracked_;
    vgl_ellipse_2d<double> centerEllipse = model.center_ellipse_;
    bool is_left_ellipse_tracked = model.is_left_ellipe_detected_;
    vgl_ellipse_2d<double> leftEllipse = model.left_ellipse_;
    vcl_vector<vgl_point_2d<double> > left_ellipse_points = model.left_ellipse_points_;
    
    
    WWoSSoccerCourt wwosCourt;
    const int w = image.ni();
    const int h = image.nj();
    
    // 5. camera optimization
    LinePointsInCameraview linePoints;
    linePoints.wld_pts_ = wld_pts;
    linePoints.img_pts_ = img_pts;
    linePoints.camera_ = initCamera;
    const double segment_image_length = 50;
    
    vil_image_view<vxl_byte> show_image;
    show_image.deep_copy(image);
    
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        PointsOnLine pol;
        vcl_vector<vgl_point_2d<double> > pts;
        pol.line_ = vgl_line_3d_2_points<double>(detectedCalibLines[i].point1_3d(), detectedCalibLines[i].point2_3d());
        
        vgl_line_segment_2d<double> img_seg = detectedCalibLines[i].detected_image_line_segment_;
        
        vgl_point_2d<double> q1 = img_seg.point1();
        vgl_point_2d<double> q2 = img_seg.point2();
        double distance = vgl_distance(q1, q2);
        int num = distance/segment_image_length;  // the number is decided by its length in image space
        
        for (int j = 0; j<num; j++) {
            double t1 = 1.0 * j / num;
            double t2 = 1.0 * (j+1)/num;
            vgl_point_2d<double> p3 = img_seg.point_t(t1);
            vgl_point_2d<double> p4 = img_seg.point_t(t2);
            
            vgl_box_2d<double> box(p3, p4);
            box.expand_about_centroid(15);
            
            VglPlus::clamp_box(box, 0, 0, w-1, h-1);
            if (box.width() > 5 && box.height() > 5) {
                vil_image_view<vxl_byte> sub_image = vil_crop(image, box.min_x(), box.width(), box.min_y(), box.height());
                vgl_line_segment_2d<double> sub_line_seg;
                bool is_detected = CvxLSD::detect_longest_line(sub_image, sub_line_seg);
                if (is_detected) {
                    vgl_point_2d<double> p5(sub_line_seg.point1().x() + box.min_x(), sub_line_seg.point1().y() + box.min_y());
                    vgl_point_2d<double> p6(sub_line_seg.point2().x() + box.min_x(), sub_line_seg.point2().y() + box.min_y());
                    
                    pol.pts_.push_back(centre(p5, p6));
                    VilPlus::draw_segment(show_image, p5, p6, VilPlus::green());
                }
            }
        }
        if (!pol.pts_.empty()) {
            linePoints.lines_.push_back(pol);
        }
    }
    if (isCenterCircleTracked) {
        PointsOnCircle circle;
        circle.circle_ = wwosCourt.getCenterCircle();
        VglPlus::sampleElliseInImage(centerEllipse, 10, w, h, circle.pts_);
        
        if (circle.pts_.size() != 0) {
            linePoints.circles_.push_back(circle);
            VilPlus::draw_cross(show_image, circle.pts_, 5, VilPlus::green());
        }
    }
    if (is_left_ellipse_tracked) {
        PointsOnCircle circle;
        circle.circle_ = wwosCourt.getLeftCircle();
        
        // randomly sample 3 points
        if (left_ellipse_points.size() > 10) {
            for (int i = 5; i<left_ellipse_points.size()-5; i += left_ellipse_points.size()/4) {
                circle.pts_.push_back(left_ellipse_points[i]);
            }
        }
        if (circle.pts_.size() != 0) {
            linePoints.circles_.push_back(circle);
            VilPlus::draw_cross(show_image, circle.pts_, 5, VilPlus::red());
        }
    }
    
    //  VilPlus::vil_save(show_image, "lsd_line_segment.jpg");
    //  VxlOpenCVImage::imshow(show_image, "LSD line, ellipse");
    
    bool isEstimated = VpglPTZEstimation::estimageCamera(linePoints, finalCamera, false);
    return isEstimated;
}


