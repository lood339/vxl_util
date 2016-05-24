//
//  SoccerVerificationModel.h
//  OnlineStereo
//
//  Created by jimmy on 8/22/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__SoccerVerificationModel__
#define __OnlineStereo__SoccerVerificationModel__

#include <vcl_vector.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vil/vil_image_view.h>

//
struct EdgeLandmark
{
    vgl_line_segment_2d<double> wld_seg_;
    vgl_line_segment_2d<double> img_seg_;
};

// verify if the camera calibration is right
class SoccerVerificationModel
{
private:
    double field_width_;     // unit is yard
    double field_heigh_;
public:
    SoccerVerificationModel(double w = 118, double h = 70);
    ~SoccerVerificationModel();
    
    // segment_image_length 100 for line, 50 for circle
    void calculate_verification_segment(const vpgl_perspective_camera<double> & camera,
                                        vcl_vector<EdgeLandmark> & landmarks,
                                        double segment_image_length);    
    
    // minMagnitude: default 0.02
    // edge_width: default 3
    vcl_vector<double> onLineRatio(const vil_image_view<vxl_byte> &image, const vcl_vector<EdgeLandmark> & landmarks,
                                   double minMagnitude, double edge_width);
    // faster version
    vcl_vector<double> onlineRatioScan(const vil_image_view<vxl_byte> &image, const vcl_vector<EdgeLandmark> & landmarks,
                                       double minMagnitude, double edge_width);
    
    
private:
    vcl_vector<vgl_line_segment_2d<double> > getLineAndCircleSegment(double width, double height);
    vcl_vector<vgl_line_segment_2d<double> > getWorldLineSegment(double width, double height);
    // approximate circle by multiple line segment
    vcl_vector<vgl_line_segment_2d<double> > getCircleLineSegment(double width, double height);
    
    double onEdgePixelRatio(const vil_image_view<double> & magnitude,
                            const vil_image_view<double> & grad_i,
                            const vil_image_view<double> & grad_j,
                            const vgl_line_segment_2d<double> & lineSeg,
                            const double min_magnitude,
                            const int edge_neighbor_size);
    
    // scan both direction
    double onEdgePixelRatioScan(const vil_image_view<double> & magnitude,
                                const vil_image_view<double> & grad_i,
                                const vil_image_view<double> & grad_j,
                                const vgl_line_segment_2d<double> & lineSeg,
                                const double min_magnitude,
                                const int edge_neighbor_size);
    // 0, 1, 2, 3 of canny drection
    //       2
    //    3     1
    //       0
    int cannyDirection(double dx, double dy);
    
};

#endif /* defined(__OnlineStereo__SoccerVerificationModel__) */
