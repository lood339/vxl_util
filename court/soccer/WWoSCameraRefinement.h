//
//  WWoSCameraRefinement.h
//  OnlineStereo
//
//  Created by jimmy on 2/19/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__WWoSCameraRefinement__
#define __OnlineStereo__WWoSCameraRefinement__

#include <vil/vil_image_view.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vgl/vgl_ellipse_2d.h>
//#include "vil_gmm.h"
//#include "vil_gmm_util.h"
#include "vxl_hough_line.h"
#include "SoccerGraphCutUtil.h"
#include "SoccerModelParameters.h"

/*
struct WhiteLineRefinementPara
{
    VilGMM green_gmm_;   // grass model
    VilGMM white_gmm_;   // white line model
    VIlGMMUTILGMMParameter white_pixel_para_;   // white line pixel should be around by green pixels
    VxlHoughParameter hough_line_para_;         // extract one line at one iteration
    int min_distance_to_project_line_;     // this parameter is to remove white pixel in players 5-10
    double sample_length_;                 // sample length from soccer court in meter
    
    // world/image line correspondence
    double line_distance_ratio_;           // ratio: first closed line and second closed line
    double line_direction_threshold_;      // cos (theta)
    
    double max_line_pair_distance_;        //
    
    
    WhiteLineRefinementPara():green_gmm_(2, true), white_gmm_(1, true)
    {
        green_gmm_.read("/Users/jimmy/Desktop/images/WWoS_soccer/gmms/green_field_2_gaussian.gmm");
        white_gmm_.read("/Users/jimmy/Desktop/images/WWoS_soccer/gmms/white_color.gmm");
        
        white_pixel_para_.blue_channel_min_threshold_ = 80;
        
        hough_line_para_.houghThreshold_ = 0.7;
        hough_line_para_.maxLineNum_ = 5;
        hough_line_para_.lineWidth_ = 10.0;
        
        min_distance_to_project_line_ = 20;
        sample_length_ = 0.5;
        
        line_distance_ratio_ = 4.0;
        line_direction_threshold_ = cos(30.0/180*3.14);  // line should have similar direction < 30 degre
        max_line_pair_distance_ = 80.0;
    }
};
 */


class WWoSCameraRefinement
{
public:
    // refine penalty circle and line intersection by pointless calibration
    // p1,p2: initial location in the image. in and out
  //  static bool refinePenaltyLineIntersection(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
      //                                        const PatchMatchParameter & para,
       //                                       vgl_point_2d<double> & p1, vgl_point_2d<double> & p2, bool isLeft);
    // fine initial camera by white line pixels
    // Not robust because only 2-3 white line are successfully detected
   // static bool refineCameraByWhiteLines(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
     //                                    const WhiteLineRefinementPara & para, vpgl_perspective_camera<double> & finalCamera);
    
    // 1, tracking line centers to estimate line
    // 2, using line intersection to refine camera
    // 3, not use ellipse
    static bool refineCameraByLineTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                           const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera);
    
    // only use ellipse
    static bool refineCameraByEllipseTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                              const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera);
    // refine camera by both line and ellipse
    static bool refineCameraByLineAndEllipseTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                     const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent = true);
    
    // refine camera by short line and ellipse tracking, working horse of this project
    // 
    static bool refineCameraByShortLineAndEllipseTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                          const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent = true);
    // change optimization by edge landmark
    static bool refineCameraByShortLineAndEllipseOptimizeByPointOnline(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                                       const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent = true);
    // output tracking result
    static bool modelTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                              const LineTrackingParameter & para, SoccerModelTrackingResult & model, bool silent = true);
    
    // detect short line segment, its center is on the line
    static bool refineCameraByShortLineAndEllipseOptimizeByDetectPointOnline(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                                             const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent = true);
    
    static bool refineCameraByShortLineAndEllipseTracking_debug(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                   const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent = true);
    
    // detect intersection between lines and ellipse
    static bool intersectionDetectionByLineAndEllipseTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                              const LineTrackingParameter & para, vcl_vector<NodeIntersection> & intersections);
    
    // lu's line tracking method
    // to make lu's method work well, use our ellipse tracking method
    static bool refineCameraByShortLineAndEllipseTrackingLuMethod(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                          const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent = true);
    
    // Thomas's line tracking method
    // to make Thoma's method work well, out our ellipse tracking method
    static bool refineCameraByShortLineAndEllipseTrackingThomasMethod(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                                      const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent = true);
    
    // refine center line location
    static bool refineCenterLine(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                 vgl_line_segment_2d<double> & centerLine, bool silent = true);
    
    
    
};

#endif /* defined(__OnlineStereo__WWoSCameraRefinement__) */
