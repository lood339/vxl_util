//
//  WWoSModelTracking.h
//  OnlineStereo
//
//  Created by jimmy on 9/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__WWoSModelTracking__
#define __OnlineStereo__WWoSModelTracking__

#include "vxl_head_file.h"
#include "SoccerModelParameters.h"


// WWoS soccer court model tracking
class WWoSModelTracking
{
public:
    // output tracking result
    static bool modelTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                              const LineTrackingParameter & para, SoccerModelTrackingResult & model, bool silent = true);
    
    // detect short line segment, its center is on the line
    static bool refineCameraByShortLineAndEllipseOptimizeByDetectPointOnline(const vil_image_view<vxl_byte> & image,
                                                                             const vpgl_perspective_camera<double> & initCamera,
                                                                             const LineTrackingParameter & para,
                                                                             vpgl_perspective_camera<double> & finalCamera,
                                                                             bool silent = true);
    
};

#endif /* defined(__OnlineStereo__WWoSModelTracking__) */
