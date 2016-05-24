//
//  VpglPTZCameraUtil.h
//  OnlineStereo
//
//  Created by jimmy on 6/30/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__VpglPTZCameraUtil__
#define __OnlineStereo__VpglPTZCameraUtil__

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_infinite_line_3d.h>
#include "vpgl_ptz_camera.h"

// Utils for WWoS soccer video
class VpglPTZCameraUtil
{
public:
    
    struct Correspondence
    {
        vcl_vector<vgl_point_2d<double> > wld_pts;
        vcl_vector<vgl_point_2d<double> > img_pts;
        
        // line sets converge into a vanishing point
        vcl_vector<vgl_point_2d<double> > vanishingPoints;
        vcl_vector<vcl_vector<vgl_infinite_line_3d<double> > > lineSets;
    };
    
public:
    // by point correspondence and vanishing points
    static bool refineCameraByPtsAndVPs(const vpgl_ptz_camera & initPTZ, const Correspondence & corres, vpgl_ptz_camera & finalCamera);
    
};


#endif /* defined(__OnlineStereo__VpglPTZCameraUtil__) */
