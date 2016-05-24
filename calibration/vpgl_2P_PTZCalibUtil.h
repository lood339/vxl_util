//
//  vpgl_2P_PTZCalibUtil.h
//  QuadCopter
//
//  Created by jimmy on 6/28/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vpgl_2P_PTZCalibUtil__
#define __QuadCopter__vpgl_2P_PTZCalibUtil__

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vnl/vnl_vector_fixed.h>
#include <vpgl/vpgl_perspective_camera.h>
#include "vpgl_2P_PTZ_calib.h"
#include "vpgl_ptz_camera.h"


class Vpgl2PPTZCalibUtil
{
public:
    // estimate PTZ camera parameters from 3 points
    // only for soccer main PTZ camera in WWoS
    static bool wwosSoccerMainPTZ_calib(const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                        const vcl_vector<vgl_point_2d <double> > & img_pts,
                                        vnl_vector_fixed<double, 3> & ptz,
                                        vpgl_perspective_camera<double> & camera);
    // data only for soccer sideview PTZ in WWoS
    static bool wwosSoccerLeftSideviewPTZ_calib(const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                                const vcl_vector<vgl_point_2d <double> > & img_pts,
                                                vnl_vector_fixed<double, 3> & ptz,
                                                vpgl_perspective_camera<double> & camera,
                                                vpgl_ptz_camera & ptzCamera);
    
    // for simulated data
    static bool simulatedPTZ_calib(const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                   const vcl_vector<vgl_point_2d <double> > & img_pts,
                                   vnl_vector_fixed<double, 3> & ptz,
                                   vpgl_ptz_camera & ptzCamera);
    
    // for simulated data
    static bool simulatedPTZ_calib_fl_from_TUM(const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                               const vcl_vector<vgl_point_2d <double> > & img_pts,
                                               vnl_vector_fixed<double, 3> & ptz,
                                               vpgl_ptz_camera & ptzCamera);
    
    
    
};

#endif /* defined(__QuadCopter__vpgl_2P_PTZCalibUtil__) */
