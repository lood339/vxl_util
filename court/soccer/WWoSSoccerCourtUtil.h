//
//  WWoSSoccerCourtUtil.h
//  QuadCopter
//
//  Created by jimmy on 6/20/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__WWoSSoccerCourtUtil__
#define __QuadCopter__WWoSSoccerCourtUtil__

#include "wwosSoccerCourt.h"
#include "vpgl_ptz_camera.h"

// utility function for wwos soccer field
// only for main PTZ camera
class WWoSSoccerCourtUtil
{
public:
    //
    static bool mapToOverviewImage(const vpgl_perspective_camera<double> & camera, const vil_image_view<vxl_byte> & image,
                                   const vil_image_view<vxl_byte> &overviewTemplate, vil_image_view<vxl_byte> & composedImage);
    // estimate pan, tilt and focal length by
    static bool cameraToPTZ(const vpgl_perspective_camera<double> & camera, double & pan, double & tilt, double & fl);
    static bool cameraToPTZOldNotUsed(const vpgl_perspective_camera<double> & camera, double & pan, double & tilt, double & fl);
    
    static bool cameraToPTZ(const vpgl_perspective_camera<double> & camera, vpgl_ptz_camera & ptzCamera);
    
    static bool PTZToCamera(const double pan, const double tilt, const double fl, vpgl_perspective_camera<double> & camera);
    // old version of ptz to camera because different image undistortion method
    static bool PTZToCameraOldNotUsed(const double pan, const double tilt, const double fl, vpgl_perspective_camera<double> & camera);
    static bool interpolateCameras(const vpgl_perspective_camera<double> & camera1, const int fn1,
                                   const vpgl_perspective_camera<double> & camera2, const int fn2,
                                   const vcl_vector<int> & fns, vcl_vector<vpgl_perspective_camera<double> > & cameras);
    
    // syncnize PTZ camera with stationary camera (Malkin)
    static int WWoS2014PTZToMalkinCamera(const int ptzFrame);
};






#endif /* defined(__QuadCopter__WWoSSoccerCourtUtil__) */
