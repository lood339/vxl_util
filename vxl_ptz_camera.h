//
//  vxl_ptz_camera.h
//  PlanarAlign
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __PlanarAlign__vxl_ptz_camera__
#define __PlanarAlign__vxl_ptz_camera__

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vnl/vnl_vector_fixed.h>
#include <vil/vil_image_view.h>


// note: function has hard code, only work for WWoS basketball field
class VxlPTZCamera
{
public:
    struct PtsPairs
    {
        vcl_vector<vgl_point_2d<double> > wld_pts;
        vcl_vector<vgl_point_2d<double> > img_pts;
    };
    
    static bool CameraToPTZ(const vpgl_perspective_camera<double> & camera, double & pan, double & tilt, double & fl);
    
    // output camera is composed from fl, pan, tilt
    static bool CameraToPTZ(const vpgl_perspective_camera<double> & inputCamera, double & pan, double & tilt, double &fl, vpgl_perspective_camera<double> & outputCamera);
    
    // vnl_vector_fixed<double, 6> & coeff
    // cx = c1 + c4 * fl
    // cy = c2 + c5 * fl
    // cz = c3 + c6 * fl
    static bool CameraToPTZ(const vpgl_perspective_camera<double> & inputCamera, const vnl_vector_fixed<double, 6> & coeff,
                            double & pan, double & tilt, double &fl, vpgl_perspective_camera<double> & outputCamera);
    
    static bool CameraToPTZ(const vpgl_perspective_camera<double> & inputCamera, const vnl_vector_fixed<double, 6> & coeff, const vgl_point_3d<double> & cc,
                            double & pan, double & tilt, double &fl, vpgl_perspective_camera<double> & outputCamera);
    
    // cx = c1
    // cy = c2
    // cz = c3 + c4 * fl
    static bool CameraToPTZ(const vpgl_perspective_camera<double> & inputCamera, const vnl_vector_fixed<double, 4> & coeff,
                            double & pan, double & tilt, double &fl, vpgl_perspective_camera<double> & outputCamera);
    
    
    // compose camera from PTZ by pre-setted coeff, pp, S, camera center
    static bool PTZToCamera(double fl, double pan, double tilt, vpgl_perspective_camera<double> & camera);
    
    static bool PTZToCamera(const vnl_vector_fixed<double, 6> & coeff, double fl, double pan, double tilt, vpgl_perspective_camera<double> & camera);
    
    
    // fix fl, cc, rod, coeff to estiamte pan/tilt
    static bool estimatePanTiltByFixingModelPositionRotationFl(const PtsPairs & corre, const vpgl_perspective_camera<double> & initCamera,
                                                               const double fl,
                                                               const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                               const vnl_vector_fixed<double, 6> & coeff, vnl_vector_fixed<double, 2> & panTilt,
                                                               vpgl_perspective_camera<double> & estimatedCamera);
    
    // overlay ratio from ground truth camera to estimated camera
    static double overlayRatio(const vpgl_perspective_camera<double> & gd_camera, const vpgl_perspective_camera<double> & estimated_camera, int width, int height);
    
    //fn1,fn2,fn3: frame numbers
    static bool interpolateCamera(const vpgl_perspective_camera<double> & camera1, const int fn1,
                                  const vpgl_perspective_camera<double> & camera2, const int fn2,
                                  const int fn3, vpgl_perspective_camera<double> & camera3);
    
    //interpolate a group of frames
    //fns: Frame NumberS
    static bool interpolateCameras(const vpgl_perspective_camera<double> & camera1, const int fn1,
                                   const vpgl_perspective_camera<double> & camera2, const int fn2,
                                   const vcl_vector<int> & fns, vcl_vector<vpgl_perspective_camera<double> > & cameras);
    
       
   


    // P = K*C*Q*S*D by given C,S, DFd
    // estimate Q (pan, tilt) angles
    // cameraCenter: estimated common camera rotation center from multiple frames
    // rod: rodrigues angle of model rotation
    // coeff: c123456, [0,0,0,0,0,0] for projection center just as rotation center
    // flPanTilt: focal length, pan, tilt.
    // from vpgl_ptz_model_estimation.h estimateFlPanTiltByFixingModelPositionRotation
    static bool estimateFlPanTiltByFixingModelPositionRotation(const PtsPairs & corre, const vpgl_perspective_camera<double> & initCamera,
                                                               const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                               const vnl_vector_fixed<double, 6> & coeff, vnl_vector_fixed<double, 3> & flPanTilt,
                                                               vpgl_perspective_camera<double> & estimatedCamera);

    // P = K*C*Q*S*D by given C,S, DFd
    // estimate Q (pan, tilt) angles
    // cameraCenter: estimated common camera rotation center from multiple frames
    // rod: rodrigues angle of model rotation
    // coeff: c1234, [0,0,0,0] for projection center just as rotation center
    // flPanTilt: focal length, pan, tilt.
    // cx = c1, cy = c2, cz = c3 + c4 * f
    static bool estimateFlPanTiltByFixingModelPositionRotationProjectioncentercoeff(const PtsPairs & corre, const vpgl_perspective_camera<double> & initCamera,
                                                                                    const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                                    const vnl_vector_fixed<double, 4> & coeff, vnl_vector_fixed<double, 3> & flPanTilt,
                                                                                    vpgl_perspective_camera<double> & estimatedCamera);
    
    
    
    // P = K*C*Q*S*D -> perspectice camera
    // cameraCenter: rotation center
    // rod:
    static bool composeCamera(double fl, const vnl_vector_fixed<double, 6> & coeff,
                              const vgl_point_2d<double> & principlePoint,
                              double pan, double tilt,
                              const vnl_vector_fixed<double, 3> & rod,
                              const vgl_point_3d<double> & cameraCenter,
                              vpgl_perspective_camera<double> & camera);
public:
    
    // (idx1, y1) --- (idx3, y?) --- (idx2, y2)
    static double linearInterpolate(double y1, double y2, int idx1, int idx2, int idx3)
    {
        assert(idx1 < idx3 && idx3 < idx2);
        double dy = y2 - y1;
        return y1 + dy/(idx2 - idx1) * (idx3 - idx1);
    }
    
};

#endif /* defined(__PlanarAlign__vxl_ptz_camera__) */
