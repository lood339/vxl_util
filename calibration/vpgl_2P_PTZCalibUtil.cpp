//
//  vpgl_2P_PTZCalibUtil.cpp
//  QuadCopter
//
//  Created by jimmy on 6/28/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vpgl_2P_PTZCalibUtil.h"


bool Vpgl2PPTZCalibUtil::wwosSoccerMainPTZ_calib(const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                                 const vcl_vector<vgl_point_2d <double> > & img_pts,
                                                 vnl_vector_fixed<double, 3> & ptz,
                                                 vpgl_perspective_camera<double> & camera)
{
    assert(wld_pts.size() == 2);
    assert(img_pts.size() == 2);
    
    vgl_point_3d<double> cc(53.8528, -8.37071, 15.0785); //camera center
    vnl_vector_fixed<double, 3> rod;    //
    rod[0] =   1.57061;
    rod[1] =  -0.00440067;
    rod[2] =   0.021745;
    vgl_rotation_3d<double> Rs(rod);
    
    double init_fl = 0.0;
    bool isEstimated = Vpgl2PPTZCalib::approximate_focal_length(Rs, cc, wld_pts, img_pts, init_fl);
    if (!isEstimated) {
        printf("Warning: initial focal length estimation failed.\n");
        return false;
    }
    vgl_point_2d<double> pp(1280/2.0, 720/2.0);
    TwopointCalibParameter para;
    para.pan_convergence_  = 0.05 / 180.0 * vnl_math::pi;
    para.tilt_convergence_ = 0.05 / 180.0 * vnl_math::pi;
    para.fl_convergence_   = 1.0;
    isEstimated = Vpgl2PPTZCalib::iterative_calib(cc, Rs, pp, init_fl, wld_pts, img_pts, ptz, camera, para, 200, false);
    if (!isEstimated) {
        printf("Warning: pan, tilt, focal length estimation failed.\n");
        return false;
    }
    return true;
}

bool Vpgl2PPTZCalibUtil::wwosSoccerLeftSideviewPTZ_calib(const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                                         const vcl_vector<vgl_point_2d <double> > & img_pts,
                                                         vnl_vector_fixed<double, 3> & ptz,
                                                         vpgl_perspective_camera<double> & camera,
                                                         vpgl_ptz_camera & ptzCamera)
{
    assert(wld_pts.size() == 2);
    assert(img_pts.size() == 2);
    
    vgl_point_3d<double> cc(-15.213795, 14.944021, 5.002864);
    vnl_vector_fixed<double, 3> rod(1.220866, -1.226907, 1.201566);
    vgl_rotation_3d<double> Rs(rod);
    vnl_vector_fixed<double, 6> coefficient;
    coefficient.fill(0);
    
    double init_fl = 0.0;
    bool isEstimated = Vpgl2PPTZCalib::approximate_focal_length(Rs, cc, wld_pts, img_pts, init_fl);
    if (!isEstimated) {
        printf("Warning: initial focal length estimation failed.\n");
        return false;
    }
    vgl_point_2d<double> pp(1280/2.0, 720/2.0);
    TwopointCalibParameter para;
    para.pan_convergence_  = 0.05 / 180.0 * vnl_math::pi;
    para.tilt_convergence_ = 0.05 / 180.0 * vnl_math::pi;
    para.fl_convergence_   = 1.0;
    isEstimated = Vpgl2PPTZCalib::iterative_calib(cc, Rs, pp, init_fl, wld_pts, img_pts, ptz, camera, para, 200, false);
    if (!isEstimated) {
        printf("Warning: pan, tilt, focal length estimation failed.\n");
        return false;
    }

    ptzCamera = vpgl_ptz_camera(pp, cc, rod, coefficient, ptz[0], ptz[1], ptz[2]);
    return true;
}

bool Vpgl2PPTZCalibUtil::simulatedPTZ_calib(const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                            const vcl_vector<vgl_point_2d <double> > & img_pts,
                                            vnl_vector_fixed<double, 3> & ptz,
                                            vpgl_ptz_camera & ptzCamera)
{
    assert(wld_pts.size() == 2);
    assert(img_pts.size() == 2);
    
    vgl_point_2d<double> pp(640, 360);
    vgl_point_3d<double> cc(0, 0, 0);
    vnl_vector_fixed<double, 3> Rs;
    Rs[0] = 0.5 * vnl_math::pi;
    Rs[1] = 0.0;
    Rs[1] = 0.0;
    vgl_rotation_3d<double> R(Rs);
    vnl_vector_fixed<double, 6> coefficient;
    coefficient.fill(0.0);
    
    double init_fl = 0.0;
    bool isEstimated = Vpgl2PPTZCalib::approximate_focal_length(R, cc, wld_pts, img_pts, init_fl);
    if (!isEstimated) {
        printf("Warning: initial focal length estimation failed.\n");
        return false;
    }
    
    TwopointCalibParameter para;
    para.pan_convergence_  = 0.05 / 180.0 * vnl_math::pi;
    para.tilt_convergence_ = 0.05 / 180.0 * vnl_math::pi;
    para.fl_convergence_   = 1.0;
    vpgl_perspective_camera<double> camera;
    isEstimated = Vpgl2PPTZCalib::iterative_calib(cc, R, pp, init_fl, wld_pts, img_pts, ptz, camera, para, 200, false);
    if (!isEstimated) {
        printf("Warning: pan, tilt, focal length estimation failed.\n");
        return false;
    }
    
    ptzCamera = vpgl_ptz_camera(pp, cc, Rs, coefficient, ptz[0], ptz[1], ptz[2]);
    return true;
}

bool Vpgl2PPTZCalibUtil::simulatedPTZ_calib_fl_from_TUM(const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                                        const vcl_vector<vgl_point_2d <double> > & img_pts,
                                                        vnl_vector_fixed<double, 3> & ptz,
                                                        vpgl_ptz_camera & ptzCamera)
{
    assert(wld_pts.size() == 2);
    assert(img_pts.size() == 2);
    
    vgl_point_2d<double> pp(640, 360);
    vgl_point_3d<double> cc(0, 0, 0);
    vnl_vector_fixed<double, 3> Rs;
    Rs[0] = 0.5 * vnl_math::pi;
    Rs[1] = 0.0;
    Rs[1] = 0.0;
    vgl_rotation_3d<double> R(Rs);
    vnl_vector_fixed<double, 6> coefficient;
    coefficient.fill(0.0);
    
    double init_fl = 0.0; 
    bool isEstimated = Vpgl2PPTZCalibTUM::focal_length(cc, pp, wld_pts, img_pts, init_fl);
    if (!isEstimated) {
        printf("Warning: initial focal length estimation failed.\n");
        return false;
    }
    
    TwopointCalibParameter para;
    para.pan_convergence_  = 0.05 / 180.0 * vnl_math::pi;
    para.tilt_convergence_ = 0.05 / 180.0 * vnl_math::pi;
    para.fl_convergence_   = 1.0;
    vpgl_perspective_camera<double> camera;
    isEstimated = Vpgl2PPTZCalib::iterative_calib(cc, R, pp, init_fl, wld_pts, img_pts, ptz, camera, para, 200, true);
    if (!isEstimated) {
        printf("Warning: pan, tilt, focal length estimation failed.\n");
        return false;
    }
    
    ptzCamera = vpgl_ptz_camera(pp, cc, Rs, coefficient, ptz[0], ptz[1], ptz[2]);
    return true;
}

