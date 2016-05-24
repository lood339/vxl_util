//
//  vpgl_2P_PTZ_calib.h
//  QuadCopter
//
//  Created by jimmy on 6/26/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vpgl_2P_PTZ_calib__
#define __QuadCopter__vpgl_2P_PTZ_calib__

// 2 point fixed location PTZ camera calibration

#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector_fixed.h>
#include <vpgl/vpgl_perspective_camera.h>

struct TwopointCalibParameter
{
    double pan_convergence_ ;
    double tilt_convergence_ ;
    double fl_convergence_ ;
    
    TwopointCalibParameter()
    {
        pan_convergence_  = 0.05 / 180.0 * vnl_math::pi;
        tilt_convergence_ = 0.05 / 180.0 * vnl_math::pi;
        fl_convergence_   = 1.0;
    }
};
class Vpgl2PPTZCalib
{
public:
    // cc: camera center
    // Rs: stationary rotation R = R_tilt * R_pan * R_s
    // pp: principle point
    // init_fl: initial value of focal length
    // wld_pts: 3D points in world coordinate, size 2
    // img_pts: correspondent 2D points in image coordinate, size 3
    // return:
    // ptz: pan, tilt, zoom. The angles are in degree
    // camera: projective (PTZ) camera
    // max_iter_num: maxinum iteration number
    // camera model from "mimicking human camera operators" wacv 2015
    static bool iterative_calib(const vgl_point_3d<double> & cc, const vgl_rotation_3d<double> & Rs,
                                const vgl_point_2d<double> & pp, const double init_fl,
                                const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                const vcl_vector<vgl_point_2d <double> > & img_pts,
                                vnl_vector_fixed<double, 3> & ptz,
                                vpgl_perspective_camera<double> & camera,
                                const TwopointCalibParameter & para, 
                                int max_iter_num = 100, bool verbose = true);
    // approximate focal length assume: R_tilt * R_theta = I
    // weak perspective camera
    // Rs: stationary rotation
    // cc: camera center
    // img_pts: two points in the image should have significant distance
    static bool approximate_focal_length(const vgl_rotation_3d<double> & Rs,
                                         const vgl_point_3d<double> & cc,
                                         const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                         const vcl_vector<vgl_point_2d <double> > & img_pts,
                                         double & fl);    
    
private:
    // pan tilt in radian,
    static bool optimizePTZ(const vgl_point_3d<double> & cc, const vgl_rotation_3d<double> & Rs,
                             const vgl_point_2d<double> & pp,
                             const vcl_vector<vgl_point_3d<double> > & wld_pts,
                             const vcl_vector<vgl_point_2d <double> > & img_pts,
                             const vnl_vector_fixed<double, 3> & init_ptz,
                             vnl_vector_fixed<double, 3> & opt_ptz);
    
    static bool getValidPanTilt(double & pan1, double & tilt1, double & pan2, double & tilt2,
                                const vnl_vector_fixed<double, 2> & threshold,
                                vnl_vector_fixed<double, 2> & panTilt);
};

// method from "Continual and Robust Estimation of Camera Parameters in Broadcasted Sports Games"
class Vpgl2PPTZCalibTUM
{
public:
    //wld_pts: points in world coordinate
    static bool focal_length(const vgl_point_3d<double> & cc,
                             const vgl_point_2d<double> & pp,
                             const vcl_vector<vgl_point_3d<double> > & wld_pts,
                             const vcl_vector<vgl_point_2d <double> > & img_pts,
                             double & fl);
    
    // two point correspondeces from two images, one image is calibrated
    // pts_1 from calibrated camera
    static bool focal_length_from_calibrated_camera(const vpgl_perspective_camera<double> & camera,
                                                    const vcl_vector<vgl_point_2d<double> > & pts_1,
                                                    const vcl_vector<vgl_point_2d<double> > & pts_2,
                                                    double &fl);

    
    
};


#endif /* defined(__QuadCopter__vpgl_2P_PTZ_calib__) */
