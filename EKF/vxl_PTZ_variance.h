//
//  vxl_PTZ_variance.h
//  OnlineStereo
//
//  Created by jimmy on 11/28/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_PTZ_variance__
#define __OnlineStereo__vxl_PTZ_variance__

// calculate the variance matrix of PTZ camera for pan, tilt angle and focal length
#include <vnl/vnl_matrix.h>
#include "vxl_plus.h"
#include "vpgl_plus.h"

struct PTZCameraPinhole
{
    vnl_matrix<double> K_;
    vnl_matrix<double> R_tilt_;
    vnl_matrix<double> R_pan_;
    vnl_matrix<double> R_S_;
    vnl_matrix<double> T_;   // translation
    
    // fixed stationary rotation and camera center
    PTZCameraPinhole(double pan, double tilt, double fl)
    {
        K_ = vnl_matrix<double>(3, 3, 0);
        K_[0][0] = fl;
        K_[1][1] = fl;
        K_[2][2] = 1.0;
        K_[0][2] = 1280/2.0;
        K_[1][2] = 720/2.0;
        
        R_tilt_ = VpglPlus::matrixTiltX(tilt);
        R_pan_  = VpglPlus::matrixPanY(pan);
        
        // R_S is fixed
        vnl_vector_fixed<double, 3> rod;    // 1.58044 -0.118628 0.124857
        rod[0] =    1.58044;
        rod[1] = -0.118628;
        rod[2] =  0.124857;
        vgl_rotation_3d<double> Rs(rod);
        R_S_ = Rs.as_matrix().as_matrix();
        
        // cc(12.9456, -14.8695, 6.21215);
        T_ = vnl_matrix<double>(3, 4, 0);
        T_[0][0] = T_[1][1] = T_[2][2]  = 1.0;
        T_[0][3] = -(12.9456);
        T_[1][3] = -(-14.8695);
        T_[2][3] = -6.21215;
    }
    ~PTZCameraPinhole(){;}
    
    // projection matrix
    vnl_matrix<double> P() const
    {
        vnl_matrix<double> p = K_ * R_tilt_ * R_pan_ * R_S_ * T_;
        return p;
    }
};

class VxlPTZVariance {
public:
    // u/vl and v/vl
    // jacobian vector according to back-propagation
    static vnl_vector<double> fl_jacobian(const PTZCameraPinhole & camera,
                                          const vgl_point_3d<double> & point3d,
                                          const vgl_point_2d<double> & imagePoint);
    // return 2 * 1
    static vnl_vector<double> pan_jacobian(const PTZCameraPinhole & camera,
                                           const vgl_point_3d<double> & point3d,
                                           const vgl_point_2d<double> & imagePoint,
                                           const double pan);
    // return 2 * 1
    static vnl_vector<double> tilt_jacobian(const PTZCameraPinhole & camera,
                                            const vgl_point_3d<double> & point3d,
                                            const vgl_point_2d<double> & imagePoint,
                                            const double tilt);
    // return 2 * 3
    // row: x', y'  (image position)
    // col: x, y, z (3D position)
    static vnl_matrix<double> xyz_jacobian(const PTZCameraPinhole & camera,
                                           const vgl_point_3d<double> & point3d,
                                           const vgl_point_2d<double> & imagePoint);
    
    
    
    // simulated method to get jacobian matrix
    // jacobian vector according to back-propagation
    static vnl_vector<double> fl_jacobian(const double pan, const double tilt, const double fl,
                                          const vgl_point_3d<double> & point3d);
    // return 2 * 1
    static vnl_vector<double> pan_jacobian(const double pan, const double tilt, const double fl,
                                           const vgl_point_3d<double> & point3d);
    // return 2 * 1
    static vnl_vector<double> tilt_jacobian(const double pan, const double tilt, const double fl,
                                            const vgl_point_3d<double> & point3d);
    // return 2 * 3
    // row: x', y'  (image position)
    // col: x, y, z (3D position)
    static vnl_matrix<double> xyz_jacobian(const double pan, const double tilt, const double fl,
                                           const vgl_point_3d<double> & point3d);
};



#endif /* defined(__OnlineStereo__vxl_PTZ_variance__) */
