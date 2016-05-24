//
//  vxl_PTZ_variance.cpp
//  OnlineStereo
//
//  Created by jimmy on 11/28/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_PTZ_variance.h"
#include "vxl_ptz_camera.h"

vnl_vector<double> VxlPTZVariance::fl_jacobian(const PTZCameraPinhole & camera,
                                               const vgl_point_3d<double> & point3d,
                                               const vgl_point_2d<double> & imagePoint)
{
    vnl_vector<double> jaco = vnl_vector<double>(2, 0);
    vnl_matrix<double> A = camera.R_tilt_ * camera.R_pan_ * camera.R_S_ * camera.T_;
    assert(A.rows() == 3 && A.cols() == 4);
    
    /*
    vnl_vector<double> pt3d(4, 0);
    pt3d[0] = point3d.x();
    pt3d[1] = point3d.y();
    pt3d[2] = point3d.z();
    pt3d[3] = 1.0;
    double D = dot_product(pt3d, A.get_row(0));
    double E = dot_product(pt3d, A.get_row(1));
    double B = dot_product(pt3d, A.get_row(2));
    assert(B != 0.0);
    jaco[0] = D/B;
    jaco[1] = E/B;
     */
    
    vnl_vector<double> pt3d(4, 0);
    pt3d[0] = point3d.x();
    pt3d[1] = point3d.y();
    pt3d[2] = point3d.z();
    pt3d[3] = 1.0;
    vnl_vector<double> camera_pt = A * pt3d;
    assert(camera_pt.size() == 3);
    assert(camera_pt[2] != 0);
    jaco[0] = camera_pt[0]/camera_pt[2];
    jaco[1] = camera_pt[1]/camera_pt[2];
    
    return jaco;
}

static double valueByIndex(const vnl_matrix<double> & m, int ind)
{
    int r = ind/m.cols();
    int c = ind - r * m.cols();
    assert(r < m.rows() && c < m.cols());
    return m[r][c];
}

vnl_vector<double> VxlPTZVariance::pan_jacobian(const PTZCameraPinhole & camera,
                                                const vgl_point_3d<double> & point3d,
                                                const vgl_point_2d<double> & imagePoint,
                                                const double pan)
{
    vnl_vector<double> jaco = vnl_vector<double>(2, 0);
    
    
    vnl_vector<double> pt3d(4, 0);
    pt3d[0] = point3d.x();
    pt3d[1] = point3d.y();
    pt3d[2] = point3d.z();
    pt3d[3] = 1.0;
    vnl_matrix<double> A = camera.K_ * camera.R_tilt_;
    assert(A.rows() == 3 && A.cols() == 3);
    vnl_vector<double> B = camera.R_S_ * camera.T_ * pt3d;
    assert(B.size() == 3);
    
    double A1 = valueByIndex(A, 0);
    double A2 = valueByIndex(A, 1);
    double A3 = valueByIndex(A, 2);
    double A4 = valueByIndex(A, 3);
    double A5 = valueByIndex(A, 4);
    double A6 = valueByIndex(A, 5);
    double A7 = valueByIndex(A, 6);
    double A8 = valueByIndex(A, 7);
    double A9 = valueByIndex(A, 8);
    
    double B1 = B[0];
    double B2 = B[1];
    double B3 = B[2];
    
    double C1 = A3 * B1 - A1*B3;
    double C2 = A1 * B1 + A3*B3;
    double C3 = A2 * B2;
    double C4 = A6*B1 - A4*B3;
    double C5 = A4*B1 + A6*B3;
    double C6 = A5*B2;
    double C7 = A9*B1 - A7*B3;
    double C8 = A7*B1 + A9*B3;
    double C9 = A8*B2;
    
    double pan_rad = pan/180.0 * vnl_math::pi;
    double sThe = sin(pan_rad);
    double cThe = cos(pan_rad);
    
    double D  = C1*sThe + C2*cThe + C3;
    double dD = C1*cThe - C2*sThe;
    double E  = C4*sThe + C5*cThe + C6;
    double dE = C4*cThe - C5*sThe;
    double F  = C7*sThe + C8*cThe + C9;
    double dF = C7*cThe - C8*sThe;
    
    assert(F != 0.0);
    jaco[0] = (dD*F - D*dF)/(F*F);
    jaco[1] = (dE*F - E*dF)/(F*F);
        
    /*
    method one
    vnl_matrix<double> A = camera.K_ * camera.R_tilt_;
    assert(A.rows() == 3 && A.cols() == 3);
    
    vnl_matrix<double> B = camera.R_S_ * camera.T_;
    assert(B.rows() == 3 && B.cols() == 4);
    
    double A1 = valueByIndex(A, 0);
    double A2 = valueByIndex(A, 1);
    double A3 = valueByIndex(A, 2);
    double A4 = valueByIndex(A, 3);
    double A5 = valueByIndex(A, 4);
    double A6 = valueByIndex(A, 5);
    double A7 = valueByIndex(A, 6);
    double A8 = valueByIndex(A, 7);
    double A9 = valueByIndex(A, 8);
    
    double B1 = valueByIndex(B, 0);
    double B2 = valueByIndex(B, 1);
    double B3 = valueByIndex(B, 2);
    double B4 = valueByIndex(B, 3);
    double B5 = valueByIndex(B, 4);
    double B6 = valueByIndex(B, 5);
    double B7 = valueByIndex(B, 6);
    double B8 = valueByIndex(B, 7);
    double B9 = valueByIndex(B, 8);
    double B10 = valueByIndex(B, 9);
    double B11 = valueByIndex(B, 10);
    double B12 = valueByIndex(B, 11);
    
    double C11 = B1*A3 - B9*A1;
    double C12 = B1*A1 + B9*A3;
    double C13 = B5*A2;
    double C14 = B2*A3 - B10*A1;
    double C15 = B2*A1 + B10*A3;
    double C16 = B6*A2;
    double C17 = B3*A3 - B11*A1;
    double C18 = B3*A1 + B11*A3;
    double C19 = B7*A2;
    double C110 = B4*A3 - B12*A1;
    double C111 = B4*A1 + B12*A3;
    double C112 = B8*A2;
    
    double C21 = B1*A6 - B9*A4;
    double C22 = B1*A4 + B9*A6;
    double C23 = B5*A5;
    double C24 = B2*A6 - B10*A4;
    double C25 = B2*A4 + B10*A6;
    double C26 = B6*A5;
    double C27 = B3*A6 - B11*A4;
    double C28 = B3*A4 + B11*A6;
    double C29 = B7*A5;
    double C210 = B4*A6 - B12*A4;
    double C211 = B4*A4 + B12*A6;
    double C212 = B8*A5;
    
    double C31 = B1*A9 - B9*A7;
    double C32 = B1*A7 + B9*A9;
    double C33 = B5*A8;
    double C34 = B2*A9 - B10*A7;
    double C35 = B2*A7 + B10*A9;
    double C36 = B6*A8;
    double C37 = B3*A9 - B11*A7;
    double C38 = B3*A7 + B11*A9;
    double C39 = B7*A8;
    double C310 = B4*A9 - B12*A7;
    double C311 = B4*A7 + B12*A9;
    double C312 = B8*A8;
    
    double x = point3d.x();
    double y = point3d.y();
    double z = point3d.z();
    
    // D matrix
    double D1 = x*C11 + y*C14 + z*C17 + C110;
    double D2 = x*C12 + y*C15 + z*C18 + C111;
    double D3 = x*C13 + y*C16 + z*C19 + C112;
    double D4 = x*C21 + y*C24 + z*C27 + C210;
    double D5 = x*C22 + y*C25 + z*C28 + C211;
    double D6 = x*C23 + y*C26 + z*C29 + C212;
    double D7 = x*C31 + y*C34 + z*C37 + C310;
    double D8 = x*C32 + y*C35 + z*C38 + C311;
    double D9 = x*C33 + y*C36 + z*C39 + C312;   
    
    double pan_rad = pan/180.0 * vnl_math::pi;
    double sThe = sin(pan_rad);
    double cThe = cos(pan_rad);
    double E = D7*sThe + D8*cThe + D9;
    double F = D1*sThe + D2*cThe + D3;
    double G = D4*sThe + D5*cThe + D6;
    double dE = D7*cThe - D8*sThe;
    double dF = D1*cThe - D2*sThe;
    double dG = D4*cThe - D5*sThe;
    
    assert(E != 0.0);
    jaco[0] = (dF * E - F * dE)/(E*E);
    jaco[1] = (dG * E - G * dE)/(E*E);
    */
    
    return jaco;
}


vnl_vector<double> VxlPTZVariance::tilt_jacobian(const PTZCameraPinhole & camera,
                                                 const vgl_point_3d<double> & point3d,
                                                 const vgl_point_2d<double> & imagePoint,
                                                 const double tilt)
{
    vnl_vector<double> jaco(2, 0);
    vnl_matrix<double> A = camera.R_pan_*camera.R_S_*camera.T_;
    assert(A.rows() == 3 && A.cols() == 4);
    
    double A1 = valueByIndex(A, 0);
    double A2 = valueByIndex(A, 1);
    double A3 = valueByIndex(A, 2);
    double A4 = valueByIndex(A, 3);
    double A5 = valueByIndex(A, 4);
    double A6 = valueByIndex(A, 5);
    double A7 = valueByIndex(A, 6);
    double A8 = valueByIndex(A, 7);
    double A9 = valueByIndex(A, 8);
    double A10 = valueByIndex(A, 9);
    double A11 = valueByIndex(A, 10);
    double A12 = valueByIndex(A, 11);
    
    double u = camera.K_[0][2];
    double v = camera.K_[1][2];
    double f = camera.K_[0][0];
    
    double B11 = -u*A5;
    double B12 =  u*A9;
    double B13 = f*A1;
    double B14 = -u*A6;
    double B15 = u*A10;
    double B16 = f*A2;
    double B17 = -u*A7;
    double B18 = u*A11;
    double B19 = f*A3;
    double B110 = -u*A8;
    double B111 = u*A12;
    double B112 = f*A4;
    
    
    double B21 = f*A9 - v*A5;
    double B22 = f*A5 - v*A9;
    double B24 = f*A10 - v*A6;
    double B25 = f*A6 + v*A10;
    double B27 = f*A11 - v*A7;
    double B28 = f*A7 + v*A11;
    double B210 = f*A12 - v*A8;
    double B211 = f*A8 + v*A12;
    
    double B31 = -A5;
    double B32 = A9;
    double B34 = -A6;
    double B35 = A10;
    double B37 = -A7;
    double B38 = A11;
    double B310 = -A8;
    double B311 = A12;
    
    double x = point3d.x();
    double y = point3d.y();
    double z = point3d.z();
    
    double C1 = x*B11 + y*B14 + z*B17 + B110;
    double C2 = x*B12 + y*B15 + z*B18 + B111;
    double C3 = x*B13 + y*B16 + z*B19 + B112;
    double C4 = x*B21 + y*B24 + z*B27 + B210;
    double C5 = x*B22 + y*B25 + z*B28 + B211;
    double C7 = x*B31 + y*B34 + z*B37 + B310;
    double C8 = x*B32 + y*B35 + z*B38 + B311;
    
    double tilt_rad = tilt / 180.0 * vnl_math::pi;
    double sinTilt = sin(tilt_rad);
    double cosTilt = cos(tilt_rad);
    
    double D = C7*sinTilt + C8*cosTilt;
    double dD = C7*cosTilt - C8*sinTilt;
    assert(D != 0.0);
    
    double E = C1*sinTilt + C2*cosTilt + C3;
    double dE = C1*cosTilt - C2*sinTilt;
    double F = C4*sinTilt + C5*cosTilt;
    double dF = C4*cosTilt - C5*sinTilt;
    
    jaco[0] = (dE*D - E*dD)/(D*D);
    jaco[1] = (dF*D - F*dD)/(D*D);
    return jaco;
}

vnl_matrix<double> VxlPTZVariance::xyz_jacobian(const PTZCameraPinhole & camera,
                                                const vgl_point_3d<double> & point3d,
                                                const vgl_point_2d<double> & imagePoint)
{
    vnl_matrix<double> jaco(2, 3, 0);
    
    vnl_matrix<double> P = camera.P();
    double P1 = P(0, 0); double P2  = P(0, 1);  double P3 = P(0, 2);  double P4 = P(0, 3);
    double P5 = P(1, 0); double P6  = P(1, 1);  double P7 = P(1, 2);  double P8 = P(1, 3);
    double P9 = P(2, 0); double P10 = P(2, 1); double P11 = P(1, 2); double P12 = P(2, 3);
    
    double x = point3d.x();
    double y = point3d.y();
    double z = point3d.z();
    double px = imagePoint.x();
    double py = imagePoint.y();
    
    double C = x * P9 + y * P10 + z*P11 + P12;
    assert(C != 0.0);
    
    double invC = 1.0 / C;
    jaco(0, 0) = (P1 - px * P9)  * invC;
    jaco(0, 1) = (P2 - px * P10) * invC;
    jaco(0, 2) = (P3 - px * P11) * invC;
    jaco(1, 0) = (P5 - py * P9)  * invC;
    jaco(1, 1) = (P6 - py * P10) * invC;
    jaco(1, 2) = (P7 - py * P11) * invC;
    
    return jaco;
}


vnl_vector<double> VxlPTZVariance::fl_jacobian(const double pan, const double tilt, const double fl,
                                               const vgl_point_3d<double> & point3d)
{
    vnl_vector<double> jaco(2, 0);
    vpgl_perspective_camera<double> camera;
    vpgl_perspective_camera<double> fl_camera;
    double delta = 1.0;
    bool isCamera = VxlPTZCamera::PTZToCamera(fl, pan, tilt, camera);
    assert(isCamera);
    isCamera = VxlPTZCamera::PTZToCamera(fl+delta, pan, tilt, fl_camera);
    assert(isCamera);
    
    vgl_point_2d<double> p1 = camera.project(point3d);
    vgl_point_2d<double> p2 = fl_camera.project(point3d);
    jaco[0] = (p2.x() - p1.x())/delta;
    jaco[1] = (p2.y() - p1.y())/delta;
    return jaco;
}

vnl_vector<double> VxlPTZVariance::pan_jacobian(const double pan, const double tilt, const double fl,
                                                const vgl_point_3d<double> & point3d)
{
    vnl_vector<double> jaco(2, 0);
    vpgl_perspective_camera<double> camera;
    vpgl_perspective_camera<double> pan_camera;
    double delta = 0.005;
    bool isCamera = VxlPTZCamera::PTZToCamera(fl, pan, tilt, camera);
    assert(isCamera);
    isCamera = VxlPTZCamera::PTZToCamera(fl, pan+delta, tilt, pan_camera);
    assert(isCamera);
    
    vgl_point_2d<double> p1 = camera.project(point3d);
    vgl_point_2d<double> p2 = pan_camera.project(point3d);
    jaco[0] = (p2.x() - p1.x())/delta;
    jaco[1] = (p2.y() - p1.y())/delta;
    return jaco;
}

vnl_vector<double> VxlPTZVariance::tilt_jacobian(const double pan, const double tilt, const double fl,
                                        const vgl_point_3d<double> & point3d)
{
    vnl_vector<double> jaco(2, 0);
    vpgl_perspective_camera<double> camera;
    vpgl_perspective_camera<double> tilt_camera;
    double delta = 0.005;
    bool isCamera = VxlPTZCamera::PTZToCamera(fl, pan, tilt, camera);
    assert(isCamera);
    isCamera = VxlPTZCamera::PTZToCamera(fl, pan, tilt+delta, tilt_camera);
    assert(isCamera);
    
    vgl_point_2d<double> p1 = camera.project(point3d);
    vgl_point_2d<double> p2 = tilt_camera.project(point3d);
    jaco[0] = (p2.x() - p1.x())/delta;
    jaco[1] = (p2.y() - p1.y())/delta;
    return jaco;
}

vnl_matrix<double> VxlPTZVariance::xyz_jacobian(const double pan, const double tilt, const double fl,
                                                const vgl_point_3d<double> & point3d)
{
    vnl_matrix<double> jaco(2, 3, 0);
    vpgl_perspective_camera<double> camera;
    double delta = 0.05;
    bool isCamera = VxlPTZCamera::PTZToCamera(fl, pan, tilt, camera);
    assert(isCamera);
    
    vgl_point_2d<double> p1 = camera.project(point3d);
    vgl_point_2d<double> px = camera.project(vgl_point_3d<double>(point3d.x() + delta, point3d.y(), point3d.z()));
    vgl_point_2d<double> py = camera.project(vgl_point_3d<double>(point3d.x(), point3d.y() + delta, point3d.z()));
    vgl_point_2d<double> pz = camera.project(vgl_point_3d<double>(point3d.x(), point3d.y(), point3d.z() + delta));
    
    jaco(0, 0) = (px.x() - p1.x())/delta;
    jaco(1, 0) = (px.y() - p1.y())/delta;
    jaco(0, 1) = (py.x() - p1.x())/delta;
    jaco(1, 1) = (py.y() - p1.y())/delta;
    jaco(0, 2) = (pz.x() - p1.x())/delta;
    jaco(1, 2) = (pz.y() - p1.y())/delta;
    
    return jaco;
}






























