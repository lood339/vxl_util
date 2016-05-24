//
//  vxl_ptz_camera.cpp
//  PlanarAlign
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_ptz_camera.h"
#include "vxl_plus.h"
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include "basketballCourt.h"
#include "vpgl_plus.h"



bool VxlPTZCamera::CameraToPTZ(const vpgl_perspective_camera<double> & camera, double & pan, double & tilt, double & fl)
{
    vpgl_perspective_camera<double> dumpCamera;
    return VxlPTZCamera::CameraToPTZ(camera, pan, tilt, fl, dumpCamera);
}

bool VxlPTZCamera::CameraToPTZ(const vpgl_perspective_camera<double> & inputCamera, double & pan, double & tilt, double &fl, vpgl_perspective_camera<double> & outputCamera)
{
    
    //estimte fl, pan, tilt from camera
    vgl_point_3d<double> cc(12.9456, -14.8695, 6.21215); //camera center
    vnl_vector_fixed<double, 3> rod;    // 1.58044 -0.118628 0.124857
    rod[0] =    1.58044;
    rod[1] = -0.118628;
    rod[2] =  0.124857;
    
    vnl_vector_fixed<double, 6> coeff;  // 0.570882 0.0310795 -0.533881 -0.000229727 -5.68634e-06 0.000266362
    
    coeff[0] =  0.570882;
    coeff[1] =  0.0310795;
    coeff[2] = -0.533881;
    coeff[3] = -0.000229727;
    coeff[4] = -5.68634e-06;
    coeff[5] =  0.000266362;
    
    const int width = 1280;
    const int height = 720;
    
    VxlPTZCamera::PtsPairs corres;
    
    DisneyWorldBasketballCourt::projectCourtPoints(inputCamera, width, height, corres.wld_pts, corres.img_pts);
    if (corres.wld_pts.size() < 4) {
        printf("Error: point pairs less than 4\n");
        return false;
    }
    
    vnl_vector_fixed<double, 3> flPanTilt;
    vpgl_perspective_camera<double> estimatedCamera;
    
    bool isEstimated = VxlPTZCamera::estimateFlPanTiltByFixingModelPositionRotation(corres, inputCamera, cc, rod, coeff, flPanTilt, estimatedCamera);
    if (!isEstimated) {
        return false;
    }
    fl  = flPanTilt[0];
    pan = flPanTilt[1];
    tilt   = flPanTilt[2];
    outputCamera = estimatedCamera;
    return true;
}

bool VxlPTZCamera::CameraToPTZ(const vpgl_perspective_camera<double> & inputCamera, const vnl_vector_fixed<double, 6> & coeff, double & pan, double & tilt, double &fl, vpgl_perspective_camera<double> & outputCamera)
{
    //estimte fl, pan, tilt from camera
    vgl_point_3d<double> cc(12.9456, -14.8695, 6.21215); //camera center
    vnl_vector_fixed<double, 3> rod;    // 1.58044 -0.118628 0.124857
    rod[0] =    1.58044;
    rod[1] = -0.118628;
    rod[2] =  0.124857;
    
    const int width = 1280;
    const int height = 720;
    
    VxlPTZCamera::PtsPairs corres;
    
    DisneyWorldBasketballCourt::projectCourtPoints(inputCamera, width, height, corres.wld_pts, corres.img_pts);
    if (corres.wld_pts.size() < 4) {
        printf("Error: point pairs less than 4\n");
        return false;
    }
    
    vnl_vector_fixed<double, 3> flPanTilt;
    vpgl_perspective_camera<double> estimatedCamera;
    
    bool isEstimated = VxlPTZCamera::estimateFlPanTiltByFixingModelPositionRotation(corres, inputCamera, cc, rod, coeff, flPanTilt, estimatedCamera);
    if (!isEstimated) {
        return false;
    }
    fl  = flPanTilt[0];
    pan = flPanTilt[1];
    tilt   = flPanTilt[2];
    outputCamera = estimatedCamera;
    return true;
}

bool VxlPTZCamera::CameraToPTZ(const vpgl_perspective_camera<double> & inputCamera, const vnl_vector_fixed<double, 6> & coeff, const vgl_point_3d<double> & cc,
                               double & pan, double & tilt, double &fl, vpgl_perspective_camera<double> & outputCamera)
{
    //estimte fl, pan, tilt from camera    
    vnl_vector_fixed<double, 3> rod;    // 1.58044 -0.118628 0.124857
    rod[0] =    1.58044;
    rod[1] = -0.118628;
    rod[2] =  0.124857;
    
    const int width = 1280;
    const int height = 720;
    
    VxlPTZCamera::PtsPairs corres;
    
    DisneyWorldBasketballCourt::projectCourtPoints(inputCamera, width, height, corres.wld_pts, corres.img_pts);
    if (corres.wld_pts.size() < 4) {
        printf("Error: point pairs less than 4\n");
        return false;
    }
    
    vnl_vector_fixed<double, 3> flPanTilt;
    vpgl_perspective_camera<double> estimatedCamera;
    
    bool isEstimated = VxlPTZCamera::estimateFlPanTiltByFixingModelPositionRotation(corres, inputCamera, cc, rod, coeff, flPanTilt, estimatedCamera);
    if (!isEstimated) {
        return false;
    }
    fl  = flPanTilt[0];
    pan = flPanTilt[1];
    tilt   = flPanTilt[2];
    outputCamera = estimatedCamera;
    return true;
}

bool VxlPTZCamera::CameraToPTZ(const vpgl_perspective_camera<double> & inputCamera, const vnl_vector_fixed<double, 4> & coeff,
                               double & pan, double & tilt, double &fl, vpgl_perspective_camera<double> & outputCamera)
{
    //estimte fl, pan, tilt from camera
    vgl_point_3d<double> cc(12.9456, -14.8695, 6.21215); //camera center
    vnl_vector_fixed<double, 3> rod;    // 1.58044 -0.118628 0.124857
    rod[0] =    1.58044;
    rod[1] = -0.118628;
    rod[2] =  0.124857;
    
    const int width = 1280;
    const int height = 720;
    
    VxlPTZCamera::PtsPairs corres;
    
    DisneyWorldBasketballCourt::projectCourtPoints(inputCamera, width, height, corres.wld_pts, corres.img_pts);
    if (corres.wld_pts.size() < 4) {
        printf("Error: point pairs less than 4\n");
        return false;
    }
    
    vnl_vector_fixed<double, 3> flPanTilt;
    vpgl_perspective_camera<double> estimatedCamera;
    
    bool isEstimated = VxlPTZCamera::estimateFlPanTiltByFixingModelPositionRotationProjectioncentercoeff(corres, inputCamera, cc, rod, coeff, flPanTilt, estimatedCamera);
    if (!isEstimated) {
        return false;
    }
    fl  = flPanTilt[0];
    pan = flPanTilt[1];
    tilt   = flPanTilt[2];
    outputCamera = estimatedCamera;
    return true;
}

bool VxlPTZCamera::PTZToCamera(double fl, double pan, double tilt, vpgl_perspective_camera<double> & camera)
{
    //assert(fl > 1000);
    if (fl < 1000) {
        return false;
    }
    
    //estimte fl, pan, tilt from camera
    vgl_point_3d<double> cc(12.9456, -14.8695, 6.21215); //camera center
    vnl_vector_fixed<double, 3> rod;    // 1.58044 -0.118628 0.124857
    rod[0] =    1.58044;
    rod[1] = -0.118628;
    rod[2] =  0.124857;
    
    vnl_vector_fixed<double, 6> coeff;  // 0.570882 0.0310795 -0.533881 -0.000229727 -5.68634e-06 0.000266362
    coeff[0] =  0.570882;
    coeff[1] =  0.0310795;
    coeff[2] = -0.533881;
    coeff[3] = -0.000229727;
    coeff[4] = -5.68634e-06;
    coeff[5] =  0.000266362;
    
    vgl_point_2d<double> pp(640, 360);  //principle point
    
    return VxlPTZCamera::composeCamera(fl, coeff, pp, pan, tilt, rod, cc, camera);
}

bool VxlPTZCamera::PTZToCamera(const vnl_vector_fixed<double, 6> & coeff, double fl, double pan, double tilt, vpgl_perspective_camera<double> & camera)
{
    //estimte fl, pan, tilt from camera
    vgl_point_3d<double> cc(12.9456, -14.8695, 6.21215); //camera center
    vnl_vector_fixed<double, 3> rod;    // 1.58044 -0.118628 0.124857
    rod[0] =    1.58044;
    rod[1] = -0.118628;
    rod[2] =  0.124857;
    
    vgl_point_2d<double> pp(640, 360);  //principle point
    
    return VxlPTZCamera::composeCamera(fl, coeff, pp, pan, tilt, rod, cc, camera);    
}

bool VxlPTZCamera::composeCamera(double fl, const vnl_vector_fixed<double, 6> & coeff,
                                  const vgl_point_2d<double> & principlePoint,
                                  double pan, double tilt,
                                  const vnl_vector_fixed<double, 3> & rod,
                                  const vgl_point_3d<double> & cameraCenter,
                                  vpgl_perspective_camera<double> & camera)
{
    double c1 = coeff[0];
    double c2 = coeff[1];
    double c3 = coeff[2];
    double c4 = coeff[3];
    double c5 = coeff[4];
    double c6 = coeff[5];
    
    vgl_rotation_3d<double> Rs(rod);  // model rotation
    
 //   vcl_cout<<"R matrix is "<<Rs.as_matrix()<<vcl_endl;
    
    vpgl_calibration_matrix<double> K(fl, principlePoint);
    vnl_matrix_fixed<double, 3, 4> C;
    C.set_identity();
    C(0, 3) = - (c1 + c4 * fl);
    C(1, 3) = - (c2 + c5 * fl);
    C(2, 3) = - (c3 + c6 * fl);
    
    vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
    VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
    
    vnl_matrix_fixed<double, 4, 4> RSD;
    VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter,  RSD);
    
    vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
    
    bool isDecompose = vpgl_perspective_decomposition(P, camera);
    if (!isDecompose) {
        return false;
    }
    
    double fl_n = camera.get_calibration().get_matrix()[0][0];
    vgl_point_2d<double> pp = camera.get_calibration().principal_point();
    vpgl_calibration_matrix<double> KK(fl_n, pp);
    camera.set_calibration(KK);
    return true;
}


class estimatePanTiltByFixsingModelPositionRotationResidual: public vnl_least_squares_function
{
protected:
    const VxlPTZCamera::PtsPairs corre_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> rod_; //rodrigue angle of model rotation
    const vnl_vector_fixed<double, 6> coeff_;
    
public:
    estimatePanTiltByFixsingModelPositionRotationResidual(const VxlPTZCamera::PtsPairs & corre, const vgl_point_2d<double> & pp,
                                                          const vgl_point_3d<double> & cc, const vnl_vector_fixed<double, 3> & rod,
                                                          const vnl_vector_fixed<double, 6> & coeff,
                                                          int pts_num):
    vnl_least_squares_function(3, 2 * pts_num, no_gradient),
    corre_(corre),
    principlePoint_(pp),
    cameraCenter_(cc),
    rod_(rod),
    coeff_(coeff)
    {
        assert(corre.wld_pts.size() >= 4);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double fl   = x[0];
        double pan  = x[1];
        double tilt = x[2];
        double c1 = coeff_[0];
        double c2 = coeff_[1];
        double c3 = coeff_[2];
        double c4 = coeff_[3];
        double c5 = coeff_[4];
        double c6 = coeff_[5];
        
        vgl_rotation_3d<double> Rs(rod_);  // model rotation
        
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        vnl_matrix_fixed<double, 3, 4> C;
        C.set_identity();
        C(0, 3) = - (c1 + c4 * fl);
        C(1, 3) = - (c2 + c5 * fl);
        C(2, 3) = - (c3 + c6 * fl);
        
        vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
        VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
        
        vnl_matrix_fixed<double, 4, 4> RSD;
        VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
        
        vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
        vpgl_proj_camera<double> camera(P);
        
        // loop each points
        int idx = 0;
        for (int i = 0; i<corre_.wld_pts.size(); i++) {
            vgl_point_2d<double> p = corre_.wld_pts[i];
            vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
            
            fx[idx] = corre_.img_pts[i].x() - q.x();
            idx++;
            fx[idx] = corre_.img_pts[i].y() - q.y();
            idx++;
        }
    }
    
    void getProjection(vnl_vector<double> const &x, vpgl_proj_camera<double> & projection)
    {
        double fl   = x[0];
        double pan  = x[1];
        double tilt = x[2];
        double c1 = coeff_[0];
        double c2 = coeff_[1];
        double c3 = coeff_[2];
        double c4 = coeff_[3];
        double c5 = coeff_[4];
        double c6 = coeff_[5];
        
        vgl_rotation_3d<double> Rs(rod_);  // model rotation
        
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        vnl_matrix_fixed<double, 3, 4> C;
        C.set_identity();
        C(0, 3) = - (c1 + c4 * fl);
        C(1, 3) = - (c2 + c5 * fl);
        C(2, 3) = - (c3 + c6 * fl);
        
        vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
        VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
        
        vnl_matrix_fixed<double, 4, 4> RSD;
        VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
        
        vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
        projection = vpgl_proj_camera<double>(P);
    }
};

bool VxlPTZCamera::estimateFlPanTiltByFixingModelPositionRotation (const PtsPairs & corre, const vpgl_perspective_camera<double> & initCamera,
                                                                             const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                             const vnl_vector_fixed<double, 6> & coeff, vnl_vector_fixed<double, 3> & flPanTilt,
                                                                             vpgl_perspective_camera<double> & estimatedCamera)
{
    assert(corre.wld_pts.size() >= 4);
    
    estimatePanTiltByFixsingModelPositionRotationResidual residual(corre, initCamera.get_calibration().principal_point(), cameraCenter, rod, coeff, (int)corre.img_pts.size());
    
    // init values
    vnl_vector<double> x(3, 0.0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    double wx = initCamera.get_rotation().as_matrix()[2][0];
    double wy = initCamera.get_rotation().as_matrix()[2][1];
    double wz = initCamera.get_rotation().as_matrix()[2][2];
    double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
    double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
    x[1] = pan;
    x[2] = tilt;
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimized = lmq.minimize(x);
    if (!isMinimized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
 //   lmq.diagnose_outcome();
    
    flPanTilt[0] = x[0];
    flPanTilt[1] = x[1];
    flPanTilt[2] = x[2];
    
    vpgl_proj_camera<double> projection;
    residual.getProjection(x, projection);
    
    return vpgl_perspective_decomposition(projection.get_matrix(), estimatedCamera);
}

class estimateFlPanTiltByFixingModelPositionRotationProjectioncentercoeffResidual:public vnl_least_squares_function
{
protected:
    const VxlPTZCamera::PtsPairs corre_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> rod_; //rodrigue angle of model rotation
    const vnl_vector_fixed<double, 4> coeff_;
    
public:
    estimateFlPanTiltByFixingModelPositionRotationProjectioncentercoeffResidual(const VxlPTZCamera::PtsPairs & corre, const vgl_point_2d<double> & pp,
                                                                                const vgl_point_3d<double> & cc, const vnl_vector_fixed<double, 3> & rod,
                                                                                const vnl_vector_fixed<double, 4> & coeff,
                                                                                int pts_num):
    vnl_least_squares_function(3, 2 * pts_num, no_gradient),
    corre_(corre),
    principlePoint_(pp),
    cameraCenter_(cc),
    rod_(rod),
    coeff_(coeff)
    {
        assert(corre.wld_pts.size() >= 4);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double fl   = x[0];
        double pan  = x[1];
        double tilt = x[2];
        double c1 = coeff_[0];
        double c2 = coeff_[1];
        double c3 = coeff_[2];
        double c4 = coeff_[3];
        
        vgl_rotation_3d<double> Rs(rod_);  // model rotation
        
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        vnl_matrix_fixed<double, 3, 4> C;
        C.set_identity();
        C(0, 3) = - (c1);
        C(1, 3) = - (c2);
        C(2, 3) = - (c3 + c4 * fl);
        
        vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
        VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
        
        vnl_matrix_fixed<double, 4, 4> RSD;
        VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
        
        vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
        vpgl_proj_camera<double> camera(P);
        
        // loop each points
        int idx = 0;
        for (int i = 0; i<corre_.wld_pts.size(); i++) {
            vgl_point_2d<double> p = corre_.wld_pts[i];
            vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
            
            fx[idx] = corre_.img_pts[i].x() - q.x();
            idx++;
            fx[idx] = corre_.img_pts[i].y() - q.y();
            idx++;
        }
    }
    
    void getProjection(vnl_vector<double> const &x, vpgl_proj_camera<double> & projection)
    {
        double fl   = x[0];
        double pan  = x[1];
        double tilt = x[2];
        double c1 = coeff_[0];
        double c2 = coeff_[1];
        double c3 = coeff_[2];
        double c4 = coeff_[3];
        
        vgl_rotation_3d<double> Rs(rod_);  // model rotation
        
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        vnl_matrix_fixed<double, 3, 4> C;
        C.set_identity();
        C(0, 3) = - (c1);
        C(1, 3) = - (c2);
        C(2, 3) = - (c3 + c4 * fl);
        
        vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
        VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
        
        vnl_matrix_fixed<double, 4, 4> RSD;
        VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
        
        vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
        projection = vpgl_proj_camera<double>(P);
    }
};

bool VxlPTZCamera::estimateFlPanTiltByFixingModelPositionRotationProjectioncentercoeff(const PtsPairs & corre, const vpgl_perspective_camera<double> & initCamera,
                                                                                       const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                                       const vnl_vector_fixed<double, 4> & coeff, vnl_vector_fixed<double, 3> & flPanTilt,
                                                                                       vpgl_perspective_camera<double> & estimatedCamera)
{
    assert(corre.wld_pts.size() >= 4);
    
    estimateFlPanTiltByFixingModelPositionRotationProjectioncentercoeffResidual residual(corre, initCamera.get_calibration().principal_point(), cameraCenter, rod, coeff, (int)corre.img_pts.size());
    
    // init values
    vnl_vector<double> x(3, 0.0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    double wx = initCamera.get_rotation().as_matrix()[2][0];
    double wy = initCamera.get_rotation().as_matrix()[2][1];
    double wz = initCamera.get_rotation().as_matrix()[2][2];
    double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
    double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
    x[1] = pan;
    x[2] = tilt;
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimized = lmq.minimize(x);
    if (!isMinimized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
   //    lmq.diagnose_outcome();
    
    flPanTilt[0] = x[0];
    flPanTilt[1] = x[1];
    flPanTilt[2] = x[2];
    
    vpgl_proj_camera<double> projection;
    residual.getProjection(x, projection);
    
    return vpgl_perspective_decomposition(projection.get_matrix(), estimatedCamera);
}

class estimatePanTiltByFixingModelPositionRotationFlResidual: public vnl_least_squares_function
{
protected:
    const VxlPTZCamera::PtsPairs corre_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> rod_; //rodrigue angle of model rotation
    const vnl_vector_fixed<double, 6> coeff_;
    const double fl_;
    
public:
    estimatePanTiltByFixingModelPositionRotationFlResidual(const VxlPTZCamera::PtsPairs & corre, const vgl_point_2d<double> & pp,
                                                             const vgl_point_3d<double> & cc, const vnl_vector_fixed<double, 3> & rod,
                                                             const vnl_vector_fixed<double, 6> & coeff,
                                                             const double fl,
                                                             int pts_num):
    vnl_least_squares_function(2, 2 * pts_num, no_gradient),
    corre_(corre),
    principlePoint_(pp),
    cameraCenter_(cc),
    rod_(rod),
    coeff_(coeff),
    fl_(fl)
    {
        assert(corre.wld_pts.size() >= 4);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double pan  = x[0];
        double tilt = x[1];
        double c1 = coeff_[0];
        double c2 = coeff_[1];
        double c3 = coeff_[2];
        double c4 = coeff_[3];
        double c5 = coeff_[4];
        double c6 = coeff_[5];
        
        vgl_rotation_3d<double> Rs(rod_);  // model rotation
        
        vpgl_calibration_matrix<double> K(fl_, principlePoint_);
        vnl_matrix_fixed<double, 3, 4> C;
        C.set_identity();
        C(0, 3) = - (c1 + c4 * fl_);
        C(1, 3) = - (c2 + c5 * fl_);
        C(2, 3) = - (c3 + c6 * fl_);
        
        vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
        VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
        
        vnl_matrix_fixed<double, 4, 4> RSD;
        VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
        
        vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
        vpgl_proj_camera<double> camera(P);
        
        // loop each points
        int idx = 0;
        for (int i = 0; i<corre_.wld_pts.size(); i++) {
            vgl_point_2d<double> p = corre_.wld_pts[i];
            vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
            
            fx[idx] = corre_.img_pts[i].x() - q.x();
            idx++;
            fx[idx] = corre_.img_pts[i].y() - q.y();
            idx++;
        }
    }
    
    void getProjection(vnl_vector<double> const &x, vpgl_proj_camera<double> & projection)
    {
        double pan  = x[0];
        double tilt = x[1];
        double c1 = coeff_[0];
        double c2 = coeff_[1];
        double c3 = coeff_[2];
        double c4 = coeff_[3];
        double c5 = coeff_[4];
        double c6 = coeff_[5];
        
        vgl_rotation_3d<double> Rs(rod_);  // model rotation
        
        vpgl_calibration_matrix<double> K(fl_, principlePoint_);
        vnl_matrix_fixed<double, 3, 4> C;
        C.set_identity();
        C(0, 3) = - (c1 + c4 * fl_);
        C(1, 3) = - (c2 + c5 * fl_);
        C(2, 3) = - (c3 + c6 * fl_);
        
        vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
        VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
        
        vnl_matrix_fixed<double, 4, 4> RSD;
        VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
        
        vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
        projection = vpgl_proj_camera<double>(P);
    }
};


bool VxlPTZCamera::estimatePanTiltByFixingModelPositionRotationFl(const PtsPairs & corre, const vpgl_perspective_camera<double> & initCamera,
                                                                    const double fl,
                                                                    const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                    const vnl_vector_fixed<double, 6> & coeff, vnl_vector_fixed<double, 2> & panTilt,
                                                                    vpgl_perspective_camera<double> & estimatedCamera)
{
    assert(corre.wld_pts.size() >= 4);
    
    estimatePanTiltByFixingModelPositionRotationFlResidual residual(corre, initCamera.get_calibration().principal_point(), cameraCenter, rod, coeff, fl, (int)corre.img_pts.size());
    
    // init values
    vnl_vector<double> x(2, 0.0);
    double wx = initCamera.get_rotation().as_matrix()[2][0];
    double wy = initCamera.get_rotation().as_matrix()[2][1];
    double wz = initCamera.get_rotation().as_matrix()[2][2];
    double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
    double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
    x[0] = pan;
    x[1] = tilt;
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimized = lmq.minimize(x);
    if (!isMinimized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();    
    
    panTilt[0] = x[0];
    panTilt[1] = x[1];
    
    vpgl_proj_camera<double> projection;
    residual.getProjection(x, projection);
    
    vpgl_perspective_decomposition(projection.get_matrix(), estimatedCamera);
    return true;
}

double VxlPTZCamera::overlayRatio(const vpgl_perspective_camera<double> & gd_camera, const vpgl_perspective_camera<double> & estimated_camera, int width, int height)
{
    vgl_h_matrix_2d<double> H1;
    vgl_h_matrix_2d<double> H2;
    
    DisneyWorldBasketballCourt court;
    
    bool isH = court.worldToCameraHomography(gd_camera, H1, width, height);
    assert(isH);
    isH = court.worldToCameraHomography(estimated_camera, H2, width, height);
    assert(isH);
    
    vgl_h_matrix_2d<double> H = H2 * H1.get_inverse();
    
    int num = 0;
    for (int j = 0; j<height; j++) {
        for (int i = 0; i<width; i++) {
            vgl_homg_point_2d<double> p(i, j, 1.0);
            vgl_homg_point_2d<double> q = H(p);
            double x = q.x()/q.w();
            double y = q.y()/q.w();
            if (x >= 0 && x < width && y >= 0 && y < height) {
                num++;
            }
        }
    }
    
    return 1.0 * num/(width * height);
}

bool VxlPTZCamera::interpolateCamera(const vpgl_perspective_camera<double> & camera1, const int fn1,
                                     const vpgl_perspective_camera<double> & camera2, const int fn2,
                                     const int fn3, vpgl_perspective_camera<double> & camera3)
{
    assert(fn1 < fn3);
    assert(fn2 > fn3);
    
    double pan1, tilt1, fl1;
    double pan2, tilt2, fl2;
    bool isPTZ = VxlPTZCamera::CameraToPTZ(camera1, pan1, tilt1, fl1);
    if (!isPTZ) {
        return false;
    }
    isPTZ = VxlPTZCamera::CameraToPTZ(camera2, pan2, tilt2, fl2);
    if (!isPTZ) {
        return false;
    }
    
    //
    double pan3, tilt3, fl3;
    pan3  = VxlPTZCamera::linearInterpolate(pan1, pan2, fn1, fn2, fn3);
    tilt3 = VxlPTZCamera::linearInterpolate(tilt1, tilt2, fn1, fn2, fn3);
    fl3   = VxlPTZCamera::linearInterpolate(fl1, fl2, fn1, fn2, fn3);
    
    return VxlPTZCamera::PTZToCamera(fl3, pan3, tilt3, camera3);
}

bool VxlPTZCamera::interpolateCameras(const vpgl_perspective_camera<double> & camera1, const int fn1,
                                      const vpgl_perspective_camera<double> & camera2, const int fn2,
                                      const vcl_vector<int> & fns, vcl_vector<vpgl_perspective_camera<double> > & cameras)
{
    assert(fn1 < fn2);
    
    //check each frame number
    for (int i = 0; i<fns.size(); i++) {
        int fn = fns[i];
        if (fn < fn1 || fn > fn2) {
            printf("fn = %d not in side of (%d %d)\n", fn, fn1, fn2);
            return false;
        }
    }
    
    double pan1, tilt1, fl1;
    double pan2, tilt2, fl2;
    bool isPTZ = VxlPTZCamera::CameraToPTZ(camera1, pan1, tilt1, fl1);
    if (!isPTZ) {
        return false;
    }
    isPTZ = VxlPTZCamera::CameraToPTZ(camera2, pan2, tilt2, fl2);
    if (!isPTZ) {
        return false;
    }
    
    for (int i = 0; i<fns.size(); i++) {
        int fn = fns[i];
        double pan3, tilt3, fl3;
        pan3  = VxlPTZCamera::linearInterpolate(pan1, pan2, fn1, fn2, fn);
        tilt3 = VxlPTZCamera::linearInterpolate(tilt1, tilt2, fn1, fn2, fn);
        fl3   = VxlPTZCamera::linearInterpolate(fl1, fl2, fn1, fn2, fn);
        vpgl_perspective_camera<double> camera;
        bool isCamera = VxlPTZCamera::PTZToCamera(fl3, pan3, tilt3, camera);
        if (!isCamera) {
            printf("Error: can not recovery camera from (fl, pan, tilt) %f %f %f\n", fl3, pan3, tilt3);
            return false;
        }
        cameras.push_back(camera);
    }
    assert(fns.size() == cameras.size());   
    
    return true;
}






