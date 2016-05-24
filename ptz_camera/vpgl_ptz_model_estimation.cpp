//
//  vpgl_ptz_model_estimation.cpp
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vpgl_ptz_model_estimation.h"
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vpgl/vpgl_proj_camera.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vcl_algorithm.h>
#include "vxl_plus.h"
//#include "basketballCourt.h"
#include "vpgl_plus.h"
#include "vnl_plus.h"

class estimateCommonCameraCenterResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence > corres_;
    const vcl_vector<double>  focalLengthes_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    estimateCommonCameraCenterResidual( const vcl_vector<VpglPTZModelEstimation::Correspondence > & corres, const vcl_vector<double>  &focalLengthes, const vgl_point_2d<double> & pp, int pts_num):
    vnl_least_squares_function(3 + 3 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    focalLengthes_(focalLengthes),
    principlePoint_(pp)
    {
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        // for each camera
        vgl_point_3d<double> cc(x[0], x[1], x[2]);
        
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_perspective_camera<double> camera;
            vpgl_calibration_matrix<double> K(focalLengthes_[i], principlePoint_);
            vnl_vector_fixed<double, 3> rod(x[3 + 3 * i + 0], x[3 + 3 * i + 1], x[3 + 3 * i + 2]);
            vgl_rotation_3d<double> R(rod);
            camera.set_calibration(K);
            camera.set_rotation(R);
            camera.set_camera_center(cc);
            
            //loop each correspondence
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getCameras(vnl_vector<double> const &x, vcl_vector<vpgl_perspective_camera<double> > & cameras)
    {
        // for each camera
        vgl_point_3d<double> cc(x[0], x[1], x[2]);
        
        cameras.resize(focalLengthes_.size());
        for (int i = 0; i<focalLengthes_.size(); i++) {
            vpgl_perspective_camera<double> camera;
            vpgl_calibration_matrix<double> K(focalLengthes_[i], principlePoint_);
            vnl_vector_fixed<double, 3> rod(x[3 + 3 * i + 0], x[3 + 3 * i + 1], x[3 + 3 * i + 2]);
            vgl_rotation_3d<double> R(rod);
            camera.set_calibration(K);
            camera.set_rotation(R);
            camera.set_camera_center(cc);
            
            cameras[i] = camera;
        }
    }
    
};




bool VpglPTZModelEstimation::estimateCommonCameraCenter(const vcl_vector<Correspondence > & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                        vgl_point_3d<double> & cameraCenter, vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras)
{
    assert(corres.size() >= 0);
    assert(corres.size() == initCameras.size());
    
    vcl_vector<double> fls;
    int pts_num = 0;
    double cx = 0.0, cy = 0.0, cz = 0.0;
    for (int i = 0; i<corres.size(); i++) {
        fls.push_back(initCameras[i].get_calibration().get_matrix()[0][0]);
        pts_num += corres[i].wld_pts.size();
        cx += initCameras[i].get_camera_center().x();
        cy += initCameras[i].get_camera_center().y();
        cz += initCameras[i].get_camera_center().z();
    }
    cx /= corres.size();
    cy /= corres.size();
    cz /= corres.size();
    
    estimateCommonCameraCenterResidual residual(corres, fls, initCameras[0].get_calibration().principal_point(), pts_num);
    
    vnl_vector<double> x(3 + 3 * (unsigned int)initCameras.size(), 0);
    x[0] = cx;
    x[1] = cy;
    x[2] = cz;
    
    for (int i = 0; i<initCameras.size(); i++) {
        x[3 + i * 3 + 0] = initCameras[i].get_rotation().as_rodrigues()[0];
        x[3 + i * 3 + 1] = initCameras[i].get_rotation().as_rodrigues()[1];
        x[3 + i * 3 + 2] = initCameras[i].get_rotation().as_rodrigues()[2];
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimized = lmq.minimize(x);
    if (!isMinimized) {
        vcl_cerr<<"Error: minimization failed.\n"<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    cameraCenter.set(x[0], x[1], x[2]);
    residual.getCameras(x, estimatedCameras);
    return true;
}

class estimateProjectionCentersResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence > corres_;
    const vcl_vector<double>   focalLengthes_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    
public:
    estimateProjectionCentersResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence > & corres, const vcl_vector<double>  & focalLengthes,
                                      const vgl_point_2d<double> &pp, const vgl_point_3d<double> & cc, int pts_num):
    vnl_least_squares_function(6 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    focalLengthes_(focalLengthes),
    principlePoint_(pp),
    cameraCenter_(cc)
    {
        assert(corres.size() == focalLengthes.size());
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        // loop each corres
        // projection center, rotation angle, ...
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_calibration_matrix<double> K(focalLengthes_[i], principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C = vnl_matrix_fixed<double, 3, 4>().set_identity();
            C[0][3] = - x[6 * i + 0];
            C[1][3] = - x[6 * i + 1];
            C[2][3] = - x[6 * i + 2];
            
            vnl_vector_fixed<double, 3> rod(x[6 * i + 3], x[6 * i + 4], x[6 * i + 5]);
            vgl_rotation_3d<double> R(rod);
         
            vnl_matrix_fixed<double, 4, 4> RT;  // rotation and translation
            VpglPlus::matrixFromRotationCameraCenter(R.as_matrix(), cameraCenter_, RT);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * RT;
            vpgl_proj_camera<double> camera(P);
            
            // image points correspondence
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjection(vnl_vector<double> &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        projections.resize(corres_.size());
        for (int i = 0; i<focalLengthes_.size(); i++) {
            vpgl_calibration_matrix<double> K(focalLengthes_[i], principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C = vnl_matrix_fixed<double, 3, 4>().set_identity();
            C[0][3] = - x[6 * i + 0];
            C[1][3] = - x[6 * i + 1];
            C[2][3] = - x[6 * i + 2];
            
            vnl_vector_fixed<double, 3> rod(x[6 * i + 3], x[6 * i + 4], x[6 * i + 5]);
            vgl_rotation_3d<double> R(rod);
            
            vnl_matrix_fixed<double, 4, 4> RT;  // rotation and translation
            VpglPlus::matrixFromRotationCameraCenter(R.as_matrix(), cameraCenter_, RT);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * RT;
            vpgl_proj_camera<double> camera(P);
            
            projections[i] = camera;
        }
        assert(projections.size() == corres_.size());
    }
    
};


bool VpglPTZModelEstimation::estimateProjectionCenters(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                       const vgl_point_3d<double> & cameraCenter,
                                                       vcl_vector< vgl_point_3d<double> > & projectionCenters, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() >= 0);
    assert(corres.size() == initCameras.size());
    
    vcl_vector<double> fls;
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        fls.push_back(initCameras[i].get_calibration().get_matrix()[0][0]);
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjectionCentersResidual residual(corres, fls, initCameras[0].get_calibration().principal_point(), cameraCenter, pts_num);
    
    vnl_vector<double> x(6 * (unsigned int)initCameras.size());
    for (int i = 0; i<initCameras.size(); i++) {
        x[6 * i + 0] = 0.0;
        x[6 * i + 1] = 0.0;
        x[6 * i + 2] = 0.0;
        x[6 * i + 3] = initCameras[i].get_rotation().as_rodrigues()[0];
        x[6 * i + 4] = initCameras[i].get_rotation().as_rodrigues()[1];
        x[6 * i + 5] = initCameras[i].get_rotation().as_rodrigues()[2];
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    projectionCenters.resize(initCameras.size());
    for (int i = 0; i<initCameras.size(); i++) {
        vgl_point_3d<double> pc(x[6 * i + 0], x[6 * i + 1], x[6 * i + 2]);
        projectionCenters[i] = pc;
    }
    
    residual.getProjection(x, estimatedCameras);
    
    return true;
}

class estimateProjectionCentersFixPanTiltResidual: public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence > corres_;
    const vcl_vector<double>   focalLengthes_;
    const vcl_vector<double>   panAngles_;
    const vcl_vector<double>   tiltAngles_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    
public:
    estimateProjectionCentersFixPanTiltResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence > & corres, const vcl_vector<double> & focalLengthes, const vcl_vector<double> & panAngles,
                                                const vcl_vector<double> & tiltAngles, const vgl_point_2d<double> & pp, const vgl_point_3d<double> & cc, int pts_num):
    vnl_least_squares_function(3 + 3 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    focalLengthes_(focalLengthes),
    panAngles_(panAngles),
    tiltAngles_(tiltAngles),
    principlePoint_(pp),
    cameraCenter_(cc)
    {
        assert(corres.size() == focalLengthes.size());
        assert(corres.size() == panAngles.size());
        assert(corres.size() == tiltAngles.size());
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        // loop each camera
        // stationary rotation, cxyz1, cxyz2
        vnl_vector_fixed<double, 3> rod(x[0], x[1], x[2]);
        vgl_rotation_3d<double> Rs(rod);
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_calibration_matrix<double> K(focalLengthes_[i], principlePoint_);
            
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - x[3 + 3 * i + 0];
            C(1, 3) = - x[3 + 3 * i + 1];
            C(2, 3) = - x[3 + 3 * i + 2];
            
            vnl_matrix_fixed<double, 4, 4> panTiltR;
            VpglPlus::matrixFromPanYTiltX(panAngles_[i], tiltAngles_[i], panTiltR);
            
            vnl_matrix_fixed<double, 4, 4> RsT;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_, RsT);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * panTiltR * RsT;
            vpgl_proj_camera<double> camera(P);
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        vnl_vector_fixed<double, 3> rod(x[0], x[1], x[2]);
        vgl_rotation_3d<double> Rs(rod);
        
        projections.resize(focalLengthes_.size());
        for (int i = 0; i<focalLengthes_.size(); i++) {
            vpgl_calibration_matrix<double> K(focalLengthes_[i], principlePoint_);
            
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - x[3 + 3 * i + 0];
            C(1, 3) = - x[3 + 3 * i + 1];
            C(2, 3) = - x[3 + 3 * i + 2];
            
            vnl_matrix_fixed<double, 4, 4> panTiltR;
            VpglPlus::matrixFromPanYTiltX(panAngles_[i], tiltAngles_[i], panTiltR);
            
            vnl_matrix_fixed<double, 4, 4> RsT;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_, RsT);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * panTiltR * RsT;
            vpgl_proj_camera<double> camera(P);
            projections[i] = camera;
        }
    }
};



bool VpglPTZModelEstimation::estimateProjectionCentersFixPanTilt(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                 const vgl_point_3d<double> & cameraCenter, const vcl_vector<double> & panAngles, const vcl_vector<double> & tiltAngles,
                                                                 vcl_vector< vgl_point_3d<double> > & projectionCenters, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() == panAngles.size());
    assert(corres.size() == tiltAngles.size());
    
    vcl_vector<double> fls;
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
        fls.push_back(initCameras[i].get_calibration().get_matrix()[0][0]);
    }
    
    estimateProjectionCentersFixPanTiltResidual residual(corres, fls, panAngles, tiltAngles, initCameras[0].get_calibration().principal_point(), cameraCenter, pts_num);
    
    vnl_vector<double> x(3 + 3 * (unsigned int)initCameras.size(), 0);
    
    vnl_matrix_fixed<double, 3, 3> R_stationary;
    R_stationary.fill(0);
    R_stationary[0][0] = 1.0;
    R_stationary[1][2] = -1.0;
    R_stationary[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(R_stationary);
    x[0] = RS.as_rodrigues()[0];
    x[1] = RS.as_rodrigues()[1];
    x[2] = RS.as_rodrigues()[2];
    for (int i = 0; i<initCameras.size(); i++) {
        x[3 + 3 * i + 0] = 0;
        x[3 + 3 * i + 1] = 0;
        x[3 + 3 * i + 2] = 0;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    projectionCenters.resize(initCameras.size());
    for (int i = 0; i<initCameras.size(); i++) {
        vgl_point_3d<double> pc(x[3 + 3 * i + 0], x[3 + 3 * i + 1], x[3 + 3 * i + 2]);
        projectionCenters[i] = pc;
    }
    residual.getProjections(x, estimatedCameras);
    return true;
}

bool  VpglPTZModelEstimation::estimateProjectionCentersFixFlPanTilt(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                    const vgl_point_3d<double> & cameraCenter,
                                                                    const vcl_vector<vnl_vector_fixed<double, 3> > & fl_pan_tilt_s,
                                                                    vcl_vector< vgl_point_3d<double> > & projectionCenters, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() == fl_pan_tilt_s.size());   
    
    vcl_vector<double> fls;
    vcl_vector<double> panAngles;
    vcl_vector<double> tiltAngles;
    for (int i = 0; i<fl_pan_tilt_s.size(); i++) {
        fls.push_back(fl_pan_tilt_s[i][0]);
        panAngles.push_back(fl_pan_tilt_s[i][1]);
        tiltAngles.push_back(fl_pan_tilt_s[i][2]);
    }
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjectionCentersFixPanTiltResidual residual(corres, fls, panAngles, tiltAngles, initCameras[0].get_calibration().principal_point(), cameraCenter, pts_num);
    
    vnl_vector<double> x(3 + 3 * (unsigned int)initCameras.size(), 0);
    
    vnl_matrix_fixed<double, 3, 3> R_stationary;
    R_stationary.fill(0);
    R_stationary[0][0] = 1.0;
    R_stationary[1][2] = -1.0;
    R_stationary[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(R_stationary);
    x[0] = RS.as_rodrigues()[0];
    x[1] = RS.as_rodrigues()[1];
    x[2] = RS.as_rodrigues()[2];
    for (int i = 0; i<initCameras.size(); i++) {
        x[3 + 3 * i + 0] = 0;
        x[3 + 3 * i + 1] = 0;
        x[3 + 3 * i + 2] = 0;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    projectionCenters.resize(initCameras.size());
    for (int i = 0; i<initCameras.size(); i++) {
        vgl_point_3d<double> pc(x[3 + 3 * i + 0], x[3 + 3 * i + 1], x[3 + 3 * i + 2]);
        projectionCenters[i] = pc;
    }
    residual.getProjections(x, estimatedCameras);
    return true;
}


class estimateCommomRotationCenterAndStationaryRotationResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    estimateCommomRotationCenterAndStationaryRotationResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp, int pts_num):
    vnl_least_squares_function(6 + 3 * (unsigned int)corres.size(), pts_num * 2 , no_gradient),
    corres_(corres),
    principlePoint_(pp)
    {
        assert(corres.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double>     Rs(rod);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_perspective_camera<double> curCamera;
            
            double fl = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
            
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 3> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix<double> QS = Q.as_matrix() * Rs.as_matrix().as_matrix();
            vnl_matrix_fixed<double, 3, 3> QS33(QS);
            
            curCamera.set_calibration(K);
            curCamera.set_camera_center(cc);
            curCamera.set_rotation(vgl_rotation_3d<double>(QS33));
            
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(curCamera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getCameras(vnl_vector<double> const & x, vcl_vector<vpgl_perspective_camera<double> > & cameras)
    {
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        
        // loop each camera
        cameras.clear();
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_perspective_camera<double> curCamera;
            
            double fl   = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            
            vnl_matrix_fixed<double, 3, 3> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix<double> QS = Q.as_matrix() * Rs.as_matrix().as_matrix();
            vnl_matrix_fixed<double, 3, 3> QS33(QS);
            
            curCamera.set_calibration(K);
            curCamera.set_camera_center(cc);
            curCamera.set_rotation(vgl_rotation_3d<double>(QS33));
            cameras.push_back(curCamera);
        }
    }
};


bool VpglPTZModelEstimation::estimateCommomRotationCenterAndStationaryRotation(const vcl_vector<Correspondence> & corres,
                                                              const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                              vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateCommomRotationCenterAndStationaryRotationResidual residual(corres, initCameras[0].get_calibration().principal_point(), pts_num);
    
    vnl_vector<double> x(6 + 3 * (int)initCameras.size(), 0.0);
    
    double dx = 0;
    double dy = 0;
    double dz = 0;
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        x[6 + 3 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[6 + 3 * i + 1] = pan;
        x[6 + 3 * i + 2] = tilt;
        
     //   printf("initial pan tilt is %f %f\n", pan, tilt);
        
        dx += curCamera.get_camera_center().x();
        dy += curCamera.get_camera_center().y();
        dz += curCamera.get_camera_center().z();
    }
    
    x[0] = dx/initCameras.size();
    x[1] = dy/initCameras.size();
    x[2] = dz/initCameras.size();
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[3] = RS.as_rodrigues()[0];
    x[4] = RS.as_rodrigues()[1];
    x[5] = RS.as_rodrigues()[2];
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    residual.getCameras(x, estimatedCameras);
    
    vcl_cout<<"D S is: "<<vcl_endl;
    for (int i = 0; i<6; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    
    return true;
}

class estimateCommomRotationCenterAndStationaryRotationFixFirstPanResidual: public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    estimateCommomRotationCenterAndStationaryRotationFixFirstPanResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp, int pts_num):
    vnl_least_squares_function(6 + 3 * (unsigned int)corres.size(), pts_num * 2 , no_gradient),
    corres_(corres),
    principlePoint_(pp)
    {
        assert(corres.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double>     Rs(rod);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_perspective_camera<double> curCamera;
            
            double fl = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
            if (i == 0) {
                pan = 0;
            }
            
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 3> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix<double> QS = Q.as_matrix() * Rs.as_matrix().as_matrix();
            vnl_matrix_fixed<double, 3, 3> QS33(QS);
            
            curCamera.set_calibration(K);
            curCamera.set_camera_center(cc);
            curCamera.set_rotation(vgl_rotation_3d<double>(QS33));
            
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(curCamera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getCameras(vnl_vector<double> const & x, vcl_vector<vpgl_perspective_camera<double> > & cameras)
    {
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        
        // loop each camera
        cameras.clear();
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_perspective_camera<double> curCamera;
            
            double fl   = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
            if (i == 0) {
                pan = 0;
            }
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            
            vnl_matrix_fixed<double, 3, 3> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix<double> QS = Q.as_matrix() * Rs.as_matrix().as_matrix();
            vnl_matrix_fixed<double, 3, 3> QS33(QS);
            
            curCamera.set_calibration(K);
            curCamera.set_camera_center(cc);
            curCamera.set_rotation(vgl_rotation_3d<double>(QS33));
            cameras.push_back(curCamera);
        }
    }
};


bool VpglPTZModelEstimation::estimateCommomRotationCenterAndStationaryRotationFixFirstPan(const vcl_vector<Correspondence> & corres,
                                                                  const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                  vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateCommomRotationCenterAndStationaryRotationFixFirstPanResidual residual(corres, initCameras[0].get_calibration().principal_point(), pts_num);
    
    vnl_vector<double> x(6 + 3 * (int)initCameras.size(), 0.0);
    
    double dx = 0;
    double dy = 0;
    double dz = 0;
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        x[6 + 3 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[6 + 3 * i + 1] = pan;
        x[6 + 3 * i + 2] = tilt;
        
        //   printf("initial pan tilt is %f %f\n", pan, tilt);
        
        dx += curCamera.get_camera_center().x();
        dy += curCamera.get_camera_center().y();
        dz += curCamera.get_camera_center().z();
    }
    
    x[0] = dx/initCameras.size();
    x[1] = dy/initCameras.size();
    x[2] = dz/initCameras.size();
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[3] = RS.as_rodrigues()[0];
    x[4] = RS.as_rodrigues()[1];
    x[5] = RS.as_rodrigues()[2];
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    residual.getCameras(x, estimatedCameras);
    
    vcl_cout<<"D S is: "<<vcl_endl;
    for (int i = 0; i<6; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    
    vcl_cout<<"first pan tilt (pan should close to 0): "<<vcl_endl;
    for (int i = 7; i<9; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    
    return true;
}

class estimateProjecionCenterWithFocalLengthResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    estimateProjecionCenterWithFocalLengthResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp, int pts_num):
    vnl_least_squares_function(12 + 3 * (unsigned int)corres.size(), pts_num * 2 , no_gradient),
    corres_(corres),
    principlePoint_(pp)
    {
        assert(corres.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, C_123456, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[6];
        double c2 = x[7];
        double c3 = x[8];
        double c4 = x[9];
        double c5 = x[10];
        double c6 = x[11];
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl = x[12 + 3 * i + 0];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(x[12 + 3 * i + 1], x[12 + 3 * i + 2], Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const & x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //D_123, S_123, C_123456, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[6];
        double c2 = x[7];
        double c3 = x[8];
        double c4 = x[9];
        double c5 = x[10];
        double c6 = x[11];
        
        // loop each camera
        projections.resize(corres_.size());
        for (int i = 0; i<corres_.size(); i++) {
            double fl = x[12 + 3 * i + 0];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(x[12 + 3 * i + 1], x[12 + 3 * i + 2], Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            projections[i] = camera;
        }
    }
    
};


bool VpglPTZModelEstimation::estimateProjecionCenterWithFocalLength(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                    vnl_vector_fixed<double, 6> & coeff, 
                                                                    vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjecionCenterWithFocalLengthResidual residual(corres, initCameras[0].get_calibration().principal_point(), pts_num);
    
    vnl_vector<double> x(12 + 3 * (int)initCameras.size(), 0.0);
    
    double dx = 0;
    double dy = 0;
    double dz = 0;
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        x[12 + 3 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[12 + 3 * i + 1] = pan;
        x[12 + 3 * i + 2] = tilt;
        
        dx += curCamera.get_camera_center().x();
        dy += curCamera.get_camera_center().y();
        dz += curCamera.get_camera_center().z();
    }
    
    x[0] = dx/initCameras.size();
    x[1] = dy/initCameras.size();
    x[2] = dz/initCameras.size();
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[3] = RS.as_rodrigues()[0];
    x[4] = RS.as_rodrigues()[1];
    x[5] = RS.as_rodrigues()[2];
    
    for (int i = 6; i<12; i++) {
        x[i] = 0.0;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    // out put result
    for (int i = 0; i<6; i++) {
        coeff[i] = x[6 + i];
    }
    
    residual.getProjections(x, estimatedCameras);
    
    vcl_cout<<"D S is: "<<vcl_endl;
    for (int i = 0; i<6; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    return true;
}


class checkProjectionCenterCoefficientResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vnl_vector_fixed<double, 6> coeff_;
    
public:
    checkProjectionCenterCoefficientResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres,  const vgl_point_2d<double> & pp, const vnl_vector_fixed<double, 6> & coeff, int pts_num):
    vnl_least_squares_function(6 + 3 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    coeff_(coeff)
    {
        assert(corres_.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = coeff_[0];
        double c2 = coeff_[1];
        double c3 = coeff_[2];
        double c4 = coeff_[3];
        double c5 = coeff_[4];
        double c6 = coeff_[5];
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl = x[6 + 3 * i + 0];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(x[6 + 3 * i + 1], x[6 + 3 * i + 2], Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = coeff_[0];
        double c2 = coeff_[1];
        double c3 = coeff_[2];
        double c4 = coeff_[3];
        double c5 = coeff_[4];
        double c6 = coeff_[5];
        
        // loop each camera
        projections.resize(corres_.size());
        for (int i = 0; i<corres_.size(); i++) {
            double fl = x[6 + 3 * i + 0];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(x[6 + 3 * i + 1], x[6 + 3 * i + 2], Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            projections[i] = camera;
        }
    }
};

bool VpglPTZModelEstimation::checkProjectionCenterCoefficientWihFolcalLength(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                             const vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    checkProjectionCenterCoefficientResidual residual(corres, initCameras[0].get_calibration().principal_point(), coeff, pts_num);
    
    vnl_vector<double> x(6 + 3 * (int)initCameras.size(), 0.0);
    
    double dx = 0;
    double dy = 0;
    double dz = 0;
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        x[6 + 3 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[6 + 3 * i + 1] = pan;
        x[6 + 3 * i + 2] = tilt;
        
        dx += curCamera.get_camera_center().x();
        dy += curCamera.get_camera_center().y();
        dz += curCamera.get_camera_center().z();
    }
    
    x[0] = dx/initCameras.size();
    x[1] = dy/initCameras.size();
    x[2] = dz/initCameras.size();
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[3] = RS.as_rodrigues()[0];
    x[4] = RS.as_rodrigues()[1];
    x[5] = RS.as_rodrigues()[2];
   
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    
    residual.getProjections(x, estimatedCameras);
    
    vcl_cout<<"D S is: "<<vcl_endl;
    for (int i = 0; i<6; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    return true;
}

class estimateProjecionCenterWithFixedFocalLengthTiltResidual:public vnl_least_squares_function
{
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    estimateProjecionCenterWithFixedFocalLengthTiltResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> & pp, int pts_num):
    vnl_least_squares_function(14 + (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp)
    {
        assert(corres_.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, C_123456, fl, tilt pan0, pan2...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[6];
        double c2 = x[7];
        double c3 = x[8];
        double c4 = x[9];
        double c5 = x[10];
        double c6 = x[11];
        double fl = x[12];
        double tilt = x[13];
      //  double tilt = -14.078568;
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double pan = x[14 + i];
            
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const & x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //D_123, S_123, C_123456, fl, tilt pan0, pan2...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[6];
        double c2 = x[7];
        double c3 = x[8];
        double c4 = x[9];
        double c5 = x[10];
        double c6 = x[11];
        double fl = x[12];
        double tilt = x[13];
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        
        // loop each camera       
        for (int i = 0; i<corres_.size(); i++) {
            double pan = x[14 + i];
            
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            projections.push_back(P);
        }
    }
};

bool VpglPTZModelEstimation::estimateProjectionCenterWithFixedFocalLengthTilt(const vcl_vector<Correspondence> & corres,
                                                                              const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                              vnl_vector_fixed<double, 6> & coeff,
                                                                              vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjecionCenterWithFixedFocalLengthTiltResidual residual(corres, initCameras[0].get_calibration().principal_point(), pts_num);
    
    vnl_vector<double> x(14 + (int)initCameras.size(), 0.0);
    double fl_avg   = 0.0;
    double tilt_avg = 0.0;
    double dx = 0;
    double dy = 0;
    double dz = 0;
    const int N = (int)initCameras.size();
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        fl_avg += curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[14 + i] = pan;
        tilt_avg += tilt;
        
        dx += curCamera.get_camera_center().x();
        dy += curCamera.get_camera_center().y();
        dz += curCamera.get_camera_center().z();
    }
    
    x[0] = dx/N;
    x[1] = dy/N;
    x[2] = dz/N;
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[3] = RS.as_rodrigues()[0];
    x[4] = RS.as_rodrigues()[1];
    x[5] = RS.as_rodrigues()[2];
    
    for (int i = 6; i<12; i++) {
        x[i] = 0.0;
    }
    x[12] = fl_avg /N;
    x[13] = tilt_avg/N;
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    // out put result
    for (int i = 0; i<6; i++) {
        coeff[i] = x[6 + i];
    }
    
    residual.getProjections(x, estimatedCameras);
    
    vcl_cout<<"D S is: "<<vcl_endl;
    for (int i = 0; i<6; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    
    printf("fl, tilt is %f %f\n", x[12], x[13]);
    vcl_cout<<"pan is: "<<vcl_endl;
    for (int i = 14; i<x.size(); i++) {
        printf("%f ", x[i]);
    }
    printf("\n");
    return true;
}

class pinHoleModelEstimateProjectionCenterWithFixedFocalLengthTiltResidual:public vnl_least_squares_function
{
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    pinHoleModelEstimateProjectionCenterWithFixedFocalLengthTiltResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> & pp, int pts_num):
    vnl_least_squares_function(8 + (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp)
    {
        assert(corres_.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, fl, tilt pan0, pan2...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double fl = x[6];
        double tilt = x[7];
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double pan = x[8 + i];            
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = 0;
            C(1, 3) = 0;
            C(2, 3) = 0;
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const & x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //D_123, S_123, fl, tilt pan0, pan2...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double fl = x[6];
        double tilt = x[7];
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        
        // loop each camera
        for (int i = 0; i<corres_.size(); i++) {
            double pan = x[8 + i];
            
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = 0;
            C(1, 3) = 0;
            C(2, 3) = 0;
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            projections.push_back(P);
            
        }
    }
};

bool VpglPTZModelEstimation::pinHoleModelEstimateProjectionCenterWithFixedFocalLengthTilt(const vcl_vector<Correspondence> & corres,
                                                                                          const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                                          vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    pinHoleModelEstimateProjectionCenterWithFixedFocalLengthTiltResidual residual(corres, initCameras[0].get_calibration().principal_point(), pts_num);
    
    vnl_vector<double> x(8 + (int)initCameras.size(), 0.0);
    double fl_avg   = 0.0;
    double tilt_avg = 0.0;
    double dx = 0;
    double dy = 0;
    double dz = 0;
    const int N = (int)initCameras.size();
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        fl_avg += curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[8 + i] = pan;
        tilt_avg += tilt;
        
        dx += curCamera.get_camera_center().x();
        dy += curCamera.get_camera_center().y();
        dz += curCamera.get_camera_center().z();
    }
    
    x[0] = dx/N;
    x[1] = dy/N;
    x[2] = dz/N;
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[3] = RS.as_rodrigues()[0];
    x[4] = RS.as_rodrigues()[1];
    x[5] = RS.as_rodrigues()[2];
    x[6] = fl_avg /N;
    x[7] = tilt_avg/N;
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    residual.getProjections(x, estimatedCameras);
    
    vcl_cout<<"D S is: "<<vcl_endl;
    for (int i = 0; i<6; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    
    printf("fl, tilt is %f %f\n", x[6], x[7]);
    vcl_cout<<"pan is: "<<vcl_endl;
    for (int i = 8; i<x.size(); i++) {
        printf("%f ", x[i]);
    }
    printf("\n");
    return true;
}

class estimateProjecionCenterWithFocalLengthCzResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    estimateProjecionCenterWithFocalLengthCzResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp, int pts_num):
    vnl_least_squares_function(9 + 3 * (unsigned int)corres.size(), pts_num * 2 , no_gradient),
    corres_(corres),
    principlePoint_(pp)
    {
        assert(corres.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, C_123456, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[6];
        double c2 = x[7];
        double c3 = x[8];
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[9 + 3 * i + 0];
            double pan  = x[9 + 3 * i + 1];
            double tilt = x[9 + 3 * i + 2];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = 0;
            C(1, 3) = - (c1);
            C(2, 3) = - (c2 + c3 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //D_123, S_123, C_123456, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[6];
        double c2 = x[7];
        double c3 = x[8];
        
        // loop each camera
        projections.resize(corres_.size());
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[9 + 3 * i + 0];
            double pan  = x[9 + 3 * i + 1];
            double tilt = x[9 + 3 * i + 2];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = 0;
            C(1, 3) = - (c1);
            C(2, 3) = - (c2 + c3 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            projections[i] = camera;
        }
    }
};

bool VpglPTZModelEstimation::estimateProjecionCenterWithFocalLengthCz(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                      vnl_vector_fixed<double, 3> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjecionCenterWithFocalLengthCzResidual residual(corres, initCameras[0].get_calibration().principal_point(), pts_num);
    
    vnl_vector<double> x(9 + 3 * (int)initCameras.size(), 0.0);
    
    double dx = 0;
    double dy = 0;
    double dz = 0;
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        x[9 + 3 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[9 + 3 * i + 1] = pan;
        x[9 + 3 * i + 2] = tilt;
        
        dx += curCamera.get_camera_center().x();
        dy += curCamera.get_camera_center().y();
        dz += curCamera.get_camera_center().z();
    }
    
    x[0] = dx/initCameras.size();
    x[1] = dy/initCameras.size();
    x[2] = dz/initCameras.size();
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[3] = RS.as_rodrigues()[0];
    x[4] = RS.as_rodrigues()[1];
    x[5] = RS.as_rodrigues()[2];
    
    for (int i = 6; i<9; i++) {
        x[i] = 0.0;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    // out put result
    for (int i = 0; i<coeff.size(); i++) {
        coeff[i] = x[6 + i];
    }
    
    residual.getProjections(x, estimatedCameras);
    
    vcl_cout<<"D S is: "<<vcl_endl;
    for (int i = 0; i<6; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;

    return true;
}

class estimateProjectionCenterIndividuallyResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    
public:
    estimateProjectionCenterIndividuallyResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres,
                                                 const vgl_point_2d<double> & pp,
                                                 const vgl_point_3d<double> & cc,
                                                 int pts_num):
    vnl_least_squares_function(3 + 6 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    cameraCenter_(cc)
    {
        assert(corres.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        // RSxyz (fl, pan, tilt, C_xyz)...
        vnl_vector_fixed<double, 3> rod(x[0], x[1], x[2]);
        vgl_rotation_3d<double> Rs(rod);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[3 + 6 * i + 0];
            double pan  = x[3 + 6 * i + 1];
            double tilt = x[3 + 6 * i + 2];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - x[3 + 6 * i + 3];
            C(1, 3) = - x[3 + 6 * i + 4];
            C(2, 3) = - x[3 + 6 * i + 5];
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        vnl_vector_fixed<double, 3> rod(x[0], x[1], x[2]);
        vgl_rotation_3d<double> Rs(rod);
        
        // loop each camera
        projections.resize(corres_.size());
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[3 + 6 * i + 0];
            double pan  = x[3 + 6 * i + 1];
            double tilt = x[3 + 6 * i + 2];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - x[3 + 6 * i + 3];
            C(1, 3) = - x[3 + 6 * i + 4];
            C(2, 3) = - x[3 + 6 * i + 5];
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            projections[i] = camera;
        }
    }
};

bool VpglPTZModelEstimation::estimateProjectionCenterIndividually(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                  const vgl_point_3d<double> & cameraCenter, vcl_vector< vgl_point_3d<double> > & projectionCenters,
                                                                  vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
  //  vcl_vector<double> fls;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjectionCenterIndividuallyResidual residual(corres, initCameras[0].get_calibration().principal_point(), cameraCenter, pts_num);
    
    vnl_vector<double> x(3 + 6 * (int)initCameras.size(), 0.0);
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[0] = RS.as_rodrigues()[0];
    x[1] = RS.as_rodrigues()[1];
    x[2] = RS.as_rodrigues()[2];
    
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        x[3 +  6 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[3 + 6 * i + 1] = pan;
        x[3 + 6 * i + 2] = tilt;
        
        x[3 + 6 * i + 3] = 0;
        x[3 + 6 * i + 4] = 0;
        x[3 + 6 * i + 5] = 0;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    projectionCenters.resize(initCameras.size());
    for (int i = 0; i<initCameras.size(); i++) {
        vgl_point_3d<double> pc(x[3 + 6 * i + 3], x[3 + 6 * i + 4], x[3 + 6 * i + 5]);
        projectionCenters[i] = pc;
    }
    
    residual.getProjections(x, estimatedCameras);
    return true;
}

class estimateProjecionCenterWithFocalLengthFixFLResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vcl_vector<double> focalLengths_;
    
public:
    estimateProjecionCenterWithFocalLengthFixFLResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp,
                                                        const vgl_point_3d<double> &cc,
                                                        const vcl_vector<double> & fls, int pts_num):
    vnl_least_squares_function(9 + 2 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    cameraCenter_(cc),
    focalLengths_(fls)
    {
        assert(corres.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //S_123, C_123456, (pan, tilt)...
        vnl_vector_fixed<double, 3> rod(x[0], x[1], x[2]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[3];
        double c2 = x[4];
        double c3 = x[5];
        double c4 = x[6];
        double c5 = x[7];
        double c6 = x[8];
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl = focalLengths_[i];
            double pan  = x[9 + 2 * i + 0];
            double tilt = x[9 + 2 * i + 1];
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
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        vnl_vector_fixed<double, 3> rod(x[0], x[1], x[2]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[3];
        double c2 = x[4];
        double c3 = x[5];
        double c4 = x[6];
        double c5 = x[7];
        double c6 = x[8];
        
        // loop each camera
        projections.resize(focalLengths_.size());
        for (int i = 0; i<focalLengths_.size(); i++) {
            double fl = focalLengths_[i];
            double pan  = x[9 + 2 * i + 0];
            double tilt = x[9 + 2 * i + 1];
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
            projections[i] = camera;
        }
    }
    
};

bool VpglPTZModelEstimation::estimateProjecionCenterWithFocalLengthFixFL(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                         const vcl_vector<double> & focalLengths, const vgl_point_3d<double> & cameraCenter,
                                                                         vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjecionCenterWithFocalLengthFixFLResidual residual(corres, initCameras[0].get_calibration().principal_point(), cameraCenter, focalLengths, pts_num);
    
    vnl_vector<double> x(9 + 2 * (int)initCameras.size(), 0.0);
   
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[0] = RS.as_rodrigues()[0];
    x[1] = RS.as_rodrigues()[1];
    x[2] = RS.as_rodrigues()[2];
    
    for (int i = 0; i<coeff.size(); i++) {
        x[3 + i] = 0.0;
    }
    
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[9 + 2 * i + 0] = pan;
        x[9 + 2 * i + 1] = tilt;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    for (int i = 0; i<coeff.size(); i++) {
        coeff[i] = x[3 + i];
    }
    
    residual.getProjections(x, estimatedCameras);
    
    vcl_cout<<"S is: "<<vcl_endl;
    for (int i = 0; i<3; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    return true;
}

class estimateProjectionCenterWithCameracenterRotationFixResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> rod_;
    
public:
    estimateProjectionCenterWithCameracenterRotationFixResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp,
                                                                const vgl_point_3d<double> &cc,
                                                                const vnl_vector_fixed<double, 3> & rod, int pts_num):
    vnl_least_squares_function(6 + 3 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    cameraCenter_(cc),
    rod_(rod)
    {
        assert(corres.size() > 3);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //C_123456, (fl, pan, tilt)...
        double c1 = x[0];
        double c2 = x[1];
        double c3 = x[2];
        double c4 = x[3];
        double c5 = x[4];
        double c6 = x[5];
        vgl_rotation_3d<double> Rs(rod_);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
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
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }       
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //C_123456, (fl, pan, tilt)...
        double c1 = x[0];
        double c2 = x[1];
        double c3 = x[2];
        double c4 = x[3];
        double c5 = x[4];
        double c6 = x[5];
        vgl_rotation_3d<double> Rs(rod_);
        
        // loop each camera
        projections.clear();
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
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
            projections.push_back(camera);
        }
    }
    
};

bool VpglPTZModelEstimation::estimateProjectionCenterWithCameracenterRotationFix(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                                 const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                                 vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjectionCenterWithCameracenterRotationFixResidual residual(corres, initCameras[0].get_calibration().principal_point(), cameraCenter, rod, pts_num);
    
    vnl_vector<double> x(6 + 3 * (int)initCameras.size(), 0.0);
    
    for (int i = 0; i<6; i++) {
        x[i] = 0;
    }
    
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        double fl = initCameras[i].get_calibration().focal_length();
        x[6 + 3 * i + 0] = fl;
        x[6 + 3 * i + 1] = pan;
        x[6 + 3 * i + 2] = tilt;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    residual.getProjections(x, estimatedCameras);
    
    for (int i = 0; i<6; i++) {
        coeff[i] = x[i];
    }
    vcl_cout<<"projection center coefficient is "<<coeff<<vcl_endl;
    
    return true;
}

class estimateProjectionCenterWithCameracenterRotationFixRestrictedModelResidual :public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> rod_;
    
public:
    estimateProjectionCenterWithCameracenterRotationFixRestrictedModelResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp,
                                                                               const vgl_point_3d<double> &cc,
                                                                               const vnl_vector_fixed<double, 3> & rod, int pts_num):
    vnl_least_squares_function(4 + 3 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    cameraCenter_(cc),
    rod_(rod)
    {
        assert(corres.size() > 3);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //C_1234, (fl, pan, tilt)...
        double c1 = x[0];
        double c2 = x[1];
        double c3 = x[2];
        double c4 = x[3];
        vgl_rotation_3d<double> Rs(rod_);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[4 + 3 * i + 0];
            double pan  = x[4 + 3 * i + 1];
            double tilt = x[4 + 3 * i + 2];
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
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //C_1234, (fl, pan, tilt)...
        double c1 = x[0];
        double c2 = x[1];
        double c3 = x[2];
        double c4 = x[3];
        vgl_rotation_3d<double> Rs(rod_);
        
        // loop each camera
        projections.clear();
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[4 + 3 * i + 0];
            double pan  = x[4 + 3 * i + 1];
            double tilt = x[4 + 3 * i + 2];
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
            projections.push_back(camera);
        }
    }
};

bool VpglPTZModelEstimation::estimateProjectionCenterWithCameracenterRotationFixRestrictedModel(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                                                const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                                                vnl_vector_fixed<double, 4> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjectionCenterWithCameracenterRotationFixRestrictedModelResidual residual(corres, initCameras[0].get_calibration().principal_point(), cameraCenter, rod, pts_num);
    
    vnl_vector<double> x(4 + 3 * (int)initCameras.size(), 0.0);
    
    for (int i = 0; i<4; i++) {
        x[i] = 0;
    }
    
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        double fl = initCameras[i].get_calibration().focal_length();
        x[4 + 3 * i + 0] = fl;
        x[4 + 3 * i + 1] = pan;
        x[4 + 3 * i + 2] = tilt;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    residual.getProjections(x, estimatedCameras);
    
    for (int i = 0; i<4; i++) {
        coeff[i] = x[i];
    }
    vcl_cout<<"projection center coefficient is "<<coeff<<vcl_endl;
    
    return true;
}

class estimateProjectionCenterWithCameracenterRotationFocalLengthFixResidual :public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> rod_;
    const vcl_vector<double> focalLengths_;
    
public:
    estimateProjectionCenterWithCameracenterRotationFocalLengthFixResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp,
                                                                           const vgl_point_3d<double> &cc,
                                                                           const vnl_vector_fixed<double, 3> & rod,
                                                                           const vcl_vector<double> & focalLengths,
                                                                           int pts_num):
    vnl_least_squares_function(6 + 2 * (unsigned int)corres.size(), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    cameraCenter_(cc),
    rod_(rod),
    focalLengths_(focalLengths)
    {
        assert(corres.size() > 3);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //C_123456, (pan, tilt)...
        double c1 = x[0];
        double c2 = x[1];
        double c3 = x[2];
        double c4 = x[3];
        double c5 = x[4];
        double c6 = x[5];
        vgl_rotation_3d<double> Rs(rod_);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = focalLengths_[i];
            double pan  = x[6 + 2 * i + 0];
            double tilt = x[6 + 2 * i + 1];
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
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //C_123456, (pan, tilt)...
        double c1 = x[0];
        double c2 = x[1];
        double c3 = x[2];
        double c4 = x[3];
        double c5 = x[4];
        double c6 = x[5];
        vgl_rotation_3d<double> Rs(rod_);
        
        // loop each camera
        projections.clear();
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = focalLengths_[i];
            double pan  = x[6 + 2 * i + 0];
            double tilt = x[6 + 2 * i + 1];
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
            projections.push_back(camera);
        }
    }
};



bool VpglPTZModelEstimation::estimateProjectionCenterWithCameracenterRotationFocalLengthFix(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                                            const vcl_vector<double> & focalLengths,
                                                                                            const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                                            vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjectionCenterWithCameracenterRotationFocalLengthFixResidual residual(corres, initCameras[0].get_calibration().principal_point(), cameraCenter, rod, focalLengths, pts_num);
    
    vnl_vector<double> x(6 + 2 * (int)initCameras.size(), 0.0);
    
    for (int i = 0; i<6; i++) {
        x[i] = 0;
    }
    
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[6 + 2 * i + 0] = pan;
        x[6 + 2 * i + 1] = tilt;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    residual.getProjections(x, estimatedCameras);
    
    for (int i = 0; i<6; i++) {
        coeff[i] = x[i];
    }
    vcl_cout<<"projection center coefficient is "<<coeff<<vcl_endl;
    
    return true;
}

class estimateProjectionCenterFixAllOthersResidual: public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> rod_;
    const vcl_vector<vnl_vector_fixed<double, 3> > fl_pan_tilts_;
    
public:
    estimateProjectionCenterFixAllOthersResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> &pp,
                                                                           const vgl_point_3d<double> &cc,
                                                                           const vnl_vector_fixed<double, 3> & rod,
                                                                           const vcl_vector<vnl_vector_fixed<double, 3> > & fl_pan_tilts,
                                                                           int pts_num):
    vnl_least_squares_function(6, pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    cameraCenter_(cc),
    rod_(rod),
    fl_pan_tilts_(fl_pan_tilts)
    {
        assert(corres.size() > 3);
        assert(fl_pan_tilts.size() == corres.size());
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //C_123456
        double c1 = x[0];
        double c2 = x[1];
        double c3 = x[2];
        double c4 = x[3];
        double c5 = x[4];
        double c6 = x[5];
        vgl_rotation_3d<double> Rs(rod_);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = fl_pan_tilts_[i][0];
            double pan  = fl_pan_tilts_[i][1];
            double tilt = fl_pan_tilts_[i][2];
            vpgl_calibration_matrix<double> K(fl, principlePoint_);
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;           //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter_,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //C_123456
        double c1 = x[0];
        double c2 = x[1];
        double c3 = x[2];
        double c4 = x[3];
        double c5 = x[4];
        double c6 = x[5];
        vgl_rotation_3d<double> Rs(rod_);
        
        // loop each camera
        projections.clear();
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = fl_pan_tilts_[i][0];
            double pan  = fl_pan_tilts_[i][1];
            double tilt = fl_pan_tilts_[i][2];
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
            projections.push_back(camera);
        }
    }
};

bool VpglPTZModelEstimation::estimateProjectionCenterFixAllOthers(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                  const vcl_vector<vnl_vector_fixed<double, 3> >  & fl_pan_tilts,
                                                                  const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                  vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateProjectionCenterFixAllOthersResidual residual(corres, initCameras[0].get_calibration().principal_point(), cameraCenter, rod, fl_pan_tilts, pts_num);
    
    vnl_vector<double> x(6, 0.0);
    
    for (int i = 0; i<6; i++) {
        x[i] = 0;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    residual.getProjections(x, estimatedCameras);
    
    for (int i = 0; i<6; i++) {
        coeff[i] = x[i];
    }
    vcl_cout<<"projection center coefficient is "<<coeff<<vcl_endl;    
    return true;
}


class estimateRotationCenterFixFLResidual: public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vcl_vector<double> focalLengths_;
    
public:
    estimateRotationCenterFixFLResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> & pp,
                                        const vcl_vector<double> & fls, int pts_num):
    vnl_least_squares_function(6 * (unsigned int)(corres.size()), pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    focalLengths_(fls)
    {
        assert(corres.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //loop each camera
        //Dxyz Rxyz
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_perspective_camera<double> camera;
            vpgl_calibration_matrix<double> K(focalLengths_[i], principlePoint_);
            vgl_point_3d<double>         cc(x[6 * i + 0], x[6 * i + 1], x[6 * i + 2]);
            vnl_vector_fixed<double, 3> rod(x[6 * i + 3], x[6 * i + 4], x[6 * i + 5]);
            camera.set_calibration(K);
            camera.set_rotation(vgl_rotation_3d<double>(rod));
            camera.set_camera_center(cc);
            
            //loop each correspondence
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getCameras(vnl_vector<double> const &x, vcl_vector<vpgl_perspective_camera<double> > & cameras)
    {
        cameras.resize(focalLengths_.size());
        for (int i = 0; i<focalLengths_.size(); i++) {
            vpgl_perspective_camera<double> camera;
            vpgl_calibration_matrix<double> K(focalLengths_[i], principlePoint_);
            vgl_point_3d<double> cc(x[6 * i + 0], x[6 * i + 1], x[6 * i + 2]);
            vnl_vector_fixed<double, 3> rod(x[6 * i + 3], x[6 * i + 4], x[6 * i + 5]);
            camera.set_calibration(K);
            camera.set_rotation(vgl_rotation_3d<double>(rod));
            camera.set_camera_center(cc);
            cameras[i] = camera;
        }
    }
    
};




bool VpglPTZModelEstimation::estimateRotationCenterFixFL(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                         const vcl_vector<double> & focalLengths,  vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras)
{
    assert(corres.size() >= 5);
    assert(corres.size() == initCameras.size());
    assert(corres.size() == focalLengths.size());
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }
    
    estimateRotationCenterFixFLResidual residual(corres, initCameras[0].get_calibration().principal_point(), focalLengths, pts_num);
    
    vnl_vector<double> x(6 * (unsigned int)initCameras.size());
    for (int i = 0; i<initCameras.size(); i++) {
        x[6 * i + 0] = initCameras[i].camera_center().x();
        x[6 * i + 1] = initCameras[i].camera_center().y();
        x[6 * i + 2] = initCameras[i].camera_center().z();
        x[6 * i + 3] = initCameras[i].get_rotation().as_rodrigues()[0];
        x[6 * i + 4] = initCameras[i].get_rotation().as_rodrigues()[1];
        x[6 * i + 5] = initCameras[i].get_rotation().as_rodrigues()[2];
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    residual.getCameras(x, estimatedCameras);
    return true;
}
/*
class estimateProjectionCenterByMaximumPropertyResidual: public vnl_least_squares_function
{
protected:
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> stds_;    // standard deviation
    const int pts_num_;
    
public:
    estimateProjectionCenterByMaximumPropertyResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> &corres,  const vgl_point_2d<double> & pp,
                                                      const vgl_point_3d<double> & cc, const vnl_vector_fixed<double, 3> & stds, int pts_num):
    vnl_least_squares_function(9 + 3 * (unsigned int)corres.size(), 2 * pts_num + (unsigned int)corres.size() - 1, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    cameraCenter_(cc),
    stds_(stds),
    pts_num_(pts_num)
    {
        assert(corres.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        vnl_vector_fixed<double, 3> rod(x[0], x[1], x[2]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[3];
        double c2 = x[4];
        double c3 = x[5];
        double c4 = x[6];
        double c5 = x[7];
        double c6 = x[8];
        
        vnl_single_guassian x_gaussian(0, stds_[0]);
        vnl_single_guassian y_gaussian(0, stds_[1]);
        vnl_single_guassian delta_fl_gaussian(0, stds_[2]);
        // loop each frame
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[9 + 3 * i + 0];
            double pan  = x[9 + 3 * i + 1];
            double tilt = x[9 + 3 * i + 2];
            
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
            
            //loop each correspondence
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                double dx = corres_[i].img_pts[j].x() - q.x();
                double dy = corres_[i].img_pts[j].y() - q.y();
                
                fx[idx] = x_gaussian.log_prob_density(dx);
                idx++;
                fx[idx] = y_gaussian.log_prob_density(dy);
                idx++;
            }
            
            //assume focal length is smooth
            if (i > 0) {
                double pre_fl = x[9 + 3 * (i - 1) + 0];
                fx[2 * pts_num_ + (i - 1)] = delta_fl_gaussian.log_prob_density(fl - pre_fl);
            }
        }
    }
    
    void getProjections(vnl_vector<double> const &x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        vnl_vector_fixed<double, 3> rod(x[0], x[1], x[2]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[3];
        double c2 = x[4];
        double c3 = x[5];
        double c4 = x[6];
        double c5 = x[7];
        double c6 = x[8];
      
        // loop each frame
        projections.resize(corres_.size());
        for (int i = 0; i<corres_.size(); i++) {
            double fl   = x[9 + 3 * i + 0];
            double pan  = x[9 + 3 * i + 1];
            double tilt = x[9 + 3 * i + 2];
            
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
            projections[i] = camera;
        }
    }
    
    void getFlPanTilts(vnl_vector<double> const &x, vnl_matrix<double> & flPanTilt)
    {
        flPanTilt = vnl_matrix<double>((unsigned int)corres_.size(), 3, 0);
        for (int i = 0; i<corres_.size(); i++) {
            flPanTilt(i, 0) = x[9 + 3 * i + 0];
            flPanTilt(i, 1) = x[9 + 3 * i + 1];
            flPanTilt(i, 2) = x[9 + 3 * i + 2];
        }
    }
    
};

bool VpglPTZModelEstimation::estimateProjectionCenterByMaximumProbability(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                          const vnl_vector_fixed<double, 3> & std_dev_s, const vgl_point_3d<double> & cameraCenter,
                                                                          vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras,
                                                                          const vcl_string & saveMatlabFile)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 1);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].img_pts.size();
    }
    
    estimateProjectionCenterByMaximumPropertyResidual residual(corres, initCameras[0].get_calibration().principal_point(), cameraCenter, std_dev_s, pts_num);
    
    vnl_vector<double> x(9 + 3 * (int)initCameras.size(), 0.0);
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] = 1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] = 1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[0] = RS.as_rodrigues()[0];
    x[1] = RS.as_rodrigues()[1];
    x[2] = RS.as_rodrigues()[2];
    
    x[3] = 0.0;
    x[4] = 0.0;
    x[5] = 0.0;
    x[6] = 0.0;
    x[7] = 0.0;
    x[8] = 0.0;
    
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        x[9 + 3 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        double wx = curCamera.get_rotation().as_matrix()[2][0];
        double wy = curCamera.get_rotation().as_matrix()[2][1];
        double wz = curCamera.get_rotation().as_matrix()[2][2];
        double pan  = atan2(wx, wy) * 180.0 /vnl_math::pi;
        double tilt = atan2(wz, wy) * 180.0 /vnl_math::pi;
        x[9 + 3 * i + 1] = pan;
        x[9 + 3 * i + 2] = tilt;
    }
    
    vnl_levenberg_marquardt lmq(residual);
    
    lmq.set_f_tolerance(1.0e-5);
    
    bool isMinimized = lmq.minimize(x);
    if (!isMinimized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    
    lmq.diagnose_outcome();
    
    // output result
    for (int i = 0; i<6; i++) {
        coeff[i] = x[3 + i];
    }
    
    residual.getProjections(x, estimatedCameras);
    
    //wirte to matlab
    vnl_matlab_filewrite writer(saveMatlabFile.c_str());
    
    vnl_matrix<double> flPanTilts;
    residual.getFlPanTilts(x, flPanTilts);
    writer.write(flPanTilts, "FlPanTilt");
    
    vcl_cout<<"save to : "<<saveMatlabFile<<vcl_endl;
    return true;
}
 */

class estimatePanTiltByFixsingModelPositionRotationResidual: public vnl_least_squares_function
{
protected:
    const VpglPTZModelEstimation::Correspondence corre_;
    const vgl_point_2d<double> principlePoint_;
    const vgl_point_3d<double> cameraCenter_;
    const vnl_vector_fixed<double, 3> rod_; //rodrigue angle of model rotation
    const vnl_vector_fixed<double, 6> coeff_;
    
public:
    estimatePanTiltByFixsingModelPositionRotationResidual(const VpglPTZModelEstimation::Correspondence & corre, const vgl_point_2d<double> & pp,
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

bool VpglPTZModelEstimation::estimateFlPanTiltByFixingModelPositionRotation (const Correspondence & corre, const vpgl_perspective_camera<double> & initCamera,
                                                                             const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                             const vnl_vector_fixed<double, 6> & coeff, vnl_vector_fixed<double, 3> & flPanTilt,
                                                                             vpgl_proj_camera<double> & estimatedCamera)
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
    
    flPanTilt[0] = x[0];
    flPanTilt[1] = x[1];
    flPanTilt[2] = x[2];
    
    residual.getProjection(x, estimatedCamera);
    return true;
}
/*
bool VpglPTZModelEstimation::pickupPoorEstimationInImageSequence(const vcl_vector< vpgl_perspective_camera<double> > & initCameras,
                                                                 const vgl_point_3d<double> & cameraCenter,
                                                                 const vnl_vector_fixed<double, 3> & rod,
                                                                 const vnl_vector_fixed<double, 6> & coeff,
                                                                 const vnl_vector_fixed<double, 3> & thresholds,
                                                                 vcl_vector<int> & poorEstimationIndex,
                                                                 vcl_vector<vpgl_perspective_camera<double>> & interpolatedCameras)
{
    assert(initCameras.size() >= 100 && initCameras.size() <= 250);
    
    int destWidth  = initCameras[0].get_calibration().principal_point().x() * 2;
    int destHeight = initCameras[0].get_calibration().principal_point().y() * 2;
    
    vgl_point_2d<double> pp(initCameras[0].get_calibration().principal_point());
    
    // get focal length, pan, tilt angles from individual frame
    vcl_vector<double> fls(initCameras.size(), 0);
    vcl_vector<double> pans(initCameras.size(), 0);
    vcl_vector<double> tilts(initCameras.size(), 0);
    for (int i = 0; i<initCameras.size(); i++) {
        VpglPTZModelEstimation::Correspondence corres;
        
        DisneyWorldBasketballCourt::projectCourtPoints(initCameras[i], destWidth, destHeight, corres.wld_pts, corres.img_pts);
        assert(corres.wld_pts.size() >= 4);
        
        vnl_vector_fixed<double, 3> flPanTilt;
        vpgl_proj_camera<double> estimatedCamera;
        
        bool isEstimated = VpglPTZModelEstimation::estimateFlPanTiltByFixingModelPositionRotation(corres, initCameras[i], cameraCenter, rod, coeff, flPanTilt, estimatedCamera);
        if (!isEstimated) {
            vcl_cerr<<"Error: estimation error, set default values.\n";
            if (i > 0) {
                fls[i]   = fls[i-1];
                pans[i]  = pans[i-1];
                tilts[i] = tilts[i-1];
            }
            else
            {
                fls[i]   = 2000;
                pans[i]  = 0;
                tilts[i] = 11;
            }
            poorEstimationIndex.push_back(i);
            continue;
        }
        fls[i]   = flPanTilt[0];
        pans[i]  = flPanTilt[1];
        tilts[i] = flPanTilt[2];
    }
    
    //fit curve from focal length
    vnl_polynomial<double> fl_poly;
    int degree = 3;
    bool isFitted = VnlPlus::vnl_fit_polynomial(fls, fl_poly, degree);
    if (!isFitted) {
        vcl_cerr<<"Error: focal length poly fitting failed.\n";
    }
    else
    {
        
        //check threshold
        for (int i = 0; i<fls.size(); i++) {
            double val = fl_poly.evaluate(i);
            double delta = fabs(val - fls[i]);
            if (delta > thresholds[0]) {
                poorEstimationIndex.push_back(i);
            }
        }
    }
    
    // curve fitting for pan angles
    vnl_polynomial<double> pan_poly;
    isFitted = VnlPlus::vnl_fit_polynomial(pans, pan_poly, degree - 1);
    if (!isFitted) {
        vcl_cerr<<"Error: pan angle poly fitting failed.\n";
    }
    else
    {
        for (int i =0; i<pans.size(); i++) {
            double val = pan_poly.evaluate(i);
            double delta = fabs(val - pans[i]);
            if (delta > thresholds[1]) {
                poorEstimationIndex.push_back(i);
            }
            
        }
    }
    
    // curve fitting for tilt angles
    vnl_polynomial<double> tilt_poly;
    isFitted = VnlPlus::vnl_fit_polynomial(tilts, tilt_poly, degree - 1);
    if (!isFitted) {
        vcl_cerr<<"Error: tilt angle poly fitting failed.\n";
    }
    else
    {
        for (int i = 0; i<tilts.size(); i++) {
            double val = tilt_poly.evaluate(i);
            double delta = fabs(val - tilts[i]);
            if (delta > thresholds[2]) {
                poorEstimationIndex.push_back(i);
            }     
        }
    }
    
    vcl_sort(poorEstimationIndex.begin(), poorEstimationIndex.end());
    vcl_vector<int>::iterator ite = vcl_unique(poorEstimationIndex.begin(), poorEstimationIndex.end());
    poorEstimationIndex.resize(ite - poorEstimationIndex.begin());
    
    // interpolate camera for poor estimated camera
    interpolatedCameras.resize(poorEstimationIndex.size());
    for (int i = 0; i<poorEstimationIndex.size(); i++) {
        int idx = poorEstimationIndex[i];
        double fl   = fl_poly.evaluate(idx);
        double pan  = pan_poly.evaluate(idx);
        double tilt = tilt_poly.evaluate(idx);
        bool isDecomposed = VpglPlus::decomposeCameraPTZ(fl, coeff, pp, pan, tilt, rod, cameraCenter, interpolatedCameras[i]);
        if (!isDecomposed) {
            vcl_cerr<<"Warning: decompose camera failed.\n";
        }
    }    
    return true;
}
 */


class estimateRotationCenterStationaryRotationByPanTiltResidual:public vnl_least_squares_function
{
    const vcl_vector<VpglPTZModelEstimation::Correspondence> corres_;
    const vgl_point_2d<double> principlePoint_;
    const vcl_vector<double> pans_;
    const vcl_vector<double> tilts_;
    
public:
    estimateRotationCenterStationaryRotationByPanTiltResidual(const vcl_vector<VpglPTZModelEstimation::Correspondence> & corres, const vgl_point_2d<double> & pp,
                                                              int pts_num, const vcl_vector<double> & pans, const vcl_vector<double> & tilts):
    vnl_least_squares_function(13, pts_num * 2, no_gradient),
    corres_(corres),
    principlePoint_(pp),
    pans_(pans),
    tilts_(tilts)
    {
        assert(corres_.size() >= 5);
        assert(corres_.size() == pans_.size());
        assert(pans_.size() == tilts_.size());
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, C_123456, fl
        vgl_point_3d<double>        cc( x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[6];
        double c2 = x[7];
        double c3 = x[8];
        double c4 = x[9];
        double c5 = x[10];
        double c6 = x[11];
        double fl = x[12];
        
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        // loop each camera
        int idx = 0;
        for (int i = 0; i<corres_.size(); i++) {
            double pan  = pans_[i];
            double tilt = tilts_[i];
            
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts.size(); j++) {
                vgl_point_2d<double> p = corres_[i].wld_pts[j];
                vgl_point_2d<double> q = (vgl_point_2d<double>)(camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0)));
                
                fx[idx] = corres_[i].img_pts[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts[j].y() - q.y();
                idx++;
            }
        }
    }
    
    
    void getProjections(vnl_vector<double> const & x, vcl_vector<vpgl_proj_camera<double> > & projections)
    {
        //D_123, S_123, C_123456, fl
        vgl_point_3d<double>        cc( x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double> Rs(rod);
        double c1 = x[6];
        double c2 = x[7];
        double c3 = x[8];
        double c4 = x[9];
        double c5 = x[10];
        double c6 = x[11];
        double fl = x[12];
        
        vpgl_calibration_matrix<double> K(fl, principlePoint_);
        // loop each camera
        for (int i = 0; i<corres_.size(); i++) {
            double pan  = pans_[i];
            double tilt = tilts_[i];
            
            vnl_matrix_fixed<double, 3, 4> C;
            C.set_identity();
            C(0, 3) = - (c1 + c4 * fl);
            C(1, 3) = - (c2 + c5 * fl);
            C(2, 3) = - (c3 + c6 * fl);
            
            vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix_fixed<double, 4, 4> RSD;
            VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cc,  RSD);
            
            vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
            vpgl_proj_camera<double> camera(P);
            
            projections.push_back(P);
        }
    }
    
    
};




bool VpglPTZModelEstimation::estimateRotationCenterStationaryRotationByPanTilt(const vcl_vector<Correspondence> & corres,
                                                                               const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                               const vcl_vector<double> & pans, const vcl_vector<double> & tilts,
                                                                               vcl_vector<vpgl_proj_camera<double> > & estimatedCameras)
{
    assert(corres.size() == initCameras.size());
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts.size();
    }    
    
    estimateRotationCenterStationaryRotationByPanTiltResidual residual(corres, initCameras[0].get_calibration().principal_point(), pts_num, pans, tilts);
    
    vnl_vector<double> x(13);
    double fl_avg   = 0.0;
    double dx = 0;
    double dy = 0;
    double dz = 0;
    const int N = (int)initCameras.size();
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        fl_avg += curCamera.get_calibration().get_matrix()[0][0];
        
        dx += curCamera.get_camera_center().x();
        dy += curCamera.get_camera_center().y();
        dz += curCamera.get_camera_center().z();
    }
    
    x[0] = dx/N;
    x[1] = dy/N;
    x[2] = dz/N;
    
    vnl_matrix_fixed<double, 3, 3> RSinit;
    RSinit.fill(0);
    RSinit[0][0] =  1.0;
    RSinit[1][2] = -1.0;
    RSinit[2][1] =  1.0;
    
    vgl_rotation_3d<double> RS(RSinit);
    x[3] = RS.as_rodrigues()[0];
    x[4] = RS.as_rodrigues()[1];
    x[5] = RS.as_rodrigues()[2];
    
    for (int i = 6; i<12; i++) {
        x[i] = 0.0;
    }
    x[12] = fl_avg /N;
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    residual.getProjections(x, estimatedCameras);
    
    vcl_cout<<"D S is: "<<vcl_endl;
    for (int i = 0; i<6; i++) {
        vcl_cout<<x[i]<<" ";
    }
    vcl_cout<<vcl_endl;
    
    printf("coefficient is \n");
    for (int i = 6; i<12; i++) {
        printf("%f ", x[i]);
    }
    printf("\n");
    
    printf("%f \n", x[13]);
    
    return true;
}





















