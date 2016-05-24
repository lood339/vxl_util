//
//  vpgl_ptz_estimation.cpp
//  QuadCopter
//
//  Created by jimmy on 6/28/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vpgl_ptz_estimation.h"
#include <vnl/vnl_least_squares_function.h>
#include "vpgl_plus.h"
#include <vnl/vnl_inverse.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vgl/vgl_distance.h>
#include <vgl/algo/vgl_homg_operators_2d.h>

class estimateCommomCameraCenterAndStationaryRotationResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<vcl_vector<vgl_point_3d<double> > > wld_pts_;
    const vcl_vector<vcl_vector<vgl_point_2d<double> > > img_pts_;
    const vgl_point_2d<double> pp_;
    
public:
    estimateCommomCameraCenterAndStationaryRotationResidual(const vcl_vector<vcl_vector<vgl_point_3d<double> > > & wld_pts,
                                                            vcl_vector<vcl_vector<vgl_point_2d<double> > > img_pts,
                                                            const vgl_point_2d<double> &pp, int pts_num):
    vnl_least_squares_function(6 + 3 * (unsigned int)wld_pts.size(), pts_num * 2 , no_gradient),
    wld_pts_(wld_pts),
    img_pts_(img_pts),
    pp_(pp)
    {
        assert(wld_pts.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double>     Rs(rod);
        
        // loop each camera
        int idx = 0;
        for (int i = 0; i<wld_pts_.size(); i++) {
            vpgl_perspective_camera<double> curCamera;
            
            double fl   = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
            
            vpgl_calibration_matrix<double> K(fl, pp_);
            vnl_matrix_fixed<double, 3, 3> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix<double> QS = Q.as_matrix() * Rs.as_matrix().as_matrix();
            vnl_matrix_fixed<double, 3, 3> QS33(QS);
            
            curCamera.set_calibration(K);
            curCamera.set_camera_center(cc);
            curCamera.set_rotation(vgl_rotation_3d<double>(QS33));
            
            //loop each points
            for (int j = 0; j<wld_pts_[i].size(); j++) {
                vgl_point_2d<double> q =curCamera.project(wld_pts_[i][j]);
                fx[idx] = img_pts_[i][j].x() - q.x();
                idx++;
                fx[idx] = img_pts_[i][j].y() - q.y();
                idx++;
            }
        }
    }
    
    void getCameras(vnl_vector<double> const & x, vcl_vector<vpgl_perspective_camera<double> > & cameras)
    {
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double>     Rs(rod);
        
        cameras.clear();
        // loop each camera
        for (int i = 0; i<wld_pts_.size(); i++) {
            vpgl_perspective_camera<double> curCamera;
            double fl   = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
            vpgl_calibration_matrix<double> K(fl, pp_);
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


bool VpglPTZEstimation::estimateCommomCameraCenterAndStationaryRotation(const vcl_vector< vcl_vector<vgl_point_3d<double> > > & wld_pts,
                                                     const vcl_vector< vcl_vector<vgl_point_2d<double> > > & img_pts,
                                                     const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                     const vnl_matrix_fixed<double, 3, 3> & initSR,
                                                     vgl_rotation_3d<double> & estimatedRs,
                                                     vgl_point_3d<double> & estimatedCameraCenter,
                                                     vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras)
{
    assert(wld_pts.size() == img_pts.size());
    assert(wld_pts.size() == initCameras.size());
    assert(initCameras.size() >= 5);
    
    int pts_num = 0;
    for (int i = 0; i<wld_pts.size(); i++) {
        if(wld_pts[i].size() < 3)
        {
            printf("Error: at least 3 correspondences for a camera to robust estimate PTZ.\n");
            return false;
        }
        assert(wld_pts[i].size() == img_pts[i].size());
        pts_num += wld_pts[i].size();
    }
    
    estimateCommomCameraCenterAndStationaryRotationResidual residual(wld_pts, img_pts,
                                                                     initCameras[0].get_calibration().principal_point(), pts_num);
    // set initial values for D_123, S_123, (fl, pan, tilt)...
    vnl_vector<double> x(6 + 3 * (int)initCameras.size(), 0.0);
    
    double cx = 0;
    double cy = 0;
    double cz = 0;
    vnl_matrix_fixed<double, 3, 3> Rs_inv = vnl_inverse(initSR.as_matrix());
    for (int i = 0; i<initCameras.size(); i++) {
        vpgl_perspective_camera<double> curCamera = initCameras[i];
        x[6 + 3 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        // R_pan_tilt * Rs = R--> R_pt = R * inv(Rs)
        vnl_matrix_fixed<double, 3, 3> R_pan_tilt = curCamera.get_rotation().as_matrix() * Rs_inv;
        double cos_pan = R_pan_tilt(0, 0);
        double sin_pan = -R_pan_tilt(0, 2);
        double cos_tilt = R_pan_tilt(1, 1);
        double sin_tilt = -R_pan_tilt(2, 1);
        double pan  = atan2(sin_pan, cos_pan) * 180.0 /vnl_math::pi;
        double tilt = atan2(sin_tilt, cos_tilt) * 180.0 /vnl_math::pi;
        x[6 + 3 * i + 1] = pan;
        x[6 + 3 * i + 2] = tilt;
        
        cx += curCamera.get_camera_center().x();
        cy += curCamera.get_camera_center().y();
        cz += curCamera.get_camera_center().z();
    }
    
    x[0] = cx/initCameras.size();
    x[1] = cy/initCameras.size();
    x[2] = cz/initCameras.size();
    
    vgl_rotation_3d<double> RS(initSR);
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
    
    printf("CC, Rs is: ");
    for (int i = 0; i<6; i++) {
        printf("%f ", x[i]);
    }
    printf("\n");    
    
    estimatedCameraCenter = vgl_point_3d<double>(x[0], x[1], x[2]);    
    vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
    estimatedRs = vgl_rotation_3d<double>(rod);
    
    residual.getCameras(x, estimatedCameras);
    return true;
}


class estimateCommomCameraCenterAndStationaryRotationByPointsOnLinesnResidual:public vnl_least_squares_function
{
protected:
    const vcl_vector<LinePointsInCameraview> & corres_;
    const vgl_point_2d<double> pp_;
    
public:
    estimateCommomCameraCenterAndStationaryRotationByPointsOnLinesnResidual(const vcl_vector<LinePointsInCameraview> & corres,
                                                                            const vgl_point_2d<double> &pp, int pts_num, int line_pts_num):
    vnl_least_squares_function(6 + 3 * (unsigned int)corres.size(), pts_num * 2 + line_pts_num , no_gradient),
    corres_(corres),
    pp_(pp)
    {
        assert(pts_num * 2 + line_pts_num >= 6 + 3 * (unsigned int)corres.size());
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
            
            double fl   = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
            
            vpgl_calibration_matrix<double> K(fl, pp_);
            vnl_matrix_fixed<double, 3, 3> Q;  //rotation from pan tilt angle
            VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
            
            vnl_matrix<double> QS = Q.as_matrix() * Rs.as_matrix().as_matrix();
            vnl_matrix_fixed<double, 3, 3> QS33(QS);
            
            curCamera.set_calibration(K);
            curCamera.set_camera_center(cc);
            curCamera.set_rotation(vgl_rotation_3d<double>(QS33));
            
            //loop each points
            for (int j = 0; j<corres_[i].wld_pts_.size(); j++) {
                vgl_point_2d<double> q =curCamera.project(corres_[i].wld_pts_[j]);
                fx[idx] = corres_[i].img_pts_[j].x() - q.x();
                idx++;
                fx[idx] = corres_[i].img_pts_[j].y() - q.y();
                idx++;
            }
            
            //loop each lines in the camera
            vcl_vector<PointsOnLine> pols = corres_[i].lines_;
            for (int j = 0; j<pols.size(); j++) {
                vgl_point_2d<double> p1 = curCamera.project(pols[j].line_.point1());
                vgl_point_2d<double> p2 = curCamera.project(pols[j].line_.point2());
                vgl_line_2d<double> line(p1, p2);
                 // for points locate on the line
                for (int k = 0; k<pols[j].pts_.size(); k++) {
                    vgl_point_2d<double> p3 = pols[j].pts_[k];
                    fx[idx] = vgl_distance(line, p3);
                    idx++;
                }
            }
        }        
    }
    
    void getCameras(vnl_vector<double> const & x, vcl_vector<vpgl_perspective_camera<double> > & cameras)
    {        
        //D_123, S_123, (fl, pan, tilt)...
        vgl_point_3d<double>        cc(x[0], x[1], x[2]);
        vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
        vgl_rotation_3d<double>     Rs(rod);
        
        cameras.clear();
        // loop each camera
        for (int i = 0; i<corres_.size(); i++) {
            vpgl_perspective_camera<double> curCamera;
            double fl   = x[6 + 3 * i + 0];
            double pan  = x[6 + 3 * i + 1];
            double tilt = x[6 + 3 * i + 2];
            vpgl_calibration_matrix<double> K(fl, pp_);
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


bool VpglPTZEstimation::estimateCommomCameraCenterAndStationaryRotationByPointsOnLines(const vcl_vector<LinePointsInCameraview> & corres,
                                                     const vnl_matrix_fixed<double, 3, 3> & initSR,
                                                     vgl_rotation_3d<double> & estimatedRs,
                                                     vgl_point_3d<double> & estimatedCameraCenter,
                                                     vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras)
{
    assert(corres.size() >= 5);
    
    int pts_num = 0;
    int on_line_pts_num = 0;
    for (int i = 0; i<corres.size(); i++) {
        pts_num += corres[i].wld_pts_.size();
        assert(corres[i].wld_pts_.size() == corres[i].img_pts_.size());
        for (int j = 0; j<corres[i].lines_.size(); j++) {
            on_line_pts_num += corres[i].lines_[j].pts_.size();
        }
    }
   
    vgl_point_2d<double> pp = corres[0].camera_.get_calibration().principal_point();
    estimateCommomCameraCenterAndStationaryRotationByPointsOnLinesnResidual residual(corres, pp, pts_num, on_line_pts_num);
    
    // set initial values for D_123, S_123, (fl, pan, tilt)...
    const int camera_num = (int)corres.size();
    vnl_vector<double> x(6 + 3 * camera_num, 0.0);
    
    double cx = 0;
    double cy = 0;
    double cz = 0;
    vnl_matrix_fixed<double, 3, 3> Rs_inv = vnl_inverse(initSR.as_matrix());
    for (int i = 0; i<corres.size(); i++) {
        vpgl_perspective_camera<double> curCamera = corres[i].camera_;
        x[6 + 3 * i + 0] = curCamera.get_calibration().get_matrix()[0][0];
        
        // R_pan_tilt * Rs = R--> R_pt = R * inv(Rs)
        vnl_matrix_fixed<double, 3, 3> R_pan_tilt = curCamera.get_rotation().as_matrix() * Rs_inv;
        double cos_pan = R_pan_tilt(0, 0);
        double sin_pan = -R_pan_tilt(0, 2);
        double cos_tilt = R_pan_tilt(1, 1);
        double sin_tilt = -R_pan_tilt(2, 1);
        double pan  = atan2(sin_pan, cos_pan) * 180.0 /vnl_math::pi;
        double tilt = atan2(sin_tilt, cos_tilt) * 180.0 /vnl_math::pi;
        x[6 + 3 * i + 1] = pan;
        x[6 + 3 * i + 2] = tilt;
        
        cx += curCamera.get_camera_center().x();
        cy += curCamera.get_camera_center().y();
        cz += curCamera.get_camera_center().z();
    }
    
    x[0] = cx/camera_num;
    x[1] = cy/camera_num;
    x[2] = cz/camera_num;
    
    vgl_rotation_3d<double> RS(initSR);
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
    
    printf("CC, Rs is: ");
    for (int i = 0; i<6; i++) {
        printf("%f ", x[i]);
    }
    printf("\n");
    
    estimatedCameraCenter = vgl_point_3d<double>(x[0], x[1], x[2]);
    vnl_vector_fixed<double, 3> rod(x[3], x[4], x[5]);
    estimatedRs = vgl_rotation_3d<double>(rod);
    
    residual.getCameras(x, estimatedCameras);    
    return true;
}

class camera2PTZByPointsOnLinesResidual:public vnl_least_squares_function
{
protected:
    const LinePointsInCameraview corre_;
    const vgl_point_3d<double> cc_;
    const vgl_rotation_3d<double> Rs_;
    const vgl_point_2d<double> pp_;
    
public:
    camera2PTZByPointsOnLinesResidual(const LinePointsInCameraview & corre,
                                      const vgl_point_3d<double> & cc,
                                      const vgl_rotation_3d<double> & Rs,
                                      const vgl_point_2d<double> &pp, int pts_num, int line_pts_num):
    vnl_least_squares_function(3, pts_num * 2 + line_pts_num , no_gradient),
    corre_(corre),
    cc_(cc),
    Rs_(Rs),
    pp_(pp)
    {
        assert(3 <= pts_num * 2 + line_pts_num);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double pan  = x[0];
        double tilt = x[1];
        double fl   = x[2];
        
        vpgl_calibration_matrix<double> K(fl, pp_);
        vnl_matrix_fixed<double, 3, 3> Q;             //rotation from pan tilt angle
        VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
        
        vnl_matrix<double> QS = Q.as_matrix() * Rs_.as_matrix().as_matrix();
        vnl_matrix_fixed<double, 3, 3> QS33(QS);
        
        vpgl_perspective_camera<double> curCamera;
        curCamera.set_calibration(K);
        curCamera.set_camera_center(cc_);
        curCamera.set_rotation(vgl_rotation_3d<double>(QS33));
        
        //loop each points
        int idx = 0;
        for (int j = 0; j<corre_.wld_pts_.size(); j++) {
            vgl_point_2d<double> q =curCamera.project(corre_.wld_pts_[j]);
            fx[idx] = corre_.img_pts_[j].x() - q.x();
            idx++;
            fx[idx] = corre_.img_pts_[j].y() - q.y();
            idx++;
        }
        
        //loop each lines in the camera
        vcl_vector<PointsOnLine> pols = corre_.lines_;
        for (int j = 0; j<pols.size(); j++) {
            vgl_point_2d<double> p1 = curCamera.project(pols[j].line_.point1());
            vgl_point_2d<double> p2 = curCamera.project(pols[j].line_.point2());
            vgl_line_2d<double> line(p1, p2);
            // for points locate on the line
            for (int k = 0; k<pols[j].pts_.size(); k++) {
                vgl_point_2d<double> p3 = pols[j].pts_[k];
                fx[idx] = vgl_distance(line, p3);
                idx++;
            }
        }
        
    }
    
    void getCamera(vnl_vector<double> const & x, vpgl_perspective_camera<double> & camera)
    {
        double pan  = x[0];
        double tilt = x[1];
        double fl   = x[2];
        
        vpgl_calibration_matrix<double> K(fl, pp_);
        vnl_matrix_fixed<double, 3, 3> Q;                   //rotation from pan tilt angle
        VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
        
        vnl_matrix<double> QS = Q.as_matrix() * Rs_.as_matrix().as_matrix();
        vnl_matrix_fixed<double, 3, 3> QS33(QS);
        
        camera.set_calibration(K);
        camera.set_camera_center(cc_);
        camera.set_rotation(vgl_rotation_3d<double>(QS33));
    }
};

bool VpglPTZEstimation::camera2PTZByPointsOnLines(const LinePointsInCameraview & corre,
                                                  const vgl_point_3d<double> & cc,
                                                  const vgl_rotation_3d<double> & SR,
                                                  vpgl_perspective_camera<double> & estimatedCamera,
                                                  vnl_vector_fixed<double, 3> & ptz, bool verbose)
{
    assert(corre.wld_pts_.size() == corre.img_pts_.size());
    
    int pts_num = (int)corre.wld_pts_.size();
    int line_pts_num = 0;
    for (int i = 0; i<corre.lines_.size(); i++) {
        line_pts_num += (int)corre.lines_[i].pts_.size();
    }
    if (pts_num * 2 + line_pts_num < 3) {
        return false;
    }
    camera2PTZByPointsOnLinesResidual residual(corre, cc, SR, corre.camera_.get_calibration().principal_point(), pts_num, line_pts_num);
    
    vnl_matrix_fixed<double, 3, 3> Rs_inv = vnl_inverse(SR.as_matrix());
    vnl_matrix_fixed<double, 3, 3> R_pan_tilt = corre.camera_.get_rotation().as_matrix() * Rs_inv;
    double cos_pan  = R_pan_tilt(0, 0);
    double sin_pan  = -R_pan_tilt(0, 2);
    double cos_tilt = R_pan_tilt(1, 1);
    double sin_tilt = -R_pan_tilt(2, 1);
    double pan  = atan2(sin_pan, cos_pan) * 180.0 /vnl_math::pi;
    double tilt = atan2(sin_tilt, cos_tilt) * 180.0 /vnl_math::pi;
    vnl_vector<double> x(3);
    x[0] = pan;
    x[1] = tilt;
    x[2] = corre.camera_.get_calibration().get_matrix()[0][0];
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    if (verbose) {
        lmq.diagnose_outcome();
    }
    
    ptz[0] = x[0];
    ptz[1] = x[1];
    ptz[2] = x[2];
    
    residual.getCamera(x, estimatedCamera);    
    return true;
}

class estimageCamera7DOFResidual:public vnl_least_squares_function
{
protected:
    const LinePointsInCameraview corre_;
    const vgl_point_2d<double> pp_;
    
public:
    estimageCamera7DOFResidual(const LinePointsInCameraview & corre,
                               const vgl_point_2d<double> &pp, int pts_num, int line_pts_num):
    vnl_least_squares_function(7, pts_num * 2 + line_pts_num , no_gradient),
    corre_(corre),
    pp_(pp)
    {
        assert(7 <= pts_num * 2 + line_pts_num);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double fl = x[0];
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double> R(rod);
        vgl_point_3d<double> cc(x[4], x[5], x[6]);
        
        vpgl_calibration_matrix<double> K(fl, pp_);
        vpgl_perspective_camera<double> curCamera;
        curCamera.set_calibration(K);
        curCamera.set_camera_center(cc);
        curCamera.set_rotation(R);
        
        //loop each points
        int idx = 0;
        for (int j = 0; j<corre_.wld_pts_.size(); j++) {
            vgl_point_2d<double> q =curCamera.project(corre_.wld_pts_[j]);
            fx[idx] = corre_.img_pts_[j].x() - q.x();
            idx++;
            fx[idx] = corre_.img_pts_[j].y() - q.y();
            idx++;
        }
        
        //loop each lines in the camera
        vcl_vector<PointsOnLine> pols = corre_.lines_;
        for (int j = 0; j<pols.size(); j++) {
            vgl_point_2d<double> p1 = curCamera.project(pols[j].line_.point1());
            vgl_point_2d<double> p2 = curCamera.project(pols[j].line_.point2());
            vgl_line_2d<double> line(p1, p2);
            // for points locate on the line
            for (int k = 0; k<pols[j].pts_.size(); k++) {
                vgl_point_2d<double> p3 = pols[j].pts_[k];
                fx[idx] = vgl_distance(line, p3);
                idx++;
            }
        }
        
        // loop each circle
        // for points locate on the conics
        vnl_matrix_fixed<double, 3, 3> H = VpglPlus::homographyFromProjectiveCamera(curCamera);
        vcl_vector<PointsOnCircle> circles = corre_.circles_;
        for (int i = 0; i<circles.size(); i++) {
            vgl_conic<double> conic_proj = VpglPlus::projectConic(H, circles[i].circle_);
            for (int j = 0; j<circles[i].pts_.size(); j++) {
                vgl_point_2d<double> p = circles[i].pts_[j];
                double dis = vgl_homg_operators_2d<double>::distance_squared(conic_proj, vgl_homg_point_2d<double>(p.x(), p.y(), 1.0));
                dis = sqrt(dis + 0.0000001);
                fx[idx] = dis;
                idx++;
            }
        }        
    }
    
    void getCamera(vnl_vector<double> const & x, vpgl_perspective_camera<double> & camera)
    {
        double fl = x[0];
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double> R(rod);
        vgl_point_3d<double> cc(x[4], x[5], x[6]);
        
        vpgl_calibration_matrix<double> K(fl, pp_);
        camera.set_calibration(K);
        camera.set_camera_center(cc);
        camera.set_rotation(R);
    }

    
};


bool VpglPTZEstimation::estimageCamera(const LinePointsInCameraview & corre,
                                       vpgl_perspective_camera<double> & estimatedCamera,
                                       bool verbose)
{
    assert(corre.wld_pts_.size() == corre.img_pts_.size());
    
    int pts_num = (int)corre.wld_pts_.size();
    int line_pts_num = 0;
    for (int i = 0; i<corre.lines_.size(); i++) {
        line_pts_num += (int)corre.lines_[i].pts_.size();
    }
    for (int i = 0; i<corre.circles_.size(); i++) {
        line_pts_num += (int)corre.circles_[i].pts_.size();
    }
    if (pts_num * 2 + line_pts_num < 7) {
        return false;
    }
 //   printf("constraint is number is %d\n", pts_num * 2 + line_pts_num);
  
    vgl_point_2d<double> pp = corre.camera_.get_calibration().principal_point();
    estimageCamera7DOFResidual residual(corre, pp, pts_num, line_pts_num);
    
    vnl_vector<double> x(7);
    x[0] = corre.camera_.get_calibration().focal_length();
    x[1] = corre.camera_.get_rotation().as_rodrigues()[0];
    x[2] = corre.camera_.get_rotation().as_rodrigues()[1];
    x[3] = corre.camera_.get_rotation().as_rodrigues()[2];
    x[4] = corre.camera_.get_camera_center().x();
    x[5] = corre.camera_.get_camera_center().y();
    x[6] = corre.camera_.get_camera_center().z();
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    if (verbose) {
        lmq.diagnose_outcome();
    }
    residual.getCamera(x, estimatedCamera);
    return true;
}

class modifyCameraCenterResidual:public vnl_least_squares_function
{
protected:
    const LinePointsInCameraview corre_;
    const vgl_point_3d<double> cc_;
    const vgl_point_2d<double> pp_;
    
public:
    modifyCameraCenterResidual(const LinePointsInCameraview & corre, const vgl_point_3d<double> & cc,
                               const vgl_point_2d<double> & pp, int pts_num, int line_pts_num)
    :vnl_least_squares_function(4, pts_num * 2 + line_pts_num , no_gradient),
    corre_(corre),
    cc_(cc),
    pp_(pp)
    {
        assert(4 <= pts_num * 2 + line_pts_num);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double fl = x[0];
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double> R(rod);
        
        vpgl_perspective_camera<double> camera;
        vpgl_calibration_matrix<double> K(fl, pp_);
        camera.set_calibration(K);
        camera.set_camera_center(cc_);
        camera.set_rotation(R);
        
        //loop each points
        int idx = 0;
        for (int j = 0; j<corre_.wld_pts_.size(); j++) {
            vgl_point_2d<double> q =camera.project(corre_.wld_pts_[j]);
            fx[idx] = corre_.img_pts_[j].x() - q.x();
            idx++;
            fx[idx] = corre_.img_pts_[j].y() - q.y();
            idx++;
        }
        
        //loop each lines in the camera
        vcl_vector<PointsOnLine> pols = corre_.lines_;
        for (int j = 0; j<pols.size(); j++) {
            vgl_point_2d<double> p1 = camera.project(pols[j].line_.point1());
            vgl_point_2d<double> p2 = camera.project(pols[j].line_.point2());
            vgl_line_2d<double> line(p1, p2);
            // for points locate on the line
            for (int k = 0; k<pols[j].pts_.size(); k++) {
                vgl_point_2d<double> p3 = pols[j].pts_[k];
                fx[idx] = vgl_distance(line, p3);
                idx++;
            }
        }
    }
    
    void getCamera(vnl_vector<double> const & x, vpgl_perspective_camera<double> & camera)
    {
        double fl = x[0];
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double> R(rod);
        
        vpgl_calibration_matrix<double> K(fl, pp_);
        camera.set_calibration(K);
        camera.set_camera_center(cc_);
        camera.set_rotation(R);
    }
    
};

bool VpglPTZEstimation::modifyCameraCenter(const LinePointsInCameraview & corre,
                                           const vgl_point_3d<double> & cc,
                                           vpgl_perspective_camera<double> & estimatedCamera,
                                           bool verbose)
{
    assert(corre.wld_pts_.size() == corre.img_pts_.size());
    printf("untested code\n");
    assert(0);
    
    int pts_num = (int)corre.wld_pts_.size();
    int line_pts_num = 0;
    for (int i = 0; i<corre.lines_.size(); i++) {
        line_pts_num += (int)corre.lines_[i].pts_.size();
    }
    if (pts_num * 2 + line_pts_num < 4) {
        return false;
    }
    
    vpgl_perspective_camera<double> orgCamera = corre.camera_;
    modifyCameraCenterResidual residual(corre, cc, orgCamera.get_calibration().principal_point(), pts_num, line_pts_num);
    
    vnl_vector<double> x(4);
    x[0] = orgCamera.get_calibration().focal_length();
    x[1] = orgCamera.get_rotation().as_rodrigues()[0];
    x[2] = orgCamera.get_rotation().as_rodrigues()[1];
    x[3] = orgCamera.get_rotation().as_rodrigues()[2];
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    if (verbose) {
        lmq.diagnose_outcome();
    }
    residual.getCamera(x, estimatedCamera);
    return true;
}
















