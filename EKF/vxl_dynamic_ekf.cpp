//
//  vxl_dynamic_ekf.cpp
//  OnlineStereo
//
//  Created by jimmy on 12/27/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_dynamic_ekf.h"
#include "vnl_plus.h"
#include <vnl/algo/vnl_matrix_inverse.h>
#include "vxl_ptz_camera.h"
#include "vxl_PTZ_variance.h"




bool VxlDynamicEKF::update(const vnl_vector<double> &z_k, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k)
{
    // prediction
    vnl_vector<double> X_kf = this->f(Xk_);
    vnl_matrix<double> Jf = this->Jf(Xk_);
    vnl_matrix<double> Pkf = Jf * Pk_ * Jf.transpose();  // Jf * Pk_ * Jf.transpose() + Qk_;
    assert(Qc_.rows() <= Pkf.rows());
    assert(Qc_.cols() <= Pkf.cols());
    
    // add CQ
    for (int i = 0; i<Qc_.rows(); i++) {
        for (int j = 0; j<Qc_.cols(); j++) {
            Pkf(i, j) += Qc_(i, j);
        }
    }
    
    
    printf("predicted pan tilt focal length is %f %f %f\n", X_kf[0], X_kf[1], X_kf[2]);
    
    
    // correction
    vnl_matrix<double> Jh = this->Jh(X_kf);
    vnl_matrix<double> JhPkfJhT = Jh * Pkf * Jh.transpose();
    assert(JhPkfJhT.rows() == Rf_.rows());
    assert(JhPkfJhT.cols() == Rf_.cols());
    
    // add FR.
    for (int i = 0; i<Rf_.rows(); i++) {
        for (int j = 0; j<Rf_.cols(); j++) {
            JhPkfJhT(i, j) += Rf_(i, j);
        }
    }
    
//    print_part(JhPkfJhT, 8, 8, "JhPkfJhT");
    
    vnl_matrix_inverse<double> inv(JhPkfJhT); // Jh * Pkf * Jh.transpose() + Rk
    
    vnl_matrix<double> Kk = Pkf * Jh.transpose() * inv;
 //   print_part(Kk, 8, 8, "Kk");
    
    vnl_vector<double> h_xkf = this->h(X_kf);
    assert(h_xkf.size() == z_k.size());
    
    vnl_vector<double> dif = z_k -h_xkf;
    X_k_a = X_kf + Kk * (dif);
 //   vcl_cout<<"dif is "<<dif<<vcl_endl;
    
    
    
    vnl_matrix<double> I = vnl_matrix<double>(Xk_.size(), Xk_.size(), 0);  // obtain matrix dimension
    I.set_identity();
    P_k = (I - Kk * Jh) * Pkf;   // ouput
    
    // update statement
    Xk_ = X_k_a;
    Pk_ = P_k;
    
  //  print_part(Pk_, 8, 8, "Pk_");
    printf("final     pan tilt focal length is %f %f %f\n", Xk_[0], Xk_[1], Xk_[2]);
    return true;    
}


/************************************************* PTZKeypointDynamicEKF  ****************************************/

vnl_vector<double> PTZKeypointDynamicEKF::f(const vnl_vector<double> & X_k_1)
{
    assert(X_k_1.size() >= 6);
    assert(X_k_1.size() == Xk_.size());
    
    // update camera state
    vnl_matrix<double> A(6, 6, 0);
    A.set_identity();
    A(0, 3) = 1.0;
    A(1, 4) = 1.0;
    A(2, 5) = 1.0;
    vnl_vector<double> camera_state(6, 0);
    for (int i = 0; i<camera_state.size(); i++) {
        camera_state[i] = X_k_1[i];
    }
    camera_state = A * camera_state;
    
    vnl_vector<double> f = X_k_1; //keep the landmark as previous frame
    for (int i =0; i<camera_state.size(); i++) {
        f[i] = camera_state[i];
    }
    
    return f;
}

vnl_vector<double> PTZKeypointDynamicEKF::h(const vnl_vector<double> & X_k)
{
    assert(X_k.size() >= 6);
    
    const int N_lm = ( X_k.size() - 6 )/3;
    vnl_vector<double> h(N_lm * 2, 0);
    
    vpgl_perspective_camera<double> camera;
    double pan = X_k[0];
    double tilt = X_k[1];
    double fl = X_k[2];
    bool isCamera = VxlPTZCamera::PTZToCamera(fl, pan, tilt, camera);
    assert(isCamera);
    
    // project landmark to image space
    for (int i = 0; i<N_lm; i++) {
        vgl_point_3d<double> p(X_k[6 + i * 3], X_k[6 + i * 3 + 1], X_k[6 + i * 3 + 2]);        
        vgl_point_2d<double> q = camera.project(p);
        h[i * 2] = q.x();
        h[i * 2 + 1] = q.y();
    }

    return h;
}

vnl_matrix<double> PTZKeypointDynamicEKF::Jf(const vnl_vector<double> & X_k_1_a)
{
    assert(X_k_1_a.size() == Xk_.size());
    
    const int N = (int) X_k_1_a.size();
    vnl_matrix<double> J(N, N, 0);
    J.set_identity();
    J(0, 3) = 1.0; J(1, 4) = 1.0; J(2, 5) = 1.0;
    
    return J;
}

vnl_matrix<double> PTZKeypointDynamicEKF::Jh(const vnl_vector<double> & X_k_f)
{
    assert(X_k_f.size() >= 6);
    
    const int N = (int)X_k_f.size();  // state vector length
    const int M = (int)(X_k_f.size() - 6)/3; // landmark numbers
    vnl_matrix<double> J(2 * M, N, 0);
    
    double pan = X_k_f[0];
    double tilt = X_k_f[1];
    double fl = X_k_f[2];
    
    // for each landmark
    for (unsigned i = 0; i<M; i++) {
        vgl_point_3d<double> p(X_k_f[6 + 3 * i], X_k_f[6 + 3 * i + 1], X_k_f[6 + 3 * i + 2]);
        
        // with camera parameters
        vnl_vector<double> jaco_pan  = VxlPTZVariance::pan_jacobian(pan, tilt, fl, p);
        vnl_vector<double> jaco_tilt = VxlPTZVariance::tilt_jacobian(pan, tilt, fl, p);
        vnl_vector<double> jaco_fl   = VxlPTZVariance::fl_jacobian(pan, tilt, fl, p);
        
        J(i * 2,     0) = jaco_pan[0];
        J(i * 2,     1) = jaco_tilt[0];
        J(i * 2,     2) = jaco_fl[0];
        J(i * 2 + 1, 0) = jaco_pan[1];
        J(i * 2 + 1, 1) = jaco_tilt[1];
        J(i * 2 + 1, 2) = jaco_fl[1];
        
        // with 3D position
        vnl_matrix<double> jaco_xyz = VxlPTZVariance::xyz_jacobian(pan, tilt, fl, p);
        //  vnl_matrix<double> jaco_xyz(2, 3, 0.0000001);
        J(i * 2,     6 + i *3)     = jaco_xyz(0, 0);
        J(i * 2,     6 + i *3 + 1) = jaco_xyz(0, 1);
        J(i * 2,     6 + i *3 + 2) = jaco_xyz(0, 2);
        J(i * 2 + 1, 6 + i *3)     = jaco_xyz(1, 0);
        J(i * 2 + 1, 6 + i *3 + 1) = jaco_xyz(1, 1);
        J(i * 2 + 1, 6 + i *3 + 2) = jaco_xyz(1, 2);
    }
    return J;
}

/*
vnl_vector<double> Xk_;   //  - state vector nx1
vnl_matrix<double> Pk_;   //  - State vector co-variance nxn

vnl_matrix<double> Qc_;  //  − camera process noise covariance matrix  v x 1
vnl_matrix<double> Rf_;  //  - feature point measuremtn noise covariance matrix M*2 x M*2
*/

void PTZKeypointDynamicEKF::initQR(const VxlEKFCamera & camera, const vcl_list<VxlEKFFeaturePoint> & features)
{
    Qc_ = camera.Q_;
    
    const int M = (int)features.size();
    Rf_ = vnl_matrix<double>(M * 2, M *2, 0);
    int j = 0;
    for (vcl_list<VxlEKFFeaturePoint>::const_iterator it = features.begin(); it != features.end(); it++, j++) {
        Rf_(2*j, 2*j) = it->R();
        Rf_(2*j+1, 2*j+1) = it->R();
    }
}



vnl_vector<double> PTZKeypointDynamicEKF::getState(const VxlEKFCamera & camera, const vcl_list<VxlEKFFeaturePoint> & features)
{
    const int M = (int)features.size();
    assert(camera.state_.size() == 6);
    
    vnl_vector<double> s(camera.state_.size() + M * 3);
    s.update(camera.state_, 0);
    int j = 0;
    for (vcl_list<VxlEKFFeaturePoint>::const_iterator it = features.begin(); it != features.end(); it++, j++) {
        s[6 + j * 3 + 0] = it->worldPt().x();
        s[6 + j * 3 + 1] = it->worldPt().y();
        s[6 + j * 3 + 2] = it->worldPt().z();
    }
    return s;
}

void PTZKeypointDynamicEKF::setState(const vnl_vector<double> & state, VxlEKFCamera & camera, vcl_list<VxlEKFFeaturePoint> & features)
{
    const int M = (int)features.size();
    assert(camera.state_.size() == 6);
    assert(state.size() == 6 + 3 * M);
    
    for (int i = 0; i<6; i++) {
        camera.state_[i] = state[i];
    }
    int j = 0;
    for (vcl_list<VxlEKFFeaturePoint>::iterator it = features.begin(); it != features.end(); it++, j++) {
        it->setWorldPoint(state[6 + 3*j + 0], state[6 + 3*j + 1], state[6 + 3*j + 2]);
    }
}

vnl_matrix<double> PTZKeypointDynamicEKF::getP(const VxlEKFCamera & camera, const vcl_list<VxlEKFFeaturePoint> & features)
{
    const int M = (int)features.size();
    // camera parameter variance
    vnl_matrix<double> P(6 + 3 * M, 6 + 3 * M, 0);
    assert(camera.cov_.rows() == 6 && camera.cov_.cols());
    P.update(camera.cov_, 0, 0);
    
    // camera, feature point covariance, feature point variance
    int j = 0;
    for (vcl_list<VxlEKFFeaturePoint>::const_iterator it = features.begin(); it != features.end(); it++, j++) {
        assert(it->Pyx_.rows() == 3 && it->Pyx_.cols() == 6);
        P.update(it->Pyx_, 6 + 3 * j, 0);
        P.update(it->Pyx_.transpose(), 0, 6 + 3 * j);
        P.update(it->Pyy_, 6 + 3 * j, 6 + 3 * j);
    }
    return P;    
}

void PTZKeypointDynamicEKF::setP(const vnl_matrix<double> & P, VxlEKFCamera & camera, vcl_list<VxlEKFFeaturePoint> & features)
{
    const int M = (int)features.size();
    assert(P.rows() == 3 * M + camera.cov_.rows());
    assert(P.cols() == 3 * M + camera.cov_.cols());

    camera.cov_ = P.extract(6, 6, 0, 0);
    int j = 0;
    for (vcl_list<VxlEKFFeaturePoint>::iterator it = features.begin(); it != features.end(); it++, j++) {
        it->Pyx_ = P.extract(3, 6, 6 + 3 * j, 0);
        it->Pyy_ = P.extract(3, 3, 6 + 3 * j, 6 + 3 *j);
    }
}



bool PTZKeypointDynamicEKF::updateCameraFeature(VxlEKFCamera & camera, vcl_list<VxlEKFFeaturePoint> & features, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k)
{
    
    this->initQR(camera, features);
    
    // previous state vector
    Xk_ = PTZKeypointDynamicEKF::getState(camera, features);
    
    // previous covariance matrix
    Pk_ = PTZKeypointDynamicEKF::getP(camera, features);
    
    // calculate observation
    vnl_vector<double> zk((int)features.size()*2, 0);
    int j = 0;
    for (vcl_list<VxlEKFFeaturePoint>::const_iterator it = features.begin(); it != features.end(); it++) {
        zk[2*j]     = it->imagePt().x();
        zk[2*j + 1] = it->imagePt().y();
        j++;
    }
    

    VxlDynamicEKF::update(zk, X_k_a, P_k);
    
    // update state and covariance matrix
    PTZKeypointDynamicEKF::setState(X_k_a, camera, features);
    PTZKeypointDynamicEKF::setP(P_k, camera, features);
    
   // vcl_cout<<"camera state \n"<<camera.state_<<vcl_endl;
   // vcl_cout<<"camera covariance \n"<<camera.cov_<<vcl_endl;
    return true;
}




















