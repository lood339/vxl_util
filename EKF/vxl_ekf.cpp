//
//  vxl_ekf.cpp
//  OnlineStereo
//
//  Created by jimmy on 12/19/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_ekf.h"
#include <vnl/vnl_inverse.h>
#include <vcl_iostream.h>
#include <vnl/algo/vnl_matrix_inverse.h>
#include "vxl_ptz_camera.h"
#include "vxl_PTZ_variance.h"
#include <vnl/vnl_matlab_filewrite.h>
#include "vnl_plus.h"

void VxlEKF::init(const vnl_vector<double> & x0, const vnl_matrix<double> & P0)
{
    assert(P0.rows() == x0.size() && P0.cols() == x0.size());
    Xk_ = x0;
    Pk_ = P0;
}
void VxlEKF::initNoiseCovariance(const vnl_matrix<double> & Qk, const vnl_matrix<double> & Rk)
{
    Qk_ = Qk;
    Rk_ = Rk;
}
bool VxlEKF::update(const vnl_vector<double> &x_k, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k)
{
//    printf("Error: this function is discarded.\n");
//    assert(0);
    
    // prediction
    vnl_vector<double> X_kf = this->f(Xk_);
    vnl_matrix<double> Jf = this->Jf(Xk_);
    vnl_matrix<double> Pkf = Jf * Pk_ * Jf.transpose() + Qk_;

    // correction
    vnl_matrix<double> Jh = this->Jh(X_kf);
    vnl_matrix<double> temp = Jh * Pkf * Jh.transpose();
    vcl_cout<<"Jh * Pkf * Jh.transpose()"<<vcl_endl;
    vcl_cout<<temp<<vcl_endl;
    
    vnl_matrix_inverse<double> inv(Jh * Pkf * Jh.transpose() + Rk_);
    vnl_matrix<double> Kk = Pkf * Jh.transpose() * inv;
    vcl_cout<<"Kalman gain is "<<Kk<<vcl_endl;
    
    vnl_vector<double> z_k = this->h(x_k);
    vnl_vector<double> h_xkf = this->h(X_kf);
    X_k_a = X_kf + Kk * (z_k - h_xkf);  // output
    
    vnl_matrix<double> I = vnl_matrix<double>(Xk_.size(), Xk_.size(), 0);  // obtain matrix dimension
    I.set_identity();
    P_k = (I - Kk * Jh) * Pkf;   // ouput
    
    // update statement
    Xk_ = X_k_a;
    Pk_ = P_k;
    
    if (1) {
        printf("part whole state P_k\n");
        for (int i = 0; i<2; i++) {
            for (int j = 0; j<2; j++) {
                printf("%f ", P_k(i, j));
            }
            printf("\n");
        }
        printf("\n");
    }
    
    return true;
}

bool VxlEKF::updateOnce(const vnl_vector<double> &z_k, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k)
{
    // prediction
    vnl_vector<double> X_kf = this->f(Xk_);
    vnl_matrix<double> Jf = this->Jf(Xk_);
    vnl_matrix<double> Pkf = Jf * Pk_ * Jf.transpose() + Qk_;
    assert(Qk_.rows() == Pkf.rows());
    assert(Qk_.cols() == Pkf.cols());
    
 //   VnlPlus::print_part(Pkf, 12, 12, "Pkf");
    
    printf("predicted  pan, tilt, focal lenth is %f %f %f\n", X_kf[0], X_kf[1], X_kf[2]);
    
    // correction
    vnl_matrix<double> Jh = this->Jh(X_kf);
    vnl_matrix_inverse<double> inv(Jh * Pkf * Jh.transpose() + Rk_);
    
    vnl_matrix<double> Kk = Pkf * Jh.transpose() * inv;
    
    vnl_vector<double> h_xkf = this->h(X_kf);
    assert(h_xkf.size() == z_k.size());
    
    vnl_vector<double> dif = z_k -h_xkf;
//    X_k_a = X_kf + Kk * (z_k - h_xkf);  // output
    X_k_a = X_kf + Kk * (dif);
    
    
    vnl_matrix<double> I = vnl_matrix<double>(Xk_.size(), Xk_.size(), 0);  // obtain matrix dimension
    I.set_identity();
    P_k = (I - Kk * Jh) * Pkf;   // ouput
    
    // update statement
  //  X_k_a[1] = Xk_[1]; // fix tilt
  //  X_k_a[2] = Xk_[2]; // fix focal length
    Xk_ = X_k_a;
    Pk_ = P_k;
    VnlPlus::print_part(P_k, 12, 12, "Pk");
    return true;
}


/***********************  PTZCalibrationEKF  **************************/

vnl_vector<double> PTZCalibrationEKF::f(const vnl_vector<double> & X_k_1)
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
    
    Xk_ = f;
    
    return f;
}

vnl_vector<double> PTZCalibrationEKF::h(const vnl_vector<double> & X_k)
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
    
  //  printf("h pan tilt fl is: %f %f %f\n\n", pan, tilt, fl);
    
    // project landmark to image space
    for (int i = 0; i<N_lm; i++) {
        vgl_point_3d<double> p(X_k[6 + i * 3], X_k[6 + i * 3 + 1], X_k[6 + i * 3 + 2]);
     //   vcl_cout<<"3d point is "<<p<<vcl_endl;
        vgl_point_2d<double> q = camera.project(p);
        h[i * 2] = q.x();
        h[i * 2 + 1] = q.y();
    }
    return h;
    
}

vnl_matrix<double> PTZCalibrationEKF::Jf(const vnl_vector<double> & X_k_1_a)
{
    const int N = (int) X_k_1_a.size();
    vnl_matrix<double> J(N, N, 0);
    J.set_identity();
    J(0, 3) = 1.0; J(1, 4) = 1.0; J(2, 5) = 1.0;
    
    return J;
}

vnl_matrix<double> PTZCalibrationEKF::Jh(const vnl_vector<double> & X_k_f)
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
     //   vcl_cout<<"jaco xyz "<<vcl_endl;
     //   vcl_cout<<jaco_xyz<<vcl_endl;
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

/***********************  PanCalibrationEKF  **************************/

void PanCalibrationEKF::set_tilt_focal_length(const double tilt, const double fl)
{
    tilt_ = tilt;
    fl_ = fl;
}

vnl_vector<double> PanCalibrationEKF::f(const vnl_vector<double> & X_k_1)
{
    vnl_vector<double> f;
    f = X_k_1;
    f[0] = f[0] + f[1]; // pan = pan_old + velocity_pan, pan velocity and landmark location does not change
    return f;
}
vnl_vector<double> PanCalibrationEKF::h(const vnl_vector<double> & X_k)
{
    assert(X_k.size() >= 2);
    const int M = 2;   // camera state length
    
    const int N_lm = ( X_k.size() - 2 )/3;
    vnl_vector<double> h(N_lm * 2, 0);
    
    vpgl_perspective_camera<double> camera;
    double pan = X_k[0];
    bool isCamera = VxlPTZCamera::PTZToCamera(fl_, pan, tilt_, camera);
    assert(isCamera);
    
    // project landmark to image space
    for (int i = 0; i<N_lm; i++) {
        vgl_point_3d<double> p(X_k[M + i * 3], X_k[M + i * 3 + 1], X_k[M + i * 3 + 2]);
        //   vcl_cout<<"3d point is "<<p<<vcl_endl;
        vgl_point_2d<double> q = camera.project(p);
        h[i * 2] = q.x();
        h[i * 2 + 1] = q.y();
    }
    return h;
    
}
vnl_matrix<double> PanCalibrationEKF::Jf(const vnl_vector<double> & X_k_1_a)
{
    const unsigned N = (unsigned)X_k_1_a.size();
    vnl_matrix<double> Jf(N, N);
    Jf.set_identity();
    Jf(0, 1) = 1.0;
    return Jf;
}
vnl_matrix<double> PanCalibrationEKF::Jh(const vnl_vector<double> & X_k_f)
{
    const int C_M = 2; // state vector length
    assert(X_k_f.size() >= C_M);
    
    const int N = (int)X_k_f.size();  // state vector length
    const int M = (int)(X_k_f.size() - C_M)/3; // landmark numbers
    vnl_matrix<double> J(2 * M, N, 0);
    
    double pan = X_k_f[0];
    vpgl_perspective_camera<double> camera;
    
    PTZCameraPinhole pinhole(pan, tilt_, fl_);
    bool isCamera = VxlPTZCamera::PTZToCamera(fl_, pan, tilt_, camera);
    assert(isCamera);
    
    vpgl_perspective_camera<double> delta_pan_Camera;
    double delta_angle = 0.005;
    isCamera = VxlPTZCamera::PTZToCamera(fl_, pan + delta_angle, tilt_, delta_pan_Camera);
    assert(isCamera);
    
    // for each landmark
    for (unsigned i = 0; i<M; i++) {
        vgl_point_3d<double> p(X_k_f[C_M + 3 * i], X_k_f[C_M + 3 * i + 1], X_k_f[C_M + 3 * i + 2]);
        vgl_point_2d<double> q     = camera.project(p);
        vgl_point_2d<double> qPan  = delta_pan_Camera.project(p);
        
        // with camera parameters
     //   vnl_vector<double> jaco_pan  = VxlPTZVariance::pan_jacobian(pinhole, p, q, pan);
     //   jaco_pan[0]  = (qPan.x() - q.x())/delta_angle;
     //   jaco_pan[1]  = (qPan.y() - q.y())/delta_angle;
        vnl_vector<double> jaco_pan(2, 0);
        jaco_pan[0]  = (qPan.x() - q.x())/delta_angle;
        jaco_pan[1]  = (qPan.y() - q.y())/delta_angle;
        
        /*
         jaco_pan /= 180.0/vnl_math::pi;
         jaco_tilt /= 180.0/vnl_math::pi;
         
         vcl_cout<<"jaco tilt "<<jaco_pan<<vcl_endl;
         vcl_cout<<"jaco pan  "<<jaco_tilt<<vcl_endl;
         vcl_cout<<"jaco fl   "<<jaco_fl<<vcl_endl<<vcl_endl;
         */
        J(i * 2,     0) = jaco_pan[0];
        J(i * 2 + 1, 0) = jaco_pan[1];
        
        // with 3D position
     //   vnl_matrix<double> jaco_xyz = VxlPTZVariance::xyz_jacobian(pinhole, p, q);
        vnl_matrix<double> jaco_xyz(2, 3, 0.000001);
        J(i * 2,     C_M + i *3)     = jaco_xyz(0, 0);
        J(i * 2,     C_M + i *3 + 1) = jaco_xyz(0, 1);
        J(i * 2,     C_M + i *3 + 2) = jaco_xyz(0, 2);
        J(i * 2 + 1, C_M + i *3)     = jaco_xyz(1, 0);
        J(i * 2 + 1, C_M + i *3 + 1) = jaco_xyz(1, 1);
        J(i * 2 + 1, C_M + i *3 + 2) = jaco_xyz(1, 2);
        
        if (i == 0) {
            vcl_cout<<"jaco pan "<<jaco_pan<<vcl_endl;
            vcl_cout<<"jaco xyz "<<jaco_xyz<<vcl_endl;
        }
     }

    return J;
}

bool PanCalibrationEKF::update(const vnl_vector<double> &z_k, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k)
{
    // prediction
    vnl_vector<double> X_kf = this->f(Xk_);
    vnl_matrix<double> Jf = this->Jf(Xk_);
    vnl_matrix<double> Pkf = Jf * Pk_ * Jf.transpose() + Qk_;
    assert(Qk_.rows() == Pkf.rows());
    assert(Qk_.cols() == Pkf.cols());
    
    
    printf("predicted pan, pan_velocity is %f %f\n", X_kf[0], X_kf[1]);
    
    if (0) {
        printf("part whole state Pkf\n");
        for (int i = 0; i<8; i++) {
            for (int j = 0; j<8; j++) {
                printf("%lf ", Pk_(i, j));
            }
            printf("\n");
        }
        printf("\n");
    }
    
    // correction
    vnl_matrix<double> Jh = this->Jh(X_kf);
    
    
    vnl_matrix<double> JhPkfJhT = Jh * Pkf * Jh.transpose();
    
    if (0) {
        printf("part measurement jacobian Jh\n");
        for (int i = 0; i<8; i++) {
            for (int j = 0; j<8; j++) {
                printf("%lf ", Jh(i, j));
            }
            printf("\n");
        }
        printf("\n");
    }

    
    if(0)
    {
        printf("camera state JhPkfJhT\n");
        for (int i = 0; i<2; i++) {
            for (int j = 0; j<2; j++) {
                printf("%lf ", JhPkfJhT(i, j));
            }
            printf("\n");
        }
        printf("\n");
        
        printf("part whole state JhPkfJhT\n");
        for (int i = 0; i<8; i++) {
            for (int j = 0; j<8; j++) {
                printf("%lf ", JhPkfJhT(i, j));
            }
            printf("\n");
        }
        printf("\n");
        
        printf("observation \n");
        for (int i = 0; i<4; i++) {
            printf("%f %f\n", z_k[2*i], z_k[2*i+1]);
        }
        printf("\n");
        
        /*
        vnl_matlab_filewrite awriter("temp.mat");
        awriter.write(Jh, "Jh");
        awriter.write(Pkf, "Pkf");
        awriter.write(Jf, "Jf");
        awriter.write(Pk_, "Jk_");
        awriter.write(Qk_, "Qk_");
        awriter.write(JhPkfJhT, "JhPkfJhT");
        printf("save to %s\n", "temp.mat");
        return false;
         */

        
       
    }
    
    vnl_matrix_inverse<double> inv(JhPkfJhT + Rk_);
    vnl_matrix<double> Kk = Pkf * Jh.transpose() * inv;
    
    
    vnl_vector<double> h_xkf = this->h(X_kf);
    assert(h_xkf.size() == z_k.size());
    
    vnl_vector<double> dif = z_k -h_xkf;
    X_k_a = X_kf + Kk * (dif);
    
    
    vnl_matrix<double> I = vnl_matrix<double>(Xk_.size(), Xk_.size(), 0);  // obtain matrix dimension
    I.set_identity();
    P_k = (I - Kk * Jh) * Pkf;   // ouput
    
    Xk_ = X_k_a;
    Pk_ = P_k;
    
    if (0) {
        printf("camera state P_k\n");
        for (int i = 0; i<2; i++) {
            for (int j = 0; j<2; j++) {
                printf("%lf ", P_k(i, j));
            }
            printf("\n");
        }
        printf("\n");
        
        printf("part whole state P_k\n");
        for (int i = 0; i<8; i++) {
            for (int j = 0; j<8; j++) {
                printf("%lf ", P_k(i, j));
            }
            printf("\n");
        }
        printf("\n");
    }

    return true;
}














