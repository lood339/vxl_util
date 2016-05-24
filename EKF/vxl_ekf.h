//
//  vxl_ekf.h
//  OnlineStereo
//
//  Created by jimmy on 12/19/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_ekf__
#define __OnlineStereo__vxl_ekf__

// Extended Kalman Filter from "Extended Kalman Filter Tutorial" by Gabriel A. Terejanu

#include <iostream>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

class VxlEKF
{
protected:
    vnl_vector<double> Xk_;  //   − State vector n×1
 //   vnl_vector<double> Wk_;  //   − Process noise vector n×1
 //   vnl_vector<double> Zk_;  //   − Observation vector m×1
 //   vnl_vector<double> Vk_;  //   − Measurement noise vector m×1
 //   vnl_vector<double> f_;   //   − Process nonlinear vector function n×1
 //   vnl_vector<double> h_;   //   − Observation nonlinear vector function m×1
    vnl_matrix<double> Qk_;  //   − Process noise covariance matrix n×n
    vnl_matrix<double> Rk_;  //   − Measurement noise covariance matrix m×m
    
    vnl_matrix<double> Pk_;  //   - State vector co-variance n×n
    
public:
    virtual vnl_vector<double> f(const vnl_vector<double> & X_k_1) = 0;
    virtual vnl_vector<double> h(const vnl_vector<double> & X_k) = 0;
    virtual vnl_matrix<double> Jf(const vnl_vector<double> & X_k_1_a) = 0;
    virtual vnl_matrix<double> Jh(const vnl_vector<double> & X_k_f) = 0;
    
    // initial state and its initial covariance
    void init(const vnl_vector<double> & x0, const vnl_matrix<double> & P0);
    // processing noise covariance and measurement noise covariance
    void initNoiseCovariance(const vnl_matrix<double> & Qk, const vnl_matrix<double> & Rk);
    
    // x_k is pure observation. From vision algorithm or ordometry etc
    // Error x_k should be z_k, x_k is previous state vector which is kept inside the EKF
    // this funiton is discarded.
    virtual bool update(const vnl_vector<double> &x_k, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k);
    
    //
    virtual bool updateOnce(const vnl_vector<double> &z_k, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k);

};


// EKF model based PTZ camera calibration
class PTZCalibrationEKF: public VxlEKF
{
public:
    // input: previous frame state vector
    //        pan, tilt, fl, v_pan, v_tilt, v_fl, landmark1 (x,y,z), landmark2 (z,y,z), ....
    // output: predicted current frame state vector
    virtual vnl_vector<double> f(const vnl_vector<double> & X_k_1);
    
    // output: 2 * M, M is landmark number
    virtual vnl_vector<double> h(const vnl_vector<double> & X_k);
    virtual vnl_matrix<double> Jf(const vnl_vector<double> & X_k_1_a);
    virtual vnl_matrix<double> Jh(const vnl_vector<double> & X_k_f);
};

// EKF based, pan only camera
class PanCalibrationEKF: public VxlEKF
{
    double tilt_;
    double fl_;
public:
    void set_tilt_focal_length(const double tilt, const double fl);
    
    // input: pan, velocity_pan
    virtual vnl_vector<double> f(const vnl_vector<double> & X_k_1);
    virtual vnl_vector<double> h(const vnl_vector<double> & X_k);
    virtual vnl_matrix<double> Jf(const vnl_vector<double> & X_k_1_a);
    virtual vnl_matrix<double> Jh(const vnl_vector<double> & X_k_f);
    
    virtual bool update(const vnl_vector<double> &z_k, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k);
};



#endif /* defined(__OnlineStereo__vxl_ekf__) */
