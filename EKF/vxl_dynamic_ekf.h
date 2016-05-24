//
//  vxl_dynamic_ekf.h
//  OnlineStereo
//
//  Created by jimmy on 12/27/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_dynamic_ekf__
#define __OnlineStereo__vxl_dynamic_ekf__

// extended kalman filter with dynamic feature point

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>
#include <vcl_list.h>
#include "vxl_ekf_feature_point.h"


// interface for dynamic EKF model
class VxlDynamicEKF
{
protected:
    vnl_vector<double> Xk_;   //  - state vector nx1
    vnl_matrix<double> Pk_;   //  - State vector co-variance nxn
    
    vnl_matrix<double> Qc_;  //  − camera process noise covariance matrix  v x 1
    vnl_matrix<double> Rf_;  //  - feature point measuremtn noise covariance matrix M*2 x M*2
    
public:
    virtual vnl_vector<double> f(const vnl_vector<double> & X_k_1) = 0;
    virtual vnl_vector<double> h(const vnl_vector<double> & X_k) = 0;
    virtual vnl_matrix<double> Jf(const vnl_vector<double> & X_k_1_a) = 0;
    virtual vnl_matrix<double> Jh(const vnl_vector<double> & X_k_f) = 0;
   
    // z_k  : observation
    // X_k_a: output state vector
    // P_k  : output covariance matrix
    virtual bool update(const vnl_vector<double> &z_k, vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k);
};


// assume FQk and FRk is zero
class PTZKeypointDynamicEKF :public VxlDynamicEKF
{
public:
    virtual vnl_vector<double> f(const vnl_vector<double> & X_k_1);
    virtual vnl_vector<double> h(const vnl_vector<double> & X_k);
    virtual vnl_matrix<double> Jf(const vnl_vector<double> & X_k_1_a);
    virtual vnl_matrix<double> Jh(const vnl_vector<double> & X_k_f);
    
    void initQR(const VxlEKFCamera & camera, const vcl_list<VxlEKFFeaturePoint> & features);
       
    // camera and feature point state
    static vnl_vector<double> getState(const VxlEKFCamera & camera, const vcl_list<VxlEKFFeaturePoint> & features);
    static void setState(const vnl_vector<double> & state, VxlEKFCamera & camera, vcl_list<VxlEKFFeaturePoint> & features);
    
    // P: covariance matrix of previous state
    static vnl_matrix<double> getP(const VxlEKFCamera & camera, const vcl_list<VxlEKFFeaturePoint> & features);
    // set P back to camera and features. Divide P into small patches
    static void setP(const vnl_matrix<double> & P, VxlEKFCamera & camera, vcl_list<VxlEKFFeaturePoint> & features);
    
  //  void init(const vnl_vector<double> & C0, const vnl_matrix<double> & CP0, const vcl_list<VxlEKFFeaturePoint> & features);
  //  void initFeatureNoiseCovariane(const vcl_list<VxlEKFFeaturePoint> & features);
  //  void addFeature(const VxlEKFFeaturePoint & feature);
    
    bool updateCameraFeature(VxlEKFCamera & camera, vcl_list<VxlEKFFeaturePoint> & features,
                             vnl_vector<double> & X_k_a, vnl_matrix<double> & P_k);
    
};



#endif /* defined(__OnlineStereo__vxl_dynamic_ekf__) */
