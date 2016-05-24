//
//  vxl_optimize_kalman_filter.h
//  CameraPlaning
//
//  Created by jimmy on 8/13/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CameraPlaning__vxl_optimize_kalman_filter__
#define __CameraPlaning__vxl_optimize_kalman_filter__

#include <vcl_vector.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_matrix_fixed.h>

// http://bilgin.esme.org/BitsBytes/KalmanFilterforDummies.aspx

struct KF_parameter // Kalman filter parameter
{

    vnl_matrix<double> X_prior_;
    vnl_matrix<double> P_prior_;
    vnl_matrix<double> H_; // fixed  observation (measurement) model
    vnl_matrix<double> R_; // fixed from ground truth data
    vnl_matrix<double> I_;
    vnl_matrix<double> A_; //  fixed  dynamic model
    vnl_matrix<double> Q_; //
    
public:
    // init estimation and estimated velocity
    KF_parameter(const double init, const double init_velocity)
    {
        X_prior_ = vnl_matrix<double>(2, 1);
        X_prior_(0, 0) = init;
        X_prior_(1, 0) = init_velocity;
        
        P_prior_ = vnl_matrix<double>(2, 2, 0); // don not know, just guess
        P_prior_(0, 0) = 0.1;                   // trust of previous prediction
        P_prior_(1, 1) = 0.1;
        
        H_ = vnl_matrix<double>(2, 2, 0);    // fixed
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
        
        R_ = vnl_matrix<double>(2, 2, 0);   // fixed, from ground truth data
        R_(0, 0) = 0.06;
        R_(1, 1) = 0.01;
        
        I_ = vnl_matrix<double>(2, 2, 0);
        I_(0, 0) = 1.0;
        I_(1, 1) = 1.0;
        
        A_ = vnl_matrix<double>(2, 2, 0);  // fixed
        A_(0, 0) = 1.0;
        A_(0, 1) = 1.0;
        A_(1, 0) = 0;
        A_(1, 1) = 1.0;
        
        Q_ = vnl_matrix<double>(2, 2, 0);   // Q can be a function of pan angle, pan moves faster, reduce Q
        Q_(0, 0) = 0.00000004;
        Q_(1, 1) = 0.00000001;
    }
    
    // constant velocity, constant acceleration
    KF_parameter(const double init_position, const double init_velocity, const double init_acceleration)
    {
        X_prior_ = vnl_matrix<double>(3, 1);
        X_prior_(0, 0) = init_position;
        X_prior_(1, 0) = init_velocity;
        X_prior_(2, 0) = init_acceleration;
        
        P_prior_ = vnl_matrix<double>(3, 3, 0); // don not know, just guess
        P_prior_(0, 0) = 0.1;                   // trust of previous prediction
        P_prior_(1, 1) = 0.1;
        P_prior_(2, 2) = 0.1;
        
        H_ = vnl_matrix<double>(3, 3, 0);    // fixed
        H_.set_identity();
        
        R_ = vnl_matrix<double>(3, 3, 0);   // fixed, from ground truth data
        R_(0, 0) = 0.06;
        R_(1, 1) = 0.01;
        R_(2, 2) = 0.00001;
        
        I_ = vnl_matrix<double>(3, 3, 0);
        I_.set_identity();
        
        A_ = vnl_matrix<double>(3, 3, 0);    // fixed
        A_(0, 0) = 1.0; A_(0, 1) = 1.0; A_(0, 2) = 0.5;
        A_(1, 0) = 0.0; A_(1, 1) = 1.0; A_(1, 2) = 1.0;
        A_(2, 0) = 0.0; A_(2, 1) = 0.0; A_(2, 2) = 0.0;

        // t^4/4 t^3/2 t^2/2
        // t^3/2 t^2   t       sigma_v^2
        // t^2/2 t     1
        // sigma_v = 1/2 * acceleration_max
        
        Q_ = vnl_matrix<double>(3, 3, 0);   // Q can be a function of pan angle, pan moves faster, reduce Q
        Q_(0, 0) = 0.00000004;
        Q_(1, 1) = 0.00000001;
        Q_(2, 2) = 0.0000000001;
        
    }
    
    void setInit(const double init1, const double init2)
    {
        X_prior_(0, 0) = init1;
        X_prior_(1, 0) = init2;
    }
    
    void setLowerBound(const double lb1, const double lb2)
    {
        R_(0, 0) = lb1;
        R_(1, 1) = lb2;
    }
    
    void setQ(const double sigma1, const double sigma2)
    {
        Q_(0, 0) = sigma1;
        Q_(1, 1) = sigma2;
    }
    
    void setInit(const double init1, const double init2, const double init3)
    {
        X_prior_(0, 0) = init1;
        X_prior_(1, 0) = init2;
        X_prior_(2, 0) = init3;
    }
    
    void setLowerBound(const double lb1, const double lb2, const double lb3)
    {
        R_(0, 0) = lb1;
        R_(1, 1) = lb2;
        R_(2, 2) = lb3;
    }
    
    void setQ(const double sigma1, const double sigma2, const double sigma3)
    {
        Q_(0, 0) = sigma1;
        Q_(1, 1) = sigma2;
        Q_(2, 2) = sigma3;
    }
    
    void set_dynamic_model(const vnl_matrix_fixed<double, 3, 3> & A)
    {
        A_ = A;
    }
    
    void set_observation_model(const vnl_matrix_fixed<double, 3, 3> & H)
    {
        H_ = H;
    }
};

// 1 Dimension Kalman filter parameter
struct KF_1D_parameter
{
    
    vnl_matrix<double> X_prior_;
    vnl_matrix<double> P_prior_;
    vnl_matrix<double> H_; // fixed
    vnl_matrix<double> R_; // fixed from ground truth data
    vnl_matrix<double> I_;
    vnl_matrix<double> A_; //fixed
    vnl_matrix<double> Q_; //
    
public:
    // init estimation and estimated velocity
    KF_1D_parameter(const double init)
    {
        X_prior_ = vnl_matrix<double>(1, 1);
        X_prior_(0, 0) = init;
        
        P_prior_ = vnl_matrix<double>(1, 1, 0); // don not know, just guess
        P_prior_(0, 0) = 0.1;                   // trust of previous prediction
        
        H_ = vnl_matrix<double>(1, 1, 0);    // fixed
        H_(0, 0) = 1.0;
        
        R_ = vnl_matrix<double>(1, 1, 0);   // fixed, from ground truth data
        R_(0, 0) = 0.06;
        
        I_ = vnl_matrix<double>(1, 1, 0);
        I_(0, 0) = 1.0;
        
        A_ = vnl_matrix<double>(1, 1, 0);  // fixed
        A_(0, 0) = 1.0;
        
        Q_ = vnl_matrix<double>(1, 1, 0);   // Q can be a function of pan angle, pan moves faster, reduce Q
        Q_(0, 0) = 0.00000004;
    }
    
    void setInit(const double init1)
    {
        X_prior_(0, 0) = init1;
    }
    
    void setLowerBound(const double lb1)
    {
        R_(0, 0) = lb1;
    }
    
    void setQ(const double sigma1)
    {
        Q_(0, 0) = sigma1;
    }
    
};


class VxlOptimizeKalmanFilter
{
public:
    /*
     [x] = [1 1 1/2] * [x]     + [0   0]        * [u_t 1]^t
     [v] = [0 1 1 ]  * [v]     + [0   0]
     [a] = [0 0 0 ]  * [a_t-1] + [0.1 0.9a_t-1]
     
     estimate Ut
     */
    static bool optimize_Ut(const vcl_vector<double> &pans, vcl_vector<double> & smoothedPan, vcl_vector<double> & Ut);
    
    // default (w1, w2, w3) = (1.0, 1.0, 100.0)
    static bool L2_norm_data(const vcl_vector<double> & data, vcl_vector<double> & smoothedData, double w1, double w2, double w3);
    
    static bool L2_norm_data(const vcl_vector<vnl_vector<double> > & data, vcl_vector<vnl_vector<double> > & smoothedData,
                             double w1, double w2, double w3);
    
    // online L2 norm when look_ahead_window_size = 0
    static bool online_L2_norm_data(const vcl_vector<double> & data, vcl_vector<double> & smoothedData,
                                    double w1, double w2, double w3, int look_ahead_window_size);
    
    static bool online_L2_norm_data(const vcl_vector<vnl_vector<double> > & data, vcl_vector<vnl_vector<double> > & smoothedData,
                                    double w1, double w2, double w3, int look_ahead_window_size);
    
    // smooth signal by fixed parameters
    static bool filter(const vcl_vector<double> & observed_signal, const vcl_vector<double> & observed_velocity,
                       const KF_parameter & para,
                       //output
                       vcl_vector<double> & smoothed_signal, vcl_vector<double> & smoothed_velocity);
    // piece-wise constant velocity
    static bool filter(const vcl_vector<double> & observed_signal,
                       const KF_parameter & para,
                       //output
                       vcl_vector<double> & smoothed_signal,
                       vcl_vector<double> & predicted_velocity); // with piecewise velocity assumption
    
    // piece-wise constant velocity, piece-wise constant acceleration
    static bool filter(const vcl_vector<double> & observed_signal,
                       const KF_parameter & para,
                       // output
                       vcl_vector<double> & smoothed_signal,
                       vcl_vector<double> & predicted_velocity,
                       vcl_vector<double> & predicted_acceleration);
    
    static bool filter(const vcl_vector<double> & observed_signal,
                       const KF_1D_parameter & para,
                       //output
                       vcl_vector<double> & smoothed_signal);
    
    // constant model
    // [ a b ]
    // [ c d ]
    static bool dynamic_model_estimation(const vcl_vector<int> & fns, const vcl_vector<double> & pans,
                                         vnl_matrix_fixed<double, 2, 2> & model);
    // constant model
    // [ a b ]
    // [ 0 d ]
    static bool dynamic_model_estimation_three_parameter(const vcl_vector<int> & fns, const vcl_vector<double> & pans,
                                                         vnl_matrix_fixed<double, 2, 2> & model);
    // constant model
    // [A B C]
    // [0 D E]
    // [0 0 F]
    static bool dynamic_model_estimation(const vcl_vector<int> & fns, const vcl_vector<double> & pans,
                                         vnl_matrix_fixed<double, 3, 3> & model);
    
    // dynamic model
    // [A B C]
    // [0 D E]
    // [0 0 0]
    
    // observation model
    // [A 0 0]
    // [0 B 0]
    // [0 0 0]
    static bool kalman_smooth_estimate_models(const vcl_vector<double> & observed_signal,
                                              const vcl_vector<double> & smoothed_signal,
                                              double sigma_process,
                                              double sigma_observation,
                                              vnl_matrix_fixed<double, 3, 3> & dynamic_model,
                                              vnl_matrix_fixed<double, 3, 3> & observation_model);
};



#endif /* defined(__CameraPlaning__vxl_optimize_kalman_filter__) */
