//
//  vxl_optimize_kalman_filter.cpp
//  CameraPlaning
//
//  Created by jimmy on 8/13/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_optimize_kalman_filter.h"
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vcl_iostream.h>
#include <vcl_map.h>
#include <vnl/algo/vnl_matrix_inverse.h>
#include "vxl_least_square.h"

class optimize_UtResidual: public vnl_least_squares_function
{
protected:
    const vcl_vector<double> observedData_;
    double lambda_;
    vcl_vector<double> velocity_;
    vcl_vector<double> acceleration_;
    
public:
    optimize_UtResidual(const vcl_vector<double> & observation, double lambda, int ut_num):
    vnl_least_squares_function(ut_num, ut_num * 2, no_gradient),
    observedData_(observation),
    lambda_(lambda)
    {
        for (int i = 0; i<observedData_.size(); i++) {
            if (i == 0) {
                velocity_.push_back(observedData_[1] - observedData_[0]);
            }
            else
            {
                velocity_.push_back(observedData_[i] - observedData_[i-1]);
            }
        }
        for (int i = 0; i<velocity_.size(); i++) {
            if (i == 0) {
                acceleration_.push_back(velocity_[1] - velocity_[0]);
            }
            else
            {
                acceleration_.push_back(velocity_[i] - velocity_[i-1]);
            }
        }
        
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> & fx)
    {
        assert(x.size() + 2 == observedData_.size());
        
        // discard first two observation data
        int idx = 0;
        for (int i = 2; i<observedData_.size(); i++) {
            double ut = x[i-2];
            double a_t_1 = acceleration_[i-1];
            double v_t_1 = velocity_[i-1];
            double x_t_1 = observedData_[i-1];
            double at = 0.1 * ut  + 0.9 * a_t_1;
            double vt = v_t_1 + a_t_1;
            double x_t = x_t_1 + v_t_1 + 0.5 * a_t_1 * a_t_1;
            
            fx[idx] = x_t - observedData_[i];
            idx++;
            fx[idx] = ut * ut * lambda_;
            idx++;
            
            if (i != observedData_.size() -1) {
                acceleration_[i+1] = at;
                velocity_[i+1] = vt;
            }
        }
    }
    
    void getSmoothedPan(vnl_vector<double> const &x, vcl_vector<double> & smoothedPan)
    {
        assert(x.size() + 2 == observedData_.size());
        
        // discard first two observation data
        smoothedPan.resize(observedData_.size());
        smoothedPan[0] = observedData_[0];
        smoothedPan[1] = observedData_[1];
        
        for (int i = 2; i<observedData_.size(); i++) {
     //       double ut = x[i-2];
            double a_t_1 = acceleration_[i-1];
            double v_t_1 = velocity_[i-1];
            double x_t_1 = observedData_[i-1];
     //       double at = 0.1 * ut  + 0.9 * a_t_1;
     //       double vt = v_t_1 + a_t_1;
            double x_t = x_t_1 + v_t_1 + 0.5 * a_t_1 * a_t_1;
            smoothedPan[i] = x_t;
        }
    }
};

bool VxlOptimizeKalmanFilter::optimize_Ut(const vcl_vector<double> &pans, vcl_vector<double> & smoothedPan, vcl_vector<double> & Ut)
{
    assert(pans.size() > 10);
    
    int opt_num = (int)pans.size() - 2;
    optimize_UtResidual residual(pans, 100.0, opt_num);
    
    vnl_vector<double> x(opt_num, 0);
    
    vnl_levenberg_marquardt lmq(residual);
    bool isMinimized = lmq.minimize(x);
    if (!isMinimized) {
        vcl_cout<<"Error: optimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    
    Ut.clear();
    for (int i = 0; i<x.size(); i++) {
        Ut.push_back(x[i]);
    }
    
    //
    residual.getSmoothedPan(x, smoothedPan);
    assert(smoothedPan.size() == pans.size());
    return true;
}

bool VxlOptimizeKalmanFilter::L2_norm_data(const vcl_vector<double> & data, vcl_vector<double> & smoothedData, double w1, double w2, double w3)
{
    vcl_vector<vcl_map<int, double> > leftVec;
    vcl_vector<double> rightVec;
    for (int i = 0; i<data.size(); i++) {        
        // constraint to original signal
        {
            vcl_map<int, double> imap;
            double right_val = data[i] * w1;
            imap[i] = 1.0 * w1;
            
            leftVec.push_back(imap);
            rightVec.push_back(right_val);
        }
        
        // constraint on velocity
        {
            if (i > 0) {
                vcl_map<int, double> imap;
                double right_val = 0.0;
                imap[i]   =  1.0 * w2;
                imap[i-1] = -1.0 * w2;
                
                leftVec.push_back(imap);
                rightVec.push_back(right_val);
            }
        }
        
        // constraint on acceleration
        {
            if (i > 0 && i<data.size() - 1) {
                vcl_map<int, double> imap;
                double right_val = 0.0;
                imap[i+1] =  1.0 * w3;
                imap[i]   = -2.0 * w3;
                imap[i-1] =  1.0 * w3;
                
                leftVec.push_back(imap);
                rightVec.push_back(right_val);
            }
        }        
    }
    assert(leftVec.size() == rightVec.size());
    
    smoothedData.resize(data.size());
    bool isSolved = VxlLeastSquare::solver(leftVec, rightVec, true, (int)smoothedData.size(), &smoothedData[0]);
    assert(isSolved);    
    return true;
}

bool VxlOptimizeKalmanFilter::L2_norm_data(const vcl_vector<vnl_vector<double> > & data, vcl_vector<vnl_vector<double> > & smoothedData,
                                           double w1, double w2, double w3)
{
    smoothedData = data;
    
    // smooth in each dimension
    const int dimNum = (int)data[0].size();
    const int sz = (int)data.size();
    for (int i = 0; i<dimNum; i++) {
        vcl_vector<double> orgData(sz);
        vcl_vector<double> L2Data;
        for (int j = 0; j<data.size(); j++) {
            orgData[j] = data[j][i];
        }
        
        bool isOpt = VxlOptimizeKalmanFilter::L2_norm_data(orgData, L2Data, w1, w2, w3);
        assert(isOpt);
        for (int j = 0; j<smoothedData.size(); j++) {
            smoothedData[j][i] = L2Data[j];
        }
    }
    return true;
}

bool VxlOptimizeKalmanFilter::online_L2_norm_data(const vcl_vector<vnl_vector<double> > & data, vcl_vector<vnl_vector<double> > & smoothedData,
                                                  double w1, double w2, double w3, int look_ahead_window_size)
{
    smoothedData = data;
    
    // smooth in each dimension
    const int dimNum = (int)data[0].size();
    const int sz = (int)data.size();
    for (int i = 0; i<dimNum; i++) {
        vcl_vector<double> orgData(sz);
        vcl_vector<double> L2Data;
        for (int j = 0; j<data.size(); j++) {
            orgData[j] = data[j][i];
        }
        
        bool isOpt = VxlOptimizeKalmanFilter::online_L2_norm_data(orgData, L2Data, w1, w2, w3, look_ahead_window_size);
        assert(isOpt);
        for (int j = 0; j<smoothedData.size(); j++) {
            smoothedData[j][i] = L2Data[j];
        }
    }
    return true;
}

bool VxlOptimizeKalmanFilter::online_L2_norm_data(const vcl_vector<double> & data, vcl_vector<double> & smoothedData,
                                                  double w1, double w2, double w3, int look_ahead_window_size)
{
    assert(look_ahead_window_size >= 0);
    assert(data.size() > look_ahead_window_size);
    
    smoothedData.clear();
    // look ahead some frames
    vcl_vector<double> curData;
    for (int i = 0; i<look_ahead_window_size; i++) {
        curData.push_back(data[i]);
    }
    
    for (int i = 0; i<data.size(); i++) {
        if (i + look_ahead_window_size < data.size()) {
            curData.push_back(data[i + look_ahead_window_size]);
        }
        
        vcl_vector<double> curSmoothedData;
        bool isSmoothed = VxlOptimizeKalmanFilter::L2_norm_data(curData, curSmoothedData, w1, w2, w3);
        assert(isSmoothed);
        smoothedData.push_back(curSmoothedData[i]);
    }
    assert(data.size() == smoothedData.size());
    return true;
}

bool VxlOptimizeKalmanFilter::filter(const vcl_vector<double> & observed_signal, const vcl_vector<double> & observed_velocity,
                                     const KF_parameter & para,
                                     // ouput
                                     vcl_vector<double> & smoothed_signal, vcl_vector<double> & smoothed_velocity)
{
    assert(observed_signal.size() == observed_velocity.size());
    
    vnl_matrix<double> X_prior = para.X_prior_;
    vnl_matrix<double> P_prior = para.P_prior_;
    vnl_matrix<double> H = para.H_;
    vnl_matrix<double> R = para.R_;
    vnl_matrix<double> I = para.I_;
    vnl_matrix<double> A = para.A_;
    vnl_matrix<double> Q = para.Q_;
    
    smoothed_signal.clear();
    smoothed_velocity.clear();
    for (int i = 0; i < observed_signal.size(); i++) {
        double x = observed_signal[i];
        double v = observed_velocity[i];
        
        vnl_matrix<double> Z(2, 1);
        Z(0, 0) = x;
        Z(1, 0) = v;
        vnl_matrix<double> K_gain = P_prior * H.transpose() * vnl_matrix_inverse<double>((H * P_prior * H.transpose() + R));
        vnl_matrix<double> X_estimate = X_prior + K_gain * ( Z - H * X_prior);
        vnl_matrix<double> P_estimate = (I - K_gain * H) * P_prior;
        
        vcl_cout<<"K_gain is: "<<vcl_endl<<K_gain;
        vcl_cout<<"P_prior is:"<<vcl_endl<<P_prior<<vcl_endl<<vcl_endl;
        
        
        X_prior = A * X_estimate; // B is zero, @todo estimate B
        P_prior = A * P_estimate * A.transpose() + Q;        
    
        smoothed_signal.push_back(X_estimate(0, 0));
        smoothed_velocity.push_back(X_estimate(1, 0));
    }
    
    assert(smoothed_signal.size() == smoothed_velocity.size());
    return true;
}

bool VxlOptimizeKalmanFilter::filter(const vcl_vector<double> & observed_signal,
                                     const KF_parameter & para,
                                     //output
                                     vcl_vector<double> & smoothed_signal,
                                     vcl_vector<double> & predicted_velocity)
{
    assert(observed_signal.size() > 0);
    
    vnl_matrix<double> X_prior = para.X_prior_;
    
    vnl_matrix<double> P_prior = para.P_prior_;
    
    vnl_matrix<double> H = para.H_;
    
    vnl_matrix<double> R = para.R_;
    
    vnl_matrix<double> I = para.I_;
    
    
    vnl_matrix<double> A = para.A_;
    vnl_matrix<double> Q = para.Q_;
    
    smoothed_signal.clear();
    for (int i = 0; i < observed_signal.size(); i++) {
        double x = observed_signal[i];
        double v = 0;
        if ( i < 2) {
            v = 0;
        }
        else
        {
            v = smoothed_signal[i-1] - smoothed_signal[i-2];
        }
        
        vnl_matrix<double> Z(2, 1);
        Z(0, 0) = x;
        Z(1, 0) = v;
        vnl_matrix<double> K_gain = P_prior * H.transpose() * vnl_matrix_inverse<double>((H * P_prior * H.transpose() + R));
        vnl_matrix<double> X_estimate = X_prior + K_gain * ( Z - H * X_prior);
        vnl_matrix<double> P_estimate = (I - K_gain * H) * P_prior;
        
      //  vcl_cout<<"K_gain is: "<<vcl_endl<<K_gain;
      //  vcl_cout<<"P_prior is:"<<vcl_endl<<P_prior<<vcl_endl<<vcl_endl;
        
        X_prior = A * X_estimate; // B is zero, @todo estimate B
        P_prior = A * P_estimate * A.transpose() + Q;
        
        smoothed_signal.push_back(X_estimate(0, 0));
        predicted_velocity.push_back(v);
    }
    assert(smoothed_signal.size() == observed_signal.size());
    
    return true;
}

bool VxlOptimizeKalmanFilter::filter(const vcl_vector<double> & observed_signal,
                                     const KF_parameter & para,
                                     // output
                                     vcl_vector<double> & smoothed_signal,
                                     vcl_vector<double> & predicted_velocity,
                                     vcl_vector<double> & predicted_acceleration)
{
    assert(observed_signal.size() > 0);
    
    vnl_matrix<double> X_prior = para.X_prior_;
    vnl_matrix<double> P_prior = para.P_prior_;
    vnl_matrix<double> H = para.H_;
    vnl_matrix<double> R = para.R_;
    vnl_matrix<double> I = para.I_;
    vnl_matrix<double> A = para.A_;
    vnl_matrix<double> Q = para.Q_;
    assert(A.rows() == 3 && A.cols() == 3);
    
    for (int i = 0; i < observed_signal.size(); i++) {
        double x = observed_signal[i];
        double v = 0;
        double a = 0;
        if ( i < 2) {
            v = 0;
        }
        else
        {
            v = smoothed_signal[i-1] - smoothed_signal[i-2];
        }
        if ( i<3) {
            a = 0;
        }
        else
        {
            a = predicted_velocity[i-1] - predicted_velocity[i-2];
        }
        
        vnl_matrix<double> Z(3, 1);
        Z(0, 0) = x;
        Z(1, 0) = v;
        Z(2, 0) = a;
        vnl_matrix<double> K_gain = P_prior * H.transpose() * vnl_matrix_inverse<double>((H * P_prior * H.transpose() + R));
        vnl_matrix<double> X_estimate = X_prior + K_gain * ( Z - H * X_prior);
        vnl_matrix<double> P_estimate = (I - K_gain * H) * P_prior;
        
        //vcl_cout<<"K_gain is: "<<vcl_endl<<K_gain;
        //  vcl_cout<<"P_prior is:"<<vcl_endl<<P_prior<<vcl_endl<<vcl_endl;
        
        X_prior = A * X_estimate; // B is zero, @todo estimate B
        P_prior = A * P_estimate * A.transpose() + Q;
        
        smoothed_signal.push_back(X_estimate(0, 0));
        predicted_velocity.push_back(v);
        predicted_acceleration.push_back(a);
    }
    assert(smoothed_signal.size() == observed_signal.size());
    assert(smoothed_signal.size() == predicted_acceleration.size());
    
    return true;
}

bool VxlOptimizeKalmanFilter::filter(const vcl_vector<double> & observed_signal,
                                     const KF_1D_parameter & para,
                                     //output
                                     vcl_vector<double> & smoothed_signal)
{
    assert(observed_signal.size() > 0);
    
    vnl_matrix<double> X_prior = para.X_prior_;
    
    vnl_matrix<double> P_prior = para.P_prior_;
    
    vnl_matrix<double> H = para.H_;
    
    vnl_matrix<double> R = para.R_;
    
    vnl_matrix<double> I = para.I_;
    
    
    vnl_matrix<double> A = para.A_;
    vnl_matrix<double> Q = para.Q_;
    
    smoothed_signal.clear();
    
    for (int i = 0; i < observed_signal.size(); i++) {
        double x = observed_signal[i];
        
        vnl_matrix<double> Z(1, 1);
        Z(0, 0) = x;
        
        vnl_matrix<double> K_gain = P_prior * H.transpose() * vnl_matrix_inverse<double>((H * P_prior * H.transpose() + R));
        vnl_matrix<double> X_estimate = X_prior + K_gain * ( Z - H * X_prior);
        vnl_matrix<double> P_estimate = (I - K_gain * H) * P_prior;
        
    //    vcl_cout<<"K_gain is: "<<vcl_endl<<K_gain;
    //    vcl_cout<<"P_prior is:"<<vcl_endl<<P_prior<<vcl_endl<<vcl_endl;
        
        X_prior = A * X_estimate; // B is zero, @todo estimate B
        P_prior = A * P_estimate * A.transpose() + Q;
        
        smoothed_signal.push_back(X_estimate(0, 0));
    }
    
    assert(smoothed_signal.size() == observed_signal.size());
    return true;
}

bool VxlOptimizeKalmanFilter::dynamic_model_estimation(const vcl_vector<int> & fns, const vcl_vector<double> & pans,
                                                       vnl_matrix_fixed<double, 2, 2> & model)
{
    assert(fns.size() == pans.size());
    
    vcl_vector<vcl_map<int, double>> matrix_vec;
    vcl_vector<double> right_vec;
    vnl_vector<double> result(4, 0);
    
    for (int i = 2; i<fns.size()-1; i++) {
        if (fns[i-2] + 1 == fns[i-1] &&
            fns[i-1] + 1 == fns[i] &&
            fns[i] + 1   == fns[i+1]) {
            double p0 = pans[i-1];
            double p1 = pans[i];
            double v0 = (pans[i]  - pans[i-2])/2.0;
            double v1 = (pans[i+1] - pans[i-1])/2.0;
            
            {
                vcl_map<int, double> iMap;
                iMap[0] = p0;
                iMap[1] = v0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(p1);
            }
            
            {
                vcl_map<int, double> iMap;
                iMap[2] = p0;
                iMap[3] = v0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(v1);
            }
        }
    }
    
    VxlLeastSquare::solver(matrix_vec, right_vec, true, 4, &result[0]);
    model(0, 0) = result[0];
    model(0, 1) = result[1];
    model(1, 0) = result[2];
    model(1, 1) = result[3];
    
    return true;
}

bool VxlOptimizeKalmanFilter::dynamic_model_estimation_three_parameter(const vcl_vector<int> & fns, const vcl_vector<double> & pans,
                                                                       vnl_matrix_fixed<double, 2, 2> & model)
{
    assert(fns.size() == pans.size());
    
    vcl_vector<vcl_map<int, double>> matrix_vec;
    vcl_vector<double> right_vec;
    vnl_vector<double> result(3, 0);
    
    for (int i = 2; i<fns.size()-1; i++) {
        if (fns[i-2] + 1 == fns[i-1] &&
            fns[i-1] + 1 == fns[i] &&
            fns[i] + 1   == fns[i+1]) {
            double p0 = pans[i-1];
            double p1 = pans[i];
            double v0 = (pans[i]  - pans[i-2])/2.0;
            double v1 = (pans[i+1] - pans[i-1])/2.0;
            
            {
                vcl_map<int, double> iMap;
                iMap[0] = p0;
                iMap[1] = v0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(p1);
            }
            
            {
                vcl_map<int, double> iMap;
                iMap[2] = v0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(v1);
            }
        }
    }
    
    VxlLeastSquare::solver(matrix_vec, right_vec, true, 3, &result[0]);
    model(0, 0) = result[0];
    model(0, 1) = result[1];
    model(1, 0) = 0.0;
    model(1, 1) = result[2];
    return true;
}

// [A B C]
// [0 D E]
// [0 0 F]
bool VxlOptimizeKalmanFilter::dynamic_model_estimation(const vcl_vector<int> & fns, const vcl_vector<double> & pans,
                                                       vnl_matrix_fixed<double, 3, 3> & model)
{
    vcl_vector<vcl_map<int, double>> matrix_vec;
    vcl_vector<double> right_vec;
    vnl_vector<double> result(6, 0);
    
    for (int i = 3; i<fns.size()-3; i++) {
        if (fns[i-3] + 1 == fns[i-2] &&
            fns[i-2] + 1 == fns[i-1] &&
            fns[i-1] + 1 == fns[i] &&
            fns[i] + 1   == fns[i+1] &&
            fns[i+1] + 1 == fns[i+2]) {
            double p0 = pans[i-1];
            double p1 = pans[i];
            double v0 = (pans[i]  - pans[i-2])/2.0;
            double v1 = (pans[i+1] - pans[i-1])/2.0;
            
            double v0_1 = (pans[i-1] - pans[i-3])/2.0;
            double v1_1 = (pans[i+2] - pans[i])/2.0;
            double a0   = (v1 - v0_1)/2.0;
            double a1   = (v1_1 - v0)/2.0;
            
            {
                // p1
                vcl_map<int, double> iMap;
                iMap[0] = p0;
                iMap[1] = v0;
                iMap[2] = a0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(p1);
            }
            
            {
                // v1
                vcl_map<int, double> iMap;
                iMap[3] = v0;
                iMap[4] = a0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(v1);
            }
            
            {
                // a1
                vcl_map<int, double> iMap;
                iMap[5] = a0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(a1);
            }
        }
    }
    
    VxlLeastSquare::solver(matrix_vec, right_vec, true, 6, &result[0]);
    model(0, 0) = result[0];
    model(0, 1) = result[1];
    model(0, 2) = result[2];
    model(1, 0) = 0.0;
    model(1, 1) = result[3];
    model(1, 2) = result[4];
    model(2, 0) = 0.0;
    model(2, 1) = 0.0;
    model(2, 2) = result[5];    
    return true;
}

bool VxlOptimizeKalmanFilter::kalman_smooth_estimate_models(const vcl_vector<double> & observed_signal,
                                                                 const vcl_vector<double> & smoothed_signal,
                                                                 double sigma_process,
                                                                 double sigma_observation,
                                                                 vnl_matrix_fixed<double, 3, 3> & dynamic_model,
                                                                 vnl_matrix_fixed<double, 3, 3> & observation_model)
{
    assert(observed_signal.size() == smoothed_signal.size());
    
    // dynamic model
    {
        vcl_vector<vcl_map<int, double>> matrix_vec;
        vcl_vector<double> right_vec;
        vnl_vector<double> result(5, 0);
        
        for (int i = 3; i<smoothed_signal.size()-3; i++) {
            double p0 = smoothed_signal[i-1];
            double p1 = smoothed_signal[i];
            double v0 = (smoothed_signal[i]   - smoothed_signal[i-2])/2.0;
            double v1 = (smoothed_signal[i+1] - smoothed_signal[i-1])/2.0;
            
            double v0_1 = (smoothed_signal[i-1] - smoothed_signal[i-3])/2.0;
            double a0   = (v1 - v0_1)/2.0;
            
            {
                // p1
                vcl_map<int, double> iMap;
                iMap[0] = p0;
                iMap[1] = v0;
                iMap[2] = a0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(p1);
            }
            
            {
                // v1
                vcl_map<int, double> iMap;
                iMap[3] = v0;
                iMap[4] = a0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(v1);
            }
        }
        
        VxlLeastSquare::solver(matrix_vec, right_vec, true, 5, &result[0]);
        dynamic_model(0, 0) = result[0];
        dynamic_model(0, 1) = result[1];
        dynamic_model(0, 2) = result[2];
        dynamic_model(1, 0) = 0.0;
        dynamic_model(1, 1) = result[3];
        dynamic_model(1, 2) = result[4];
        dynamic_model(2, 0) = 0.0;
        dynamic_model(2, 1) = 0.0;
        dynamic_model(2, 2) = 0.0;
    }
    
    // observation model
    {
        vcl_vector<vcl_map<int, double>> matrix_vec;
        vcl_vector<double> right_vec;
        vnl_vector<double> result(2, 0);
        
        for (int i = 3; i<smoothed_signal.size()-3; i++) {
            double p0 = smoothed_signal[i-1];
            double v0 = (smoothed_signal[i]   - smoothed_signal[i-2])/2.0;
            
            double z0 = observed_signal[i-1];
            double z_v0 = (observed_signal[i]  - observed_signal[i-2])/2.0;
            
            {
                // p1
                vcl_map<int, double> iMap;
                iMap[0] = p0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(z0);
            }
            
            {
                // v1
                vcl_map<int, double> iMap;
                iMap[1] = v0;
                
                matrix_vec.push_back(iMap);
                right_vec.push_back(z_v0);
            }
        }
        
        VxlLeastSquare::solver(matrix_vec, right_vec, true, 2, &result[0]);
        observation_model.fill(0.0);
        observation_model(0, 0) = result[0];
        observation_model(1, 1) = result[1];
        observation_model(2, 2) = 0.0;
    }
    
    return true;
}






