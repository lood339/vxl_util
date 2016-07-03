//
//  vnl_multiple_gaussian.cpp
//  CameraPlaning
//
//  Created by Jimmy Chen LOCAL on 8/7/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vnl_multiple_gaussian.h"
#include <vnl/algo/vnl_determinant.h>
#include <vnl/vnl_inverse.h>
#include <vnl/vnl_math.h>
#include <vcl_iostream.h>
//#include "vxl_plus.h"

/*
 **************************   vnl_multiple_gaussian     ************************
 */

vnl_multiple_gaussian::vnl_multiple_gaussian()
{
    dim_ = -1;
    coeff_ = -1;
    
}
vnl_multiple_gaussian::~vnl_multiple_gaussian()
{
    
}

bool vnl_multiple_gaussian::set_data(const vcl_vector<vnl_vector<double> > & data)
{
    const double EPSILON = 0.000001;
    if (data.size() < 2) {
        return false;
    }
    dim_ = data[0].size();
    
    // mean vector
    mu_ = vnl_matrix<double>(dim_, 1, 0);
    for (int i = 0; i<data.size(); i++) {
        vnl_matrix<double> x(dim_, 1, 0);
        x.set_column(0, data[i]);
        mu_ += x;
    }
    mu_ /= data.size();
    
    // covariance matrix
    sigma_ = vnl_matrix<double>(dim_, dim_, 0);
    for (int i = 0; i<data.size(); i++) {
        vnl_matrix<double> x(dim_, 1, 0);
        x.set_column(0, data[i]);
        
        vnl_matrix<double> dif = x - mu_;
        sigma_ += dif * dif.transpose();
    }
    sigma_ /= data.size();
    
//    vcl_cout<<"mean is "<<mu_<<vcl_endl;
//    vcl_cout<<"co variance is "<<sigma_<<vcl_endl;
    for (int i = 0; i<dim_; i++) {
        sigma_(i, i) += EPSILON;
    }
    
    double det = vnl_determinant(sigma_);
    if (det < EPSILON) {
        return false;
    }
    assert(det > EPSILON);
    inv_sigma_ = vnl_inverse(sigma_);
    coeff_ = pow(2.0 * vnl_math::pi, - 0.5 * dim_) / sqrt(det);
    
    return true;
}

double vnl_multiple_gaussian::probability(const vnl_vector<double> & x) const
{
    assert(dim_ >= 2);
    assert(x.size() == dim_);
    
    vnl_matrix<double> xMat(dim_, 1, 0);
    xMat.set_column(0, x);
    vnl_matrix<double> dif = xMat - mu_;
    vnl_matrix<double> tMat = dif.transpose() * inv_sigma_ * dif;
    
    return coeff_ * exp( -0.5 * tMat(0, 0));
}

double vnl_multiple_gaussian::independent_probability(const vnl_vector<double> & x) const
{
    // vnl_single_guassian(const double mu = 0.0, const double sigma = 1.0)
    assert(dim_ >= 2);
    assert(x.size() == dim_);

    double prob = 1.0;
    for (int i = 0; i < dim_; i++) {
        vnl_single_guassian gau(mu_(i, 0), sigma_(i, i));
        double p = gau.probility(x[i]);
        if (p < 0.001) {
            p = 0.001;
        }
        prob *= p;
    }
    return  prob;
}
double vnl_multiple_gaussian::distance_square(const vnl_vector<double> & x) const
{
    assert(x.size() == dim_);
    
    double dis = 0;
    for (int i = 0; i<x.size(); i++) {
        double dif = x[i] - mu_(i, 0);
        dis += dif * dif;
    }
    return dis;
    
    
    
    
}
