//
//  vnl_multiple_gaussian.h
//  CameraPlaning
//
//  Created by Jimmy Chen LOCAL on 8/7/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CameraPlaning__vnl_multiple_gaussian__
#define __CameraPlaning__vnl_multiple_gaussian__

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>
#include <vnl/vnl_math.h>
#include <vcl_vector.h>

class vnl_single_guassian
{
private:
    double mu_;
    double sigma_;
    double log_cach_; // lg(1/sigma*sqrt(2*pi))
public:
    vnl_single_guassian(const vcl_vector<double> & data)
    {
        mu_ = 0;
        for (int i = 0 ; i<data.size(); i++) {
            mu_ += data[i];
        }
        mu_ /= data.size();
        
        sigma_ = 0;
        for (int i = 0; i<data.size(); i++) {
            double dif = data[i] - mu_;
            sigma_ += dif * dif;
        }
        sigma_ = sqrt(sigma_/data.size());
        log_cach_ = log(1.0/sigma_ * sqrt(2 * vnl_math::pi));
    }
    vnl_single_guassian(const double mu = 0.0, const double sigma = 1.0)
    {
        assert(sigma > 0.0);
        mu_ = mu;
        sigma_ = sigma;
        log_cach_ = log(1.0/sigma_ * sqrt(2 * vnl_math::pi));
    }
    
    double log_prob_density(const double x) const
    {
        return 0.5 * (x - mu_) * (x - mu_)/sigma_ - log_cach_;
    }
    double probility(const double x) const
    {
        assert(sigma_ > 0.0);
        return 1.0/(sigma_ * sqrt(2.0 * vnl_math::pi)) * exp(-0.5 * ((x - mu_) * (x - mu_))/(sigma_ * sigma_));
    }
    
    double mean(){return mu_;}
    double sigma(){return sigma_;}
};


// multivariate gaussian distribution
// dimension >= 2

// un-finished implementation
// can not handle when variance of one dimension is zero

class vnl_multiple_gaussian
{
    int dim_;
    vnl_matrix<double> mu_;
    vnl_matrix<double> sigma_;  // covariance matrix
    vnl_matrix<double> inv_sigma_;
    double coeff_; // 1/(2pi)^D/2 * 1.0/det(sigma)
    
public:
    vnl_multiple_gaussian();
    ~vnl_multiple_gaussian();
    
    bool set_data(const vcl_vector<vnl_vector<double> > & data);  // mean covariance matrix from data set
    double probability(const vnl_vector<double> & x) const;
    double independent_probability(const vnl_vector<double> & x) const;
    double distance_square(const vnl_vector<double> & x) const;   // distance to mean vector
    
};

#endif /* defined(__CameraPlaning__vnl_multiple_gaussian__) */
