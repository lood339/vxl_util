//
//  homography_uncertainty.h
//  QuadCopter
//
//  Created by jimmy on 7/5/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__homography_uncertainty__
#define __QuadCopter__homography_uncertainty__

// measure uncertainty of homography matrix
#include <vgl/algo/vgl_h_matrix_2d.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_matrix_fixed.h>

class VglHomographyUncertainty
{
    vgl_h_matrix_2d<double> H_;
    vnl_matrix<double> sigma_h_;  //uncertainty of H, 3 * 3
public:
    VglHomographyUncertainty(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2);
    // sigma_x, sigma_y measurement error in second image. default 1.0
    VglHomographyUncertainty(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2, double sigma_x, double sigma_y);
    ~VglHomographyUncertainty();
    
    // uncertainty of a point p, projected by H
    vnl_matrix_fixed<double, 2, 2> cov(const vgl_point_2d<double> & p);
    vgl_point_2d<double> project(const vgl_point_2d<double> & p);
    
private:
    void calculateUncertainty(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2);
    void calculateUncertainty(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2,
                              double sigma_x, double sigma_y);
    
    
};


#endif /* defined(__QuadCopter__homography_uncertainty__) */
