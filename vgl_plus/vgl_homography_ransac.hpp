//
//  vgl_homography_ransac.hpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-25.
//  Copyright © 2015 jimmy. All rights reserved.
//
//  adapted code from Peter Carr

#ifndef vgl_homography_ransac_cpp
#define vgl_homography_ransac_cpp

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/algo/vgl_h_matrix_2d.h>

class homography_ransac_parameter
{
public:
    double max_outlier_frac  ;
    double desired_prob_good ;
    double error_tolerance;
    
    homography_ransac_parameter()
    {
        max_outlier_frac  = 0.5;  // 0.6, 0.7
        desired_prob_good = 0.99;
        error_tolerance   = 1.0;  // pixel unit of error
    }
};

// estimate homography matrix by at least 4 correspondences
bool vgl_homography_ransac(vcl_vector< vgl_point_2d< double > > const& first,
                           vcl_vector< vgl_point_2d< double > > const& second,
                           vgl_h_matrix_2d< double > & H);

// set ransac parameters
bool vgl_homography_ransac(vcl_vector< vgl_point_2d< double > > const& first,
                           vcl_vector< vgl_point_2d< double > > const& second,
                           vgl_h_matrix_2d< double > & H,
                           const homography_ransac_parameter & param);


#endif /* vgl_homography_ransac_cpp */
