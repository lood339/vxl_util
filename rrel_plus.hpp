//
//  rrel_plus.hpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-25.
//  Copyright © 2015 jimmy. All rights reserved.
//

#ifndef rrel_plus_cpp
#define rrel_plus_cpp

#include <vgl/algo/vgl_h_matrix_2d.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include "vgl_homography_ransac.hpp"

// wrap ransac methods
class RrelPlus
{
public:
    // match points by homography mapping constraint
    static bool homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                  vcl_vector< vgl_point_2d< double > > const& second,
                                  vcl_vector< bool > & inlier,
                                  vgl_h_matrix_2d< double > & H,
                                  double error_threshold = 1.0); // pixel
    
    static bool homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                  vcl_vector< vgl_point_2d< double > > const& second,
                                  vcl_vector< bool > & inlier,
                                  vgl_h_matrix_2d< double > & H,
                                  const homography_ransac_parameter & param);     
    
};

#endif /* rrel_plus_cpp */
