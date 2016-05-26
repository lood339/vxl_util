//
//  vgl_fundamental_ransac.hpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-11-30.
//  Copyright © 2015 jimmy. All rights reserved.
//

#ifndef vgl_fundamental_ransac_cpp
#define vgl_fundamental_ransac_cpp

// filter outlier by fundamental ransac

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/algo/vgl_h_matrix_2d.h>
#include <vpgl/vpgl_fundamental_matrix.h>
#include <vnl/vnl_matrix_fixed.h>

class fundamental_ransac_parameter
{
public:
    double max_epipolar_distance;
    double confidence_prob;
    
    fundamental_ransac_parameter()
    {
        max_epipolar_distance = 3.0;
        confidence_prob = 0.99;
    }
    
};

/*
// nor robust, has infinite loop
bool vgl_fundamental_ransac(vcl_vector< vgl_point_2d< double > > const& first,
                            vcl_vector< vgl_point_2d< double > > const& second,
                            vnl_matrix_fixed< double, 3, 3 > & F,
                            vcl_vector<bool> & inliers,
                            const fundamental_ransac_parameter & param);
 */
// using opencv version
bool vgl_fundamental_ransac_opencv(vcl_vector< vgl_point_2d< double > > const& first,
                                   vcl_vector< vgl_point_2d< double > > const& second,
                                   vnl_matrix_fixed< double, 3, 3 > & F,
                                   vcl_vector<bool> & inliers,
                                   const fundamental_ransac_parameter & param);


#endif /* vgl_fundamental_ransac_cpp */
