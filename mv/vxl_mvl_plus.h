//
//  vxl_mvl_plus.h
//  QuadCopter
//
//  Created by jimmy on 4/3/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vxl_mvl_plus__
#define __QuadCopter__vxl_mvl_plus__

// multiple view library

// wrap code in vxl/src/contrib/oxl/mvl
#include <vpgl/vpgl_fundamental_matrix.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vcl_utility.h>
#include <vnl/vnl_matrix_fixed.h>
#include <vnl/vnl_vector_fixed.h>

struct SSelfCalibSolution
{
    bool valid;
    vnl_matrix_fixed<double, 3, 4> P[3]; // camera matrices.
    vgl_homg_point_3d<double>   Q;    // last world point.
};

class VxlMvlPlus
{
public:
    static bool fundamental_RANSAC(vcl_vector< vgl_point_2d< double > > const & first,
                                   vcl_vector< vgl_point_2d< double > > const & second,
                                   vcl_vector< bool > & inlier,
                                   double error_threshold,
                                   // output
                                   vpgl_fundamental_matrix<double> & F);
    
    static bool fundamental_RANSAC(vcl_vector< vgl_point_2d< double > > const & first,
                                   vcl_vector< vgl_point_2d< double > > const & second,
                                   vcl_vector< vcl_pair<int, int> > const & initialMatchedIndices,  // matched index first --> second
                                   double error_threshold,
                                   // output
                                   vcl_vector< vcl_pair<int, int> > & finalMatchedIndices,
                                   vpgl_fundamental_matrix<double> & F);
    
    // self calibration from 6 points from 3 views
    // Q: world coordinate of last point
    
    static bool three_view_six_points_calib(const vcl_vector< vcl_vector<vgl_point_2d<double> > > & pointsVec,
                                            vcl_vector< vnl_matrix_fixed<double, 3, 4> > & Pmatrix,
                                            vgl_homg_point_3d<double> & Q);
    
    static bool three_view_six_points_calib(const vcl_vector< vcl_vector<vgl_point_2d<double> > > & pointsVec,
                                            SSelfCalibSolution solutions[3]);
    
    
    
};

#endif /* defined(__QuadCopter__vxl_mvl_plus__) */
