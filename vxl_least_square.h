//
//  vxl_least_square.h
//  CameraPlaning
//
//  Created by Jimmy Chen LOCAL on 8/8/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CameraPlaning__vxl_least_square__
#define __CameraPlaning__vxl_least_square__

#include <vcl_map.h>
#include <vcl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

// least square solver for A x = b of sparse matrix
class VxlLeastSquare
{
public:
	static bool solver(vcl_vector<vcl_map<int, double> > & A, vcl_vector<double> & b,
                       bool overConstraint, int var_Num, double *result);
    
    // solve Ax = b for x
    static bool solver(const vnl_matrix<double> & A, const vnl_vector<double> & b,
                       vnl_vector<double> & x);
    
    // L2 norm least square argmin ||Ax - b||^2 + lambda * || x ||^2 for x
    static bool solver(const vnl_matrix<double> & A, const vnl_vector<double> & b,
                       const double lambda, vnl_vector<double> & x);
    
};

#endif /* defined(__CameraPlaning__vxl_least_square__) */
