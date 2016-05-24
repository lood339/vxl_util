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

// least square solver for A x = b of sparse matrix
class VxlLeastSquare
{
public:
	static bool solver(vcl_vector<vcl_map<int, double> > & A, vcl_vector<double> & b,
                       bool overConstraint, int var_Num, double *result);
};

#endif /* defined(__CameraPlaning__vxl_least_square__) */
