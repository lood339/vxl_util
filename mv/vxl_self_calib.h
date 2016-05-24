//
//  vxl_self_calib.h
//  QuadCopter
//
//  Created by jimmy on 4/27/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vxl_self_calib__
#define __QuadCopter__vxl_self_calib__

#include <vnl/vnl_matrix_fixed.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector_fixed.h>


// experimental code from "multiple view geometry in computer vision" chapter 19 Auto-calibration
class VxlSelfCalib
{
public:
    // page 461 equation 19.5
    // assume K only has one degree of freedom (focal length)
    static bool calibIdenticalK(const vcl_vector< vnl_matrix_fixed<double, 3, 4> > & projections,
                                const vnl_matrix_fixed<double, 3, 3> & initK,
                                const vnl_vector_fixed<double, 3> & init_p,
                                vnl_matrix_fixed<double, 3, 3> & finalK,
                                vnl_vector_fixed<double, 3> & final_p);
    
};

#endif /* defined(__QuadCopter__vxl_self_calib__) */
