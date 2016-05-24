//
//  vxl_self_calib.cpp
//  QuadCopter
//
//  Created by jimmy on 4/27/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vxl_self_calib.h"
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vnl/vnl_least_squares_function.h>
#include <vcl_numeric.h>
#include <vcl_iostream.h>
#include <vnl/algo/vnl_svd.h>
#include <vnl/algo/vnl_matrix_inverse.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vnl/algo/vnl_determinant.h>
#include <assert.h>
#include <vgl/vgl_point_2d.h>


class calibIdenticalKEqu195_Residual: public vnl_least_squares_function
{
protected:
    const vcl_vector< vnl_matrix_fixed<double, 3, 4> > & projections_;
    const vgl_point_2d<double> pp_;
    
public:
    // unknow 4, f, p_3x1
    // constraint, 5: up triangle in 3x3 matrix, may have meaning less constraint such as  1 - 1 = 0
    // but at least 2 meaningful constraits
    calibIdenticalKEqu195_Residual(const vcl_vector< vnl_matrix_fixed<double, 3, 4> > & projections,
                                   const vgl_point_2d<double> & pp):
    vnl_least_squares_function(4, 5 * (int)projections.size(), no_gradient),
    projections_(projections),
    pp_(pp)
    {
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double f  = x[0];
        double p1 = x[1];
        double p2 = x[2];
        double p3 = x[3];
        double u  = pp_.x();
        double v  = pp_.y();
        
        vnl_matrix<double> K(3, 3, 0);
        K[0][0] = K[1][1] = f;
        K[0][2] = u;
        K[1][2] = v;
        K[2][2] = 1.0;
        
        // dual image of the absolute conic
        vnl_matrix<double> diac = K * K.transpose();
        vnl_matrix<double> pVec(3, 1, 0);
        pVec[0][0] = p1;
        pVec[1][0] = p2;
        pVec[2][0] = p3;
        int idx = 0;
        for (int i = 0; i<projections_.size(); i++) {
            vnl_matrix<double> P = projections_[i].as_matrix();
            vnl_matrix<double> A = P.extract(3, 3, 0, 0);
            vnl_matrix<double> a = P.extract(3, 1, 0, 3);
            
            vnl_matrix<double> tmp = A - a * pVec.transpose();
            vnl_matrix<double> KKt_right = tmp * diac * tmp.transpose();
            KKt_right /= KKt_right[2][2];  // normalize
            
            // constraint in up triangle
            fx[idx] = diac[0][0] - KKt_right[0][0];
            idx++;
            fx[idx] = diac[1][1] - KKt_right[1][1];
            idx++;
            
            fx[idx] = diac[0][1] - KKt_right[0][1];
            idx++;
            fx[idx] = diac[0][2] - KKt_right[0][2];
            idx++;
            fx[idx] = diac[1][2] - KKt_right[1][2];
            idx++;            
        }
    }
    
};



bool VxlSelfCalib::calibIdenticalK(const vcl_vector< vnl_matrix_fixed<double, 3, 4> > & projections,
                                   const vnl_matrix_fixed<double, 3, 3> & initK,
                                   const vnl_vector_fixed<double, 3> & init_p,
                                   vnl_matrix_fixed<double, 3, 3> & finalK,
                                   vnl_vector_fixed<double, 3> & final_p)
{
    assert(projections.size() >= 3);
    
    vgl_point_2d<double> pp(initK[0][2], initK[1][2]);  
    calibIdenticalKEqu195_Residual residual(projections, pp);
    
    vnl_vector<double> x(4, 0.0);
    x[0] = initK[0][0];
    x[1] = init_p[0];
    x[2] = init_p[1];
    x[3] = init_p[2];
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        vcl_cerr<<"x = "<<x<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    vcl_cerr<<"x = "<<x<<vcl_endl;
    
    
    
    return true;
}