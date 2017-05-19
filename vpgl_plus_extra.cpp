//
//  vpgl_plus_extra.cpp
//  Relocalization
//
//  Created by jimmy on 2016-08-14.
//  Copyright (c) 2016 Nowhere Planet. All rights reserved.
//

#include "vpgl_plus_extra.h"
#include <vpgl/algo/vpgl_calibration_matrix_compute.h>
#include <vpgl/algo/vpgl_camera_compute+.h>
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>



bool VpglPlusExtra::init_calib(const vcl_vector<vgl_point_2d<double> > &wldPts,
                               const vcl_vector<vgl_point_2d<double> > &imgPts,
                               const vgl_point_2d<double> &principlePoint,
                               vpgl_perspective_camera<double> &camera)
{
    if (wldPts.size() < 4 && imgPts.size() < 4) {
        return false;
    }
    if (wldPts.size() != imgPts.size()) {
        return false;
    }
    assert(wldPts.size() >= 4 && imgPts.size() >= 4);
    assert(wldPts.size() == imgPts.size());
    
    vpgl_calibration_matrix<double> K;
    if (vpgl_calibration_matrix_compute::natural(imgPts, wldPts, principlePoint, K) == false) {
        vcl_cerr<<"Failed to compute K"<<vcl_endl;
        vcl_cerr<<"Default principle point: "<<principlePoint<<vcl_endl;
        return false;
    }
    
    camera.set_calibration(K);
    
    // vpgl_perspective_camera_compute_positiveZ
    if (vpgl_perspective_camera_compute::compute(imgPts, wldPts, camera) == false) {
        vcl_cerr<<"Failed to computer R, C"<<vcl_endl;
        return false;
    }
    return true;
}

/*
bool VpglPlusExtra::init_calib_positive_z(const vcl_vector<vgl_point_2d<double> > &wldPts, const vcl_vector<vgl_point_2d<double> > &imgPts,
                                        const vgl_point_2d<double> &principlePoint, vpgl_perspective_camera<double> &camera)
{
    if (wldPts.size() < 4 && imgPts.size() < 4) {
        return false;
    }
    if (wldPts.size() != imgPts.size()) {
        return false;
    }
    assert(wldPts.size() >= 4 && imgPts.size() >= 4);
    assert(wldPts.size() == imgPts.size());
    
    
    vpgl_calibration_matrix<double> K;
    if (vpgl_calibration_matrix_compute::natural(imgPts, wldPts, principlePoint, K) == false) {
        vcl_cerr<<"Failed to compute K"<<vcl_endl;
        vcl_cerr<<"Default principle point: "<<principlePoint<<vcl_endl;
        return false;
    }
    
    camera.set_calibration(K);
    
    if (vpgl_perspective_camera_compute_positiveZ(imgPts, wldPts, camera) == false) {
        vcl_cerr<<"Failed to computer R, C"<<vcl_endl;
        return false;
    }
    return true;
}
 */


class vpgl_AffineTransform_residual: public vnl_least_squares_function
{
protected:
    vcl_vector<vgl_point_2d<double> > pts1_;
    vcl_vector<vgl_point_2d<double> > pts2_;
public:
    vpgl_AffineTransform_residual(const vcl_vector<vgl_point_2d<double>> & pts1, const vcl_vector<vgl_point_2d<double> > & pts2):
    vnl_least_squares_function(6, (unsigned int)pts1.size() * 2, no_gradient),
    pts1_(pts1),
    pts2_(pts2)
    {
        assert(pts1.size() == pts2.size());
    }
    void f(const vnl_vector<double> &x, vnl_vector<double> &fx)
    {
        //  0 1 2
        //  3 4 5
        //   ....
        vgl_transform_2d< double > M;
        M[0][0] = x[0];
        M[0][1] = x[1];
        M[0][2] = x[2];
        M[1][0] = x[3];
        M[1][1] = x[4];
        M[1][2] = x[5];
        M[2][0] = 0;
        M[2][1] = 0;
        M[2][2] = 1;
        
        int idx = 0;
        for (int i = 0; i<pts1_.size(); i++) {
            vgl_point_2d<double> p = M(pts1_[i]);
            fx[idx] = pts2_[i].x() - p.x();
            idx++;
            fx[idx] = pts2_[i].y() - p.y();
            idx++;
        }
    }
    
    vgl_transform_2d<double> getTransform(const vnl_vector<double> & x)
    {
        vnl_matrix_fixed<double, 3, 3> m;
        m[0][0] = x[0];
        m[0][1] = x[1];
        m[0][2] = x[2];
        m[1][0] = x[3];
        m[1][1] = x[4];
        m[1][2] = x[5];
        m[2][0] = 0;
        m[2][1] = 0;
        m[2][2] = 1;
        
        return vgl_transform_2d< double >(m);
    }
};

bool VpglPlusExtra::vpgl_AffineTransform(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2,
                                    vgl_transform_2d<double> & affine)
{
    assert(pts1.size() >= 3);
    assert(pts1.size() == pts2.size());
    
    //  init
    vnl_vector<double> x(6, 0);
    x[0] = 1.0;
    x[4] = 1.0;
    
    vpgl_AffineTransform_residual residual(pts1, pts2);
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimizeOk = lmq.minimize(x);
    //   lmq.diagnose_outcome();
    if (!isMinimizeOk) {
        printf("minimization failed\n");
        return false;
    }
    affine = residual.getTransform(x);
    return true;
}
