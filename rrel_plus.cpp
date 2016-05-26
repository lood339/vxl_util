//
//  rrel_plus.cpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-25.
//  Copyright © 2015 jimmy. All rights reserved.
//

#include "rrel_plus.hpp"
#include "vgl_homography_ransac.hpp"

bool RrelPlus::homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                 vcl_vector< vgl_point_2d< double > > const& second,
                                 vcl_vector< bool > & inlier, vgl_h_matrix_2d< double > & H,
                                 double error_threshold)
{
    assert(first.size() >= 4);
    assert(first.size() == second.size());
    assert(inlier.size() == 0);    
    
    bool isOk = vgl_homography_ransac(first, second, H);
    if(!isOk)
    {
        return false;
    }
    inlier.resize(first.size());
    for (int i = 0; i<first.size(); i++) {
        const vgl_homg_point_2d<double> p1 = (vgl_homg_point_2d<double>)first[i];
        const vgl_homg_point_2d<double> p2 = (vgl_homg_point_2d<double>)second[i];
        vgl_homg_point_2d<double> proj_p1 = H(p1);
        double x = proj_p1.x()/proj_p1.w();
        double y = proj_p1.y()/proj_p1.w();
        
        double dx = x - p2.x();
        double dy = y - p2.y();
        
        double dis_2 = dx * dx + dy * dy;
        if (dis_2 <= error_threshold * error_threshold) {
            inlier[i] = true;
        }
        else
        {
            inlier[i] = false;
        }
    }
    return true;
}

bool RrelPlus::homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                 vcl_vector< vgl_point_2d< double > > const& second,
                                 vcl_vector< bool > & inlier,
                                 vgl_h_matrix_2d< double > & H,
                                 const homography_ransac_parameter & param) // pixel
{
    assert(first.size() >= 4);
    assert(first.size() == second.size());
    assert(inlier.size() == 0);
    
    double error_threshold = param.error_tolerance;
    
    
    bool isOk = vgl_homography_ransac(first, second, H, param);
    if(!isOk)
    {
        return false;
    }
    inlier.resize(first.size());
    for (int i = 0; i<first.size(); i++) {
        const vgl_homg_point_2d<double> p1 = (vgl_homg_point_2d<double>)first[i];
        const vgl_homg_point_2d<double> p2 = (vgl_homg_point_2d<double>)second[i];
        vgl_homg_point_2d<double> proj_p1 = H(p1);
        double x = proj_p1.x()/proj_p1.w();
        double y = proj_p1.y()/proj_p1.w();
        
        double dx = x - p2.x();
        double dy = y - p2.y();
        
        double dis_2 = dx * dx + dy * dy;
        if (dis_2 <= error_threshold * error_threshold) {
            inlier[i] = true;
        }
        else
        {
            inlier[i] = false;
        }
    }
    return true;
}

