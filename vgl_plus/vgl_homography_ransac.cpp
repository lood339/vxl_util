//
//  vgl_homography_ransac.cpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-25.
//  Copyright © 2015 jimmy. All rights reserved.
//

#include "vgl_homography_ransac.hpp"

#include <rrel/rrel_homography2d_est.h>
#include <rrel/rrel_ran_sam_search.h>
#include <rrel/rrel_ransac_obj.h>
#include <vcl_iostream.h>

bool vgl_homography_ransac(vcl_vector< vgl_point_2d< double > > const& first,
                           vcl_vector< vgl_point_2d< double > > const& second,
                           vgl_h_matrix_2d< double > & H)
{
    assert(first.size() == second.size() );
    assert(first.size() >= 4);
    
    vcl_vector< vnl_vector< double > > from_pts;
    vcl_vector< vnl_vector< double > > to_pts;
    vnl_vector< double > p(3);
    
    for ( unsigned int i = 0; i < first.size(); i++ )
    {
        p[0] = first[i].x();
        p[1] = first[i].y();
        p[2] = 1.0;
        from_pts.push_back(p);
        
        p[0] = second[i].x();
        p[1] = second[i].y();
        p[2] = 1.0;
        to_pts.push_back(p);
    }
    
    
    rrel_homography2d_est * hg = new rrel_homography2d_est( from_pts, to_pts );
    
    double max_outlier_frac  = 0.5;
    double desired_prob_good = 0.99;
    int max_pops    = 1;
    int trace_level = 0;
    
    // solve using RANSAC
    rrel_ransac_obj* ransac = new rrel_ransac_obj();
    hg->set_prior_scale( 1.0 );
    
    rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
    ransam->set_trace_level(trace_level);
    ransam->set_sampling_params( max_outlier_frac, desired_prob_good, max_pops);
    
    if ( !ransam->estimate( hg, ransac ) ) {
        vcl_cout << "RANSAC failed!!\n";
        H.set_identity();
        
        delete ransac;
        delete ransam;
        delete hg;
        return false;
    }
    
    vnl_matrix< double > elements( 3, 3 );
    hg->params_to_homog( ransam->params(), elements );
    
    delete ransac;
    delete ransam;
    delete hg;
    
    // normalise the homography
    elements /= elements[2][2];
    
    H = vgl_h_matrix_2d< double >( elements );
    return true;
}


bool vgl_homography_ransac(vcl_vector< vgl_point_2d< double > > const& first,
                           vcl_vector< vgl_point_2d< double > > const& second,
                           vgl_h_matrix_2d< double > & H,
                           const homography_ransac_parameter & param)
{
    assert(first.size() == second.size() );
    assert(first.size() >= 4);
    
    vcl_vector< vnl_vector< double > > from_pts;
    vcl_vector< vnl_vector< double > > to_pts;
    vnl_vector< double > p(3);
    
    for ( unsigned int i = 0; i < first.size(); i++ )
    {
        p[0] = first[i].x();
        p[1] = first[i].y();
        p[2] = 1.0;
        from_pts.push_back(p);
        
        p[0] = second[i].x();
        p[1] = second[i].y();
        p[2] = 1.0;
        to_pts.push_back(p);
    }
    
    rrel_homography2d_est * hg = new rrel_homography2d_est( from_pts, to_pts );
    
    double max_outlier_frac  = param.max_outlier_frac;
    double desired_prob_good = param.desired_prob_good;
    double error_tolerance = param.error_tolerance;
    int max_pops    = 1;  // ?
    int trace_level = 0;  // ?
    
    // solve using RANSAC
    rrel_ransac_obj* ransac = new rrel_ransac_obj();
    hg->set_prior_scale( error_tolerance/2.0 );  // ransac has default value of 2.0
    
    rrel_ran_sam_search* ransam = new rrel_ran_sam_search;
    ransam->set_trace_level(trace_level);
    ransam->set_sampling_params( max_outlier_frac, desired_prob_good, max_pops);
    
    if ( !ransam->estimate( hg, ransac ) ) {
        vcl_cout << "RANSAC failed!!\n";
        H.set_identity();
        
        delete ransac;
        delete ransam;
        delete hg;
        return false;
    }
    
    vnl_matrix< double > elements( 3, 3 );
    hg->params_to_homog( ransam->params(), elements );
    
    delete ransac;
    delete ransam;
    delete hg;
    
    // normalise the homography
    elements /= elements[2][2];
    
    H = vgl_h_matrix_2d< double >( elements );
    return true;
}


