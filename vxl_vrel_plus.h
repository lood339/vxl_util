//
//  vxl_vrel_plus.h
//  CameraPlaning
//
//  Created by Jimmy Chen LOCAL on 8/9/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CameraPlaning__vxl_vrel_plus__
#define __CameraPlaning__vxl_vrel_plus__

#include <vgl/algo/vgl_h_matrix_2d.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_ellipse_2d.h>


class VrelPlus
{
public:
    // assert unsafe
    static vgl_h_matrix_2d< double > homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                                                 vcl_vector< vgl_point_2d< double > > const& second,
                                                                 vcl_vector< bool > & inlier,
                                                                 double error_threshold);
    static bool homography_RANSAC(vcl_vector< vgl_point_2d< double > > const& first,
                                                       vcl_vector< vgl_point_2d< double > > const& second,
                                                       vcl_vector< bool > & inlier, vgl_h_matrix_2d< double > & H,
                                                       double error_threshold = 1.0);
    
    // detect one ellipse from the edge map
    // algorithm from CSE 5323 assignment 2
    /************************************************************************/
    /*
     RANSAC to fit an ellipse
     labelPt:  edge points     
     threshold:distance threshold from a point to an ellipse, less than threshold be
     regarded as inlier points
     fail_ratio: the probability of failure, 0.001
     */
    /************************************************************************/
    static bool fit_ellipse_RANSAC(const vcl_vector<vcl_vector<vgl_point_2d<double> > > &labelPt,
                                   const double threshold, const double fail_ratio, vgl_ellipse_2d<double> & ellipse,
                                   vnl_vector_fixed<double, 5> & ellipseEquation, vcl_vector<vgl_point_2d<double> > & fittingPoints, int maxIter = 3000);
    
    // pts1 and pts2 from different arc
    static bool fit_ellipse_RANSAC(const vcl_vector<vgl_point_2d<double> > & pts1,
                                   const vcl_vector<vgl_point_2d<double> > & pts2,
                                   double threshold, double fail_ratio,
                                   vgl_ellipse_2d<double> & ellipse, vcl_vector<vgl_point_2d<double> > & inliers, int maxIter = 3000);
};



#endif /* defined(__CameraPlaning__vxl_vrel_plus__) */
