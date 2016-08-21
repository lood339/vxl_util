//
//  vpgl_plus_extra.h
//  Relocalization
//
//  Created by jimmy on 2016-08-14.
//  Copyright (c) 2016 Nowhere Planet. All rights reserved.
//

#ifndef __Relocalization__vpgl_plus_extra__
#define __Relocalization__vpgl_plus_extra__

#include <stdio.h>
#include <vgl/vgl_transform_2d.h>

class VpglPlusExtra
{
public:    
    static bool init_calib(const vcl_vector<vgl_point_2d<double> > &wldPts, const vcl_vector<vgl_point_2d<double> > &imgPts,
                           const vgl_point_2d<double> &principlePoint, vpgl_perspective_camera<double> &camera);
    
    
    // camera look at a direction that z > 0
    static bool init_calib_positive_z(const vcl_vector<vgl_point_2d<double> > &wldPts, const vcl_vector<vgl_point_2d<double> > &imgPts,
                                      const vgl_point_2d<double> &principlePoint, vpgl_perspective_camera<double> &camera);
    
    // affine matrix [a b t_x; c d t_y; 0 0 1]
    static bool vpgl_AffineTransform(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2,
                                     vgl_transform_2d<double> & affine);

    
};



#endif /* defined(__Relocalization__vpgl_plus_extra__) */
