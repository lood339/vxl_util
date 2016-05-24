//
//  vxl_sparse_3D_point.h
//  CalibFromScene
//
//  Created by Jimmy Chen LOCAL on 6/9/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CalibFromScene__vxl_sparse_3D_point__
#define __CalibFromScene__vxl_sparse_3D_point__

#include "vil_sift_feature.h"
#include <vcl_utility.h>

// sparse 3D point cloud from key frames
class VxlSparse3DPoint
{
public:
    
    static bool triangulateFromTwoCamera(const vpgl_perspective_camera<double> & leftCamera,
                                         const vpgl_perspective_camera<double> & rightCamera,
                                         const vcl_vector<vgl_point_2d<double> > & ptsLeft,
                                         const vcl_vector<vgl_point_2d<double> > & ptsRight,
                                         vcl_vector<vgl_point_3d<double> > & pts3D);
    // Pmatrix >= 3
    static bool triangulation(const vcl_vector< vnl_matrix_fixed<double, 3, 4> > & Pmatrix,
                              const vcl_vector< vcl_vector<vgl_point_2d<double> > > & points,
                              vcl_vector<vgl_homg_point_3d<double> > & points_3d);
          
    
    
};

#endif /* defined(__CalibFromScene__vxl_sparse_3D_point__) */
