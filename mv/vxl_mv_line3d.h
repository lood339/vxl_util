//
//  vxl_mv_line3d.h
//  QuadCopter
//
//  Created by jimmy on 6/11/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vxl_mv_line3d__
#define __QuadCopter__vxl_mv_line3d__

// multiple view 3D line reconstruction
#include <vgl/vgl_homg_line_3d_2_points.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_segment_3d.h>

class VxlMVLine3D
{
public:
    // reconstruct a 3d line from (>=2) calibrated camera view
    // line_segments: line segments in the image
    // return       : 
    static bool reconstruct_line(const vcl_vector<vgl_line_segment_2d<double> > & line_segments,
                                 const vcl_vector<vpgl_perspective_camera<double> > & cameras,
                                 vgl_infinite_line_3d<double> &line_3d);
    
    // 3d line segment projected to the multiple image
    // the line segment is the longest of all candidate segment (union)
    static bool longest_line_segment(const vcl_vector<vpgl_perspective_camera<double> > & cameras,
                                         const vcl_vector<vgl_line_segment_2d<double> > & segments_2d,
                                         const vgl_infinite_line_3d<double> & line_3d,
                                         vgl_line_segment_3d<double> & segment_3d);
    

};


#endif /* defined(__QuadCopter__vxl_mv_line3d__) */
