//
//  cvx_LSD.h
//  QuadCopter
//
//  Created by jimmy on 6/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__cvx_LSD__
#define __QuadCopter__cvx_LSD__

// wrap lsd code
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vnl/vnl_vector.h>

struct LSDLineSegment
{
    //    The seven values are:
    //    - x1,y1,x2,y2,width,p,-log10(NFA)
    //    for a line segment from coordinates (x1,y1) to (x2,y2),
    //    a width 'width', an angle precision of p in (0,1) given
    //    by angle_tolerance/180 degree, and NFA value 'NFA'.
    
    vgl_line_segment_2d<double> seg_;
    double width_;
    double angle_precision_;
    double NFA_; // ?
};


class CvxLSD
{
public:
    // use default parameters
    static void detect_lines(const vil_image_view<vxl_byte> & image, vcl_vector<LSDLineSegment> & line_segments);
    
    // only output longest line
    static bool detect_longest_line(const vil_image_view<vxl_byte> & image, vgl_line_segment_2d<double> & line_seg);
    
};

#endif /* defined(__QuadCopter__cvx_LSD__) */
