//
//  opengl_utils.h
//  PlanarAlign
//
//  Created by Jimmy Chen LOCAL on 1/21/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __PlanarAlign__opengl_utils__
#define __PlanarAlign__opengl_utils__

#include <vxl_config.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_line_segment_2d.h>
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>

class OpenglUtils
{
public:
    static void drawCross(const vcl_vector<vgl_point_2d<double> > &pts, float width);
    static void drawLineSegment(const vcl_vector< vgl_line_segment_2d< double > > &lines);
    static void drawCircle(const vcl_vector<vgl_point_2d<double> > & centers, float radius);
    static void drawBox(const vcl_vector<vgl_point_2d<double> > &pts, float width);
};



#endif /* defined(__PlanarAlign__opengl_utils__) */
