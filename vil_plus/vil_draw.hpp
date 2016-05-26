//
//  vil_draw.hpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-25.
//  Copyright © 2015 jimmy. All rights reserved.
//

#ifndef vil_draw_cpp
#define vil_draw_cpp

// draw line, circle, ellipse on the image
#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_line_segment_2d.h>

class VilDraw
{
public:
    // only support thickness = 1
    static void draw_match(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                           const vcl_vector< vgl_point_2d<double> > & pts1,
                           const vcl_vector< vgl_point_2d<double> > & pts2,
                           vil_image_view<vxl_byte> &matches, const int thickness = 1);
    
    static void draw_match_vertical(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                                    const vcl_vector< vgl_point_2d<double> > & pts1,
                                    const vcl_vector< vgl_point_2d<double> > & pts2,
                                    vil_image_view<vxl_byte> &matches, const int sample_num = 1);
    
    static void draw_match_dense(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                                 const vcl_vector< vgl_point_2d<double> > & pts1,
                                 const vcl_vector< vgl_point_2d<double> > & pts2,
                                 const vcl_vector<bool> & inliers,
                                 vil_image_view<vxl_byte> &matches,
                                 const int thickness = 1);
    
    
    static void draw_cross(vil_image_view<vxl_byte> &image,
                           const vcl_vector< vgl_point_2d<double> > &pts,
                           int crossWidth,
                           const vcl_vector< vxl_byte >& colour, int lineWidth = 1);
    
    static void draw_segment(vil_image_view<vxl_byte> &image,
                             const vgl_line_segment_2d<double> & seg,
                             const vcl_vector<vxl_byte> & colour,
                             int lineWidth);
    
    
    static vcl_vector<vxl_byte> blue(void);
    static vcl_vector<vxl_byte> green(void);
    static vcl_vector<vxl_byte> red(void);
    static vcl_vector<vxl_byte> white(void);
    static vcl_vector<vxl_byte> hotPink(void);
    static vcl_vector<vxl_byte> yellow(void);
    
    
    
    
};

#endif /* vil_draw_cpp */
