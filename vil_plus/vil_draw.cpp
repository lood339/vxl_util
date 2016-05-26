//
//  vil_draw.cpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-25.
//  Copyright © 2015 jimmy. All rights reserved.
//

#include "vil_draw.hpp"
#include <vcl_algorithm.h>
#include <vil/vil_copy.h>
#include "vil_algo_plus.h"
#include "vil_util.hpp"


void VilDraw::draw_match(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                         const vcl_vector< vgl_point_2d<double> > & pts1,
                         const vcl_vector< vgl_point_2d<double> > & pts2,
                         vil_image_view<vxl_byte> &matches, const int thickness)
{
    assert(image1.nplanes() == 3 || image1.nplanes() == 1);
    assert(image2.nplanes() == 3 || image2.nplanes() == 1);
    assert(pts1.size() == pts2.size());
    assert(thickness == 1);
    
    vil_image_view<vxl_byte> rgb_image1 = image1;
    vil_image_view<vxl_byte> rgb_image2 = image2;
    if (image1.nplanes() == 1) {
        rgb_image1 = VilUtil::gray_2_rgb(image1);
    }
    if (image2.nplanes() == 1) {
        rgb_image2 = VilUtil::gray_2_rgb(image2);
    }
    
    int gap_width = 10;
    int x_shift = gap_width + image1.ni();
    int width  = image2.ni() + x_shift;
    int height = vcl_max(image1.nj(), image2.nj());
    
    matches = vil_image_view<vxl_byte>(width, height, 3);
    matches.fill(0);
    
    //copy image 1 to left
    vil_copy_to_window(rgb_image1, matches, 0, 0);
    
    //copy image 2 to right
    vil_copy_to_window(rgb_image2, matches, x_shift, 0);
    
    for (int i = 0; i<pts1.size(); i++) {
        // int r = rand()%255;
        // int g = rand()%255;
        //  int b = rand()%255;
        int r = 0;
        int g = 255;
        int b = 0;
        
        vcl_vector< vxl_byte> colour;
        colour.push_back(r);
        colour.push_back(g);
        colour.push_back(b);
        
        vgl_point_2d<double> p1 = pts1[i];
        vgl_point_2d<double> p2(vgl_point_2d<double>(pts2[i].x() + x_shift, pts2[i].y()));
        
        VilAlgoPlus::fill_line(matches, p1, p2, colour);
        
        // cross in left
        
        vgl_point_2d<double> q1, q2, q3, q4;
        double h_l = 5;
        q1.set(p1.x() - h_l, p1.y());
        q2.set(p1.x() + h_l, p1.y());
        q3.set(p1.x(), p1.y() - h_l);
        q4.set(p1.x(), p1.y() + h_l);
     
        VilAlgoPlus::fill_line(matches, q1, q2, colour);
        VilAlgoPlus::fill_line(matches, q3, q4, colour);
        
        // cross in right
        q1.set(p2.x() - h_l, p2.y());
        q2.set(p2.x() + h_l, p2.y());
        q3.set(p2.x(), p2.y() - h_l);
        q4.set(p2.x(), p2.y() + h_l);
        
        VilAlgoPlus::fill_line(matches, q1, q2, colour);
        VilAlgoPlus::fill_line(matches, q3, q4, colour);
    }
}

void VilDraw::draw_match_vertical(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                                  const vcl_vector< vgl_point_2d<double> > & pts1,
                                  const vcl_vector< vgl_point_2d<double> > & pts2,
                                  vil_image_view<vxl_byte> &matches, const int sample_num)
{
    assert(image1.nplanes() == 3 || image1.nplanes() == 1);
    assert(image2.nplanes() == 3 || image2.nplanes() == 1);
    assert(pts1.size() == pts2.size());
    
    vil_image_view<vxl_byte> rgb_image1 = image1;
    vil_image_view<vxl_byte> rgb_image2 = image2;
    if (image1.nplanes() == 1) {
        rgb_image1 = VilUtil::gray_2_rgb(image1);
    }
    if (image2.nplanes() == 1) {
        rgb_image2 = VilUtil::gray_2_rgb(image2);
    }

    
    int gap_width = 10;
    int y_shift = gap_width + image1.nj();
    int width  = vcl_max(image1.ni(), image2.ni());
    int height = image1.nj() + gap_width + image2.nj();
    
    matches = vil_image_view<vxl_byte>(width, height, 3);
    matches.fill(0);
    
    //copy image 1 to left
    vil_copy_to_window(rgb_image1, matches, 0, 0);
    
    //copy image 2 to right
    vil_copy_to_window(rgb_image2, matches, 0, y_shift);
    
    for (int i = 0; i<pts1.size(); i += sample_num) {
        int r = rand()%255;
        int g = rand()%255;
        int b = rand()%255;
        //int r = 0;
        //int g = 255;
        //int b = 0;
        
        vcl_vector< vxl_byte> colour;
        colour.push_back(r);
        colour.push_back(g);
        colour.push_back(b);
        
        vgl_point_2d<double> p1 = pts1[i];
        vgl_point_2d<double> p2(vgl_point_2d<double>(pts2[i].x(), pts2[i].y() + y_shift));
        
        VilAlgoPlus::fill_line(matches, p1, p2, colour);
        
        // cross in left
        
        vgl_point_2d<double> q1, q2, q3, q4;
        double h_l = 5;
        q1.set(p1.x() - h_l, p1.y());
        q2.set(p1.x() + h_l, p1.y());
        q3.set(p1.x(), p1.y() - h_l);
        q4.set(p1.x(), p1.y() + h_l);
        
        VilAlgoPlus::fill_line(matches, q1, q2, colour);
        VilAlgoPlus::fill_line(matches, q3, q4, colour);
        
        // cross in right
        q1.set(p2.x() - h_l, p2.y());
        q2.set(p2.x() + h_l, p2.y());
        q3.set(p2.x(), p2.y() - h_l);
        q4.set(p2.x(), p2.y() + h_l);
        
        VilAlgoPlus::fill_line(matches, q1, q2, colour);
        VilAlgoPlus::fill_line(matches, q3, q4, colour);
    }
}


void VilDraw::draw_match_dense(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                               const vcl_vector< vgl_point_2d<double> > & pts1,
                               const vcl_vector< vgl_point_2d<double> > & pts2,
                               const vcl_vector<bool> & inliers,
                               vil_image_view<vxl_byte> &matches,
                               const int thickness)
{
    assert(image1.nplanes() == 1 || image1.nplanes() == 3);
    assert(image2.nplanes() == 1 || image1.nplanes() == 3);
    assert(pts1.size() == pts2.size());
    assert(inliers.size() == pts1.size());
    
    vcl_vector<vgl_point_2d<double> > inlier1;
    vcl_vector<vgl_point_2d<double> > inlier2;
    for (int i = 0; i<inliers.size(); i++) {
        if (inliers[i]) {
            inlier1.push_back(pts1[i]);
            inlier2.push_back(pts2[i]);
        }
    }
    printf("inlier number is %lu\n", inlier1.size());    
    VilDraw::draw_match(image1, image2, inlier1, inlier2, matches, thickness);
}

void VilDraw::draw_cross(vil_image_view<vxl_byte> &image,
                         const vcl_vector< vgl_point_2d<double>> &pts,
                         int crossWidth,
                         const vcl_vector< vxl_byte >& colour, int lineWidth)
{
    assert(image.nplanes() == 3 || image.nplanes() == 1);
    assert(colour.size() == 3);
    
    vil_image_view<vxl_byte> rgb_image = image;
    if (image.nplanes() == 1) {
        image = VilUtil::gray_2_rgb(image);
    }
    
    for (unsigned int i = 0; i<pts.size(); i++)
    {
        //center point
        double px = pts[i].x();
        double py = pts[i].y();
        
        vgl_point_2d<double> p1, p2, p3, p4;
        
        double h_l = crossWidth;
        p1.set(px - h_l, py);
        p2.set(px + h_l, py);
        p3.set(px, py - h_l);
        p4.set(px, py + h_l);
        
        VilAlgoPlus::fill_line(image, p1, p2, colour);
        VilAlgoPlus::fill_line(image, p3, p4, colour);
    }
}

void VilDraw::draw_segment(vil_image_view<vxl_byte> &image,
                           const vgl_line_segment_2d<double> & seg,
                           const vcl_vector<vxl_byte> & colour,
                           int lineWidth)
{
    assert(image.nplanes() == 3 || image.nplanes() == 1);
    assert(colour.size() == 3);
    
    vil_image_view<vxl_byte> rgb_image = image;
    if (image.nplanes() == 1) {
        image = VilUtil::gray_2_rgb(image);
    }
    VilAlgoPlus::fill_line(image, seg.point1(), seg.point2(), colour);
}


vcl_vector<vxl_byte> VilDraw::blue(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(0);
    color.push_back(0);
    color.push_back(255);
    return color;
}

vcl_vector<vxl_byte> VilDraw::green(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(0);
    color.push_back(255);
    color.push_back(0);
    return color;
}

vcl_vector<vxl_byte> VilDraw::red(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(255);
    color.push_back(0);
    color.push_back(0);
    return color;
}

vcl_vector<vxl_byte> VilDraw::white(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(255);
    color.push_back(255);
    color.push_back(255);
    return color;
}

vcl_vector<vxl_byte> VilDraw::hotPink(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(255);
    color.push_back(105);
    color.push_back(180);
    return color;
}

vcl_vector<vxl_byte> VilDraw::yellow()
{
    vcl_vector<vxl_byte> color;
    color.push_back(255);
    color.push_back(255);
    color.push_back(0);
    return color;
}





