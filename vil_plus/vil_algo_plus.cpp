//
//  vil_algo_plus.cpp
//  QuadCopter
//
//  Created by jimmy on 7/9/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_algo_plus.h"
#include "vnl_plus.h"
#include <vil/vil_flip.h>
#include <vil/vil_fill.h>
#include <vil/vil_plane.h>

// for test
//#include "vil_plus.h"


int VilAlgoPlus::cannyDirection(double dx, double dy)
{
    int dir = 0;
    if(VnlPlus::isEqualZero(dx))
    {
        if(VnlPlus::isEqualZero(dy))
        {
            dir = 0;
        }
        else
        {
            dir = 2;
        }
    }
    else
    {
        // 0.4142 --> 22.5^o, 2.4142--> 657.5^o
        double tanV = dy/dx;
        if(tanV > -0.4142f && tanV <= 0.4142f)
        {
            dir = 0;
        }
        else if(tanV > 0.4142f && tanV < 2.4142f)
        {
            dir = 1;
        }
        else if(fabs(tanV) >= 2.4142f)
        {
            dir = 2;
        }
        else if(tanV > -2.4142f && tanV <= -0.4142f)
        {
            dir = 3;
        }
    }
    return dir;
}

vgl_vector_2d<int> VilAlgoPlus::scanDirectionFromOctaveIndex(int oct_index)
{
    assert(oct_index >= 0 && oct_index < 8);
    // direction is perpandicular to the canny direction
    //   3 2 1
    //   4 * 0
    //   5 6 7
    
    // (dx, dy)
    int scanDirection[][2] = { 1, 0,
                               1, -1, 0, -1, -1, -1,
                               -1, 0,
                               -1, 1, 0, 1, 1, 1};
    assert(sizeof(scanDirection)/sizeof(scanDirection[0][0]) == 8 * 2);
    return vgl_vector_2d<int>(scanDirection[oct_index][0], scanDirection[oct_index][1]);
}

vgl_vector_2d<int> VilAlgoPlus::scanDirection(const vgl_point_2d<double> & fromPt, const vgl_point_2d<double> & toPt)
{
    double dx = toPt.x() - fromPt.x();
    double dy = toPt.y() - fromPt.y();
    int index = VilAlgoPlus::octaveIndex(dx, dy);
    return VilAlgoPlus::scanDirectionFromOctaveIndex(index);
}

int VilAlgoPlus::octaveIndex(double dx, double dy)
{
    int dir = 0;
    
    // printf("dx dy is %f %f\n", dx, dy);
    // horizontal
    if(VnlPlus::isEqualZero(dx))
    {
        if (dy >= 0) {
            dir = 6;
        }
        else
        {
            dir = 2;
        }
    }
    else if(VnlPlus::isEqualZero(dy))
    {
        if (dx >= 0) {
            dir = 0;
        }
        else
        {
            dir = 4;
        }
    }
    else
    {
        double tanV = dy/dx;
        //    printf("tan v is %f\n", tanV);
        if(tanV > -0.4142f && tanV <= 0.4142f)
        {
            if (dx > 0) {
                dir = 0;
            }
            else
            {
                dir = 4;
            }
        }
        else if(tanV > 0.4142f && tanV < 2.4142f)
        {
            if (dx < 0) {
                dir = 3;
            }
            else
            {
                dir = 7;
            }
        }
        else if(fabs(tanV) >= 2.4142f)
        {
            if (dy < 0) {
                dir = 2;
            }
            else
            {
                dir = 6;
            }
        }
        else if(tanV > -2.4142f && tanV <= -0.4142f)
        {
            if (dx > 0) {
                dir = 1;
            }
            else
            {
                dir = 5;
            }
        }
    }
    //   printf("tan dir is %d\n\n", dir);
    
    return dir;
}



bool VilAlgoPlus::linePixels(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts, int w, int h)
{
    vcl_vector<vgl_point_2d<double> > pts;
    VilAlgoPlus::linePixels(p0, p1, pts);
    for (int i = 0; i<pts.size(); i++) {
        vgl_point_2d<double> p = pts[i];
        if (p.x() >= 0 && p.x() < w && p.y() >= 0 && p.y() < h) {
            linePts.push_back(p);
        }
    }
    return true;
}

bool VilAlgoPlus::linearInterpolateFromCenter(int imageW, int imageH, vil_image_view<double> & wtImage)
{
    if (imageW%2 != 0 || imageH%2 != 0) {
        printf("Error: linearInterpolateFromCenter failed. The width, height of image must be 4x.\n");
        return false;
    }
    // left top patch of the image
    int w = imageW/2;
    int h = imageH/2;
    vil_image_view<double> patch = vil_image_view<double>(w, h, 1);
    patch.fill(0.0);
    patch(w-1, h-1) = 1.0;
    // right side linear
    for (int j = 1; j<h-1; j++) {
        patch(w-1, j) = 1.0 * j / h;
    }
    // bottom side linear
    for (int i = 1; i<w-1; i++) {
        patch(i, h-1) = 1.0 * i / w;
    }
    
    // bilinear interpolate inside
    for (int j = 1; j<h-1; j++) {
        for (int i = 1; i<w-1; i++) {
            double w1 = 1.0 * (h-j)/h * 0.0 + 1.0*j/h * patch(i, h-1);
            double w2 = 1.0 * (w-i)/w * 0.0 + 1.0*i/w * patch(w-1, j);
            patch(i, j) = (w1 + w2)/2.0;
        }
    }
    
    wtImage = vil_image_view<double>(imageW, imageH, 1);
    wtImage.fill(0.0);
    // left top patch
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            wtImage(i, j) = patch(i, j);
        }
    }
    
    vil_image_view<double> lrPatch = vil_flip_lr(patch);
    // right top patch
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            wtImage(i+w, j) = lrPatch(i, j);
        }
    }
    // left bottom
    vil_image_view<double> upPatch = vil_flip_ud(patch);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            wtImage(i, j+h) = upPatch(i, j);
        }
    }
    // right bottom
    vil_image_view<double> lrupPatch = vil_flip_ud(lrPatch);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            wtImage(i+w, j+h) = lrupPatch(i, j);
        }
    }
    return true;
}


// http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
// Bresenham's line algorithm
bool VilAlgoPlus::linePixels(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts)
{
    /*
     function line(x0, x1, y0, y1)
     real deltax := x1 - x0
     real deltay := y1 - y0
     real error := 0
     real deltaerr := abs (deltay / deltax)    // Assume deltax != 0 (line is not vertical),
     // note that this division needs to be done in a way that preserves the fractional part
     int y := y0
     for x from x0 to x1
     plot(x,y)
     error := error + deltaerr
     while error ≥ 0.5 then
     plot(x, y)
     y := y + sign(y1 - y0)
     error := error - 1.0
     */
    double x0 = p0.x();
    double y0 = p0.y();
    double x1 = p1.x();
    double y1 = p1.y();
    if (p0.x() > p1.x()) {
        vcl_swap(x0, x1);
        vcl_swap(y0, y1);
    }
    
    double deltaX = x1 - x0;
    double deltaY = y1 - y0;
    double error = 0;
    if (fabs(deltaX) < 0.5) {
        // vertical line
        if (y0 > y1) {
            vcl_swap(y0, y1);
        }
        int x = (x0 + x1)/2.0;
        for (int y = y0; y <= y1; y++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
        }
    }
    else if(fabs(deltaY) < 0.5)
    {
        // horizontal line
        int y = (y0 + y1)/2.0;
        for (int x = x0; x <= x1; x++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
        }
    }
    else
    {
        double deltaErr = fabs(deltaY/deltaX);
        int y = (int)y0;
        int sign = y1 > y0 ? 1:-1;
        for (int x = x0; x <= x1; x++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
            error += deltaErr;
            while (error >= 0.5) {
                linePts.push_back(vgl_point_2d<double>(x, y));  // may have duplicated (x, y)
                y += sign;
                error -= 1.0;
            }
        }
    }
    return true;
}

bool VilAlgoPlus::fill_line(vil_image_view<vxl_byte> & image,
                            const vgl_point_2d<double> & p1,
                            const vgl_point_2d<double> & p2,
                            const vcl_vector<vxl_byte> & colour)
{
    int ai = p1.x();
    int aj = p1.y();
    int bi = p2.x();
    int bj = p2.y();
    
    for (int i = 0; i<image.nplanes(); i++) {
        vil_image_view<vxl_byte> plane = vil_plane(image, i);
        vil_fill_line(plane, ai, aj, bi, bj, colour[i]);
    }
    
    return true;
}


