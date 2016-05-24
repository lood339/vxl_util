//
//  vxl_hough_line.h
//  OnlineStereo
//
//  Created by jimmy on 2/16/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_hough_line__
#define __OnlineStereo__vxl_hough_line__

#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vgl/vgl_line_2d.h>

struct VxlHoughParameter
{
    float rhoResolution_   ;    // hough accumulation resolution
    float thetaResolution_ ;
    double houghThreshold_ ;    // hough threshold
    
    double lineWidth_;     // line width in image space
    int maxLineNum_;       // line numbers in one image
    int minPixeNum_;       // minimum number in image
    double inlierDistance_;
    VxlHoughParameter()
    {
        rhoResolution_   = 1.0;
        thetaResolution_ = 1.0;
        houghThreshold_  = 0.8;
        lineWidth_ = 3.0;
        maxLineNum_ = 6;
        minPixeNum_ = 100;
        inlierDistance_ = 1.0;
    }
};

class VxlHoughLine
{
public:
    // detect all lines in the image using hough transform
    // remove pixels that belong to the previous detected line
    static int oneByOneLineDetection(const vcl_vector<vgl_point_2d<double> > & pts, int imageW, int imageH,
                                     const VxlHoughParameter & para, vcl_vector<vgl_line_2d<double> > & lines);
    
    //  detection lines one by one: remove edge maks of previous detected lines
    static int oneByOneLineDetection(const vil_image_view<vxl_byte> & edgeMask, const VxlHoughParameter & para,
                                     vcl_vector<vgl_line_2d<double> > & lines);
    
    // detect one line and record inlier points in 1 pixel distance
    static bool detectOneLine(const vcl_vector<vgl_point_2d<double> > & pts, int imageW, int imageH,
                              const VxlHoughParameter & para, vgl_line_2d<double> & line,
                              vcl_vector<vgl_point_2d<double> > & inlierPts);
    
    
    
};

#endif /* defined(__OnlineStereo__vxl_hough_line__) */
