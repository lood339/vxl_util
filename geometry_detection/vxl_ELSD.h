//
//  vxl_ELSD.h
//  OnlineStereo
//
//  Created by jimmy on 8/26/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_ELSD__
#define __OnlineStereo__vxl_ELSD__

#include <vil/vil_image_view.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_circle_2d.h>
#include <vgl/vgl_ellipse_2d.h>
#include <vcl_vector.h>

// http://ubee.enseeiht.fr/vision/ELSD/
// Ellipse and Line Segment Detector

struct ELSDLineEllipseParameter
{
    int ellipse_min_pixel_;   // minimum number of pixels on the ellipse
    int line_min_length_;     // minimum length of line segment
    
    ELSDLineEllipseParameter()
    {
        ellipse_min_pixel_ = 100;
        line_min_length_ = 50;
    }
};

struct ELSDResult
{
    vcl_vector<vgl_line_segment_2d<double> > lines_;
    vcl_vector<vcl_vector<vgl_point_2d<double > > > line_points_;
    vcl_vector<vgl_ellipse_2d<double> > ellipses_;
    vcl_vector<vcl_vector<vgl_point_2d<double> > > ellipse_points_;
};


class VxlELSD
{
public:
    // only return lines and ellipse.
    // circles for future
    static void detectLineCirleEllipse(const vil_image_view<vxl_byte> & image,
                                       const ELSDLineEllipseParameter & para,
                                       ELSDResult & elsd);
    // only detect the largest ellipse
    // For example, some part of the image has high probability of ellipse
    // defualt ELSDLineEllipseParameter parameter
    static bool detect_largest_ellipse(const vil_image_view<vxl_byte> & image,
                                       vgl_ellipse_2d<double> & ellipse,
                                       vcl_vector<vgl_point_2d<double> > & ellipse_points);
};



#endif /* defined(__OnlineStereo__cvx_ELSD__) */
