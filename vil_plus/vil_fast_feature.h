//
//  vil_fast_feature.h
//  VpglPtzOpt
//
//  Created by Jimmy Chen LOCAL on 3/13/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VpglPtzOpt__vil_fast_feature__
#define __VpglPtzOpt__vil_fast_feature__

#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_vector_fixed.h>

struct FAST_feature
{
    vgl_point_2d<double> position;
    vnl_vector_fixed<int, 16> description;
};


void vil_fast_feature_point(const vil_image_view<vxl_byte> &gray_image,
                                   vcl_vector<vgl_point_2d<double> > &positive,
                                   vcl_vector<vgl_point_2d<double> > &negative,
                                   int pixel_num_threshold = 12,
                                   int intensity_threshold = 10);

void vil_fast_feature(const vil_image_view<vxl_byte> &grey_image,
                             vcl_vector<FAST_feature> & positive,
                             vcl_vector<FAST_feature> & negative,
                             int pixel_num_threshold = 12,
                             int intensity_threshold = 10);
// bruteforce matching
void vil_fast_feature_match(const vcl_vector<FAST_feature> & fast1,
                                   const vcl_vector<FAST_feature> & fast2,
                                   vcl_vector<int> &matchIndex);


vcl_vector<vgl_point_2d<double> > vil_fast_feature_point(const vil_image_view<vxl_byte> &gray_image, int threshold = 10);



#endif /* defined(__VpglPtzOpt__vil_fast_feature__) */
