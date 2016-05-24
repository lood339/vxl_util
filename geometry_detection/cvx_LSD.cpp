//
//  cvx_LSD.cpp
//  QuadCopter
//
//  Created by jimmy on 6/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "cvx_LSD.h"

#include "lsd.h"
#include "vil_plus.h"
#include "vgl_plus.h"
#include <vil/vil_convert.h>
#include <vgl/vgl_distance.h>

void CvxLSD::detect_lines(const vil_image_view<vxl_byte> & image, vcl_vector<LSDLineSegment> & line_segments)
{
    assert(image.nplanes() == 3);
    
    vil_image_view<vxl_byte> grayImage;
    vil_convert_planes_to_grey(image, grayImage);
   
    
    double * imageData = NULL;
    double * out = NULL;
    int width  = grayImage.ni();
    int height = grayImage.nj();
    int n = 0;
    imageData = (double *) malloc( width * height * sizeof(double) );
    assert(imageData);
    for(int j=0; j<height; j++)
    {
        for(int i=0; i<width; i++)
        {
            imageData[i + j * width] = grayImage(i, j);
        }
    }
    
    /* LSD call */
    out = lsd(&n, imageData, width, height);
    
    /* print output */
   // printf("%d line segments found:\n",n);
    line_segments.resize(n);
    for(int i=0; i<n; i++)
    {
        double x1 = out[7*i + 0];
        double y1 = out[7*i + 1];
        double x2 = out[7*i + 2];
        double y2 = out[7*i + 3];
        double line_width = out[7*i + 4];
        double p   = out[7*i + 5];
        double tmp = out[7*i + 6];
        
        line_segments[i].seg_ = vgl_line_segment_2d<double>(vgl_point_2d<double>(x1, y1), vgl_point_2d<double>(x2, y2));
        line_segments[i].width_ = line_width;
        line_segments[i].angle_precision_ = p;
        line_segments[i].NFA_ = tmp;
    }
    free( (void *) imageData );
    free( (void *) out );
}

bool CvxLSD::detect_longest_line(const vil_image_view<vxl_byte> & image, vgl_line_segment_2d<double> & line_seg)
{
    vcl_vector<LSDLineSegment> line_segments;
    CvxLSD::detect_lines(image, line_segments);
    if (line_segments.size() == 0) {
        return false;
    }
    double longest_dis = 0.0;
    for (int i = 0; i<line_segments.size(); i++) {
        double dis = vgl_distance(line_segments[i].seg_.point1(), line_segments[i].seg_.point2());
        if (dis > longest_dis) {
            longest_dis = dis;
            line_seg = line_segments[i].seg_;
        }
    }
    return true;
}


