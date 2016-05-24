//
//  vxl_half_edge_line_tracking.h
//  OpenCVCalib
//
//  Created by jimmy on 9/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OpenCVCalib__vxl_half_edge_line_tracking__
#define __OpenCVCalib__vxl_half_edge_line_tracking__

// half edge line tracking
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_line_2d.h>
#include <vil/vil_image_view.h>
#include "vil_line_tracking.h"
#include "SoccerLineParameters.h"

class vxl_half_edge_line_tracking
{
public:
    // assume lineSeg is very close to real line in the image.
    // assume the line is wider in the image
    static bool trackingLineFromSegment(const vil_image_view<vxl_byte> & image, // for test only
                                        const vil_image_view<double> & magnitude,
                                        const vil_image_view<double> & grad_i,
                                        const vil_image_view<double> & grad_j,
                                        const vgl_line_segment_2d<double> & lineSeg,
                                        const LinesegmntTrackingParameter & para, vgl_line_2d<double> & line, vgl_line_segment_2d<double> & cenerLineSeg);
    
    // estimate a center line from two points sets. These point sets are from double edge of a thick line
    // assume all points are in-liers
    static bool estimateCenterLine(const vcl_vector<vgl_point_2d<double> > & line1Pts, const vcl_vector<vgl_point_2d<double> > & line2Pts,
                                   vcl_vector<vgl_point_2d<double> > & centerPts, vgl_line_2d<double> & centerLine);
    
    // double edge line detection
    static bool detectLinePixelByAverageGradient(const vil_image_view<double> & mag, const vil_image_view<vxl_byte> & maskImage,
                                                 const AverageMagnitudeParameter & para, vcl_vector<vgl_point_2d<double> > & edge1Pixels,
                                                 vcl_vector<vgl_point_2d<double> > & edge2Pixels);
    
    // double edge ellipse detection
    // outsideUpPixels: up to down in image space, outsideDownPixels: opposite direction
    static bool detectEllipsePixelByAverageGradient(const vil_image_view<double> & mag,
                                                    const vil_image_view<vxl_byte> & mask,
                                                    const vil_image_view<vxl_byte> & nonEllipseMask,
                                                    const AverageMagnitudeParameter & para,
                                                    vcl_vector<vgl_point_2d<double> > & outsideUpPixels,
                                                    vcl_vector<vgl_point_2d<double> > & outsideDownPixels);
    
    // outsideHorizontalPixels and outsideVerticalPixels may have same pixel position
    static bool detectLeftPenaltyEllipseByAverageGradient(const vil_image_view<double> & mag,
                                                          const vil_image_view<vxl_byte> & mask,
                                                          const vil_image_view<vxl_byte> & nonEllipseMask,
                                                          const AverageMagnitudeParameter & para,
                                                          vcl_vector<vgl_point_2d<double> > & outsideHorizontalPixels,
                                                          vcl_vector<vgl_point_2d<double> > & outsideVerticalPixels);
    
    static bool detectRightPenaltyEllipseByAverageGradient(const vil_image_view<double> & mag,
                                                           const vil_image_view<vxl_byte> & mask,
                                                           const vil_image_view<vxl_byte> & nonEllipseMask,
                                                           const AverageMagnitudeParameter & para,
                                                           vcl_vector<vgl_point_2d<double> > & outsideHorizontalPixels,
                                                           vcl_vector<vgl_point_2d<double> > & outsideVerticalPixels);
    
};

#endif /* defined(__OpenCVCalib__vxl_half_edge_line_tracking__) */
