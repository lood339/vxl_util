//
//  vil_ellipse_tracking.h
//  OnlineStereo
//
//  Created by jimmy on 3/11/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vil_ellipse_tracking__
#define __OnlineStereo__vil_ellipse_tracking__

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_box_2d.h>
#include <vgl/vgl_ellipse_2d.h>

struct ElliseTrackingParameter
{
    vgl_box_2d<int> bounding_box_;
    double mag_threshold_;
    double lambda_;
    int min_inlier_num_;
    
    ElliseTrackingParameter()
    {
        mag_threshold_ = 0.05;
        lambda_    = 2.0;
        min_inlier_num_ = 20;
    }
    
};

class VilEllipseTracking
{
public:
    // tracking an ellipse inside a box in the image with magnitude
    static bool trackingInBoundingBox(const vil_image_view<vxl_byte> & image, // for test only
                                      const vil_image_view<double> & magnitude,
                                      const ElliseTrackingParameter & para,
                                      vgl_ellipse_2d<double> & outEllipse);
    
    
    
private:
    static bool detectEllipsePixelByAverageGradient(const vil_image_view<double> & mag,
                                             const ElliseTrackingParameter & para,
                                             vcl_vector<vgl_point_2d<double> > & outsideUpPixels,
                                             vcl_vector<vgl_point_2d<double> > & outsideDownPixels);
    
};

#endif /* defined(__OnlineStereo__vil_ellipse_tracking__) */
