//
//  vpgl_distortion_camera.h
//  OpenCVCalib
//
//  Created by jimmy on 9/10/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OpenCVCalib__vpgl_distortion_camera__
#define __OpenCVCalib__vpgl_distortion_camera__

// perspective camera with distortions
// distortion model is from opencv http://docs.opencv.org/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

#include <vpgl/vpgl_perspective_camera.h>
#include <vnl/vnl_vector_fixed.h>
#include <vil/vil_image_view.h>
#include <vcl_vector.h>

class vpgl_distortion_camera : public vpgl_perspective_camera<double>
{
protected:
    vnl_vector_fixed<double, 3> radial_;
    vnl_vector_fixed<double, 2> tangential_;  //
public:
    vpgl_distortion_camera();
    ~vpgl_distortion_camera();
    
    vnl_vector_fixed<double, 3> radial_coefficient(void) const {return radial_;}
    vnl_vector_fixed<double, 2> tangential_coefficient(void) const {return tangential_;}
    vnl_vector_fixed<double, 5> coefficient(void) const;
    
    vil_image_view<vxl_byte> undistort(const vil_image_view<vxl_byte> & image) const;
    vcl_vector<vgl_point_2d<double> > undistort(const vcl_vector<vgl_point_2d<double> > & pts)const;
    
    bool read(const char *file);
    bool write(const char *file) const;
    
};



#endif /* defined(__OpenCVCalib__vpgl_distortion_camera__) */
