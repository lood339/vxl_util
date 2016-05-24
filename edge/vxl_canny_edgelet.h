//
//  vxl_canny_edgelet.h
//  OnlineStereo
//
//  Created by jimmy on 12/4/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_canny_edgelet__
#define __OnlineStereo__vxl_canny_edgelet__

#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_line_2d.h>
#include <vgl/vgl_vector_3d.h>
#include <vgl/vgl_point_3d.h>
#include <vnl/vnl_matrix_fixed.h>
#include <vnl/vnl_matrix.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include "edgelet_parameter.h"

// edge landmarks in monocular SLAM by Ethan Eade in Image and Vision Computing 2009


struct CannyParameter
{
    double sigma_;                   // gaussian smooth parameter for canny-like edge, first step
    double min_gradient_magnitude2_; // magnitude^2, edges
    
    CannyParameter()
    {
        sigma_ = 1.4;
        min_gradient_magnitude2_ = 200;
    }
};

class VxlCannyEdgelet
{
public:
    
    
    // edgelet in a subwindow
    static void cannyEdgelet(const vil_image_view<vxl_byte> & image,
                             const EdgeletParameter & para,
                             vil_image_view<vxl_byte> & edgeletMask,
                             vcl_vector<Edgelet> & edgelets);
    
    // grad_mag2: maginitue^2
    // edgelet detection in an image patch
    // return:
    
    static EdgeletDetectionResult edgeletDetection(const vil_image_view<vxl_byte> & grayImage,
                                                   const EdgeletParameter & para,
                                                   vcl_vector<vgl_point_2d<double> > & edgeletPts);
    
    
    
    // canny edge detection
    // edgeMask: 255 edge, 0 non-edge
    static void canny(const vil_image_view<vxl_byte> & image,
                      const CannyParameter & para,
                      vil_image_view<vxl_byte> & edgeMask);
    
    
    static void canny(const vil_image_view<vxl_byte> & image,
                      const CannyParameter & para,
                      vil_image_view<vxl_byte> & edgeMask,
                      vil_image_view<double> & maginitude2);
    
    //
    // lines: polar coordinate (r, theta)
    // rho : The resolution of the parameter r in pixels. We use 1 pixel.
    // theta: The resolution of the parameter \theta in radians. We use 1 degree (vnl_math::pi/180)
    // threshold: The minimum number of intersections to “detect” a line
    static void houghLines(const vil_image_view<vxl_byte> & edgeMask,
                           vcl_vector<vnl_vector_fixed<double, 2> > &lines,
                           double rho = 1.0, double theta = vnl_math::pi/180.0, int threshold = 20);
    
    // rho:   distance resolution in pixel. 1.0
    // theta: angle resolution in radius.   vnl_math::pi/180.0
    static void houghVoting(const vil_image_view<vxl_byte> & edgeMask,
                            vil_image_view<vxl_int_32> & accumulatedMap,
                            double rho, double theta);
    
};


class Edgelet3D
{
private:
    vgl_point_3d<double>  x_;  //edgelet center location
    vgl_vector_3d<double> d_;  //direction
    
public:
    Edgelet3D();
    Edgelet3D(const vgl_point_3d<double> & center, const vgl_vector_3d<double> & direction)
    {
        x_ = center;
        d_ = direction;
    }
    ~Edgelet3D();
    
    // project center to image
    vgl_point_2d<double> projectCenter(const vpgl_perspective_camera<double> & camera) const;
    // project direction to image
    vgl_vector_2d<double> projectDirection(const vpgl_perspective_camera<double> & camera) const;
    
    
    vgl_point_3d<double> center()const     {return x_;}
    vgl_vector_3d<double> direction()const {return d_;}
    
    // jacobian matrix with (2D) position, dx dd (0), dT, dR
    vnl_matrix_fixed<double, 2, 12> jacobian_x(const vgl_rotation_3d<double> & R, const vgl_vector_3d<double> & T);
    
    // jacobian matrix with (2D) direction, dx dd, dT, dR
    vnl_matrix_fixed<double, 2, 12> jacobian_d(const vgl_rotation_3d<double> & R, const vgl_vector_3d<double> & T);
};





#endif /* defined(__OnlineStereo__vxl_canny_edgelet__) */
