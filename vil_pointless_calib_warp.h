//
//  vil_pointless_calib_warp.h
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VpglPtzOpt__vil_pointless_calib_warp__
#define __VpglPtzOpt__vil_pointless_calib_warp__

#include <vil/vil_image_view.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vcl_string.h>
#include "basketballCourt.h"
#include "SoccerCourt.h"
#include "vpgl_ptz_camera.h"

// warp court image to align captured image
class VilPointlessCalibWarp
{
private:
    vcl_string court_gradient_cach_file_;
    vcl_string image_gradient_cached_file_;
    
public:
    VilPointlessCalibWarp();
    ~VilPointlessCalibWarp();
    
    void setImageName(const char *fileName) {image_gradient_cached_file_ = vcl_string(fileName);}
    
    //iterator_num: pointless calibration iteration number
    //radius:       long gradient radius 5 - 10
    bool warpCourtToImage(BasketballCourt * court, const vil_image_view<vxl_byte> &image, int iterator_num, int radius, 
                          const vpgl_perspective_camera<double> &initCamera, vpgl_perspective_camera<double> &finalCamera);
    
    bool warpCourtToImage(SoccerCourt * court, const vil_image_view<vxl_byte> &image, int iterator_num, int radius,
                          const vpgl_perspective_camera<double> &initCamera, vpgl_perspective_camera<double> &finalCamera);
    
    bool warpCourtToImageWithLogo(DisneyWorldBasketballCourt * court, const vil_image_view<vxl_byte> &image, int iterator_num, int radius,
                                  const vpgl_perspective_camera<double> &initCamera, vpgl_perspective_camera<double> &finalCamera);
    // warp the pattern to image
    // return: SSD in magnitude
    double warp2DPatternToImage(const vil_image_view<vxl_byte> & pattern, const vil_image_view<vxl_byte> & image, int iterator_num, int radius,
                              const vnl_vector_fixed<double, 2> & initTranslate, vnl_vector_fixed<double, 2> & finalTranslate);
                    
    
    // warp source image to dest image by optimize the homography between them
    // no cach of long gradient, so do not put large image in this function
    static bool homographyWarp(const vil_image_view<vxl_byte> & source, const vil_image_view<vxl_byte> & dest, const vil_image_view<double> & wtImage,
                               int iterator_num, int radius,
                               const vgl_h_matrix_2d<double> & initHomo, vgl_h_matrix_2d<double> & finalHomo);
    
};

// PTZ camera pointless calibration
class VilPTZPointlessWarp
{
private:
    vcl_string court_gradient_cach_file_;
    vcl_string image_gradient_cached_file_;
    
public:
    VilPTZPointlessWarp();
    ~VilPTZPointlessWarp();
    
    void setImageName(const char *fileName) {image_gradient_cached_file_ = vcl_string(fileName);}
    
    //iterator_num: pointless calibration iteration number
    //radius:       long gradient radius 5 - 10
    bool warpCourtToImage(BasketballCourt * court, const vil_image_view<vxl_byte> &image,
                          int iterator_num, int radius, const vpgl_ptz_camera & initCamera, vpgl_ptz_camera & finalCamera);
    
};

// warp topview image to align the image
class VilTopviewPointlessCalibWarp
{
    vcl_string topview_gradient_cach_file_;
    vcl_string image_gradient_cached_file_;
public:
    VilTopviewPointlessCalibWarp();
    ~VilTopviewPointlessCalibWarp();
    
    void setImageName(const char *fileName) {image_gradient_cached_file_ = vcl_string(fileName);}
    
    // the initial camera should very close to ground truth
    // only for Disney World Court
    //    radius: radius for long gradient, 4-6
    // wt_radisu: radius weight image, 4-6
    bool warpTopviewToImageCalib(const vil_image_view<vxl_byte> & topview, const vil_image_view<vxl_byte> & image, int iterator_num, int radius, int wt_radius,
                                  const vpgl_perspective_camera<double> & initCamera, vpgl_perspective_camera<double> & finalCamera);
};


#endif /* defined(__VpglPtzOpt__vil_pointless_calib_warp__) */
