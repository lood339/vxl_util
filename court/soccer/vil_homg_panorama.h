//
//  vil_homg_panorama.h
//  OnlineStereo
//
//  Created by jimmy on 8/11/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vil_homg_panorama__
#define __OnlineStereo__vil_homg_panorama__

// panorama image from homography mapping, assume the cameras is
// approximately located in the same place
#include <vil/vil_image_view.h>
#include <vpgl/vpgl_perspective_camera.h>

// only for two image
// only for WWoS soccer game
class vil_homg_panorama
{
    vil_image_view<vxl_byte> leftImage_;
    vil_image_view<vxl_byte> rightImage_;
    vpgl_perspective_camera<double> leftCamera_;
    vpgl_perspective_camera<double> rightCamera_;
    vpgl_perspective_camera<double> midCamera_;
    
    // for speed up
    vil_image_view<double> leftWtImage_;  // weight map of warped left image
    vil_image_view<double> rightWtImage_;
    vgl_h_matrix_2d<double> leftH_;       // left camera to middle camera homography from soccer field
    vgl_h_matrix_2d<double> rightH_;
public:
    vil_homg_panorama();
    ~vil_homg_panorama();
    void setPanoramaSouce(const vpgl_perspective_camera<double> & leftCamera, const vpgl_perspective_camera<double> & rightCamera,
                          const vil_image_view<vxl_byte> & leftImage, const vil_image_view<vxl_byte> & rightImage);
    
    void setCameras(const vpgl_perspective_camera<double> & leftCamera, const vpgl_perspective_camera<double> & rightCamera);
    bool calculateWeightImage(int imageW, int imageH);
    
    // run calculateWeightImage before run this function
    bool getPanorama_cacheWeightImage(const vil_image_view<vxl_byte> & leftImage, const vil_image_view<vxl_byte> & rightImage,
                                      vil_image_view<vxl_byte> & panorama);
    
    bool getPanorama(const vil_image_view<vxl_byte> & leftImage, const vil_image_view<vxl_byte> & rightImage,
                     vil_image_view<vxl_byte> & panorama);
    
    
    vil_image_view<vxl_byte> projectToVirtualCamera(const vpgl_perspective_camera<double> & camera,
                                                    int imageW, int imageH);
    
    
    // homograpy warp from image1 to image2
    static vil_image_view<vxl_byte> projectToVirtualCamera(const vpgl_perspective_camera<double> & camera1,
                                                           const vil_image_view<vxl_byte> & image1,
                                                           const vpgl_perspective_camera<double> & camera2,
                                                           int imageW, int imageH);
    
};

#endif /* defined(__OnlineStereo__vil_homg_panorama__) */
