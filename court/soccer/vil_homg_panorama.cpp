//
//  vil_homg_panorama.cpp
//  OnlineStereo
//
//  Created by jimmy on 8/11/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_homg_panorama.h"
#include "vpgl_plus.h"
#include "vil_plus.h"
#include "wwosSoccerCourt.h"
#include "vil_algo_plus.h"
#include "vil_homo_warp.h"

vil_homg_panorama::vil_homg_panorama()
{
    
}
vil_homg_panorama::~vil_homg_panorama()
{
    
}
void vil_homg_panorama::setPanoramaSouce(const vpgl_perspective_camera<double> & leftCamera, const vpgl_perspective_camera<double> & rightCamera,
                         const vil_image_view<vxl_byte> & leftImage, const vil_image_view<vxl_byte> & rightImage)
{
    leftImage_.deep_copy(leftImage);
    rightImage_.deep_copy(rightImage);
    
    leftCamera_  = leftCamera;
    rightCamera_ = rightCamera;
    
    vgl_point_3d<double> cl = leftCamera_.get_camera_center();
    vgl_point_3d<double> cr = rightCamera_.get_camera_center();
    double dx = cl.x() - cr.x();
    double dy = cl.y() - cr.y();
    double dz = cl.z() - cr.z();
    printf("camera center displacement is %f %f %f\n", dx, dy, dz);
}

void vil_homg_panorama::setCameras(const vpgl_perspective_camera<double> & leftCamera, const vpgl_perspective_camera<double> & rightCamera)
{
    leftCamera_  = leftCamera;
    rightCamera_ = rightCamera;
    
    vnl_quaternion<double> qt1 = leftCamera_.get_rotation().as_quaternion();
    vnl_quaternion<double> qt2 = rightCamera_.get_rotation().as_quaternion();
    vnl_quaternion<double> qt_mean = (qt1 + qt2).normalize();
    
    double fl = (leftCamera.get_calibration().focal_length() + rightCamera.get_calibration().focal_length())/2.0;
    vpgl_calibration_matrix<double> K(fl, leftCamera.get_calibration().principal_point());
    vgl_point_3d<double> cc1 = leftCamera.get_camera_center();
    vgl_point_3d<double> cc2 = rightCamera.get_camera_center();
    vgl_point_3d<double> cc = centre(cc1, cc2);
    vgl_rotation_3d<double> R(qt_mean);
    midCamera_.set_calibration(K);
    midCamera_.set_rotation(R);
    midCamera_.set_camera_center(cc);
}

bool vil_homg_panorama::calculateWeightImage(int imageW, int imageH)
{
    vil_image_view<double> wtImage;
    VilAlgoPlus::linearInterpolateFromCenter(imageW, imageH, wtImage);
    
    WWoSSoccerCourt soccerCourt;
    bool isH = soccerCourt.getRelativeHomography(leftCamera_, midCamera_, imageW, imageH, leftH_);
    if (!isH) {
        return false;
    }
    VilHomoWarp::homography_wap_shift_origin<double>(wtImage, leftH_, vgl_point_2d<double>(0, 0), vgl_point_2d<double>(imageW*1.5, 0),
                                                     imageW*2.5, imageH, leftWtImage_);
    
    isH = soccerCourt.getRelativeHomography(rightCamera_, midCamera_, imageW, imageH, rightH_);
    if (!isH) {
        return false;
    }
    VilHomoWarp::homography_warp<double>(wtImage, rightH_, imageW*2, imageH, rightWtImage_);
    return true;
}

bool vil_homg_panorama::getPanorama_cacheWeightImage(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2,
                                                     vil_image_view<vxl_byte> & panorama)
{
    // hard code parameters
    assert(image1.nplanes() == image2.nplanes());
    assert(image1.ni() == image2.ni());
    assert(image1.nj() == image2.nj());
    
    vpgl_perspective_camera<double> camera1 = leftCamera_;
    vpgl_perspective_camera<double> camera2 = rightCamera_;
    vpgl_perspective_camera<double> mid_camera = midCamera_;
    
    int w = image1.ni();
    int h = image1.nj();
    
    vil_image_view<vxl_byte> leftWarpImage;
    VilHomoWarp::homography_wap_shift_origin<vxl_byte>(image1, leftH_, vgl_point_2d<double>(0, 0), vgl_point_2d<double>(w*1.5, 0), w*2.5, h, leftWarpImage);
    vil_image_view<vxl_byte> rightWarpImage;
    VilHomoWarp::homography_warp<vxl_byte>(image2, rightH_, w*2, h, rightWarpImage);
    
    // combine left and right warp
    int shift_left = 1.5 * w;
    panorama = vil_image_view<vxl_byte>(3.5*w, h, 3);
    panorama.fill(0);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++)
        {
            double w1 = leftWtImage_(i + shift_left, j);
            double w2 = rightWtImage_(i, j);
            if (w1 + w2 != 0.0) {
                double w3 = w1/(w1 + w2);
                double w4 = w2/(w1 + w2);
                panorama(i+shift_left, j, 0) = leftWarpImage(i+shift_left, j, 0) * w3 + rightWarpImage(i, j, 0) * w4;
                panorama(i+shift_left, j, 1) = leftWarpImage(i+shift_left, j, 1) * w3 + rightWarpImage(i, j, 1) * w4;
                panorama(i+shift_left, j, 2) = leftWarpImage(i+shift_left, j, 2) * w3 + rightWarpImage(i, j, 2) * w4;
            }
        }
    }
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<shift_left; i++) {
            panorama(i, j, 0) = leftWarpImage(i, j, 0);
            panorama(i, j, 1) = leftWarpImage(i, j, 1);
            panorama(i, j, 2) = leftWarpImage(i, j, 2);
        }
    }
    
    for(int j = 0; j<h; j++)
    {
        for (int i = 0; i<w; i++) {
            panorama(i+2.5*w, j, 0) = rightWarpImage(i+w, j, 0);
            panorama(i+2.5*w, j, 1) = rightWarpImage(i+w, j, 1);
            panorama(i+2.5*w, j, 2) = rightWarpImage(i+w, j, 2);
        }
    }
    return true;
}

bool vil_homg_panorama::getPanorama(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2,
                                    vil_image_view<vxl_byte> & panorama)
{
    // hard code parameters
    assert(image1.nplanes() == image2.nplanes());
    assert(image1.ni() == image2.ni());
    assert(image1.nj() == image2.nj());
    
    vpgl_perspective_camera<double> camera1 = leftCamera_;
    vpgl_perspective_camera<double> camera2 = rightCamera_;
    vpgl_perspective_camera<double> mid_camera = midCamera_;
    
    int w = image1.ni();
    int h = image1.nj();    
    vil_image_view<double> wtImage;
    VilAlgoPlus::linearInterpolateFromCenter(image1.ni(), image1.nj(), wtImage);
    
    WWoSSoccerCourt soccerCourt;
    vgl_h_matrix_2d<double> H1;
    bool isH = soccerCourt.getRelativeHomography(camera1, mid_camera, w, h, H1);
    assert(isH);
    vil_image_view<vxl_byte> leftWarpImage;
    VilHomoWarp::homography_wap_shift_origin<vxl_byte>(image1, H1, vgl_point_2d<double>(0, 0), vgl_point_2d<double>(w*1.5, 0), w*2.5, h, leftWarpImage);
    vil_image_view<double> leftWtImage;
    VilHomoWarp::homography_wap_shift_origin<double>(wtImage, H1, vgl_point_2d<double>(0, 0), vgl_point_2d<double>(w*1.5, 0), w*2.5, h, leftWtImage);
    
    vgl_h_matrix_2d<double> H2;
    isH = soccerCourt.getRelativeHomography(camera2, mid_camera, w, h, H2);
    assert(isH);
    vil_image_view<vxl_byte> rightWarpImage;
    VilHomoWarp::homography_warp<vxl_byte>(image2, H2, w*2, h, rightWarpImage);
    vil_image_view<double> rightWtImage;
    VilHomoWarp::homography_warp<double>(wtImage, H2, w*2, h, rightWtImage);
    
    // combine left and right warp
    int shift_left = 1.5 * w;
    panorama = vil_image_view<vxl_byte>(3.5*w, h, 3);
    panorama.fill(0);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++)
        {
            double w1 = leftWtImage(i + shift_left, j);
            double w2 = rightWtImage(i, j);
            if (w1 + w2 != 0.0) {
                double w3 = w1/(w1 + w2);
                double w4 = w2/(w1 + w2);
                panorama(i+shift_left, j, 0) = leftWarpImage(i+shift_left, j, 0) * w3 + rightWarpImage(i, j, 0) * w4;
                panorama(i+shift_left, j, 1) = leftWarpImage(i+shift_left, j, 1) * w3 + rightWarpImage(i, j, 1) * w4;
                panorama(i+shift_left, j, 2) = leftWarpImage(i+shift_left, j, 2) * w3 + rightWarpImage(i, j, 2) * w4;
            }
        }
    }
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<shift_left; i++) {
            panorama(i, j, 0) = leftWarpImage(i, j, 0);
            panorama(i, j, 1) = leftWarpImage(i, j, 1);
            panorama(i, j, 2) = leftWarpImage(i, j, 2);
        }
    }
    
    for(int j = 0; j<h; j++)
    {
        for (int i = 0; i<w; i++) {
            panorama(i+2.5*w, j, 0) = rightWarpImage(i+w, j, 0);
            panorama(i+2.5*w, j, 1) = rightWarpImage(i+w, j, 1);
            panorama(i+2.5*w, j, 2) = rightWarpImage(i+w, j, 2);
        }
    }

    return true;
}

vil_image_view<vxl_byte> vil_homg_panorama::projectToVirtualCamera(const vpgl_perspective_camera<double> & camera,
                                                int imageW, int imageH)
{
    assert(leftImage_.nplanes()  == 3);
    assert(rightImage_.nplanes() == 3);
    vil_image_view<vxl_byte> retImage = vil_image_view<vxl_byte>(imageW, imageH, 3);
    retImage.fill(0);
    
    // warp from left  camera
    vgl_h_matrix_2d<double> leftH = VpglPlus::homographyFromCameraToCamera(leftCamera_, camera);
    vil_image_view<vxl_byte> warpedLeftImage;
    warpedLeftImage.deep_copy(retImage);
    vil_image_view<vxl_byte> leftAlpha;
    VilPlus::homography_warp_fill(leftImage_, leftH, retImage, warpedLeftImage, leftAlpha);
    
    // warp from right camera
    vgl_h_matrix_2d<double> rightH = VpglPlus::homographyFromCameraToCamera(rightCamera_, camera);
    vil_image_view<vxl_byte> warpedRightImage;
    warpedRightImage.deep_copy(retImage);
    vil_image_view<vxl_byte> rightAlpha;
    VilPlus::homography_warp_fill(rightImage_, rightH, retImage, warpedRightImage, rightAlpha);
 
    // simple average of there are multiple bans
    for (int j = 0; j<retImage.nj(); j++) {
        for (int i = 0; i<retImage.ni(); i++) {
            if (leftAlpha(i, j) == 255 && rightAlpha(i, j) == 255) {
                retImage(i, j, 0) = warpedLeftImage(i, j, 0)/2 + warpedRightImage(i, j, 0)/2;
                retImage(i, j, 1) = warpedLeftImage(i, j, 1)/2 + warpedRightImage(i, j, 1)/2;
                retImage(i, j, 2) = warpedLeftImage(i, j, 2)/2 + warpedRightImage(i, j, 2)/2;              
            }
            else if(leftAlpha(i, j) == 255)
            {
                retImage(i, j, 0) = warpedLeftImage(i, j, 0);
                retImage(i, j, 1) = warpedLeftImage(i, j, 1);
                retImage(i, j, 2) = warpedLeftImage(i, j, 2);
            }
            else if(rightAlpha(i, j) == 255)
            {
                retImage(i, j, 0) = warpedRightImage(i, j, 0);
                retImage(i, j, 1) = warpedRightImage(i, j, 1);
                retImage(i, j, 2) = warpedRightImage(i, j, 2);
            }
        }
    }
    return retImage;
}

vil_image_view<vxl_byte> vil_homg_panorama::projectToVirtualCamera(const vpgl_perspective_camera<double> & camera1,
                                                                   const vil_image_view<vxl_byte> & image1,
                                                                   const vpgl_perspective_camera<double> & camera2,
                                                                   int imageW, int imageH)
{
    vgl_h_matrix_2d<double> H = VpglPlus::homographyFromCameraToCamera(camera1, camera2);
    vil_image_view<vxl_byte> image(imageW, imageH, 3);
    image.fill(0);
    vil_image_view<vxl_byte> warpedImage;
    warpedImage.deep_copy(image);
    VilPlus::homography_warp_fill(image1, H, image, warpedImage);
    return warpedImage;
}






