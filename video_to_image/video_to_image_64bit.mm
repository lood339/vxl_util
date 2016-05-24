//
//  video_to_image_64bit.cpp
//  VideoCalibration
//
//  Created by jimmy on 8/29/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//
#import <VisionKit/VKHomography.h>

#include "video_to_image_64bit.h"
#include <vil/vil_osx.h>
#include <vpgl/vpgl_perspective_camera.h>
#include "basketballCourt.h"
#include "vpgl_ptz_model_estimation.h"

bool video_to_image_64bit::grabFrameFromVideo(const MMCVPixelBufferReaderAVFoundation *videoReader, int frame_num, double frameRate, vil_image_view<vxl_byte> & image)
{
  //  NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    
    MMTime curTime;
    curTime = MMMakeTime(frame_num, frameRate);
    CVPixelBufferRef buf = [videoReader pixelBufferForTime:curTime];
    
    if (!buf) {
     //   [pool drain];
        return false;
    }
    image = vil_image_view_from_CVPixelBuffer(buf, true);
 //   [pool drain];
    [buf release];
    return true;
}

bool video_to_image_64bit::grabFrameFromVideo(const MMCVPixelBufferReaderAVFoundation *videoReader, int frame_num, double frameRate, CVPixelBufferRef & buf)
{
    MMTime curTime;
    curTime = MMMakeTime(frame_num, frameRate);
    buf = [videoReader pixelBufferForTime:curTime];
    return buf != nil;
}


void video_to_image_64bit::camera_to_VKHomography(const vpgl_perspective_camera<double> & camera,  VKHomography * & vkhomo)
{
    const int width  = camera.get_calibration().principal_point().x() * 2;
    const int height = camera.get_calibration().principal_point().y() * 2;
    
    vcl_vector<vgl_point_2d<double> > pts_world;
    vcl_vector<vgl_point_2d<double> > pts_image;
    DisneyWorldBasketballCourt::projectCourtPoints(camera, width, height, pts_world, pts_image);
    assert(pts_world.size() >= 4);
    assert(pts_image.size() >= 4);
    
    vcl_vector<vgl_homg_point_2d<double> > hpts1;
    vcl_vector<vgl_homg_point_2d<double> > hpts2;
    for (int i = 0; i<pts_world.size(); i++) {
        hpts1.push_back(vgl_homg_point_2d<double>(pts_world[i]));
        hpts2.push_back(vgl_homg_point_2d<double>(pts_image[i]));
    }
    vgl_h_matrix_2d<double> Hvgl = vgl_h_matrix_2d<double>(hpts1, hpts2);
    
    
    CGFloat Hvalues[9] = {0};
    for (int r = 0; r<3; r++) {
        for (int c = 0; c<3; c++) {
            Hvalues[r * 3 + c] = Hvgl.get(r, c);
        }
    }    
    
    vkhomo = [[VKHomography alloc] initWithValues: Hvalues];
}

void video_to_image_64bit::camera_to_VKHomographyFromProjectionMatrix(const vpgl_perspective_camera<double> & camera,  VKHomography * & vkhomo)
{
    vnl_matrix_fixed<double,3,4> P = camera.get_matrix();
    
    vnl_matrix<double> Pmat(3, 3, 0);
    Pmat(0, 0) = P(0, 0);
    Pmat(1, 0) = P(1, 0);
    Pmat(2, 0) = P(2, 0);
    
    Pmat(0, 1) = P(0, 1);
    Pmat(1, 1) = P(1, 1);
    Pmat(2, 1) = P(2, 1);
    
    Pmat(0, 2) = P(0, 3);
    Pmat(1, 2) = P(1, 3);
    Pmat(2, 2) = P(2, 3);
    
    CGFloat Hvalues[9] = {0};
    for (int r = 0; r<3; r++) {
        for (int c = 0; c<3; c++) {
            Hvalues[r * 3 + c] = Pmat(r, c)/Pmat(2, 2);
        }
    }
    
    vkhomo = [[VKHomography alloc] initWithValues: Hvalues];
}


