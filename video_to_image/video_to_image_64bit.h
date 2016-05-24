//
//  video_to_image_64bit.h
//  VideoCalibration
//
//  Created by jimmy on 8/29/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VideoCalibration__video_to_image_64bit__
#define __VideoCalibration__video_to_image_64bit__

#import <Foundation/Foundation.h>
#import <Marshmallows/Marshmallows.h>
#import <VisionKit/VKLensDistortedPTZProjectionMatrix.h>
#import <VisionKit/VKProjectionMatrix.h>

#include <vil/vil_image_view.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vcl_string.h>

// works for 64 bit only
class video_to_image_64bit
{
public:
    static bool grabFrameFromVideo(const MMCVPixelBufferReaderAVFoundation *videoReader, int frame_num, double frameRate, vil_image_view<vxl_byte> & image);
    
    // buf is self-release
    static bool grabFrameFromVideo(const MMCVPixelBufferReaderAVFoundation *videoReader, int frame_num, double frameRate, CVPixelBufferRef & buf);
    
    static void camera_to_VKHomography(const vpgl_perspective_camera<double> & camera,  VKHomography * & vkhomo);
    
    // [1 2 4] column of projection matrix as H matrix
    static void camera_to_VKHomographyFromProjectionMatrix(const vpgl_perspective_camera<double> & camera,  VKHomography * & vkhomo);
    
};



#endif /* defined(__VideoCalibration__video_to_image_64bit__) */
