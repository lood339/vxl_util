//
//  video_to_image.h
//  VideoCalibration
//
//  Created by Jimmy Chen LOCAL on 5/2/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VideoCalibration__video_to_image__
#define __VideoCalibration__video_to_image__

#import <Foundation/Foundation.h>
#import <Marshmallows/Marshmallows.h>
#import <VisionKit/VKLensDistortedPTZProjectionMatrix.h>
#import <VisionKit/VKProjectionMatrix.h>

#include <vil/vil_image_view.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vcl_string.h>

// works for 32 bit only

bool grabFrameFromVideo(const MMCVPixelBufferReaderQuickTime *videoReader, int frame_num, double frameRate, vil_image_view<vxl_byte> & image);

// buf is NOT self-release
bool grabFrameFromVideo(const MMCVPixelBufferReaderQuickTime *videoReader, int frame_num, double frameRate, CVPixelBufferRef & buf);

void camera_to_VKHomography(const vpgl_perspective_camera<double> & camera,  VKHomography * & vkhomo);
//void camera_to_fl_pan_tilt(const vpgl_perspective_camera<double> & camera, double & fl, double & pan, double & tilt);


void overlay_text_to_image(vil_image_view<vxl_byte> & image, MMCIRenderer * aRender, const vcl_string & text);

@interface ImageConverter : NSObject
{
}

+ (vil_image_view<vxl_byte>) createImageFromCIImage:(CIImage *)image;
+ (vil_image_view<vxl_byte>) createImageFromCIImage:(CIImage *)image renderBy: (MMCIRenderer *) aRender;

+ (CVPixelBufferRef) createCVPixelBufferFromCIImage:(CIImage*)image;
+ (CIImage *) createImageFromVilImageView:(vil_image_view<vxl_byte>) image;


@end


#endif /* defined(__VideoCalibration__video_to_image__) */
