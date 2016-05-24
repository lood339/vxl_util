//
//  video_to_image.mm
//  VideoCalibration
//
//  Created by Jimmy Chen LOCAL on 5/2/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "video_to_image.h"
#include <vil/vil_osx.h>
#include <vpgl/vpgl_perspective_camera.h>
#include "basketballCourt.h"
#include "vpgl_ptz_model_estimation.h"

bool grabFrameFromVideo(const MMCVPixelBufferReaderQuickTime *videoReader, int frame_num, double frameRate, vil_image_view<vxl_byte> & image)
{
    NSAutoreleasePool *pool = [[NSAutoreleasePool alloc] init];
    
    MMTime curTime;
    curTime = MMMakeTime(frame_num, frameRate);
    CVPixelBufferRef buf = [videoReader pixelBufferForTime:curTime];
    
    if (!buf) {
        [pool drain];
        return false;
    }
    image = vil_image_view_from_CVPixelBuffer(buf, true);
    
    [pool drain];
    return true;
}

bool grabFrameFromVideo(const MMCVPixelBufferReaderQuickTime *videoReader, int frame_num, double frameRate, CVPixelBufferRef & buf)
{    
    MMTime curTime;
    curTime = MMMakeTime(frame_num, frameRate);
    buf = [videoReader pixelBufferForTime:curTime];
    return buf != nil;
}

void camera_to_VKHomography(const vpgl_perspective_camera<double> & camera,  VKHomography * & vkhomo)
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
    
    //@ todo, autorelease because this function has no way to release by return
    vkhomo = [[[VKHomography alloc] initWithValues: Hvalues] autorelease];
}



void overlay_text_to_image(vil_image_view<vxl_byte> & image, MMCIRenderer * aRender, const vcl_string & text)
{
    assert(aRender);
    
    CIImage *pCIImage = [ImageConverter createImageFromVilImageView:image];
    
    
    // input test into image
    NSDictionary * textAttributes = [NSDictionary dictionaryWithObjectsAndKeys:
                                     [NSFont fontWithName:@"Times New Roman" size:23],NSFontAttributeName,
                                     [NSColor colorWithDeviceHue:0.33 saturation:1.0 brightness:0.87 alpha:1],NSForegroundColorAttributeName,
                                     nil];
    
    CIImage * textImage = [CIImage imageWithData:[[NSImage mmImageWithAttributedString:[[[NSAttributedString alloc] initWithString:[[NSString alloc] initWithUTF8String:text.c_str()] attributes:textAttributes] autorelease]] TIFFRepresentation]];
    
    NSMutableArray * imagesToComposite = [NSMutableArray array];
    
    // add your first "layer"
    [imagesToComposite addObject:pCIImage];
    
    // add text overlay
    [imagesToComposite addObject:textImage];
    
    // composite
    CIImage * compositeImage = nil;
    for ( CIImage * image in imagesToComposite )
    {
        CIFilter * compositeFilter = [CIFilter filterWithName:@"CISourceOverCompositing"];
        [compositeFilter setValue:compositeImage forKey:@"inputBackgroundImage"];
        [compositeFilter setValue:image forKey:@"inputImage"];
        compositeImage = [compositeFilter valueForKey:@"outputImage"];
    }
    
    image = [ImageConverter createImageFromCIImage:compositeImage renderBy:aRender];
}




@implementation ImageConverter

+ (vil_image_view<vxl_byte>) createImageFromCIImage:(CIImage *)image
{
    // memory leak
    CVPixelBufferRef pixelBuf;
    
    MMCIRenderer *aRender = [[MMCIRenderer alloc] init];
    pixelBuf = [aRender render:image];  // auto release
    
    vil_image_view< vxl_byte > outImage = vil_image_view_from_CVPixelBuffer(pixelBuf, true);
    
    [aRender release];  
    
    return outImage;
}

+ (vil_image_view<vxl_byte>) createImageFromCIImage:(CIImage *)image renderBy: (MMCIRenderer *) aRender
{
    CVPixelBufferRef pixelBuf;
   
    pixelBuf = [aRender render:image];  // auto release
    vil_image_view< vxl_byte > outImage = vil_image_view_from_CVPixelBuffer(pixelBuf, true);
    return outImage;
}


+ (CVPixelBufferRef) createCVPixelBufferFromCIImage:(CIImage*)image
{
    CVPixelBufferRef pixelBuf;
    
    MMCIRenderer *aRender = [[MMCIRenderer alloc] init];
    pixelBuf = [aRender render:image];   
    
    return pixelBuf;
}

+ (CIImage *) createImageFromVilImageView:(vil_image_view<vxl_byte>) image
{
    CVPixelBufferRef buf = CVPixelBufferCreateWithVilImageView(image);
    CIImage *img = [[[CIImage alloc] initWithCVImageBuffer:buf] autorelease];
    
    CVPixelBufferRelease(buf);
    return  img;
}


@end