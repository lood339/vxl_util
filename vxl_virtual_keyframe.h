//
//  vxl_virtual_keyframe.h
//  CalibFromScene
//
//  Created by Jimmy Chen LOCAL on 7/14/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CalibFromScene__vxl_virtual_keyframe__
#define __CalibFromScene__vxl_virtual_keyframe__

#include <vcl_vector.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vil/vil_image_view.h>

class VxlVirtualKeyframe
{
    
public:
    // refine camera by patch match
    // it is slow
    static bool refinePTZCameraByVirtualKeyFrames(const vpgl_perspective_camera<double> & initCamera, const vil_image_view<vxl_byte> & image,
                                                  const vil_image_view<vxl_byte> & topviewImage,
                                                  const int patchSize, const int searchSize, vpgl_perspective_camera<double> & finalCamera);
    
    // threshold: Two positions: one in warped topview image and another in queryImage. Their position displacement should be similar if the camera is close in PTZ space
    //            The threshold is used to filter ourliers (correspondences that is mis-matched because of occolusion) whose displacement is so different from current matchings.
    //            10 pixel works fine in 1280 * 720 resolution
    // the method downsamples the image to speed up
    static bool refinePTZCameraByVirtualKeyFrames(const vpgl_perspective_camera<double> & initCamera, const vil_image_view<vxl_byte> & queryImage,
                                                  const vil_image_view<vxl_byte> & topviewImage, const double threshold,
                                                  const int patchSize, const int searchSize, vpgl_perspective_camera<double> & finalCamera);
    
    static bool refinePTZCameraByVirtualKeyFramesFullResolution(const vpgl_perspective_camera<double> & initCamera, const vil_image_view<vxl_byte> & queryImage,
                                                                const vil_image_view<vxl_byte> & topviewImage, const double threshold,
                                                                const int patchSize, const int searchSize, vpgl_perspective_camera<double> & finalCamera);
    
    // calcualte patch size, search size by patch match in down sampled images
    static bool optimizePatchSearchSize(const vpgl_perspective_camera<double> & initCamera, const vil_image_view<vxl_byte> & queryImage,
                                        const vil_image_view<vxl_byte> & topviewImage, const double threshold, int & patchSize, int & searchSize);

    
};



#endif /* defined(__CalibFromScene__vxl_virtual_keyframe__) */
