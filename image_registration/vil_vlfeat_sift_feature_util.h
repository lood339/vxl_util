//
//  vil_vlfeat_sift_feature_util.h
//  VideoCalibVXL
//
//  Created by jimmy on 7/18/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __VideoCalibVXL__vil_vlfeat_sift_feature_util__
#define __VideoCalibVXL__vil_vlfeat_sift_feature_util__

// util function for wwos basketball field calibration

#include "vil_vlfeat_sift_feature.h"
#include <vpgl/vpgl_perspective_camera.h>



class VilVlFeatSIFTFetureUtil
{
public:
    static void getSiftFromWarpedTopview(const vil_image_view<vxl_byte> &topviewImage,
                                         int destWidth, int destHeight,
                                         const vpgl_perspective_camera<double> & keyframeCamera,
                                         const vl_feat_sift_parameter &param,
                                         vcl_vector<bapl_keypoint_sptr> & keyframeSift);
    // only for Disney basketball court
    static bool calib_sequential_frames(const vil_image_view<vxl_byte> & imageA, const vpgl_perspective_camera<double> & cameraA,
                                        const vil_image_view<vxl_byte> & imageB, vpgl_perspective_camera<double> & cameraB);
    
    static bool calibSequentialFrames(const vcl_vector<bapl_keypoint_sptr> & siftA, const vpgl_perspective_camera<double> & cameraA,
                                      const vil_image_view<vxl_byte> & imageB, vpgl_perspective_camera<double> & cameraB);
    
};

#endif /* defined(__VideoCalibVXL__vil_vlfeat_sift_feature_util__) */
