//
//  vxl_elas.h
//  SegmentationDisparity
//
//  Created by jimmy on 9/29/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __SegmentationDisparity__vxl_elas__
#define __SegmentationDisparity__vxl_elas__

// wrap code from "Efficient Large-Scale Stereo Matching" ACCV 2010
#include <vil/vil_image_view.h>
#include "elas.h"

class VXL_ELAS
{
    // void process (uint8_t* I1,uint8_t* I2,float* D1,float* D2,const int32_t* dims);
public:
    // fixed parameter as for middlebury benchmark
    // image 1, 2, dispary 1, 2, gray scale dispary 1, 2 for visualization purpose
    static bool calculate_disparity_map(const vil_image_view<vxl_byte> & I1, const vil_image_view<vxl_byte> & I2,
                                        vil_image_view<vxl_ieee_32> & D1, vil_image_view<vxl_ieee_32> & D2,
                                        vil_image_view<vxl_byte> & D1_gray, vil_image_view<vxl_byte> & D2_gray);
};

#endif /* defined(__SegmentationDisparity__vxl_elas__) */
