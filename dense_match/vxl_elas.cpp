//
//  vxl_elas.cpp
//  SegmentationDisparity
//
//  Created by jimmy on 9/29/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_elas.h"
#include <vil/vil_convert.h>

bool VXL_ELAS::calculate_disparity_map(const vil_image_view<vxl_byte> & I1, const vil_image_view<vxl_byte> & I2,
                                       vil_image_view<vxl_ieee_32> & D1, vil_image_view<vxl_ieee_32> & D2,
                                       vil_image_view<vxl_byte> & D1_gray, vil_image_view<vxl_byte> & D2_gray)
{
    assert(I1.ni() == I2.ni());
    assert(I1.nj() == I2.nj());
    
    vil_image_view<vxl_byte> grey_img_1;
    vil_image_view<vxl_byte> grey_img_2;
    if (I1.nplanes() == 3) {
        vil_convert_planes_to_grey(I1, grey_img_1);
    }
    else
    {
        grey_img_1.deep_copy(I1);
    }
    if (I2.nplanes() == 3) {
        vil_convert_planes_to_grey(I2, grey_img_2);
    }
    else
    {
        grey_img_2.deep_copy(I2);
    }
    // data = new T[w * h];
    const int w = I1.ni();
    const int h = I1.nj();
    
    uint8_t* I1_data = new uint8_t[w * h];
    uint8_t* I2_data = new uint8_t[w * h];
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            int ind = j * w + i;
            I1_data[ind] = grey_img_1(i, j, 0);
            I2_data[ind] = grey_img_2(i, j, 0);
        }
    }
    float *D1_data = new float[w * h];
    float *D2_data = new float[w * h];
    const int32_t dims[3] = {w, h, w};
    
    // process
    Elas::parameters param;
    param.postprocess_only_left = false;
    Elas elas(param);
    elas.process(I1_data, I2_data, D1_data, D2_data, dims);
    
    // output result
    D1 = vil_image_view<vxl_ieee_32>(w, h, 1);
    D2 = vil_image_view<vxl_ieee_32>(w, h, 1);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            int ind = j * w + i;
            D1(i, j, 0) = D1_data[ind];
            D2(i, j, 0) = D2_data[ind];
        }
    }
    
    // find maximum disparity for scaling output disparity images to [0..255]
    float disp_max = 0;
    for (int i=0; i<w*h; i++) {
        if (D1_data[i]>disp_max) disp_max = D1_data[i];
        if (D2_data[i]>disp_max) disp_max = D2_data[i];
    }
    
    // copy float to uchar
    D1_gray = vil_image_view<vxl_byte>(w, h, 1);
    D2_gray = vil_image_view<vxl_byte>(w, h, 1);
    for (int i=0; i<w*h; i++) {
        int x = i%w;
        int y = i/w;
        D1_gray(x, y, 0) = (vxl_byte)vcl_max(255.0*D1_data[i]/disp_max, 0.0);
        D2_gray(x, y, 0) = (vxl_byte)vcl_max(255.0*D2_data[i]/disp_max, 0.0);
    }
    
    // de-localte memory
    delete [] I1_data;
    delete [] I2_data;
    delete [] D1_data;
    delete [] D2_data;
    return true;
}