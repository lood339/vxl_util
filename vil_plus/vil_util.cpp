//
//  vil_util.cpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-25.
//  Copyright © 2015 jimmy. All rights reserved.
//

#include "vil_util.hpp"
#include <vil/vil_save.h>

void VilUtil::vil_save(const vil_image_view<vxl_byte> & image, char const* filename, bool print_logo)
{
    bool isSaveOk = ::vil_save(image, filename);
    if (print_logo && isSaveOk) {
        vcl_cout<<"save to: "<<filename<<vcl_endl;
    }
}

vil_image_view<vxl_byte> VilUtil::gray_2_rgb(const vil_image_view<vxl_byte> & image)
{    
    assert(image.nplanes() == 1);
    const int w = image.ni();
    const int h = image.nj();
    
    vil_image_view<vxl_byte> rgbImage = vil_image_view<vxl_byte>(w, h, 3);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            rgbImage(i, j, 0) = image(i, j);
            rgbImage(i, j, 1) = image(i, j);
            rgbImage(i, j, 2) = image(i, j);
        }
    }
    return rgbImage;
}
