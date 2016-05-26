//
//  vil_util.hpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-25.
//  Copyright © 2015 jimmy. All rights reserved.
//

#ifndef vil_util_cpp
#define vil_util_cpp

#include <vil/vil_image_view.h>

class VilUtil
{
public:
    static void vil_save(const vil_image_view<vxl_byte> & image, char const* filename, bool print_logo = true);
    
    static vil_image_view<vxl_byte> gray_2_rgb(const vil_image_view<vxl_byte> & image);
    
};


#endif /* vil_util_cpp */
