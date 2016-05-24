//
//  vil_deconvolution.h
//  QuadCopter
//
//  Created by jimmy on 3/14/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_deconvolution__
#define __QuadCopter__vil_deconvolution__

#include <vil/vil_image_view.h>

class VilDeconvolution
{
public:
    // Lucy Richardson algorithm with zero noise
    // return recovered image
    static vil_image_view<double> RL_deconvolution(const vil_image_view<double> &bluredImage, const vil_image_view<double> & kernel, int iterNum);
    
    
};

#endif /* defined(__QuadCopter__vil_deconvolution__) */
