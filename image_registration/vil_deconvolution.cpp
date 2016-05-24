//
//  vil_deconvolution.cpp
//  QuadCopter
//
//  Created by jimmy on 3/14/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_deconvolution.h"
#include <vil/vil_convolve_2d+.h>

vil_image_view<double> VilDeconvolution::RL_deconvolution(const vil_image_view<double> &bluredImage, const vil_image_view<double> & kernel, int iterNum)
{
    // error_image = b - Ik * kernel
    // I_k+1 = Ik + (error_image)
    
    assert(iterNum > 0);
    const int w = bluredImage.ni();
    const int h = bluredImage.nj();
    
    vil_image_view<double> preCorrectedImage;
    preCorrectedImage.deep_copy(bluredImage);
    
    for (int i = 0; i<iterNum; i++) {
        vil_image_view<double> curCorrectedImage;
        vil_convolve_2d(preCorrectedImage, curCorrectedImage, kernel, double(), vil_convolve_zero_extend);
        
        // calculate error
        vil_image_view<double> errorImage(w, h, 1);
        for (int y = 0; y<h; y++) {
            for (int x = 0; x<w; x++) {
                errorImage(x, y) = bluredImage(x, y) - curCorrectedImage(x, y);
            }
        }
        
        // add error
        for (int y = 0; y<h; y++) {
            for (int x = 0; x<w; x++) {
                curCorrectedImage(x, y) = preCorrectedImage(x, y) + errorImage(x, y);
            }
        }
        
        preCorrectedImage.deep_copy(curCorrectedImage);
    }
    
    return preCorrectedImage;

}