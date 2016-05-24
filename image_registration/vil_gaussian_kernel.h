//
//  vil_gaussian_kernel.h
//  QuadCopter
//
//  Created by jimmy on 3/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_gaussian_kernel__
#define __QuadCopter__vil_gaussian_kernel__

#include <vil/vil_image_view.h>
#include <vnl/vnl_vector.h>

class VilGaussianKernel
{
public:
    // x, y direction as image (i, j)
    static vil_image_view<double> orthogonalKernel(double sigma_x, double sigma_y);
    
    //x_dir : direction of x kernel in degree [0, 180). 0 for x direction in image space
    //
    //180     0
    //    90
    static vil_image_view<double> orientedOrthogonalKernel(double sigma_x, double sigma_y, double x_dir);
    
    static void gaussian1DKernel(double sigma, vnl_vector<double> & kernel);
    static vil_image_view<double> gaussianKernel(double sigma);    
};

class VilGaussianKernelUtil
{
public:
    // blur image by kernel, keep image size
    static vil_image_view<vxl_byte> blur_image_by_kernel(const vil_image_view<vxl_byte> & grayImage, const vil_image_view<double> & kernel);
    static vil_image_view<vxl_byte> blur_rgb_image_by_kernel(const vil_image_view<vxl_byte> & colorImage, const vil_image_view<double> & kernel);
};

#endif /* defined(__QuadCopter__vil_gaussian_kernel__) */
