//
//  vil_gaussian_kernel.cpp
//  QuadCopter
//
//  Created by jimmy on 3/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_gaussian_kernel.h"
#include <vnl/vnl_math.h>
#include <vcl_iostream.h>
#include <vil/algo/vil_convolve_1d.h>
#include <vnl/vnl_vector.h>
#include <vil/vil_transpose.h>
#include <vil/vil_convolve_2d+.h>

#include "vil_plus.h"

vil_image_view<double> VilGaussianKernel::orthogonalKernel(double sigma_x, double sigma_y)
{
    
    
    vnl_vector<double> x_kernel;// = VilGaissianKernel::gaussianKernel(sigma_x);
    vnl_vector<double> y_kernel;
    VilGaussianKernel::gaussian1DKernel(sigma_x, x_kernel);
    VilGaussianKernel::gaussian1DKernel(sigma_y, y_kernel);
    
    int sz = vcl_max(x_kernel.size(), y_kernel.size());
    assert(sz%2 == 1);
    vil_image_view<double> kernel(sz, sz, 1);
    kernel.fill(0);
    
    double scaleX = 1.0/(2.0*(sigma_x*sigma_x));
    double scaleY = 1.0/(2.0*(sigma_y*sigma_y));
    int half = sz/2;
    double sum = 0.0;
    for (int j = -half; j <= half; j++) {
        for (int i = -half; i <= half; i++) {
            kernel(i + half, j+ half) = exp(-( scaleX * (i * i) + scaleY * (j * j)));
            sum += kernel(i + half, j+ half);
        }
    }
    
    // normalize
    for (int j = -half; j <= half; j++) {
        for (int i = -half; i <= half; i++) {
            kernel(i + half, j + half) /= sum;
        }
    }   
    
    
  //  VilPlus::vil_save(kernel, "kernel.png");
 
   
    
  //  vcl_cout<<"2d kernel is "<<kernel<<vcl_endl;
    
    return kernel;
}

vil_image_view<double> VilGaussianKernel::orientedOrthogonalKernel(double sigma_x, double sigma_y, double x_dir)
{
    assert(x_dir >= 0 && x_dir < 180);
    vnl_vector<double> x_kernel;
    vnl_vector<double> y_kernel;
    VilGaussianKernel::gaussian1DKernel(sigma_x, x_kernel);
    VilGaussianKernel::gaussian1DKernel(sigma_y, y_kernel);
    
    int sz = vcl_max(x_kernel.size(), y_kernel.size());
    assert(sz%2 == 1);
    vil_image_view<double> kernel(sz, sz, 1);
    kernel.fill(0);
    
    double x_dir_radian = x_dir/180.0 * vnl_math::pi;
    // x kernel direction
    double dx1 = cos(x_dir_radian);
    double dy1 = sin(x_dir_radian);
    
    // y kernel direction
    double dx2 = -dy1;
    double dy2 =  dx1;
    
    double scaleX = 1.0/(2.0*(sigma_x*sigma_x));
    double scaleY = 1.0/(2.0*(sigma_y*sigma_y));
    int half = sz/2;
    double sum = 0.0;
    for (int j = -half; j <= half; j++) {
        for (int i = -half; i <= half; i++) {
            double dis_x = i * dx1 + j * dy1;
            double dis_y = i * dx2 + j * dy2;
            kernel(i + half, j + half) = exp(-( scaleX * (dis_x * dis_x) + scaleY * (dis_y * dis_y)));
            sum += kernel(i + half, j + half);
        }
    }
    
    // normalize
    for (int j = -half; j <= half; j++) {
        for (int i = -half; i <= half; i++) {
            kernel(i + half, j + half) /= sum;
        }
    }
    
 //   VilPlus::vil_save(kernel, "kernel_oriented.png");
    
    return kernel;
}

void VilGaussianKernel::gaussian1DKernel(double sigma, vnl_vector<double> & kernel)
{
    assert(sigma > 0);
    assert(sigma > 0.17);
    
    int half = vnl_math::ceil(3*sigma - 0.5);
    kernel = vnl_vector<double>(half * 2 + 1);
    double scale = 1.0/(2*(sigma*sigma));
    double sum = 0;
    for (int r = -half; r <= half; r++) {
        kernel[r+half] = exp(-(r*r)) * scale;
        sum += kernel[r + half];
    }
    for (int i = 0; i<kernel.size(); i++) {
        kernel[i] /= sum;
    }
   // kernel = kernel.normalize();
  //  vcl_cout<<"1 D kernel is "<<kernel<<vcl_endl;
}

vil_image_view<double> VilGaussianKernel::gaussianKernel(double sigma)
{
    assert(sigma > 0);
    assert(sigma > 0.17);
    
    int half = vnl_math::ceil(3*sigma - 0.5);
    vil_image_view<double> kernel(half * 2 + 1, 1);
    double sum = 0;
    for (int r = -half; r <= half; r++) {
        kernel(r+half, 0) = exp(-(r*r)/(2*(sigma*sigma)));
        sum += kernel(r+half, 0);
    }
    
    vcl_cout<<"kernel is "<<kernel<<vcl_endl;
    // normalize
    for (int i = 0; i<kernel.ni(); i++) {
        kernel(i, 0) /= sum;
        printf("%f ", kernel(i, 0));
    }
    printf("\n");
    return kernel;
}


/****************        VilGaussianKernelUtil           **************/

vil_image_view<vxl_byte> VilGaussianKernelUtil::blur_image_by_kernel(const vil_image_view<vxl_byte> & grayImage, const vil_image_view<double> & kernel)
{
    vil_image_view<vxl_byte> bluredImage;
    
    vil_convolve_2d(grayImage, bluredImage, kernel, double(), vil_convolve_zero_extend);
    
    //
    return bluredImage;
}

vil_image_view<vxl_byte> VilGaussianKernelUtil::blur_rgb_image_by_kernel(const vil_image_view<vxl_byte> & colorImage, const vil_image_view<double> & kernel)
{
    assert(colorImage.nplanes() == 3);
    
    const int w = colorImage.ni();
    const int h = colorImage.nj();
    vil_image_view<vxl_byte> red = vil_image_view<vxl_byte>(w, h, 1);
    vil_image_view<vxl_byte> green = vil_image_view<vxl_byte>(w, h, 1);
    vil_image_view<vxl_byte> blue = vil_image_view<vxl_byte>(w, h, 1);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            red(i, j) = colorImage(i, j, 0);
            green(i, j) = colorImage(i, j, 1);
            blue(i, j) = colorImage(i, j, 2);
        }
    }
    
    vil_image_view<vxl_byte> blurred_red = VilGaussianKernelUtil::blur_image_by_kernel(red, kernel);
    vil_image_view<vxl_byte> blurred_green = VilGaussianKernelUtil::blur_image_by_kernel(green, kernel);
    vil_image_view<vxl_byte> blurred_blue = VilGaussianKernelUtil::blur_image_by_kernel(blue, kernel);
    
    vil_image_view<vxl_byte> blurred_image = vil_image_view<vxl_byte>(w, h, 3);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            blurred_image(i, j, 0) = blurred_red(i, j);
            blurred_image(i, j, 1) = blurred_green(i, j);
            blurred_image(i, j, 2) = blurred_blue(i, j);
        }
    }
    
    return blurred_image;
}




