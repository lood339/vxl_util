//
//  vil_plus_extra.cpp
//  GenerateFeature
//
//  Created by jimmy on 2016-07-22.
//  Copyright (c) 2016 Nowhere Planet. All rights reserved.
//

#include "vil_plus.h"
#include "vil_plus_extra.h"
#include <vil/vil_save.h>
#include <vil/algo/vil_sobel_3x3.h>
#include <vil/algo/vil_gauss_filter.h>
#include <vicl/vicl_ellipse.h>
#include <vicl/vicl_colours.h>


void VilPlusExtra::draw_ellipse(vil_image_view<vxl_byte> & image, const vgl_ellipse_2d<double> & ellipse, const vcl_vector<vxl_byte> & colour)
{
    assert(image.nplanes() == 3);
    assert(colour.size() == 3);
    
    vgl_line_segment_2d<double> majDia = ellipse.major_diameter();
    vgl_line_segment_2d<double> minDia = ellipse.minor_diameter();    
    vicl_overlay_ellipse(image, ellipse, colour, 2);
}

void VilPlusExtra::draw_covariance(vil_image_view<vxl_byte> & image,
                              const vnl_matrix_fixed<double, 2, 2> & cov,
                              const vgl_point_2d<double> & loc,
                              double orientation, const vcl_vector<vxl_byte> & colour,
                              double scale)
{
    // vgl_ellipse_2d( vgl_point_2d< T > centre = vgl_point_2d< T >( 0, 0 ), T majorRadius = 2, T minorRadius = 1, T orientation = 0 );
    double major = cov[0][0] * scale;
    double minor = cov[1][1] * scale;
    if (major < minor) {
        vcl_swap(major, minor);
        orientation += vnl_math::pi/2.0;
    }
    vgl_ellipse_2d<double> ellipse(loc, major, minor, orientation);
    VilPlusExtra::draw_ellipse(image, ellipse, colour);
}


void VilPlusExtra::vil_save(const vil_image_view<double> & image, char const* filename, bool print_logo)
{
    
    bool isSaveOk = ::vil_save(vil_quantize::quantize<vxl_byte>(image, true), filename);
    if (print_logo && isSaveOk) {
        vcl_cout<<"save to: "<<filename<<vcl_endl;
    }
}


void VilPlusExtra::vil_save(const vil_image_view<int> & image, char const* filename, bool print_logo)
{
    vil_image_view<double> dImage(image.ni(), image.nj(), image.nplanes());
    for (int i = 0; i<image.ni(); i++) {
        for (int j = 0; j<image.nj(); j++) {
            for (int k = 0; k<image.nplanes(); k++) {
                dImage(i, j, k) = image(i, j, k);
            }
        }
    }
    VilPlusExtra::vil_save(dImage, filename, print_logo);
}



void VilPlusExtra::vil_smooth_gradient(const vil_image_view<vxl_byte> & image, vil_image_view<double> & magnitude,
                                  vil_image_view<double> & Ix, vil_image_view<double> & Iy)
{
    const int w = image.ni();
    const int h = image.nj();
    
    // rgb to gray
    vil_image_view<vxl_byte> gray;
    if (image.nplanes() == 3) {
        gray = VilPlus::vil_to_gray(image);
    }
    else
    {
        gray = image;
    }
    // gray to double
    vil_image_view<double> dImage = vil_quantize::dequantize<double>(gray);
    
    vil_image_view<double> smoothedImage = vil_image_view<double>(w, h, 1);
    vil_gauss_filter_5tap_params params(5);
    vil_gauss_filter_5tap(dImage, smoothedImage, params);
    
    vil_sobel_3x3(smoothedImage, Ix, Iy);
    
    //magnitude
    magnitude = vil_image_view<double>(w, h, 1);
    for (int y = 0; y<magnitude.nj(); y++) {
        for (int x = 0; x<magnitude.ni(); x++) {
            double dx = Ix(x, y, 0);
            double dy = Iy(x, y, 0);
            magnitude(x, y, 0) = sqrt(dx * dx + dy * dy);
        }
    }
}

double VilPlusExtra::vil_gradient_ssd(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> & image2)
{
    assert(image1.ni() == image2.ni());
    assert(image1.nj() == image2.nj());
    
    vil_image_view<double> mag1;
    vil_image_view<double> mag2;
    VilPlus::vil_magnitude(image1, mag1);
    VilPlus::vil_magnitude(image2, mag2);
    
    double ssd = 0;
    for (int j = 0; j<image1.nj(); j++) {
        for (int i = 0; i<image1.ni(); i++) {
            double dif = image1(i, j, 0) - image2(i, j, 0);
            ssd += dif * dif;
        }
    }
    return ssd;
}

void VilPlusExtra::draw_circle(vil_image_view<vxl_byte> & image, const vcl_vector< vgl_point_2d<double> > & pts,
                          int radius, const vcl_vector<vxl_byte> & colour)
{
    for (int i = 0; i<pts.size(); i++) {
        vgl_ellipse_2d<double> ellipse(pts[i], radius, radius);
        vicl_overlay_ellipse(image, ellipse, colour);
    }
}

void VilPlusExtra::draw_velocity(vil_image_view<vxl_byte> & image, const vcl_vector< vgl_point_2d<double> > & pts,
                            const vcl_vector< vnl_vector_fixed<double, 2> > & vlt,
                            double scale, const vcl_vector<vxl_byte> & colour)
{
    assert(pts.size() == vlt.size());
    
    VilPlusExtra::draw_circle(image, pts, 2, colour);
    for (int i = 0; i<pts.size(); i++) {
        vgl_point_2d<double> p1 = pts[i];
        double u = vlt[i][0] * scale;
        double v = vlt[i][1] * scale;
        vgl_point_2d<double> p2(p1.x() + u, p1.y() + v);
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), colour, 2);
    }
}

void VilPlusExtra::vil_cross_correlation(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2,
                                    const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2,
                                    int window_size,
                                    vcl_vector<double> & nccs)
{
    assert(pts1.size() == pts2.size());
    assert(image1.nplanes() == image2.nplanes());
    
    //   int w = image1.ni();
    //   int h = image1.nj();
    // 0 - 1.0
    vil_image_view<double> source = vil_quantize::dequantize<double>(image1);
    vil_image_view<double> dest   = vil_quantize::dequantize<double>(image2);
    
    for (int i = 0; i<pts1.size(); i++) {
        int x = pts1[i].x();
        int y = pts1[i].y();
        if (x < window_size || x > image1.ni() - window_size || y < window_size || y > image1.nj() - window_size) {
            nccs.push_back(0.0);
            continue;
        }
        
        vil_image_view<double> patch_s = vil_crop(source, x - window_size/2, window_size, y - window_size/2, window_size);
        
        x = pts2[i].x();
        y = pts2[i].y();
        if (x < window_size || x > image2.ni() - window_size || y < window_size || y > image2.nj() - window_size) {
            nccs.push_back(0.0);
            continue;
        }
        vil_image_view<double> patch_d = vil_crop(dest, x - window_size/2, window_size, y - window_size/2, window_size);
        
        vil_image_view<double> ncc    = vil_normalised_cross_correlation(patch_s, patch_d);
        assert(ncc.ni() == 1 && ncc.nj() == 1 && ncc.nplanes() == 1);
        nccs.push_back(ncc(0, 0, 0));
    }
    assert(nccs.size() == pts1.size());
}


bool VilPlusExtra::refine_patch_position(const vil_image_view<vxl_byte> & kernelImage, const vil_image_view<vxl_byte> & destImage,
                                             const vgl_point_2d<double> & initP, int patchSize, int searchSize, vgl_point_2d<double> & finalP)
{
    assert(kernelImage.nplanes() == destImage.nplanes());
    assert(kernelImage.ni() == destImage.ni());
    assert(kernelImage.nj() == destImage.nj());
    assert(patchSize < searchSize);
    
    int destWidth  = destImage.ni();
    int destHeight = destImage.nj();
    
    vil_image_view<double> source = vil_quantize::dequantize<double>(kernelImage);
    vil_image_view<double> dest   = vil_quantize::dequantize<double>(destImage);
    
    // grab patch center at initP in warped image
    vgl_point_2d<double> c = initP;
    vil_image_view<double> patch     = vil_crop(source, c.x() - patchSize/2, patchSize, c.y() - patchSize/2, patchSize);
    vil_image_view<double> destPatch = vil_crop(dest, c.x() - searchSize/2, searchSize, c.y() - searchSize/2, searchSize);
    vil_image_view<double> ncc    = vil_normalised_cross_correlation(destPatch, patch);
    assert(ncc.nplanes() == 1);
    
    // max value in ncc
    double val_max = -1.0;
    int idx_x_ = -1;
    int idx_y_ = -1;
    for (int i = 0; i<ncc.ni(); i++) {
        for (int j = 0; j<ncc.nj(); j++) {
            double val = ncc(i, j, 0);
            if (val > val_max) {
                val_max = val;
                idx_x_ = i;
                idx_y_ = j;
            }
        }
    }
    int x_center = idx_x_ + patchSize/2;  // ncc position is in the left top, but patch center is in middle
    int y_center = idx_y_ + patchSize/2;
    int x_offset = c.x() - searchSize/2;  // ncc coordinate to destImage coordinate
    int y_offset = c.y() - searchSize/2;
    
    finalP.set(x_center + x_offset  ,  y_center + y_offset);
    return finalP.x() >= 0 && finalP.x() < destWidth && finalP.y() >= 0 && finalP.y() < destHeight;
}


bool VilPlusExtra::refine_patch_position(const vil_image_view<vxl_byte> & kernelImage,
                                             const vgl_point_2d<double> & kernelP,
                                             const vil_image_view<vxl_byte> & destImage, const vgl_point_2d<double> & initP,
                                             int patchSize, int searchSize, vgl_point_2d<double> & finalP)
{
    assert(kernelImage.nplanes() == destImage.nplanes());
    assert(patchSize < searchSize);
    
    int destWidth  = destImage.ni();
    int destHeight = destImage.nj();
    
    vil_image_view<double> source = vil_quantize::dequantize<double>(kernelImage);
    vil_image_view<double> dest   = vil_quantize::dequantize<double>(destImage);
    
    // grab patch center at initP in warped image
    vil_image_view<double> patch     = vil_crop(source, kernelP.x() - patchSize/2, patchSize, kernelP.y() - patchSize/2, patchSize);
    
    int x = initP.x() - searchSize/2;
    int y = initP.y() - searchSize/2;
    if (x < 0) {
        x = 0;
    }
    if (y < 0) {
        y = 0;
    }
    int w = searchSize;
    int h = searchSize;
    if (x + w > destWidth) {
        w = destWidth - x;
    }
    if (y + h > destHeight) {
        h = destHeight - y;
    }
    if (w < patchSize || h < patchSize) {
        return false;
    }
    
    vil_image_view<double> destPatch = vil_crop(dest, x, w, y, h);
    
    vil_image_view<double> ncc    = vil_normalised_cross_correlation(destPatch, patch);
    assert(ncc.nplanes() == 1);
    
    // max value in ncc
    double val_max = -1.0;
    int idx_x_ = -1;
    int idx_y_ = -1;
    for (int i = 0; i<ncc.ni(); i++) {
        for (int j = 0; j<ncc.nj(); j++) {
            double val = fabs(ncc(i, j, 0));
            if (val > val_max) {
                val_max = val;
                idx_x_ = i;
                idx_y_ = j;
            }
        }
    }
    int x_center = idx_x_ ;      // ncc position is in the left top, but patch center is in middle
    int y_center = idx_y_ ;
    int x_offset = x + patchSize/2;  // ncc coordinate to destImage coordinate
    int y_offset = y + patchSize/2;
    
    finalP.set(x_center + x_offset,  y_center + y_offset);
    bool isInside = finalP.x() >= 0 && finalP.x() < destWidth && finalP.y() >= 0 && finalP.y() < destHeight;
    
    if (!isInside) {
        printf("out of dest image %f %f\n", finalP.x(), finalP.y());
    }
    return isInside;
}

bool VilPlusExtra::refine_patch_position(const vil_image_view<vxl_byte> & kernelImage, const vcl_vector<vgl_point_2d<double> > & kernelPts,
                                             const vil_image_view<vxl_byte> & destImage, const vcl_vector<vgl_point_2d<double> > & initPts,
                                             int patchSize, int searchSize, vcl_vector<vgl_point_2d<double> > & finalP)
{
    assert(kernelImage.nplanes() == destImage.nplanes());
    assert(patchSize < searchSize);
    assert(kernelPts.size() == initPts.size());
    assert(finalP.size() == 0);
    
    int destWidth  = destImage.ni();
    int destHeight = destImage.nj();
    vil_image_view<double> source = vil_quantize::dequantize<double>(kernelImage);
    vil_image_view<double> dest   = vil_quantize::dequantize<double>(destImage);
    
    // loop over all initial correspondence
    for (int i = 0; i<kernelPts.size(); i++) {
        vgl_point_2d<double> kp = kernelPts[i];  // kernal position
        vgl_point_2d<double> ip = initPts[i];    // init position
        vgl_point_2d<double> cp;  // corresponding position
        
        vil_image_view<double> patch     = vil_crop(source, kp.x() - patchSize/2, patchSize, kp.y() - patchSize/2, patchSize);
        
        // search top-left position
        int x = ip.x() - searchSize/2;
        int y = ip.y() - searchSize/2;
        if (x < 0) {
            x = 0;
        }
        if (y < 0) {
            y = 0;
        }
        // search size
        int w = searchSize;
        int h = searchSize;
        if (x + w > destWidth) {
            w = destWidth - x;
        }
        if (y + h > destHeight) {
            h = destHeight - y;
        }
        // search size smaller than patch size
        if (w < patchSize || h < patchSize) {
            finalP.push_back(vgl_point_2d<double>(-1, -1));
            continue;
        }
        
        vil_image_view<double> destPatch = vil_crop(dest, x, w, y, h);
        
        vil_image_view<double> ncc    = vil_normalised_cross_correlation(destPatch, patch);
        assert(ncc.nplanes() == 1);
        
        // max value in ncc
        double val_max = -1.0;
        int idx_x_ = -1;
        int idx_y_ = -1;
        for (int i = 0; i<ncc.ni(); i++) {
            for (int j = 0; j<ncc.nj(); j++) {
                double val = fabs(ncc(i, j, 0));
                if (val > val_max) {
                    val_max = val;
                    idx_x_ = i;
                    idx_y_ = j;
                }
            }
        }
        int x_center = idx_x_ ;      // ncc position is in the left top, but patch center is in middle
        int y_center = idx_y_ ;
        int x_offset = x + patchSize/2;  // ncc coordinate to destImage coordinate
        int y_offset = y + patchSize/2;
        
        cp.set(x_center + x_offset,  y_center + y_offset);
        bool isInside = cp.x() >= 0 && cp.x() < destWidth && cp.y() >= 0 && cp.y() < destHeight;
        
        if (!isInside) {
            finalP.push_back(vgl_point_2d<double>(-1, -1));
            continue;
        }
        finalP.push_back(cp);
    }
    assert(kernelPts.size() == finalP.size());
    
    return true;
}












