//
//  vxl_canny_edgelet.cpp
//  OnlineStereo
//
//  Created by jimmy on 12/4/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_canny_edgelet.h"
#include <vil/vil_convert.h>
#include <vil/algo/vil_gauss_filter.h>
#include <vil/algo/vil_sobel_1x3.h>
#include <vnl/vnl_matrix.h>
#include <vnl/algo/vnl_svd.h>
#include <vnl/vnl_vector.h>
#include <vgl/vgl_point_2d.h>
#include "vxl_plus.h"
#include <vcl_iostream.h>
#include <vcl_algorithm.h>
#include <vcl_numeric.h>
#include <vgl/vgl_fit_line_2d.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_matrix_fixed.txx>
#include "vnl_plus.h"
#include "vil_plus.h"


void VxlCannyEdgelet::cannyEdgelet(const vil_image_view<vxl_byte> & image,
                                   const EdgeletParameter & para,
                                   vil_image_view<vxl_byte> & edgeletMask,
                                   vcl_vector<Edgelet> & edgelets)
{
    int width = image.ni();
    int height = image.nj();
 
    vil_image_view<vxl_byte> intensityImage;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, intensityImage);
    }
    else
    {
        intensityImage = image;
    }
    
    // gaussian smoothing
    vil_image_view<double> smoothedImage = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(para.sigma_);
    vil_gauss_filter_5tap(intensityImage, smoothedImage, params);
    
    // only for test
    if (0)
    {
        char buf[1024] = {NULL};
        sprintf(buf, "smoothedImage_%d.jpg", rand()%1024);
       // VilPlus::vil_save(smoothedImage, buf);
    }
    
    
    // gradient
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    vil_sobel_1x3(smoothedImage, grad_i, grad_j);
    
 //   VilPlus::vil_save(grad_i, "grad_i.jpg");
 //   VilPlus::vil_save(grad_j, "grad_j.jpg");
    
    // non-maxinum suppress
    vil_image_view<double> grad_mag2 = vil_image_view<double>(width, height, 1);
    vil_image_view<unsigned char> grad_direction = vil_image_view<unsigned char>(width, height, 1);
    grad_mag2.fill(0);
    grad_direction.fill(0);
    for (int j = 1; j<height; j++) {
        for (int i = 1; i<width; i++) {
            double dx = grad_i(i, j);
            double dy = grad_j(i, j);
            
            // mag^2
            grad_mag2(i, j) = dx*dx + dy*dy;
            
            //compute 8 directions: [0, 45), [45, 90), [90, 135), [135,180), [180, 225), [225, 270), [270, 315), [315, 360)
            //   -----> x
            //    |
            //    |
            //   \ / y
            if(VnlPlus::isEqualZero(dx))
            {
                if(VnlPlus::isEqualZero(dy))
                {
                    grad_direction(i, j) = 0;
                }
                else
                {
                    grad_direction(i, j) = 2;
                }
            }
            else
            {
                // 0.4142 --> 22.5^o, 2.4142--> 657.5^o
                double tanV = dy/dx;
                if(tanV > -0.4142f && tanV <= 0.4142f)
                {
                    grad_direction(i, j) = 0;
                }
                else if(tanV > 0.4142f && tanV < 2.4142f)
                {
                    grad_direction(i, j) = 1;
                }
                else if(abs(tanV) >= 2.4142f)
                {
                    grad_direction(i, j) = 2;
                }
                else if(tanV > -2.4142f && tanV <= -0.4142f)
                {
                    grad_direction(i, j) = 3;
                }
            }
        }
    }
    
    vil_image_view<double> grad_canny_mag = vil_image_view<double>(width, height, 1);
    vil_image_view<bool> is_canny_edge = vil_image_view<bool>(width, height, 1);
    grad_canny_mag.fill(0);
    is_canny_edge.fill(false);
    
    // compare in 8 directions
    for (int j = 1; j<height-1; j++) {
        for (int i = 1; i<width-1; i++) {
            double v = grad_mag2(i, j);
            if (v < para.min_gradient_magnitude2_) {  // filter small edgelet
                continue;
            }
            switch (grad_direction(i, j)) {
                case 0:
                    if (v > grad_mag2(i-1, j) && v > grad_mag2(i+1, j)) {
                        grad_canny_mag(i, j) = sqrt(v);
                        is_canny_edge(i, j) = true;
                    }
                    break;
                case 1:
                    if (v > grad_mag2(i-1, j-1) && v > grad_mag2(i+1, j+1)) {
                        grad_canny_mag(i, j) = sqrt(v);
                        is_canny_edge(i, j) = true;
                    }
                    break;
                case 2:
                    if (v > grad_mag2(i, j-1) && v > grad_mag2(i, j+1)) {
                        grad_canny_mag(i, j) = sqrt(v);
                        is_canny_edge(i, j) = true;
                    }
                    break;
                case 3:
                    if (v > grad_mag2(i-1, j+1) && v > grad_mag2(i+1, j-1)) {
                        grad_canny_mag(i, j) = sqrt(v);
                        is_canny_edge(i, j) = true;
                    }
                    break;
                default:
                    assert(0);
                    break;
            }
        }
    }
    
    // only for test
    if (0)
    {
        char buf[1024] = {NULL};
        sprintf(buf, "grad_canny_mag_%d.jpg", rand()%1014);
        //VilPlus::vil_save(grad_canny_mag, buf);
    }
    
    
    // dominant normal
//    vil_image_view<vxl_byte> showImage;
//    showImage.deep_copy(image);
    
    // dominant direction inside the patch
    const int sz = para.patchSize_;
    const double cosThetaThreshold = cos(para.max_theta_*vnl_math::pi/180.0);
    edgeletMask = vil_image_view<vxl_byte>(width, height, 1);
    edgeletMask.fill(0);
    for (int j = 0; j + sz < height; j += sz) {
        for (int i = 0; i + sz < width; i += sz) {
            // second moment of gradient for dominent eigen vector
            vnl_matrix<double> M(2, 2, 0);
            int num = 0;
            for (int y = 0; y<sz; y++)
            {
                for (int x = 0; x<sz; x++)
                {
                    int px = x + i;
                    int py = y + j;
                    if (!is_canny_edge(px, py)) {
                        continue;
                    }
                    double gi = grad_i(px, py, 0);
                    double gj = grad_j(px, py, 0);
                    
                    double xx = gi * gi;
                    double yy = gj * gj;
                    double xy = gi * gj;
                    M(0, 0) += xx;
                    M(0, 1) += xy;
                    M(1, 0) += xy;
                    M(1, 1) += yy;
                    num++;
                }
            }
            
            if (num < para.min_edgelet_in_patch_) {
                continue;
            }
            M /= num;
            
            vnl_svd<double> svd(M);
            if (svd.singularities() != 0)
            {
                continue;
            }
            
            vnl_matrix<double> eigen_vectors = svd.V();
            vnl_vector<double> nHat = eigen_vectors.get_column(0).normalize();  //dominant eigen vector in the patch
            assert(nHat.size() == 2);
           
            //equation 9, project position to normal direction
            num = 0;
            vcl_vector<double> b;
            for (int y = 0; y<sz; y++)
            {
                for (int x = 0; x<sz; x++)
                {
                    int px = x + i;
                    int py = y + j;
                    if (!is_canny_edge(px, py)) {
                        continue;
                    }
                    
                    double gi = grad_i(px, py, 0);
                    double gj = grad_j(px, py, 0);
                    
                    double cosTheta = (nHat[0] * gi + nHat[1] * gj)/grad_canny_mag(px, py);
                    if (fabs(cosTheta) >= cosThetaThreshold)
                    {
                        
                        double bj = px * nHat[0] + py * nHat[1];
                        b.push_back(bj);
                        num++;
                    }
                }
            }
            
            
            if (num < para.min_edgelet_in_patch_) {
                continue;
            }
            
            double b_mean = 0;
            double b_sigma = 0;
            VnlPlus::mean_std(&b[0], (unsigned int)b.size(), b_mean, b_sigma);
            
            if (b_sigma >= para.location_variance_) {
                continue;
            }
            
            // get candidate points for line
            vcl_vector<vgl_point_2d<double> > line_pts;
            //vgl_point_2d<double> line_center_point(0, 0);
            double line_pts_center_x = 0.0;
            double line_pts_center_y = 0.0;
            for (int y = 0; y<sz; y++)
            {
                for (int x = 0; x<sz; x++)
                {
                    int px = x + i;
                    int py = y + j;
                    if (!is_canny_edge(px, py)) {
                        continue;
                    }
                    double gi = grad_i(px, py, 0);
                    double gj = grad_j(px, py, 0);
                    
                    double cosTheta = (nHat[0] * gi + nHat[1] * gj)/grad_canny_mag(px, py);
                    double bj = px * nHat[0] + py * nHat[1];
                    
                    if (fabs(cosTheta) >= cosThetaThreshold && fabs(bj - b_mean) <= para.location_variance_)
                    {
                        num++;
                        edgeletMask(px, py) = 255;
                        line_pts.push_back(vgl_point_2d<double>(px, py));
                        line_pts_center_x += px;
                        line_pts_center_y += py;
                        
                    }
                }
            }
            
            // line has enough point
            if (line_pts.size() >= para.min_line_points_)
            {
                line_pts_center_x /= line_pts.size();
                line_pts_center_y /= line_pts.size();
                vgl_line_2d<double> line = vgl_fit_line_2d(line_pts);
                double len = 0.0;
                for (int k = 0; k<line_pts.size(); k++) {
                    double x_dif = line_pts[k].x() - line_pts_center_x;
                    double y_dif = line_pts[k].y() - line_pts_center_y;
                    double dis = x_dif * x_dif + y_dif * y_dif;
                    if (dis > len) {
                        len = dis;
                    }
                }
                len = sqrt(len);
                
                Edgelet aEdgelet(vgl_point_2d<double>(line_pts_center_x, line_pts_center_y), line, len * 2);
                edgelets.push_back(aEdgelet);
            }
            else
            {
                for (int y = 0; y<sz; y++)
                {
                    for (int x = 0; x<sz; x++)
                    {
                        int px = x + i;
                        int py = y + j;
                        edgeletMask(px, py) = 0;
                    }
                }
            }
        }
    }
 //   VilPlus::vil_save(showImage, "dominant_edge.jpg");
}


EdgeletDetectionResult VxlCannyEdgelet::edgeletDetection(const vil_image_view<vxl_byte> & grayImage,
                                                         const EdgeletParameter & para,
                                                         vcl_vector<vgl_point_2d<double> > & edgeletPts)
{
    assert(grayImage.nplanes() == 1);
    int width = grayImage.ni();
    int height = grayImage.nj();
    
    // gaussian smoothing
    vil_image_view<double> smoothedImage = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(para.sigma_);
    vil_gauss_filter_5tap(grayImage, smoothedImage, params);
    
    
    // gradient
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    vil_sobel_1x3(smoothedImage, grad_i, grad_j);
    
    //   VilPlus::vil_save(grad_i, "grad_i.jpg");
    //   VilPlus::vil_save(grad_j, "grad_j.jpg");
    
    // non-maxinum suppress
    vil_image_view<double> grad_mag2 = vil_image_view<double>(width, height, 1);
    vil_image_view<unsigned char> grad_direction = vil_image_view<unsigned char>(width, height, 1);
    grad_mag2.fill(0);
    grad_direction.fill(0);
    for (int j = 1; j<height; j++) {
        for (int i = 1; i<width; i++) {
            double dx = grad_i(i, j);
            double dy = grad_j(i, j);
            
            // mag^2
            grad_mag2(i, j) = dx*dx + dy*dy;
            
            //compute 8 directions: [0, 45), [45, 90), [90, 135), [135,180), [180, 225), [225, 270), [270, 315), [315, 360)
            //   -----> x
            //    |
            //    |
            //   \ / y
            if(VnlPlus::isEqualZero(dx))
            {
                if(VnlPlus::isEqualZero(dy))
                {
                    grad_direction(i, j) = 0;
                }
                else
                {
                    grad_direction(i, j) = 2;
                }
            }
            else
            {
                // 0.4142 --> 22.5^o, 2.4142--> 657.5^o
                double tanV = dy/dx;
                if(tanV > -0.4142f && tanV <= 0.4142f)
                {
                    grad_direction(i, j) = 0;
                }
                else if(tanV > 0.4142f && tanV < 2.4142f)
                {
                    grad_direction(i, j) = 1;
                }
                else if(abs(tanV) >= 2.4142f)
                {
                    grad_direction(i, j) = 2;
                }
                else if(tanV > -2.4142f && tanV <= -0.4142f)
                {
                    grad_direction(i, j) = 3;
                }
            }
        }
    }
    
    vil_image_view<double> grad_canny_mag = vil_image_view<double>(width, height, 1);
    vil_image_view<bool> is_canny_edge = vil_image_view<bool>(width, height, 1);
    grad_canny_mag.fill(0);
    is_canny_edge.fill(false);
    
    // compare in 8 directions
    for (int j = 1; j<height-1; j++) {
        for (int i = 1; i<width-1; i++) {
            double v = grad_mag2(i, j);
            if (v < para.min_gradient_magnitude2_) {  // filter small edgelet
                continue;
            }
            switch (grad_direction(i, j)) {
                case 0:
                if (v > grad_mag2(i-1, j) && v > grad_mag2(i+1, j)) {
                    grad_canny_mag(i, j) = sqrt(v);
                    is_canny_edge(i, j) = true;
                }
                break;
                case 1:
                if (v > grad_mag2(i-1, j-1) && v > grad_mag2(i+1, j+1)) {
                    grad_canny_mag(i, j) = sqrt(v);
                    is_canny_edge(i, j) = true;
                }
                break;
                case 2:
                if (v > grad_mag2(i, j-1) && v > grad_mag2(i, j+1)) {
                    grad_canny_mag(i, j) = sqrt(v);
                    is_canny_edge(i, j) = true;
                }
                break;
                case 3:
                if (v > grad_mag2(i-1, j+1) && v > grad_mag2(i+1, j-1)) {
                    grad_canny_mag(i, j) = sqrt(v);
                    is_canny_edge(i, j) = true;
                }
                break;
                default:
                assert(0);
                break;
            }
        }
    }
    
    const double cosThetaThreshold = cos(para.max_theta_*vnl_math::pi/180.0);
  //  edgeletMask = vil_image_view<vxl_byte>(width, height, 1);
  //  edgeletMask.fill(0);
    
    // second moment of gradient for dominent eigen vector
    vnl_matrix<double> M(2, 2, 0);
    int num = 0;
    for (int y = 0; y<height; y++)
    {
        for (int x = 0; x<width; x++)
        {
            if (!is_canny_edge(x, y)) {
                continue;
            }
            double gi = grad_i(x, y, 0);
            double gj = grad_j(x, y, 0);
            
            double xx = gi * gi;
            double yy = gj * gj;
            double xy = gi * gj;
            M(0, 0) += xx;
            M(0, 1) += xy;
            M(1, 0) += xy;
            M(1, 1) += yy;
            num++;
        }
    }
    
    if (num < para.min_edgelet_in_patch_) {
        return TOO_FEW_EDGE_POINT;
    }
    M /= num;
    
    vnl_svd<double> svd(M);
    if (svd.singularities() != 0)
    {
        return SVD_SINGULAR;
    }
    
    vnl_matrix<double> eigen_vectors = svd.V();
    vnl_vector<double> nHat = eigen_vectors.get_column(0).normalize();  //dominant eigen vector in the patch
    assert(nHat.size() == 2);
    
    //equation 9, project position to normal direction
    num = 0;
    vcl_vector<double> b;
    for (int y = 0; y<height; y++)
    {
        for (int x = 0; x<width; x++)
        {
            if (!is_canny_edge(x, y)) {
                continue;
            }
            double gi = grad_i(x, y, 0);
            double gj = grad_j(x, y, 0);
            // equation 7
            double cosTheta = (nHat[0] * gi + nHat[1] * gj)/sqrt(grad_mag2(x, y));
            if (fabs(cosTheta) >= cosThetaThreshold)
            {
                // equation 9
                double bj = x * nHat[0] + y * nHat[1];
                b.push_back(bj);
                num++;
            }
        }
    }    
    
    if (num < para.min_edgelet_in_patch_) {
        return DOMINAT_ANGLE;
    }
    
    double b_mean = 0;
    double b_sigma = 0;
    VnlPlus::mean_std(&b[0], (unsigned int)b.size(), b_mean, b_sigma);
    
    if (b_sigma > para.location_variance_) {
        return LOCATION_VARIANCE;
    }
    
    // get candidate points for line
    vcl_vector<vgl_point_2d<double> > line_pts;
    double line_pts_center_x = 0.0;
    double line_pts_center_y = 0.0;
    for (int y = 0; y<height; y++)
    {
        for (int x = 0; x<width; x++)
        {
            if (!is_canny_edge(x, y)) {
                continue;
            }
            double gi = grad_i(x, y, 0);
            double gj = grad_j(x, y, 0);
            
            double cosTheta = (nHat[0] * gi + nHat[1] * gj)/sqrt(grad_mag2(x, y));
            double bj = x * nHat[0] + y * nHat[1];
            
            if (fabs(cosTheta) >= cosThetaThreshold && fabs(bj - b_mean) <= para.location_variance_)
            {
                num++;
           //     edgeletMask(x, y) = 255;
                line_pts.push_back(vgl_point_2d<double>(x, y));
                line_pts_center_x += x;
                line_pts_center_y += y;
            }
        }
    }
    
    // line has enough point
    if (line_pts.size() >= para.min_line_points_)
    {
        edgeletPts = line_pts;
        
        line_pts_center_x /= line_pts.size();
        line_pts_center_y /= line_pts.size();
        vgl_line_2d<double> line = vgl_fit_line_2d(line_pts);
        double len = 0.0;
        for (int k = 0; k<line_pts.size(); k++) {
            double x_dif = line_pts[k].x() - line_pts_center_x;
            double y_dif = line_pts[k].y() - line_pts_center_y;
            double dis = x_dif * x_dif + y_dif * y_dif;
            if (dis > len) {
                len = dis;
            }
        }
        len = sqrt(len);
        
      //  edgelet = Edgelet(vgl_point_2d<double>(line_pts_center_x, line_pts_center_y), line, len * 2);
        return Detected;
    }
    else
    {
      //  edgeletMask.fill(0);
        return TOO_FEW_LINE_POINTS;
    }
    assert(0);
    
    return Detected;
}

void VxlCannyEdgelet::canny(const vil_image_view<vxl_byte> & image,
                            const CannyParameter & para,
                            vil_image_view<vxl_byte> & edgeMask)
{
    int width = image.ni();
    int height = image.nj();
    
    vil_image_view<vxl_byte> intensityImage;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, intensityImage);
    }
    else
    {
        intensityImage = image;
    }
    
    // gaussian smoothing
    vil_image_view<double> smoothedImage = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(para.sigma_);
    vil_gauss_filter_5tap(intensityImage, smoothedImage, params);
    
    
    // gradient
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    vil_sobel_1x3(smoothedImage, grad_i, grad_j);
    
    // non-maxinum suppress
    vil_image_view<double> grad_mag2 = vil_image_view<double>(width, height, 1);
    vil_image_view<unsigned char> grad_direction = vil_image_view<unsigned char>(width, height, 1);
    grad_mag2.fill(0);
    grad_direction.fill(0);
    for (int j = 1; j<height; j++) {
        for (int i = 1; i<width; i++) {
            double dx = grad_i(i, j);
            double dy = grad_j(i, j);
            
            // mag^2
            grad_mag2(i, j) = dx*dx + dy*dy;
            
            //compute 8 directions: [0, 45), [45, 90), [90, 135), [135,180), [180, 225), [225, 270), [270, 315), [315, 360)
            //   -----> x
            //    |
            //    |
            //   \ / y
            if(VnlPlus::isEqualZero(dx))
            {
                if(VnlPlus::isEqualZero(dy))
                {
                    grad_direction(i, j) = 0;
                }
                else
                {
                    grad_direction(i, j) = 2;
                }
            }
            else
            {
                // 0.4142 --> 22.5^o, 2.4142--> 657.5^o
                double tanV = dy/dx;
                if(tanV > -0.4142f && tanV <= 0.4142f)
                {
                    grad_direction(i, j) = 0;
                }
                else if(tanV > 0.4142f && tanV < 2.4142f)
                {
                    grad_direction(i, j) = 1;
                }
                else if(abs(tanV) >= 2.4142f)
                {
                    grad_direction(i, j) = 2;
                }
                else if(tanV > -2.4142f && tanV <= -0.4142f)
                {
                    grad_direction(i, j) = 3;
                }
            }
        }
    }
    
    edgeMask = vil_image_view<vxl_byte>(width, height, 1);
    edgeMask.fill(0);
    
    // compare in 8 directions
    for (int j = 1; j<height-1; j++) {
        for (int i = 1; i<width-1; i++) {
            double v = grad_mag2(i, j);
            if (v < para.min_gradient_magnitude2_) {  // filter small edgelet
                continue;
            }
            switch (grad_direction(i, j)) {
                case 0:
                if (v > grad_mag2(i-1, j) && v > grad_mag2(i+1, j)) {
                    edgeMask(i, j) = 255;
                }
                break;
                case 1:
                if (v > grad_mag2(i-1, j-1) && v > grad_mag2(i+1, j+1)) {
                    edgeMask(i, j) = 255;
                }
                break;
                case 2:
                if (v > grad_mag2(i, j-1) && v > grad_mag2(i, j+1)) {
                    edgeMask(i, j) = 255;
                }
                break;
                case 3:
                if (v > grad_mag2(i-1, j+1) && v > grad_mag2(i+1, j-1)) {
                    edgeMask(i, j) = 255;
                }
                break;
                default:
                assert(0);
                break;
            }
        }
    }    
}

void VxlCannyEdgelet::canny(const vil_image_view<vxl_byte> & image,
                            const CannyParameter & para,
                            vil_image_view<vxl_byte> & edgeMask,
                            vil_image_view<double> & maginitude2)
{
    int width = image.ni();
    int height = image.nj();
    
    vil_image_view<vxl_byte> intensityImage;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, intensityImage);
    }
    else
    {
        intensityImage = image;
    }
    
    // gaussian smoothing
    vil_image_view<double> smoothedImage = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(para.sigma_);
    vil_gauss_filter_5tap(intensityImage, smoothedImage, params);
    
    
    // gradient
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    vil_sobel_1x3(smoothedImage, grad_i, grad_j);
    
    // non-maxinum suppress
    maginitude2 = vil_image_view<double>(width, height, 1);
    vil_image_view<unsigned char> grad_direction = vil_image_view<unsigned char>(width, height, 1);
    maginitude2.fill(0);
    grad_direction.fill(0);
    for (int j = 1; j<height; j++) {
        for (int i = 1; i<width; i++) {
            double dx = grad_i(i, j);
            double dy = grad_j(i, j);
            
            // mag^2
            maginitude2(i, j) = dx*dx + dy*dy;
            
            //compute 8 directions: [0, 45), [45, 90), [90, 135), [135,180), [180, 225), [225, 270), [270, 315), [315, 360)
            //   -----> x
            //    |
            //    |
            //   \ / y
            if(VnlPlus::isEqualZero(dx))
            {
                if(VnlPlus::isEqualZero(dy))
                {
                    grad_direction(i, j) = 0;
                }
                else
                {
                    grad_direction(i, j) = 2;
                }
            }
            else
            {
                // 0.4142 --> 22.5^o, 2.4142--> 657.5^o
                double tanV = dy/dx;
                if(tanV > -0.4142f && tanV <= 0.4142f)
                {
                    grad_direction(i, j) = 0;
                }
                else if(tanV > 0.4142f && tanV < 2.4142f)
                {
                    grad_direction(i, j) = 1;
                }
                else if(abs(tanV) >= 2.4142f)
                {
                    grad_direction(i, j) = 2;
                }
                else if(tanV > -2.4142f && tanV <= -0.4142f)
                {
                    grad_direction(i, j) = 3;
                }
            }
        }
    }
    
    edgeMask = vil_image_view<vxl_byte>(width, height, 1);
    edgeMask.fill(0);
    
    // compare in 8 directions
    for (int j = 1; j<height-1; j++) {
        for (int i = 1; i<width-1; i++) {
            double v = maginitude2(i, j);
            if (v < para.min_gradient_magnitude2_) {  // filter small edgelet
                continue;
            }
            switch (grad_direction(i, j)) {
                case 0:
                if (v > maginitude2(i-1, j) && v > maginitude2(i+1, j)) {
                    edgeMask(i, j) = 255;
                }
                break;
                case 1:
                if (v > maginitude2(i-1, j-1) && v > maginitude2(i+1, j+1)) {
                    edgeMask(i, j) = 255;
                }
                break;
                case 2:
                if (v > maginitude2(i, j-1) && v > maginitude2(i, j+1)) {
                    edgeMask(i, j) = 255;
                }
                break;
                case 3:
                if (v > maginitude2(i-1, j+1) && v > maginitude2(i+1, j-1)) {
                    edgeMask(i, j) = 255;
                }
                break;
                default:
                assert(0);
                break;
            }
        }
    }
}

void VxlCannyEdgelet::houghLines(const vil_image_view<vxl_byte> & edgeMask,
                                 vcl_vector<vnl_vector_fixed<double, 2> > &lines,
                                 double rho, double theta, int threshold)
{
    assert(0);
    
    int w = edgeMask.ni();
    int h = edgeMask.nj();
    
    int wTheta = 2.0 * vnl_math::pi/theta + 1;  // histogram width, store angle theta
    int hRho   = sqrt(w * w + h * h)/rho + 1;   // histogram height, store r, rho
    
    printf("wTheta, hRho is %d %d \n", wTheta, hRho);
    
    //
    vil_image_view<vxl_byte> histogram(wTheta, hRho, 1);
    histogram.fill(0);
    
    for (int y = 0; y<edgeMask.nj(); y++) {
        for (int x =0 ; x<edgeMask.ni(); x++) {
            if (edgeMask(x, y) == 255) {
                // vote in a line
                for (int k = 0; k<wTheta; k++) {
                    double theta = 1.0 * k / 180.0 * vnl_math::pi;
                    double r = x * cos(theta) + y * sin(theta);
                    int ir = vnl_math::rnd_halfintup(r);
                //    printf("rho is %d\n", ir);
                    if (ir >= 0 && ir < hRho) {
                        histogram(k, ir) += 1;
                    }
                }
            }
        }
    }
    
    
    VilPlus::vil_save(histogram, "hough_histogram.jpg");
    
    // filter local maximum above threshold
    for (int j = 1; j<histogram.nj()-1; j++) {
        for (int i = 1 ; i<histogram.ni()-1; i++) {
            if (histogram(i, j) >= threshold) {
                int val = histogram(i, j);
                if (val > histogram(i-1, j-1) && val > histogram(i, j-1) && val > histogram(i+1, j-1) &&
                    val > histogram(i-1, j) && val > histogram(i+1, j) &&
                    val > histogram(i-1, j+1) && val > histogram(i, j+1) && val > histogram(i+1, j+1)) {
                    double row   = j * rho;
                    double cur_theta = 1.0 * i/vnl_math::pi * theta;
                    lines.push_back(vnl_vector_fixed<double, 2>(row, cur_theta));
                }
            }
        }
    }
    printf("find %lu lines\n", lines.size());
}

/*
 Here image is an input raster;
 step is it's step; size characterizes it's ROI;
 rho and theta are discretization steps (in pixels and radians correspondingly).
 threshold is the minimum number of pixels in the feature for it
 to be a candidate for line. lines is the output
 array of (rho, theta) pairs. linesMax is the buffer size (number of pairs).
 Functions return the actual number of found lines.
 */

/*
static void
icvHoughLinesStandard( const CvMat* img, float rho, float theta,
                      int threshold, CvSeq *lines, int linesMax )
{
    cv::AutoBuffer<int> _accum, _sort_buf;
    cv::AutoBuffer<float> _tabSin, _tabCos;
    
    const uchar* image;
    int step, width, height;
    int numangle, numrho;
    int total = 0;
    int i, j;
    float irho = 1 / rho;
    double scale;
    
    CV_Assert( CV_IS_MAT(img) && CV_MAT_TYPE(img->type) == CV_8UC1 );
    
    image = img->data.ptr;
    step = img->step;
    width = img->cols;
    height = img->rows;
    
    numangle = cvRound(CV_PI / theta);
    numrho = cvRound(((width + height) * 2 + 1) / rho);
    
    _accum.allocate((numangle+2) * (numrho+2));
    _sort_buf.allocate(numangle * numrho);
    _tabSin.allocate(numangle);
    _tabCos.allocate(numangle);
    int *accum = _accum, *sort_buf = _sort_buf;
    float *tabSin = _tabSin, *tabCos = _tabCos;
    
    memset( accum, 0, sizeof(accum[0]) * (numangle+2) * (numrho+2) );
    
    float ang = 0;
    for(int n = 0; n < numangle; ang += theta, n++ )
    {
        tabSin[n] = (float)(sin((double)ang) * irho);
        tabCos[n] = (float)(cos((double)ang) * irho);
    }
    
    // stage 1. fill accumulator
    for( i = 0; i < height; i++ )
    for( j = 0; j < width; j++ )
    {
        if( image[i * step + j] != 0 )
        for(int n = 0; n < numangle; n++ )
        {
            int r = cvRound( j * tabCos[n] + i * tabSin[n] );
            r += (numrho - 1) / 2;
            accum[(n+1) * (numrho+2) + r+1]++;
        }
    }
    
    // stage 2. find local maximums
    for(int r = 0; r < numrho; r++ )
    for(int n = 0; n < numangle; n++ )
    {
        int base = (n+1) * (numrho+2) + r+1;
        if( accum[base] > threshold &&
           accum[base] > accum[base - 1] && accum[base] >= accum[base + 1] &&
           accum[base] > accum[base - numrho - 2] && accum[base] >= accum[base + numrho + 2] )
        sort_buf[total++] = base;
    }
    
    // stage 3. sort the detected lines by accumulator value
    icvHoughSortDescent32s( sort_buf, total, accum );
    
    // stage 4. store the first min(total,linesMax) lines to the output buffer
    linesMax = MIN(linesMax, total);
    scale = 1./(numrho+2);
    for( i = 0; i < linesMax; i++ )
    {
        CvLinePolar line;
        int idx = sort_buf[i];
        int n = cvFloor(idx*scale) - 1;
        int r = idx - (n+1)*(numrho+2) - 1;
        line.rho = (r - (numrho - 1)*0.5f) * rho;
        line.angle = n * theta;
        cvSeqPush( lines, &line );
    }
}
 */

void VxlCannyEdgelet::houghVoting(const vil_image_view<vxl_byte> & edgeMask,
                                  vil_image_view<vxl_int_32> & accumulatedMap,
                                  double rho, double theta)
{
    int width  = edgeMask.ni();
    int height = edgeMask.nj();
    
    // horizontal ---> angle
    // vertical   ---> distance
    
    double irho = 1.0 / rho; // save computation
    
    int numangle = vnl_math::rnd_halfintup(vnl_math::pi/theta);
    int numrho   = ((width + height) * 2 + 1) / rho;
    accumulatedMap = vil_image_view<vxl_int_32>(numangle+2, numrho+2);
    accumulatedMap.fill(0);
                                                
    float ang = 0;
    double *tabSin = new double[numangle];
    double *tabCos = new double[numangle];
    for(int n = 0; n < numangle; ang += theta, n++ )
    {
        tabSin[n] = (float)(sin((double)ang));
        tabCos[n] = (float)(cos((double)ang));
    }
    
    // stage 1. fill accumulator
    for(int j = 0; j < height; j++ )
    {
        for(int i = 0; i < width; i++ )
        {
            if( edgeMask(i, j) != 0 )
            {
                for(int n = 0; n < numangle; n++ )
                {
                    int r = vnl_math::rnd_halfintup((i * tabCos[n] + j * tabSin[n]) * irho);
                    r += (numrho - 1) / 2;
                    assert(r+1 >= 0 && r+1<accumulatedMap.nj());
                    accumulatedMap(n+1, r+1) += 1;
                }
            }
        }
    }
    // stage 2. find local maxinum
    
    //VilPlus::vil_save(accumulatedMap, "accumated_map.jpg");
    delete []tabSin;
    delete []tabCos;
}


/***********          Edgelet3D                           ***********/
Edgelet3D::Edgelet3D()
{
    x_ = vgl_point_3d<double>(0, 0, 0);
    d_ = vgl_vector_3d<double>(1.0, 0, 0);   
    
}
Edgelet3D::~Edgelet3D()
{
    
}

vgl_point_2d<double> Edgelet3D::projectCenter(const vpgl_perspective_camera<double> & camera) const
{
    return camera.project(x_);
}
vgl_vector_2d<double> Edgelet3D::projectDirection(const vpgl_perspective_camera<double> & camera) const
{
    vnl_matrix<double> Rmat = camera.get_rotation().as_matrix().as_matrix();
    vgl_point_3d<double> cc = camera.get_camera_center();  //camera center
    vnl_matrix<double> ccmat(3, 1, 0);
    ccmat(0, 0) = cc.x();
    ccmat(1, 0) = cc.y();
    ccmat(2, 0) = cc.z();
    
    vnl_matrix<double> xmat(3, 1, 0);
    xmat(0, 0) = x_.x();
    xmat(1, 0) = x_.y();
    xmat(2, 0) = x_.z();
    
    vnl_matrix<double> Xmat = Rmat * (xmat - ccmat); // location in camera coordinate
    
    // equation from E.Eade and T.Drummond "landmark in monocular SLAM", A.5-7    
    vnl_matrix<double> dmat(3, 1, 0);
    dmat(0, 0) = d_.x();
    dmat(1, 0) = d_.y();
    dmat(2, 0) = d_.z();
    vnl_matrix<double> D = Rmat * dmat; // rotate direction to the camera coordinate
    vgl_vector_2d<double> d_p;          // direction in image coordinate
    double X1 = Xmat(0, 0);
    double X2 = Xmat(1, 0);
    double X3 = Xmat(2, 0);
    double D1 = D(0, 0);
    double D2 = D(1, 0);
    double D3 = D(2, 0);
    double d_p_x = X3*D1 - X1*D3;
    double d_p_y = X3*D2 - X2*D3;
    d_p.set(d_p_x, d_p_y);
    double length = d_p.length();
    if (length != 0.0) {
        d_p.set(d_p.x()/length, d_p.y()/length);
    }
    return d_p;
}

vnl_matrix_fixed<double, 2, 12> Edgelet3D::jacobian_x(const vgl_rotation_3d<double> & R, const vgl_vector_3d<double> & T)
{
    // Appendex A. Edgelet observation model
    vnl_matrix<double> Rmat = R.as_matrix().as_matrix();
    vnl_matrix<double> Tmat = vnl_matrix<double>(3, 1);
    Tmat(0, 0) = T.x();
    Tmat(1, 0) = T.y();
    Tmat(2, 0) = T.z();
    vnl_matrix<double> x_mat = vnl_matrix<double>(3, 1);
    x_mat(0, 0) = x_.x();
    x_mat(1, 0) = x_.y();
    x_mat(2, 0) = x_.z();
    
    vnl_matrix_fixed<double, 2, 12> jaco;
    jaco.fill(0);
    
    vnl_matrix<double> X = Rmat * x_mat + Tmat;
    
    double X1 = X(0, 0);
    double X2 = X(1, 0);
    double X3 = X(2, 0);
    
    // denominator can't be too small
    if (VnlPlus::isEqualZero(X3)) {
        X3 = vnl_math::sqrteps;
    }
    
    // A.9
    vnl_matrix<double> d_xp_dX = vnl_matrix<double>(2, 3, 0);
    d_xp_dX(0, 0) = 1;
    d_xp_dX(0, 1) = 0;
    d_xp_dX(0, 2) = -X1/X3;
    d_xp_dX(1, 0) = 0;
    d_xp_dX(1, 1) = 1;
    d_xp_dX(1, 2) = -X2/X3;
    d_xp_dX /= X3;
    
    vnl_matrix<double> d_xp_dx = d_xp_dX * Rmat; // 1
    vnl_matrix<double> d_xp_dT = d_xp_dX;        // 2
    
    vnl_matrix<double> Xx = vnl_matrix<double>(3, 3, 0);
    Xx(0, 0) = 0.0;   Xx(0, 1) = -X3;  Xx(0, 2) = X2;
    Xx(1, 0) = X3;  Xx(1, 1) = 0.0;    Xx(1, 2) = -X1;
    Xx(2, 0) = -X2; Xx(2, 1) = X1;   Xx(2, 2) = 0.0;
    
    vnl_matrix<double> d_xp_dR = d_xp_dX * Xx;   // 3
    
    assert(d_xp_dx.rows() == 2 && d_xp_dx.cols() == 3);
    assert(d_xp_dT.rows() == 2 && d_xp_dT.cols() == 3);
    assert(d_xp_dR.rows() == 2 && d_xp_dR.cols() == 3);
    
    jaco.set_columns(0, d_xp_dx);
    // 3 as all zero
    jaco.set_columns(6, d_xp_dT);
    jaco.set_columns(9, d_xp_dR);
    
    return jaco;
}

vnl_matrix_fixed<double, 2, 12> Edgelet3D::jacobian_d(const vgl_rotation_3d<double> & R, const vgl_vector_3d<double> & T)
{
    // Appendex A. Edgelet observation model
    vnl_matrix<double> Rmat = R.as_matrix().as_matrix();
    vnl_matrix<double> Tmat = vnl_matrix<double>(3, 1);
    Tmat(0, 0) = T.x();
    Tmat(1, 0) = T.y();
    Tmat(2, 0) = T.z();
    vnl_matrix<double> x_mat = vnl_matrix<double>(3, 1);
    x_mat(0, 0) = x_.x();
    x_mat(1, 0) = x_.y();
    x_mat(2, 0) = x_.z();
    
    vnl_matrix_fixed<double, 2, 12> jaco;
    jaco.fill(0);
    
    vnl_matrix<double> X = Rmat * x_mat + Tmat;
    
    double X1 = X(0, 0);
    double X2 = X(1, 0);
    double X3 = X(2, 0);
    // denominator can't be too small
    if (VnlPlus::isEqualZero(X3)) {
        X3 = vnl_math::sqrteps;
    }
    vnl_matrix<double> dmat = vnl_matrix<double>(3, 1, 0);
    dmat(0, 0) = d_.x();
    dmat(1, 0) = d_.y();
    dmat(2, 0) = d_.z();
    
    vnl_matrix<double> D = Rmat * dmat;
    double D1 = D(0, 0);
    double D2 = D(1, 0);
    double D3 = D(2, 0);
    
    //
    vnl_matrix<double> Xx = vnl_matrix<double>(3, 3, 0);
    Xx(0, 0) = 0.0;   Xx(0, 1) = -X3;  Xx(0, 2) = X2;
    Xx(1, 0) = X3;  Xx(1, 1) = 0.0;    Xx(1, 2) = -X1;
    Xx(2, 0) = -X2; Xx(2, 1) = X1;   Xx(2, 2) = 0.0;
    
    vnl_matrix<double> Dx = vnl_matrix<double>(3, 3, 0);
    Dx(0, 0) = 0.0;   Dx(0, 1) = -D3;  Dx(0, 2) = D2;
    Dx(1, 0) = D3;    Dx(1, 1) = 0.0;  Dx(1, 2) = -D1;
    Dx(2, 0) = -D2;   Dx(2, 1) = D1;   Dx(2, 2) = 0.0;
    
    vnl_matrix<double> d_dp_d_X = vnl_matrix<double>(2, 3, 0); // A.13
    d_dp_d_X(0, 0) = -D3; d_dp_d_X(0, 1) = 0;   d_dp_d_X(0, 2) = D1;
    d_dp_d_X(1, 0) =  0;  d_dp_d_X(1, 1) = -D3; d_dp_d_X(1, 2) = D2;
    
    vnl_matrix<double> d_dp_d_x = d_dp_d_X * Rmat; // 1
    vnl_matrix<double> d_dp_d_D = vnl_matrix<double>(2, 3, 0);
    d_dp_d_D(0, 0) = X3; d_dp_d_D(0, 1) = 0;  d_dp_d_D(0, 2) = -X1;
    d_dp_d_D(1, 0) =  0; d_dp_d_D(1, 1) = X3; d_dp_d_D(1, 2) = -X2;
    
    vnl_matrix<double> d_dp_d_d = d_dp_d_D * Rmat;  // 2
    vnl_matrix<double> d_dp_d_T = d_dp_d_X;         // 3
    vnl_matrix<double> d_dp_d_R = d_dp_d_X * Xx + d_dp_d_D * Dx;  // 4
    
    assert(d_dp_d_x.rows() == 2 && d_dp_d_x.cols() == 3);
    assert(d_dp_d_d.rows() == 2 && d_dp_d_d.cols() == 3);
    assert(d_dp_d_T.rows() == 2 && d_dp_d_T.cols() == 3);
    assert(d_dp_d_R.rows() == 2 && d_dp_d_R.cols() == 3);
    
    jaco.set_columns(0, d_dp_d_x);
    jaco.set_columns(3, d_dp_d_d);
    jaco.set_columns(6, d_dp_d_T);
    jaco.set_columns(9, d_dp_d_R);
    
    return jaco;
    
}













































































