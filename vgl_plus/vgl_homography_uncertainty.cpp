//
//  homography_uncertainty.cpp
//  QuadCopter
//
//  Created by jimmy on 7/5/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vgl_homography_uncertainty.h"
#include <vgl/algo/vgl_h_matrix_2d.h>
#include "vnl_plus.h"
#include <vnl/algo/vnl_svd.h>
#include <vnl/vnl_matrix_fixed.h>
#include <vnl/vnl_matrix_fixed.txx>

VglHomographyUncertainty::VglHomographyUncertainty(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2)
{
    assert(pts1.size() == pts2.size());
    assert(pts1.size() >= 4);
    calculateUncertainty(pts1, pts2);
}

VglHomographyUncertainty::VglHomographyUncertainty(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2, double sigma_x, double sigma_y)
{
    assert(pts1.size() == pts2.size());
    assert(pts1.size() >= 4);
    assert(sigma_x > 0);
    assert(sigma_y > 0);
    calculateUncertainty(pts1, pts2, sigma_x, sigma_y);
}

VglHomographyUncertainty::~VglHomographyUncertainty()
{
    
}

// uncertainty of a point p, projected by H
vnl_matrix_fixed<double, 2, 2> VglHomographyUncertainty::cov(const vgl_point_2d<double> & p)
{
    vgl_point_2d<double> q = H_(vgl_homg_point_2d<double>(p));
    vnl_matrix_fixed<double, 2, 9> J;
    vnl_vector_fixed<double, 3> x_bar(p.x(), p.y(), 1.0);
    double x_prime = q.x();
    double y_prime = q.y();
    // dh/ dx
    vnl_vector_fixed<double, 9> Jx;
    Jx.fill(0);
    Jx.update(x_bar, 0);
    Jx.update(-x_prime * x_bar, 6);
    
    // dh/ dy
    vnl_vector_fixed<double, 9> Jy;
    Jy.fill(0);
    Jy.update(x_bar, 3);
    Jy.update(-y_prime * x_bar, 6);
    
    J.set_row(0, Jx);
    J.set_row(1, Jy);
    
    vnl_matrix_fixed<double, 2, 2> cov = J * sigma_h_ * J.transpose();
    return cov;
}

vgl_point_2d<double> VglHomographyUncertainty::project(const vgl_point_2d<double> & p)
{
    return H_(vgl_homg_point_2d<double>(p));
}

void VglHomographyUncertainty::calculateUncertainty(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2)
{
    vcl_vector<vgl_homg_point_2d<double> > h_pts1;
    vcl_vector<vgl_homg_point_2d<double> > h_pts2;
    for (int i = 0; i<pts1.size(); i++) {
        h_pts1.push_back(vgl_homg_point_2d<double>(pts1[i]));
        h_pts2.push_back(vgl_homg_point_2d<double>(pts2[i]));
    }
    
    H_ = vgl_h_matrix_2d<double>(h_pts1, h_pts2);
    
    // sigma x and sigma y in the second image
    vcl_vector<double> dx;
    vcl_vector<double> dy;
    for (int i = 0; i<h_pts1.size(); i++) {
        vgl_point_2d<double> p = (vgl_point_2d<double>)h_pts1[i]; // in first image
        vgl_point_2d<double> q = (vgl_point_2d<double>)H_(h_pts1[i]); // in second image
        
        dx.push_back(p.x() - q.x());
        dy.push_back(p.y() - q.y());
    }
    
    //
    double mean_x = 0;
    double sigma_x = 0;
    double mean_y = 0;
    double sigma_y = 0;
    VnlPlus::mean_std(&dx[0], (unsigned int)dx.size(), mean_x, sigma_x);
    VnlPlus::mean_std(&dy[0], (unsigned int)dy.size(), mean_y, sigma_y);
    
    sigma_x += 0.000001; // prevent zero
    sigma_y += 0.000001;
    
    // add all sample points togethter
    vnl_matrix_fixed<double, 2, 2> sigma_xy;
    sigma_xy.fill(0);
    sigma_xy[0][0] = 1.0/sigma_x;
    sigma_xy[1][1] = 1.0/sigma_y;
    vnl_matrix_fixed<double, 9, 9> JtJ_total;
    JtJ_total.fill(0);
    
    // p146 MVG in computer vision
    for (int i = 0; i<pts1.size(); i++) {
        vnl_matrix_fixed<double, 2, 9> J;   // for each pair of (p, q)
        vnl_vector_fixed<double, 3> x_bar(pts1[i].x(), pts1[i].y(), 1.0);
        double x_prime = pts2[i].x();
        double y_prime = pts2[i].y();
        vnl_vector_fixed<double, 9> Jix;
        Jix.fill(0);
        Jix.update(x_bar, 0);
        Jix.update(-x_prime * x_bar, 6);
        
        vnl_vector_fixed<double, 9> Jiy;
        Jiy.fill(0);
        Jiy.update(x_bar, 3);
        Jiy.update(-y_prime * x_bar, 6);
        
        J.set_row(0, Jix);
        J.set_row(1, Jiy);
     
        vnl_matrix_fixed<double, 9, 9> curJtSimgaXYJ = J.transpose() * sigma_xy * J;
        JtJ_total += curJtSimgaXYJ;
    }
    
    vnl_svd<double> svd(JtJ_total);
    sigma_h_ = svd.pinverse(8);
    assert(sigma_h_.rows() == 9 && sigma_h_.cols() == 9);
}

void VglHomographyUncertainty::calculateUncertainty(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2,
                                                 double sigma_x, double sigma_y)
{
    
    vcl_vector<vgl_homg_point_2d<double> > h_pts1;
    vcl_vector<vgl_homg_point_2d<double> > h_pts2;
    for (int i = 0; i<pts1.size(); i++) {
        h_pts1.push_back(vgl_homg_point_2d<double>(pts1[i]));
        h_pts2.push_back(vgl_homg_point_2d<double>(pts2[i]));
    }
    
    H_ = vgl_h_matrix_2d<double>(h_pts1, h_pts2);
    
    // add all sample points togethter
    vnl_matrix_fixed<double, 2, 2> sigma_xy;
    sigma_xy.fill(0);
    sigma_xy[0][0] = 1.0/sigma_x;
    sigma_xy[1][1] = 1.0/sigma_y;
    vnl_matrix_fixed<double, 9, 9> JtJ_total;
    JtJ_total.fill(0);
    
    // p146 MVG in computer vision
    for (int i = 0; i<pts1.size(); i++) {
        vnl_matrix_fixed<double, 2, 9> J;   // for each pair of (p, q)
        vnl_vector_fixed<double, 3> x_bar(pts1[i].x(), pts1[i].y(), 1.0);
        double x_prime = pts2[i].x();
        double y_prime = pts2[i].y();
        vnl_vector_fixed<double, 9> Jix;
        Jix.fill(0);
        Jix.update(x_bar, 0);
        Jix.update(-x_prime * x_bar, 6);
        
        vnl_vector_fixed<double, 9> Jiy;
        Jiy.fill(0);
        Jiy.update(x_bar, 3);
        Jiy.update(-y_prime * x_bar, 6);
        
        J.set_row(0, Jix);
        J.set_row(1, Jiy);
        
        vnl_matrix_fixed<double, 9, 9> curJtSimgaXYJ = J.transpose() * sigma_xy * J;
        JtJ_total += curJtSimgaXYJ;
    }
    
    vnl_svd<double> svd(JtJ_total);
    sigma_h_ = svd.pinverse(8);
    assert(sigma_h_.rows() == 9 && sigma_h_.cols() == 9);
 //   vcl_cout<<"pseudo inverse is \n"<<sigma_h_*18<<vcl_endl;
}
