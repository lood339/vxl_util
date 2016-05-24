//
//  edgelet_parameter.h
//  OnlineStereo
//
//  Created by jimmy on 1/31/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef OnlineStereo_edgelet_parameter_h
#define OnlineStereo_edgelet_parameter_h

#include <vgl/vgl_line_2d.h>
#include <vnl/vnl_vector_fixed.h>
#include <vgl/vgl_fit_line_2d.h>


enum EdgeletDetectionResult{Detected = 0, TOO_FEW_EDGE_POINT, SVD_SINGULAR, DOMINAT_ANGLE, LOCATION_VARIANCE, TOO_FEW_LINE_POINTS};

struct Edgelet
{
    vgl_point_2d<double> center_;       // line segment center
    vgl_vector_2d<double> direction_;   // line direction
    double length_;                     // segment length
    vgl_line_2d<double> line_;          // 2D line equation
    
    Edgelet()
    {
        length_ = 0;
    }
    
    Edgelet(const vcl_vector<vgl_point_2d<double> > & pts)
    {
        assert(pts.size() > 5);
        double cx = 0.0;  //center point
        double cy = 0.0;
        for(int i = 0; i<pts.size(); i++)
        {
            cx += pts[i].x();
            cy += pts[i].y();
        }
        cx /= pts.size();
        cy /= pts.size();
        
        double len = 0.0;
        for (int k = 0; k<pts.size(); k++) {
            double x_dif = pts[k].x() - cx;
            double y_dif = pts[k].y() - cy;
            double dis = x_dif * x_dif + y_dif * y_dif;
            if (dis > len) {
                len = dis;
            }
        }
        len = sqrt(len);
        
        center_ = vgl_point_2d<double>(cx, cy);
        line_ = vgl_fit_line_2d(pts);
        direction_ = line_.direction();
        length_ = len * 2.0;       
        
    }
    
    Edgelet(const vgl_point_2d<double> & center, const vgl_line_2d<double> &line, double length)
    {
        center_ = center;
        direction_ = line.direction();
        length_ = length;
        line_ = line;
    }   
    
    vgl_point_2d<double> startPoint()
    {
        return vgl_point_2d<double>(center_.x() - length_ * direction_.x() * 0.5, center_.y() - length_ * direction_.y() * 0.5);
        
    }
    vgl_point_2d<double> endPoint()
    {
        return vgl_point_2d<double>(center_.x() + length_ * direction_.x() * 0.5, center_.y() + length_ * direction_.y() * 0.5);
    }
};


struct EdgeletParameter
{
    double sigma_;                   // gaussian smooth parameter for canny-like edge, first step
    double min_gradient_magnitude2_; // magnitude^2, edges
    int patchSize_;                  // patch size, edgelet only considered independet in each patch
    int min_edgelet_in_patch_;       // minimum edgelet number insize a patch
    double max_theta_;               // degree, difference between dominant normal and each edgelet candidate
    double location_variance_;       // std(b_j) in equation 9
    int min_line_points_;
    
    EdgeletParameter()
    {
        sigma_ = 1.4;
        min_gradient_magnitude2_ = 200;
        patchSize_ = 16;
        max_theta_ = 5.0;
        min_edgelet_in_patch_ = patchSize_/2;
        location_variance_ = 2.0;
        min_line_points_ = 10;
    }
};


#endif
