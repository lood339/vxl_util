//
//  vxl_ekf_feature_point.h
//  OnlineStereo
//
//  Created by jimmy on 12/27/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_ekf_feature_point__
#define __OnlineStereo__vxl_ekf_feature_point__

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include "vpgl_ptz_camera.h"

class VxlEKFCamera : public vpgl_ptz_camera
{
public:
    vnl_vector<double> state_;   // camera state: pan, tilt, zoom, pan_v, tilt_v, zoom_v
    vnl_matrix<double> cov_;     // 6 * 6 covariance matrix
    vnl_matrix<double> Q_;       // 6 * 6 processing noise, fixed
    
   // VxlEKFCamera(const vnl_vector<double> & s, const vnl_matrix<double> & cov, const vnl_matrix<double> & Q):vpgl_ptz_camera(s[0], s[1], s[2])
    VxlEKFCamera(const vnl_vector<double> & s, const vnl_matrix<double> & cov, const vnl_matrix<double> & Q):vpgl_ptz_camera()
    {
        printf("VxlEKFCamera is only for WWoS basketball court.\n");
        assert(0);
        assert(s.size() == 6);
        assert(cov.rows() == 6 && cov.cols() == 6);
        assert(Q.rows() == 6 && Q.cols() == 6);
        
        state_ = s;
        cov_ = cov;
        Q_ = Q;
        this->setPTZ(state_[0], state_[1], s[2]);
    }
    ~VxlEKFCamera()
    {
        
    }
};


class VxlEKFFeaturePoint
{
    vgl_point_3d<double> worldPt_;
    vgl_point_2d<double> imagePt_;
    
    vnl_matrix<double> covImage_; // 2x2, uncertainty in image space
    double covR_;                 // measurement error variance
    
    
public:
    vnl_matrix<double> Pyx_;     // 3x6, camera state, feature position covariance matrix
    vnl_matrix<double> Pyy_;     // 3x3, feature position variance matrix
    long long int id_;                // feature Id
    
public:
    VxlEKFFeaturePoint()
    {
        worldPt_.set(0, 0, 0);
        imagePt_.set(0, 0);
        covImage_ = vnl_matrix<double>(2, 2, 100);
        covR_ = 0.0001;
        Pyx_ = vnl_matrix<double>(3, 6, 0);
        Pyy_ = vnl_matrix<double>(3, 3, 0);
        id_ = -1;
    }
    VxlEKFFeaturePoint(const vgl_point_3d<double> &p, const vgl_point_2d<double> & q)
    {
        worldPt_ = p;
        imagePt_ = q;
        covImage_ = vnl_matrix<double>(2, 2, 100);
        covR_ = 0.0001;
        Pyx_ = vnl_matrix<double>(3, 6, 0);
        Pyy_ = vnl_matrix<double>(3, 3, 0);
        id_ = -1;
    }
    ~VxlEKFFeaturePoint();
    
    vgl_point_2d<double> imagePt() const {return imagePt_;}
    
    vgl_point_3d<double> worldPt() const {return worldPt_;}
    
    void setWorldPoint(double x, double y, double z)
    {
        worldPt_.set(x, y, z);
    }
    
    void setImagePoint(double x, double y)
    {
        imagePt_.set(x, y);
    }
    
    bool operator == (const VxlEKFFeaturePoint & other )const
    {
        return this->id_ == other.id_;
    }
    
    double R()const {return covR_;}
};





#endif /* defined(__OnlineStereo__vxl_ekf_feature_point__) */
