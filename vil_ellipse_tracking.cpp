//
//  vil_ellipse_tracking.cpp
//  OnlineStereo
//
//  Created by jimmy on 3/11/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_ellipse_tracking.h"
//#include "vil_gmm.h"

#include "vxl_vrel_plus.h"

bool VilEllipseTracking::trackingInBoundingBox(const vil_image_view<vxl_byte> & image, // for test only
                                               const vil_image_view<double> & magnitude,
                                               const ElliseTrackingParameter & trackingPara,
                                               vgl_ellipse_2d<double> & outEllipse)
{
    // detect edge pixels around the ellipse
    vcl_vector<vgl_point_2d<double> > outsideUpPixels;
    vcl_vector<vgl_point_2d<double> > outsideDownPixels;
 //   printf(" VilEllipseTracking::trackingInBoundingBox 1\n");
    VilEllipseTracking::detectEllipsePixelByAverageGradient(magnitude, trackingPara, outsideUpPixels, outsideDownPixels);
 //   printf(" VilEllipseTracking::trackingInBoundingBox 2\n");
    
    // fit an ellipse by RANSAC
    double threshold  = 2.0;
    double fail_ratio = 0.001;
    vcl_vector<vgl_point_2d<double> > inliers;
    int maxIter = 2000;
   
    bool isFitEllipse = false;
    if (outsideUpPixels.size() + outsideDownPixels.size() > trackingPara.min_inlier_num_) {
  //      printf(" VilEllipseTracking::trackingInBoundingBox 3\n");
        isFitEllipse = VrelPlus::fit_ellipse_RANSAC(outsideUpPixels, outsideDownPixels, threshold,fail_ratio, outEllipse, inliers, maxIter);
  //      printf(" VilEllipseTracking::trackingInBoundingBox 4\n");
    }
    
    return isFitEllipse && inliers.size() > trackingPara.min_inlier_num_;
}


bool VilEllipseTracking::detectEllipsePixelByAverageGradient(const vil_image_view<double> & mag,
                                                             const ElliseTrackingParameter & para,
                                                             vcl_vector<vgl_point_2d<double> > & outsideUpPixels,
                                                             vcl_vector<vgl_point_2d<double> > & outsideDownPixels)
{
    const int w = mag.ni();
    const int h = mag.nj();
    
    double minMag = para.mag_threshold_;
    double lambda = para.lambda_;
    
    int minX = para.bounding_box_.min_x();
    int maxX = para.bounding_box_.max_x();
    int minY = para.bounding_box_.min_y();
    int maxY = para.bounding_box_.max_y();
    assert(minX >= 0);
    assert(maxX < w);
    assert(minY >= 0);
    assert(maxY < h);
    
    // up to down
    for (int i = minX; i < maxX; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = minY; j<maxY; j++) {
            if (num >= 1) {
                if (mag(i, j) > minMag &&
                    mag(i, j) >= lambda * sumMag/num) {
                    outsideUpPixels.push_back(vgl_point_2d<double>(i, j));
                    break;
                }
            }
            sumMag += mag(i, j);
            num++;
        }
    }
    // down to up
    for (int i = minX ; i< maxX; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = maxY; j >= minY; j--) {
            if (num >= 1 && mag(i, j) > minMag &&
                mag(i, j) >= lambda * sumMag/num) {
                outsideDownPixels.push_back(vgl_point_2d<double>(i, j));
                break;
            }
            sumMag += mag(i, j);
            num++;
        }
    }
    return true;
}
