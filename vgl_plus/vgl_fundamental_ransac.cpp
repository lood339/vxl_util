//
//  vgl_fundamental_ransac.cpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-11-30.
//  Copyright © 2015 jimmy. All rights reserved.
//

#include "vgl_fundamental_ransac.hpp"
//#include <mvl/FMatrixComputeRobust.h>
//#include <mvl/FMatrixComputeRANSAC.h>
#include "cvxImage_310.hpp"

/*
bool vgl_fundamental_ransac(vcl_vector< vgl_point_2d< double > > const& first,
                            vcl_vector< vgl_point_2d< double > > const& second,
                            vnl_matrix_fixed< double, 3, 3 > & F,
                            vcl_vector<bool> & inliers,
                            const fundamental_ransac_parameter & param)
{
    assert(0);
    assert(first.size() == second.size());
    if(first.size() <= 10)
    {
        return false;
    }
    
    vcl_vector<vgl_homg_point_2d<double> > points1;
    vcl_vector<vgl_homg_point_2d<double> > points2;
    
    for (int i = 0; i<first.size(); i++) {
        points1.push_back(vgl_homg_point_2d<double>(first[i]));
        points2.push_back(vgl_homg_point_2d<double>(second[i]));
    }
    
    // Perform the fit using Phil Torr's Robust Sampling Consensus
    FMatrixComputeRANSAC computor(true, 2);
    printf("1\n");
    FMatrix f = computor.compute(points1, points2);
    printf("2\n");
    f.set_rank2_using_svd();
    printf("3\n");
    
    inliers = computor.get_inliers();
    int inlier_num = 0;
    for (int i = 0; i<inliers.size(); i++) {
        if (inliers[i]) {
            inlier_num += 1;
        }
    }
    vcl_vector<double> residuals = computor.get_residuals();
    for (int i = 0; i<residuals.size(); i++) {
        printf("residual is %f\n", residuals[i]);
    }
    printf("\n");
    printf("matching, inlier are (%lu %d)\n", inliers.size(), inlier_num);
    return true;
}
*/

bool vgl_fundamental_ransac_opencv(vcl_vector< vgl_point_2d< double > > const& first,
                                   vcl_vector< vgl_point_2d< double > > const& second,
                                   vnl_matrix_fixed< double, 3, 3 > & F,
                                   vcl_vector<bool> & inliers,
                                   const fundamental_ransac_parameter & param)
{
    assert(first.size() == second.size());
    if(first.size() <= 8) // 8 is from opencv source code
    {
        return false;
    }
    
    std::vector<cv::Point2d> points1;
    std::vector<cv::Point2d> points2;
    for (int i = 0; i<first.size(); i++) {
        points1.push_back(cv::Point2d(first[i].x(), first[i].y()));
        points2.push_back(cv::Point2d(second[i].x(), second[i].y()));
    }    
 
    double max_epipolar_distance = param.max_epipolar_distance;
    double confidence_prob = param.confidence_prob;
    std::vector<uchar> inlier_mask;
    cv::Mat f = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, max_epipolar_distance, confidence_prob, inlier_mask);
    if (f.rows == 3 && f.cols == 3) {
        assert(inlier_mask.size() == points1.size());
        inliers.resize(inlier_mask.size());
        for (int i = 0; i<inlier_mask.size(); i++) {
            if (inlier_mask[i] == 1) {
                inliers[i] = true;
            }
            else{
                inliers[i] = false;
            }
        }
        for (int i = 0; i<3; i++) {
            for (int j = 0; j<3; j++) {
                F(i, j) = f.at<double>(i, j);
            }
        }
        
        return true;
    }
    else
    {
        return false;
    }
}
