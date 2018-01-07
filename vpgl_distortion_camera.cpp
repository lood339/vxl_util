//
//  vpgl_distortion_camera.cpp
//  OpenCVCalib
//
//  Created by jimmy on 9/10/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vpgl_distortion_camera.h"
#include "vxlOpenCV.h"

vpgl_distortion_camera::vpgl_distortion_camera()
{
    
}
vpgl_distortion_camera::~vpgl_distortion_camera()
{
    
}

vnl_vector_fixed<double, 5> vpgl_distortion_camera::coefficient(void) const
{
    vnl_vector_fixed<double, 5> coeff;
    coeff[0] = radial_[0];
    coeff[1] = radial_[1];
    coeff[2] = radial_[2];
    coeff[3] = tangential_[0];
    coeff[4] = tangential_[1];
    return coeff;
}

vil_image_view<vxl_byte> vpgl_distortion_camera::undistort(const vil_image_view<vxl_byte> & image) const
{
    Mat cvImage = VxlOpenCVImage::cv_image(image);
    
    vnl_matrix<double> K = K_.get_matrix().as_matrix();
    cv::Mat camera_mat(3, 3, CV_64F);
    for (int r = 0; r<3; r++) {
        for (int c = 0; c<3; c++) {
            camera_mat.at<double>(r, c) = K(r, c);
        }
    }
    
    vnl_vector_fixed<double, 5> coeff = this->coefficient();
    cv::Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    for (int i = 0; i<coeff.size(); i++) {
        distCoeffs.at<double>(i, 0) = coeff[i];
    }
    
    cv::Mat undistorted_image;
    cv::undistort(cvImage, undistorted_image, camera_mat, distCoeffs);
    
    return VxlOpenCVImage::to_vil_image_view(undistorted_image);
}

vcl_vector<vgl_point_2d<double> > vpgl_distortion_camera::undistort(const vcl_vector<vgl_point_2d<double> > & pts)const
{
    vnl_matrix<double> K = K_.get_matrix().as_matrix();
    cv::Mat camera_mat(3, 3, CV_64F);
    for (int r = 0; r<3; r++) {
        for (int c = 0; c<3; c++) {
            camera_mat.at<double>(r, c) = K(r, c);
        }
    }

    vnl_vector_fixed<double, 5> coeff = this->coefficient();
    cv::Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
    for (int i = 0; i<coeff.size(); i++) {
        distCoeffs.at<double>(i, 0) = coeff[i];
    }
    
    std::vector<cv::Point2d> input_pts;
    std::vector<cv::Point2d> output_pts;
    for (int i = 0; i<pts.size(); i++) {
        input_pts.push_back(cv::Point2d(pts[i].x(), pts[i].y()));
    }
    
    cv::Mat P_mat(3, 4, CV_64F);
    for (int i = 0; i<3; i++) {
        for (int j = 0; j<3; j++) {
            P_mat.at<double>(i, j) = K(i, j);
        }
    }
    vgl_vector_3d<double> T = this->get_translation();
    P_mat.at<double>(0, 3) = T.x();
    P_mat.at<double>(1, 3) = T.y();
    P_mat.at<double>(2, 3) = T.z();
    
    // work here
    vcl_vector<vgl_point_2d<double> > ret_pts;
    cv::undistortPoints(Mat(input_pts), output_pts, camera_mat, distCoeffs, Mat(), P_mat);
    for (int i = 0; i<output_pts.size(); i++) {
        ret_pts.push_back(vgl_point_2d<double>(output_pts[i].x, output_pts[i].y));
    }
    assert(ret_pts.size() == pts.size());
    return ret_pts;
}

bool vpgl_distortion_camera::read(const char *file)
{
    FILE *pf = fopen(file, "r");
    if (!pf) {
        printf("can not read camera from %s\n", file);
        return false;
    }
    char buf[1024] = {NULL};
    fgets(buf, sizeof(buf), pf);
    // K
    vnl_matrix_fixed<double, 3, 3> K;
    vnl_matrix_fixed<double, 3, 3> R;
    vgl_vector_3d<double> T;
    int ret = 0;
    for (int i = 0; i<3; i++) {
        for (int j = 0; j<3; j++) {
            ret = fscanf(pf, "%lf", &(K[i][j]));
            assert(ret == 1);
        }
    }
    for (int i = 0; i<3; i++) {
        for (int j = 0; j<3; j++) {
            ret = fscanf(pf, "%lf", &(R[i][j]));
            assert(ret == 1);
        }
    }
    ret = fscanf(pf, "%lf %lf %lf", &(T.x_), &(T.y_), &(T.z_));
    assert(ret == 3);
    double coeff[5] = {NULL};
    ret = fscanf(pf, "%lf %lf %lf %lf %lf", &coeff[0], &coeff[1], &coeff[2], &coeff[3], &coeff[4]);
    assert(ret == 5);
    fclose(pf);
    
    this->set_calibration(vpgl_calibration_matrix<double>(K));
    this->set_rotation(vgl_rotation_3d<double>(R));
    this->set_translation(T);
    
    radial_[0] = coeff[0];
    radial_[1] = coeff[1];
    radial_[2] = coeff[4];
    tangential_[0] = coeff[2];
    tangential_[1] = coeff[3];
    return true;
}

bool vpgl_distortion_camera::write(const char *file) const
{
    return true;
}
