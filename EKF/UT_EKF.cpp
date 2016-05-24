//
//  UT_EKF.cpp
//  OnlineStereo
//
//  Created by jimmy on 6/20/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "UT_EKF.h"

#include "vxl_ekf.h"
#include "vxl_dynamic_ekf.h"
#include "vxl_ekf_feature_point.h"
#include "vxl_plus.h"
#include "vgl_plus.h"
#include "vxl_ptz_camera.h"
#include <vnl/vnl_matlab_filewrite.h>
#include "vpgl_plus.h"
#include <vnl/vnl_random.h>
#include "basketballCourt.h"
#include "DetectionReader.h"
#include "vnl_plus.h"



void test_EKF()
{
    //    test_EKF_interface_simulated_data();
    //    test_EKF_noise_ptz_data();
    //    test_EKF_PTZ_landmark_noise_ptz_data();
    //    test_EKF_Panonly_noise_ptz_data();
    test_EKF_remove_feature_landmark_noise_ptz_data();
}

class cameraPlaningPan_EKF: public VxlEKF
{
public:
    virtual vnl_vector<double> f(const vnl_vector<double> & X_k_1)
    {
        vnl_matrix<double> A(2, 2, 0);
        A(0, 0) = 1.0; A(0, 1) = 1.0;
        A(1, 0) = 0.0; A(1, 1) = 1.0;
        return A * X_k_1;
    }
    virtual vnl_vector<double> h(const vnl_vector<double> & X_k)
    {
        vnl_matrix<double> H(2, 2, 0);
        H.set_identity();
        return H * X_k;
    }
    virtual vnl_matrix<double> Jf(const vnl_vector<double> & X_k_1_a)
    {
        vnl_matrix<double> A(2, 2, 0);
        A(0, 0) = 1.0; A(0, 1) = 1.0;
        A(1, 0) = 0.0; A(1, 1) = 1.0;
        
        return A;
    }
    virtual vnl_matrix<double> Jh(const vnl_vector<double> & X_k_f)
    {
        vnl_matrix<double> H(2, 2, 0);
        H.set_identity();
        return H;
    }
};

void test_EKF_interface_simulated_data()
{
    
    vcl_string panFile("/Users/jimmy/Data/pan_regression/real_data/RF_raw_output.txt");
    vcl_string velocityFile("/Users/jimmy/Data/pan_regression/real_data/RF_raw_velocity.txt");
    vcl_string panGdFile("/Users/jimmy/Data/pan_regression/real_data/quarter_2_spherical_map/testing_fn_pan_features.txt");
    
    vnl_matrix<double> panMat;
    vnl_matrix<double> velocityMat;
    vnl_matrix<double> panGdMat;
    VnlPlus::readMat(panFile.c_str(), panMat);
    VnlPlus::readMat(velocityFile.c_str(), velocityMat);
    VnlPlus::readMat(panGdFile.c_str(), panGdMat);
    
    vcl_vector<double> observed_pan;
    vcl_vector<double> observed_velocity;
    for (int i = 0; i<panMat.rows(); i++) {
        observed_pan.push_back(panMat(i, 1));
        observed_velocity.push_back(velocityMat(i, 1));
    }
    
    vcl_vector<double> smoothed_pan;
    vcl_vector<double> smoothed_velocity;
    
    vnl_vector<double> x0(2, 0);
    x0[0] = 12; x0[1] = 0.0;
    
    vnl_matrix<double> P0(2, 2, 0);
    P0(0, 0) = 0.1; P0(1, 1) = 0.1;
    
    vnl_matrix<double> R(2, 2, 0);
    R(0, 0) = 0.06; R(1, 1) = 0.01;
    vnl_matrix<double> Q(2, 2, 0);
    Q(0, 0) = 0.00000004; Q(1, 1) = 0.00000001;
    
    cameraPlaningPan_EKF aEKF;
    aEKF.init(x0, P0);
    aEKF.initNoiseCovariance(Q, R);
    
    
    for (int i = 0; i<observed_pan.size(); i++) {
        vnl_vector<double> zk(2, 0);
        zk[0] = observed_pan[i];
        zk[1] = observed_velocity[i];
        
        vnl_vector<double> xk;
        vnl_matrix<double> pk;
        aEKF.update(zk, xk, pk);
        smoothed_pan.push_back(xk[0]);
        smoothed_velocity.push_back(xk[1]);
    }
    
    vnl_vector<double> smoothedPanVec(&smoothed_pan[0], (int)smoothed_pan.size());
    vnl_vector<double> smoothedVelocityVec(&smoothed_velocity[0], (int)smoothed_velocity.size());
    
    vcl_string save_file("EKF_observed_pan_velocity_26.mat");
    vnl_matlab_filewrite awriter(save_file.c_str());
    
    awriter.write(smoothedPanVec, "sm_pan");
    awriter.write(panMat.get_column(1), "pan");
    awriter.write(panGdMat.get_column(1), "gdPan");
    awriter.write(velocityMat.get_column(1), "velo");
    awriter.write(smoothedVelocityVec, "sm_velo");
    
    printf("save to %s\n", save_file.c_str());
}


class PTZCameraPlaning_EKF: public VxlEKF
{
public:
    virtual vnl_vector<double> f(const vnl_vector<double> & X_k_1)
    {
        vnl_matrix<double> A(6, 6, 0);
        A.set_identity();
        A(0, 3) = 1.0;
        A(1, 4) = 1.0;
        A(2, 5) = 1.0;
        return A * X_k_1;
    }
    virtual vnl_vector<double> h(const vnl_vector<double> & X_k)
    {
        vnl_matrix<double> H(6, 6, 0);
        H.set_identity();
        return H * X_k;
    }
    virtual vnl_matrix<double> Jf(const vnl_vector<double> & X_k_1_a)
    {
        vnl_matrix<double> A(6, 6, 0);
        A.set_identity();
        A(0, 3) = 1.0;
        A(1, 4) = 1.0;
        A(2, 5) = 1.0;
        return A;
    }
    virtual vnl_matrix<double> Jh(const vnl_vector<double> & X_k_f)
    {
        vnl_matrix<double> H(6, 6, 0);
        H.set_identity();
        return H;
    }
};

void test_EKF_noise_ptz_data()
{
    vcl_string ptz_file("/Users/jimmy/Desktop/images/33_slam_data/ptz_145420_145719.txt");
    vcl_vector<PTZData> ptzs;
    bool isRead = readPTZCameraFile(ptz_file.c_str(), ptzs, 1);
    assert(isRead);
    
    vcl_vector<double> gdPans;
    vcl_vector<double> gdTilts;
    vcl_vector<double> gdZooms;
    vcl_vector<double> observedPans;
    vcl_vector<double> observedTilts;
    vcl_vector<double> observedZooms;
    
    const int width = 1280;
    const int height = 720;
    vnl_random rnd;
    double delta = 2.0;
    vgl_point_2d<double> pp(width/2, height/2);
    for (int i = 0; i<ptzs.size(); i++) {
        gdPans.push_back(ptzs[i].pan);
        gdTilts.push_back(ptzs[i].tilt);
        gdZooms.push_back(ptzs[i].fl);
        
        vpgl_perspective_camera<double> gdCamera;
        bool isCamera = VxlPTZCamera::PTZToCamera(ptzs[i].fl, ptzs[i].pan, ptzs[i].tilt, gdCamera);
        assert(isCamera);
        
        // project image position
        vcl_vector<vgl_point_2d<double> > worldPts;
        vcl_vector<vgl_point_2d<double> > dump;
        vcl_vector<vgl_point_2d<double> > imagePts;
        DisneyWorldBasketballCourt::projectCalibPoints(gdCamera, width, height, worldPts, dump, imagePts, 0);
        assert(worldPts.size() >= 5);
        
        // add noise to image points
        for (int j = 0; j<imagePts.size(); j++) {
            double x = imagePts[j].x();
            double y = imagePts[j].y();
            x += delta * rnd.normal();
            y += delta * rnd.normal();
            imagePts[j].set(x, y);
        }
        
        // recalibrate camera
        vpgl_perspective_camera<double> initCamera;
        vpgl_perspective_camera<double> finalCamera;
        
        bool isInit = VpglPlus::init_calib(worldPts, imagePts, pp, initCamera);
        if (!isInit) {
            printf("initiate camera error\n");
            continue;
        }
        bool isFinal = VpglPlus::optimize_perspective_camera(worldPts, imagePts, initCamera, finalCamera);
        if (!isFinal) {
            printf("final camera error\n");
            continue;
        }
        //
        double pan = 0;
        double tilt = 0;
        double zoom = 0;
        bool isPTZ = VxlPTZCamera::CameraToPTZ(finalCamera, pan, tilt, zoom);
        assert(isPTZ);
        observedPans.push_back(pan);
        observedTilts.push_back(tilt);
        observedZooms.push_back(zoom);
    }
    
    assert(gdPans.size() == observedPans.size());
    assert(gdTilts.size() == observedTilts.size());
    assert(gdZooms.size() == observedZooms.size());
    
    double pan0 = gdPans[0];
    double tilt0 = gdTilts[0];
    double zoom0 = gdZooms[0];
    vnl_vector<double> x0(6, 0);
    x0[0] = pan0; x0[1] = tilt0; x0[2] = zoom0;
    
    vnl_matrix<double> P0(6, 6, 0);
    P0(0, 0) = 0.1;  P0(1, 1) = 0.1;  P0(2, 2) = 1.0;
    P0(3, 3) = 0.01; P0(4, 4) = 0.01; P0(5, 5) = 0.1;
    
    vnl_matrix<double> R(6, 6, 0);
    R(0, 0) = 0.06; R(1, 1) = 0.02;  R(2, 2) = 0.6;
    R(3, 3) = 0.01; R(4, 4) = 0.005; R(5, 5) = 0.1;
    
    vnl_matrix<double> Q(6, 6, 0);
    Q(0, 0) = 0.00000004; Q(1, 1) = 0.00000004; Q(2, 2) = 0.0000004;
    Q(3, 3) = 0.00000001; Q(4, 4) = 0.00000001; Q(5, 5) = 0.0000001;
    
    PTZCameraPlaning_EKF aEKF;
    aEKF.init(x0, P0);
    aEKF.initNoiseCovariance(Q, R);
    
    vcl_vector<double> smoothedPans;
    vcl_vector<double> smoothedTilts;
    vcl_vector<double> smoothedZooms;
    for (int i = 0; i<observedPans.size(); i++) {
        vnl_vector<double> zk(6, 0);
        zk[0] = observedPans[i];
        zk[1] = observedTilts[i];
        zk[2] = observedZooms[i];
        if (i >= 2) {
            zk[3] = smoothedPans[i-1]  - smoothedPans[i-2];
            zk[4] = smoothedTilts[i-1] - smoothedTilts[i-2];
            zk[5] = smoothedZooms[i-1] - smoothedZooms[i-2];
        }
        
        vnl_vector<double> xk;
        vnl_matrix<double> pk;
        aEKF.update(zk, xk, pk);
        smoothedPans.push_back(xk[0]);
        smoothedTilts.push_back(xk[1]);
        smoothedZooms.push_back(xk[2]);
    }
    
    // save
    vnl_vector<double> gdPanVec(&gdPans[0], (int)gdPans.size());
    vnl_vector<double> observedPanVec(&observedPans[0], (int)observedPans.size());
    vnl_vector<double> smoothedPanVec(&smoothedPans[0], (int)smoothedPans.size());
    
    vnl_vector<double> gdZoomVec(&gdZooms[0], (int)gdZooms.size());
    vnl_vector<double> observedZoomVec(&observedZooms[0], (int)observedZooms.size());
    vnl_vector<double> smoothedZoomVec(&smoothedZooms[0], (int)smoothedZooms.size());
    
    vnl_vector<double> gdTiltVec(&gdTilts[0], (int)gdTilts.size());
    vnl_vector<double> observedTiltVec(&observedTilts[0], (int)observedTilts.size());
    vnl_vector<double> smoothedTiltVec(&smoothedTilts[0], (int)observedTilts.size());
    
    
    vcl_string save_file("EKF_simulated_ptz.mat");
    vnl_matlab_filewrite awriter(save_file.c_str());
    
    awriter.write(gdPanVec, "gdPan");
    awriter.write(observedPanVec, "observedPan");
    awriter.write(smoothedPanVec, "sm_pan");
    awriter.write(gdZoomVec, "gdZoom");
    awriter.write(observedZoomVec, "observedZoom");
    awriter.write(smoothedZoomVec, "smoothedZoom");
    awriter.write(gdTiltVec, "gdTilt");
    awriter.write(vnl_vector<double>(&observedTilts[0], (int)observedTilts.size()), "observedTilt");
    awriter.write(vnl_vector<double>(&smoothedTilts[0], (int)observedTilts.size()), "smoothedTilt");
    
    printf("save to %s\n", save_file.c_str());
}

void test_EKF_PTZ_landmark_noise_ptz_data()
{
    vcl_string ptz_file("/Users/jimmy/Desktop/images/33_slam_data/ptz_145420_145719.txt");
    vcl_vector<PTZData> ptzs;
    bool isRead = readPTZCameraFile(ptz_file.c_str(), ptzs, 1);
    assert(isRead);
    
    vcl_vector<double> gdPans;
    vcl_vector<double> gdTilts;
    vcl_vector<double> gdZooms;
    vcl_vector<double> observedPans;
    vcl_vector<double> observedTilts;
    vcl_vector<double> observedZooms;
    
    vcl_vector<vcl_vector<vgl_point_2d<double> > > observedImagePts;
    
    const int width = 1280;
    const int height = 720;
    vnl_random rnd;
    double delta = 2.0;
    vgl_point_2d<double> pp(width/2, height/2);
    
    vcl_vector<vgl_point_2d<double> > firstWorldPts;
    for (int i = 0; i<ptzs.size(); i++) {
        gdPans.push_back(ptzs[i].pan);
        gdTilts.push_back(ptzs[i].tilt);
        gdZooms.push_back(ptzs[i].fl);
        
        vpgl_perspective_camera<double> gdCamera;
        bool isCamera = VxlPTZCamera::PTZToCamera(ptzs[i].fl, ptzs[i].pan, ptzs[i].tilt, gdCamera);
        assert(isCamera);
        
        // project image position
        vcl_vector<vgl_point_2d<double> > worldPts;
        vcl_vector<vgl_point_2d<double> > dump;
        vcl_vector<vgl_point_2d<double> > imagePts;
        DisneyWorldBasketballCourt::projectCalibPoints(gdCamera, width, height, worldPts, dump, imagePts, 10000);
        assert(worldPts.size() == 35);
        
        
        if (firstWorldPts.size() == 0) {
            firstWorldPts = worldPts;
        }
        assert(firstWorldPts.size() == worldPts.size());
        // add noise to image points
        for (int j = 0; j<imagePts.size(); j++) {
            double x = imagePts[j].x();
            double y = imagePts[j].y();
            x += delta * rnd.normal();
            y += delta * rnd.normal();
            imagePts[j].set(x, y);
        }
        observedImagePts.push_back(imagePts);
        
        
        // recalibrate camera
        vpgl_perspective_camera<double> initCamera;
        vpgl_perspective_camera<double> finalCamera;
        
        bool isInit = VpglPlus::init_calib(worldPts, imagePts, pp, initCamera);
        if (!isInit) {
            printf("initiate camera error\n");
            continue;
        }
        bool isFinal = VpglPlus::optimize_perspective_camera(worldPts, imagePts, initCamera, finalCamera);
        if (!isFinal) {
            printf("final camera error\n");
            continue;
        }
        //
        double pan = 0;
        double tilt = 0;
        double zoom = 0;
        bool isPTZ = VxlPTZCamera::CameraToPTZ(finalCamera, pan, tilt, zoom);
        assert(isPTZ);
        observedPans.push_back(pan);
        observedTilts.push_back(tilt);
        observedZooms.push_back(zoom);
    }
    
    assert(gdPans.size() == observedPans.size());
    assert(gdTilts.size() == observedTilts.size());
    assert(gdZooms.size() == observedZooms.size());
    
    double pan0 = gdPans[0];
    double tilt0 = gdTilts[0];
    double zoom0 = gdZooms[0];
    
    PTZCalibrationEKF ptzEKF;
    vnl_vector<double> x0(6 + (int)firstWorldPts.size() * 3, 0);
    x0[0] = pan0; x0[1] = tilt0; x0[2] = zoom0;
    for (int i = 0; i<firstWorldPts.size(); i++) {
        x0[6 + 3 * i + 0] = firstWorldPts[i].x();
        x0[6 + 3 * i + 1] = firstWorldPts[i].y();
        x0[6 + 3 * i + 2] = 0.0;
    }
    
    //todo, needs to improve
    const int M = (int)firstWorldPts.size();
    vnl_matrix<double> P0(x0.size(), x0.size(), 0);
    P0(0, 0) = 0.01;  P0(1, 1) = 0.01;  P0(2, 2) = 10;
    P0(3, 3) = 0.001; P0(4, 4) = 0.001; P0(5, 5) = 0.1;
    
    for (int i = 0; i<M*3; i++) {
        for (int j = 0; j<M*3; j++) {
            P0(6 + i, 6 + j) = 0.01;
        }
    }
    ptzEKF.init(x0, P0);
    
    
    vnl_matrix<double> Q(6 + M * 3, 6 + M * 3, 0);
    Q(0, 0) = 0.004; Q(1, 1) = 0.004; Q(2, 2) = 0.04;
    Q(3, 3) = 0.001; Q(4, 4) = 0.001; Q(5, 5) = 0.01;
    for (int i = 0; i<M * 3; i++) {
        Q(6 + i, 6 + i) = 0.0000001;
    }
    
    vnl_matrix<double> Rk(M*2, M*2, 0);
    for (int i = 0; i<M*2; i++) {
        Rk(i, i) = 0.1;
    }
    ptzEKF.initNoiseCovariance(Q, Rk);
    
    vcl_vector<vnl_vector<double> > states;
    vcl_vector<double> smoothedPans;
    vcl_vector<double> smoothedTilts;
    vcl_vector<double> smoothedZooms;
    
    for (int i = 0; i<observedPans.size(); i++) {  // observedPans.size()
        vnl_vector<double> zk(2 * M, 0);  // initialize landmarks
        
        printf("gd       pan tilt focal length is %f %f %f\n", gdPans[i], gdTilts[i], gdZooms[i]);
        printf("observed pan tilt focal length is %f %f %f\n", observedPans[i], observedTilts[i], observedZooms[i]);
        // add observed point in the image space
        for (int j = 0; j<M; j++) {
            zk[j * 2]     = observedImagePts[i][j].x();
            zk[j * 2 + 1] = observedImagePts[i][j].y();
        }
        
        vnl_vector<double> xk;
        vnl_matrix<double> pk;
        ptzEKF.updateOnce(zk, xk, pk);
        states.push_back(xk);
        printf("final  pan, tilt, focal lenth is %f %f %f\n\n", xk[0], xk[1], xk[2]);
        smoothedPans.push_back(xk[0]);
        smoothedTilts.push_back(xk[1]);
        smoothedZooms.push_back(xk[2]);
    }
    
    //save
    
    vnl_vector<double> gdPanVec(&gdPans[0], (int)gdPans.size());
    vnl_vector<double> observedPanVec(&observedPans[0], (int)observedPans.size());
    vnl_vector<double> smoothedPanVec(&smoothedPans[0], (int)smoothedPans.size());
    
    vnl_vector<double> gdZoomVec(&gdZooms[0], (int)gdZooms.size());
    vnl_vector<double> observedZoomVec(&observedZooms[0], (int)observedZooms.size());
    vnl_vector<double> smoothedZoomVec(&smoothedZooms[0], (int)smoothedZooms.size());
    
    vnl_vector<double> gdTiltVec(&gdTilts[0], (int)gdTilts.size());
    vnl_vector<double> observedTiltVec(&observedTilts[0], (int)observedTilts.size());
    vnl_vector<double> smoothedTiltVec(&smoothedTilts[0], (int)observedTilts.size());
    
    
    vcl_string save_file("EKF_simulated_ptz.mat");
    vnl_matlab_filewrite awriter(save_file.c_str());
    
    awriter.write(gdPanVec, "gdPan");
    awriter.write(observedPanVec, "observedPan");
    awriter.write(smoothedPanVec, "sm_pan");
    awriter.write(gdZoomVec, "gdZoom");
    awriter.write(observedZoomVec, "observedZoom");
    awriter.write(smoothedZoomVec, "smoothedZoom");
    awriter.write(gdTiltVec, "gdTilt");
    awriter.write(vnl_vector<double>(&observedTilts[0], (int)observedTilts.size()), "observedTilt");
    awriter.write(vnl_vector<double>(&smoothedTilts[0], (int)observedTilts.size()), "smoothedTilt");
    
    printf("save to %s\n", save_file.c_str());
    
}

void test_EKF_Panonly_noise_ptz_data()
{
    vcl_string ptz_file("/Users/jimmy/Desktop/images/33_slam_data/ptz_145420_145719.txt");
    vcl_vector<PTZData> ptzs;
    bool isRead = readPTZCameraFile(ptz_file.c_str(), ptzs, 1);
    assert(isRead);
    
    vcl_vector<double> gdPans;
    vcl_vector<double> gdTilts;
    vcl_vector<double> gdZooms;
    vcl_vector<double> observedPans;
    vcl_vector<double> observedTilts;
    vcl_vector<double> observedZooms;
    
    vcl_vector<vcl_vector<vgl_point_2d<double> > > observedImagePts;
    
    const int width = 1280;
    const int height = 720;
    vnl_random rnd;
    double delta = 2.0;
    vgl_point_2d<double> pp(width/2, height/2);
    
    vcl_vector<vgl_point_2d<double> > firstWorldPts;
    for (int i = 0; i<ptzs.size(); i++) {
        gdPans.push_back(ptzs[i].pan);
        gdTilts.push_back(ptzs[i].tilt);
        gdZooms.push_back(ptzs[i].fl);
        
        vpgl_perspective_camera<double> gdCamera;
        bool isCamera = VxlPTZCamera::PTZToCamera(ptzs[i].fl, ptzs[i].pan, ptzs[i].tilt, gdCamera);
        assert(isCamera);
        
        // project image position
        vcl_vector<vgl_point_2d<double> > worldPts;
        vcl_vector<vgl_point_2d<double> > dump;
        vcl_vector<vgl_point_2d<double> > imagePts;
        DisneyWorldBasketballCourt::projectCalibPoints(gdCamera, width, height, worldPts, dump, imagePts, 10000);
        //  assert(worldPts.size() == 35);
        
        
        if (firstWorldPts.size() == 0) {
            firstWorldPts = worldPts;
        }
        assert(firstWorldPts.size() == worldPts.size());
        // add noise to image points
        for (int j = 0; j<imagePts.size(); j++) {
            double x = imagePts[j].x();
            double y = imagePts[j].y();
            x += delta * rnd.normal();
            y += delta * rnd.normal();
            imagePts[j].set(x, y);
        }
        observedImagePts.push_back(imagePts);
        
        
        // recalibrate camera
        vpgl_perspective_camera<double> initCamera;
        vpgl_perspective_camera<double> finalCamera;
        
        bool isInit = VpglPlus::init_calib(worldPts, imagePts, pp, initCamera);
        if (!isInit) {
            printf("initiate camera error\n");
            continue;
        }
        bool isFinal = VpglPlus::optimize_perspective_camera(worldPts, imagePts, initCamera, finalCamera);
        if (!isFinal) {
            printf("final camera error\n");
            continue;
        }
        //
        double pan = 0;
        double tilt = 0;
        double zoom = 0;
        bool isPTZ = VxlPTZCamera::CameraToPTZ(finalCamera, pan, tilt, zoom);
        assert(isPTZ);
        observedPans.push_back(pan);
        observedTilts.push_back(tilt);
        observedZooms.push_back(zoom);
    }
    
    assert(gdPans.size() == observedPans.size());
    assert(gdTilts.size() == observedTilts.size());
    assert(gdZooms.size() == observedZooms.size());
    
    double pan0 = gdPans[0];
    
    const int C_M = 2; //camera state length
    PanCalibrationEKF panEKF;
    vnl_vector<double> x0(C_M + (int)firstWorldPts.size() * 3, 0);
    x0[0] = pan0; x0[1] = 1e-10;
    for (int i = 0; i<firstWorldPts.size(); i++) {
        x0[C_M + 3 * i + 0] = firstWorldPts[i].x();
        x0[C_M + 3 * i + 1] = firstWorldPts[i].y();
        x0[C_M + 3 * i + 2] = 0.0;
    }
    
    //todo, needs to improve
    vnl_matrix<double> P0(x0.size(), x0.size(), 0);
    // P0(0, 0) = 0.1;  P0(1, 1) = INT_MAX;
    panEKF.init(x0, P0);
    
    const int M = (int)firstWorldPts.size();
    vnl_matrix<double> Q(C_M + M * 3, C_M + M * 3, 0);
    Q(0, 0) = 0.0004; Q(1, 1) = 0.0001;
    for (int i = 0; i<M; i++) {
        Q(C_M + i, C_M + i) = 0.0;
    }
    
    vnl_matrix<double> Rk(M*2, M*2, 0);
    for (int i = 0; i<M*2; i++) {
        Rk(i, i) = 0.05;
    }
    panEKF.initNoiseCovariance(Q, Rk);
    
    vcl_vector<vnl_vector<double> > states;
    vcl_vector<double> smoothedPan;
    
    for (int i = 0; i<observedPans.size(); i++) {
        vnl_vector<double> zk(2 * M, 0);  // initialize landmarks
        
        printf("observed pan             is %f\n", observedPans[i]);
        // add observed point in the image space
        for (int j = 0; j<M; j++) {
            zk[j * 2]     = observedImagePts[i][j].x();
            zk[j * 2 + 1] = observedImagePts[i][j].y();
        }
        
        vnl_vector<double> xk;
        vnl_matrix<double> pk;
        panEKF.set_tilt_focal_length(gdTilts[i], gdZooms[i]);
        panEKF.update(zk, xk, pk);
        states.push_back(xk);
        
        printf("final  pan, pan_velocity is %f %f\n", xk[0], xk[1]);
        printf("gd    pan,               is %f \n\n", gdPans[i]);
        //  break;
        smoothedPan.push_back(xk[0]);
    }
    
    vcl_string save_file("EKF_panOnly_simulated_noise.mat");
    vnl_matlab_filewrite awriter(save_file.c_str());
    
    awriter.write(vnl_vector<double>(&observedPans[0], (int)observedPans.size()), "observed_pan");
    awriter.write(vnl_vector<double>(&smoothedPan[0], (int)smoothedPan.size()), "smoothed_pan");
    awriter.write(vnl_vector<double>(&gdPans[0], (int)gdPans.size()), "gdPans");
    
    printf("save to %s\n", save_file.c_str());
}

void test_EKF_remove_feature_landmark_noise_ptz_data()
{
    vcl_string ptz_file("/Users/jimmy/Desktop/images/33_slam_data/ptz_145420_145719.txt");
    vcl_vector<PTZData> ptzs;
    bool isRead = readPTZCameraFile(ptz_file.c_str(), ptzs, 1);
    assert(isRead);
    
    vcl_vector<double> gdPans;
    vcl_vector<double> gdTilts;
    vcl_vector<double> gdZooms;
    vcl_vector<double> observedPans;
    vcl_vector<double> observedTilts;
    vcl_vector<double> observedZooms;
    
    vcl_vector<vcl_vector<vgl_point_2d<double> > > observedImagePts;
    
    const int width = 1280;
    const int height = 720;
    vnl_random rnd;
    double delta = 2.0;
    vgl_point_2d<double> pp(width/2, height/2);
    
    PTZKeypointDynamicEKF ptzDynamicEKF;
    
    //  const int M = (int)firstFeatures.size();
    
    vnl_vector<double> C0(6, 0);
    C0[0] = ptzs[0].pan;
    C0[1] = ptzs[0].tilt;
    C0[2] = ptzs[0].fl;
    
    vnl_matrix<double> CP0(6, 6, 0);
    CP0(0, 0) = 0.01;  CP0(1, 1) = 0.01;  CP0(2, 2) = 10;
    CP0(3, 3) = 0.001; CP0(4, 4) = 0.001; CP0(5, 5) = 0.1;
    
    vnl_matrix<double> CQ0(6, 6, 0);
    CQ0(0, 0) = 0.00000004; CQ0(1, 1) = 0.00000004; CQ0(2, 2) = 0.0000004;
    CQ0(3, 3) = 0.00000001; CQ0(4, 4) = 0.00000001; CQ0(5, 5) = 0.0000001;
    
    // construct feature a database
    vcl_vector<vgl_point_2d<double> > courtPoints = DisneyWorldBasketballCourt::getCalibPoints();
    vcl_list<VxlEKFFeaturePoint> featureDatabase;
    for (int i = 0; i<courtPoints.size(); i++) {
        vgl_point_3d<double> p(courtPoints[i].x(), courtPoints[i].y(), 0);
        vgl_point_2d<double> q(0, 0);
        
        VxlEKFFeaturePoint feat(p, q);
        feat.id_ = i;
        featureDatabase.push_back(feat);
    }
    
    
    // init camera and feature from ground truth of data set
    VxlEKFCamera camera(C0, CP0, CQ0);
    vcl_list<VxlEKFFeaturePoint> features;
    
    for (int i = 0; i<courtPoints.size(); i++) {
        vgl_homg_point_3d<double> p(courtPoints[i].x(), courtPoints[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        vgl_point_2d<double> q= camera.project(p);
        if (vgl_inside_image(q, width, height, 10)) {
            VxlEKFFeaturePoint feat(p, q);
            feat.id_ = i;
            features.push_back(feat);
        }
    }
    printf("initiate feature number is %lu\n", features.size());
    vnl_vector<double> Xk;
    vnl_matrix<double> Pk;
    bool isInit = ptzDynamicEKF.updateCameraFeature(camera, features, Xk, Pk);
    assert(isInit);
    
    vcl_vector<double> smoothedPans;
    vcl_vector<double> smoothedTilts;
    vcl_vector<double> smoothedZooms;
    
    for (int i = 1; i<ptzs.size(); i++) {  // ptzs.size()
        gdPans.push_back(ptzs[i].pan);
        gdTilts.push_back(ptzs[i].tilt);
        gdZooms.push_back(ptzs[i].fl);
        
        vpgl_perspective_camera<double> gdCamera;
        bool isCamera = VxlPTZCamera::PTZToCamera(ptzs[i].fl, ptzs[i].pan, ptzs[i].tilt, gdCamera);
        assert(isCamera);
        
        // remove features
        vcl_list<VxlEKFFeaturePoint>::iterator it = features.begin();
        while (it != features.end()) {
            vgl_point_3d<double> p = it->worldPt();
            if (gdCamera.is_behind_camera(vgl_homg_point_3d<double>(p.x(), p.y(), p.z(), 1.0)))
            {
                it = features.erase(it);
                printf("erase a feature\n");
                continue;
            }
            vgl_point_2d<double> q = gdCamera.project(p);
            if (!vgl_inside_image(q, width, height, 10)) {
                it = features.erase(it);
                printf("erase a feature\n");
                continue;
            }
            it++;
        }
        printf("feature number is %lu\n", features.size());
        
        // add features
        for (vcl_list<VxlEKFFeaturePoint>::iterator it = featureDatabase.begin(); it != featureDatabase.end(); it++) {
            vgl_point_3d<double> p = it->worldPt();
            if (gdCamera.is_behind_camera(vgl_homg_point_3d<double>(p.x(), p.y(), p.z(), 1.0)))
            {
                continue;
            }
            vgl_point_2d<double> q = gdCamera.project(p);
            if (vgl_inside_image(q, width, height, 10))
            {
                vcl_list<VxlEKFFeaturePoint>::iterator findIt = std::find(features.begin(), features.end(), *it);
                // cannot find same featuere (defined by id) in the features
                if (findIt == features.end()) {
                    VxlEKFFeaturePoint feat(p, q);
                    feat.id_ = it->id_;
                    features.push_back(feat);
                    
                    printf("add a new feature with id %d\n", (int)feat.id_);
                }
            }
            
        }
        
        vcl_vector<vgl_point_2d<double> > worldPts;
        vcl_vector<vgl_point_2d<double> > imagePts;
        // add noise to current observation
        for (vcl_list<VxlEKFFeaturePoint>::iterator it = features.begin(); it != features.end(); it++) {
            vgl_point_3d<double> p = it->worldPt();
            vgl_point_2d<double> q = gdCamera.project(p);
            double x = q.x();
            double y = q.y();
            x += delta * rnd.normal();
            y += delta * rnd.normal();
            it->setImagePoint(x, y);
            
            worldPts.push_back(vgl_point_2d<double>(p.x(), p.y()));
            imagePts.push_back(it->imagePt());
        }
        vpgl_perspective_camera<double> initCamera;
        vpgl_perspective_camera<double> finalCamera;
        
        bool isInit = VpglPlus::init_calib(worldPts, imagePts, pp, initCamera);
        if (!isInit) {
            printf("initiate camera error\n");
            continue;
        }
        bool isFinal = VpglPlus::optimize_perspective_camera(worldPts, imagePts, initCamera, finalCamera);
        if (!isFinal) {
            printf("final camera error\n");
            continue;
        }
        //
        double pan = 0;
        double tilt = 0;
        double zoom = 0;
        bool isPTZ = VxlPTZCamera::CameraToPTZ(finalCamera, pan, tilt, zoom);
        assert(isPTZ);
        observedPans.push_back(pan);
        observedTilts.push_back(tilt);
        observedZooms.push_back(zoom);
        
        printf("observed  pan tilt focal length is %f %f %f\n", pan, tilt, zoom);
        ptzDynamicEKF.updateCameraFeature(camera, features, Xk, Pk);
        printf("gd        pan tilt focal length is %f %f %f\n\n", ptzs[i].pan, ptzs[i].tilt, ptzs[i].fl);
        
        smoothedPans.push_back(Xk[0]);
        smoothedTilts.push_back(Xk[1]);
        smoothedZooms.push_back(Xk[2]);
    }
    
    //save
    
    vnl_vector<double> gdPanVec(&gdPans[0], (int)gdPans.size());
    vnl_vector<double> observedPanVec(&observedPans[0], (int)observedPans.size());
    vnl_vector<double> smoothedPanVec(&smoothedPans[0], (int)smoothedPans.size());
    
    vnl_vector<double> gdZoomVec(&gdZooms[0], (int)gdZooms.size());
    vnl_vector<double> observedZoomVec(&observedZooms[0], (int)observedZooms.size());
    vnl_vector<double> smoothedZoomVec(&smoothedZooms[0], (int)smoothedZooms.size());
    
    vnl_vector<double> gdTiltVec(&gdTilts[0], (int)gdTilts.size());
    vnl_vector<double> observedTiltVec(&observedTilts[0], (int)observedTilts.size());
    vnl_vector<double> smoothedTiltVec(&smoothedTilts[0], (int)observedTilts.size());
    
    
    vcl_string save_file("EKF_dynamic_ptz.mat");
    vnl_matlab_filewrite awriter(save_file.c_str());
    
    awriter.write(gdPanVec, "gdPan");
    awriter.write(observedPanVec, "observedPan");
    awriter.write(smoothedPanVec, "sm_pan");
    awriter.write(gdZoomVec, "gdZoom");
    awriter.write(observedZoomVec, "observedZoom");
    awriter.write(smoothedZoomVec, "smoothedZoom");
    awriter.write(gdTiltVec, "gdTilt");
    awriter.write(vnl_vector<double>(&observedTilts[0], (int)observedTilts.size()), "observedTilt");
    awriter.write(vnl_vector<double>(&smoothedTilts[0], (int)observedTilts.size()), "smoothedTilt");
    
    printf("save to %s\n", save_file.c_str());
}
