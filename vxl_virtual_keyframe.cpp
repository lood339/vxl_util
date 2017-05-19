//
//  vxl_virtual_keyframe.cpp
//  CalibFromScene
//
//  Created by Jimmy Chen LOCAL on 7/14/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_virtual_keyframe.h"
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include "vxl_ptz_camera.h"
#include "basketballCourt.h"
#include "vxl_plus.h"
#include "vxl_vrel_plus.h"
#include "vpgl_plus.h"
#include <vil/vil_image_pyramid.h>
#include <vil/vil_image_pyramid.txx>
#include <vcl_algorithm.h>
#include "vil_plus.h"
#include "vpgl_plus_extra.h"
#include "vil_plus_extra.h"


bool VxlVirtualKeyframe::refinePTZCameraByVirtualKeyFrames(const vpgl_perspective_camera<double> & initCamera, const vil_image_view<vxl_byte> & image,
                                                           const vil_image_view<vxl_byte> & topviewImage,
                                                           const int patchSize, const int searchSize, vpgl_perspective_camera<double> & finalCamera)
{
    assert(image.nplanes() == 3);
    
    const bool isTest = false;
    
    int width = image.ni();
    int height = image.nj();
    assert(width == 1280);
    assert(height == 720);
    
    double init_fl = 0;
    double init_pan = 0;
    double init_tilt = 0;
    
    bool isPTZ = VxlPTZCamera::CameraToPTZ(initCamera, init_pan, init_tilt, init_fl);
    if (!isPTZ) {
        return false;
    }
    
    // generate nearby key frames
    vcl_vector<vil_image_view<vxl_byte> > warpedTopviews;
    vcl_vector<vpgl_perspective_camera<double> > cameras;
    DisneyWorldBasketballCourt court;
    for (double pan = init_pan - 1; pan <= init_pan + 1; pan += 1.0)
    {
        for (double tilt = init_tilt - 1; tilt <= init_tilt + 1; tilt += 1.0) {
            vil_image_view<vxl_byte> warpedImage;
            vpgl_perspective_camera<double> camera;
            bool isCamera = VxlPTZCamera::PTZToCamera(init_fl, pan, tilt, camera);
            if (isCamera) {
                bool isWarped = court.projectTopviewImage(topviewImage, camera, width, height, warpedImage, 20);
                if (isWarped) {
                    warpedTopviews.push_back(warpedImage);
                    cameras.push_back(camera);
                }
            }
            else
            {
                printf("Warning: PTZ to camera failed.\n");
            }
        }
    }
    assert(warpedTopviews.size() == cameras.size());
    
    // matching between image and warped key frames by pre-defined positions
    vcl_vector<vgl_point_2d<double> > wld_pts_sampled;
    vcl_vector<vgl_point_2d<double> > img_pts_sampled;
    
    for (int i = 0; i<warpedTopviews.size(); i++) {
        vpgl_perspective_camera<double> curCamera = cameras[i];
        vcl_vector<vgl_point_2d<double> > wld_2d_pts = DisneyWorldBasketballCourt::getPatchMatchingPoints();
        // project to query image and warped topview image and match them
        
        // for test
        vcl_vector<vgl_point_2d<double> > pts1;
        vcl_vector<vgl_point_2d<double> > pts2;
        
        for (int j = 0; j<wld_2d_pts.size(); j++) {
            vgl_point_3d<double> p(wld_2d_pts[j].x(), wld_2d_pts[j].y(), 0.0);
            
            vgl_point_2d<double> q1 = (vgl_point_2d<double>)curCamera.project(p);
            vgl_point_2d<double> q2 = (vgl_point_2d<double>)initCamera.project(p);
            
            if (!vgl_inside_image(q1, width, height, -patchSize)) {
                continue;
            }
            if (!vgl_inside_image(q2, width, height, -patchSize)) {
                continue;
            }
            
            vgl_point_2d<double> finalP;
            bool isRefine = VilPlusExtra::vil_refine_patch_position(warpedTopviews[i], q1, image, q2, patchSize, searchSize, finalP);
            if (isRefine) {
                if (isTest) {
                    pts1.push_back(q1);
                    pts2.push_back(finalP);
                }
                
                wld_pts_sampled.push_back(wld_2d_pts[j]);
                img_pts_sampled.push_back(finalP);
            }
        }
        if (isTest) {
            vil_image_view<vxl_byte> matchedImage;
            VilPlus::draw_match(warpedTopviews[i], image, pts1, pts2, matchedImage);
            char buf[1024] = {NULL};
            sprintf(buf, "patch_match_%d.jpg", i);
            VilPlus::vil_save(matchedImage, buf);
        }
    }
    
    // homography from world to image
    vcl_vector<bool> inlier;
    vgl_h_matrix_2d< double > H = VrelPlus::homography_RANSAC(wld_pts_sampled, img_pts_sampled, inlier, 1.0);
    
    // camera from H
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    vcl_vector<vgl_point_2d<double> > wld_2d_pts = DisneyWorldBasketballCourt::getCourtKeyPoints();
    
    for (int i = 0; i<wld_2d_pts.size(); i++) {
        vgl_point_2d<double> p = H(vgl_homg_point_2d<double>(wld_2d_pts[i]));
        if (vgl_inside_image(p, width, height, 10)) {
            wld_pts.push_back(wld_2d_pts[i]);
            img_pts.push_back(p);
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    if (wld_pts.size() < 5) {
        printf("Warning: matching number less 5, can't get optimized camera\n");
        return false;
    }
    
    //
    vpgl_perspective_camera<double> curInitCamera;
    bool isInit = VpglPlusExtra::init_calib(wld_pts, img_pts, vgl_point_2d<double>(width/2, height/2), curInitCamera);
    if (!isInit) {
        return false;
    }
    bool isOptimize = VpglPlus::optimize_perspective_camera(wld_pts, img_pts, curInitCamera, finalCamera);
    if (isOptimize && isTest) {
        double fl = 0;
        double pan = 0;
        double tilt = 0;
        VxlPTZCamera::CameraToPTZ(finalCamera, pan, tilt, fl);
        printf("dif in pan, tilt, fl: %f %f %f\n", pan - init_pan, tilt - init_tilt, fl - init_fl);
        
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        court.overlayAllLines(initCamera, showImage, VilPlus::green());
        VilPlus::vil_save(showImage, "camera_before_patch_match.jpg");
        
        showImage.deep_copy(image);
        court.overlayAllLines(finalCamera, showImage, VilPlus::green());
        VilPlus::vil_save(showImage, "camera_after_patch_match.jpg");
    }
    return isOptimize;
}

static void filterOutlierFromMedian(const vcl_vector<double> & data, vcl_vector<bool> & isInlier, double threshold)
{
    vcl_vector<double> data_copy = data;
    vcl_sort(data_copy.begin(), data_copy.end());
    double v_m = data_copy[data_copy.size()/2];
    for (int i = 0; i<data.size(); i++) {
        double v = data[i];
        if (fabs(v - v_m) < threshold) {
            isInlier.push_back(true);
        }
        else {
            isInlier.push_back(false);
        }
    }
    assert(isInlier.size() == data.size());
}

bool VxlVirtualKeyframe::refinePTZCameraByVirtualKeyFrames(const vpgl_perspective_camera<double> & initCamera, const vil_image_view<vxl_byte> & queryImage,
                                                           const vil_image_view<vxl_byte> & topviewImage, const double threshold,
                                                           const int patchSize, const int searchSize, vpgl_perspective_camera<double> & finalCamera)
{
    assert(queryImage.nplanes() == 3);
    
    const bool isTest = false;
    
    int width  = queryImage.ni();
    int height = queryImage.nj();
    assert(width == 1280);
    assert(height == 720);
    
    double init_fl = 0;
    double init_pan = 0;
    double init_tilt = 0;
    
    bool isPTZ = VxlPTZCamera::CameraToPTZ(initCamera, init_pan, init_tilt, init_fl);
    if (!isPTZ) {
        return false;
    }
    
    // generate nearby key frames
    vcl_vector<vil_image_view<vxl_byte> > warpedTopviews;
    vcl_vector<vpgl_perspective_camera<double> > cameras;
    DisneyWorldBasketballCourt court;
    for (double pan = init_pan - 1; pan <= init_pan + 1; pan += 1.0)
    {
        double tilt = init_tilt;
        //for (double tilt = init_tilt - 1; tilt <= init_tilt + 1; tilt += 1.0)
        {
            vil_image_view<vxl_byte> warpedImage;
            vpgl_perspective_camera<double> camera;
            bool isCamera = VxlPTZCamera::PTZToCamera(init_fl, pan, tilt, camera);
            if (isCamera) {
                bool isWarped = court.projectTopviewImage(topviewImage, camera, width, height, warpedImage, 20);
                if (isWarped) {
                    warpedTopviews.push_back(warpedImage);
                    cameras.push_back(camera);
                }
            }
            else
            {
                printf("Warning: PTZ to camera failed.\n");
            }
        }
    }
    assert(warpedTopviews.size() == cameras.size());
    
    vil_image_pyramid<vxl_byte>::parameters params(32, 2);
    vil_image_pyramid<vxl_byte> queryPyramid(queryImage, params);
    vil_image_view<vxl_byte> half_queryImage = queryPyramid[1];
    
    // matching between image and warped key frames by pre-defined positions
    vcl_vector<vgl_point_2d<double> > wld_pts_sampled;
    vcl_vector<vgl_point_2d<double> > img_pts_sampled;
    for (int ite = 0; ite<warpedTopviews.size(); ite++) {
        vpgl_perspective_camera<double> curCamera = cameras[ite];
        vcl_vector<vgl_point_2d<double> > wld_2d_pts = DisneyWorldBasketballCourt::getPatchMatchingPoints();
        // project to query image and warped topview image and match them
        
        vcl_vector<vgl_point_2d<double> > pts1;
        vcl_vector<vgl_point_2d<double> > pts2;
        vcl_vector<vgl_point_2d<double> > wld_pts;
        for (int j = 0; j<wld_2d_pts.size(); j++) {
            vgl_point_3d<double> p(wld_2d_pts[j].x(), wld_2d_pts[j].y(), 0.0);
            vgl_point_2d<double> q1 = (vgl_point_2d<double>)curCamera.project(p);
            vgl_point_2d<double> q2 = (vgl_point_2d<double>)initCamera.project(p);
            
            // patch corners have to be insider image
            if (!vgl_inside_image(q1, width, height, -patchSize/2 - 1)) {
                continue;
            }
            if (!vgl_inside_image(q2, width, height, -patchSize/2 - 1)) {
                continue;
            }
            pts1.push_back(q1);
            pts2.push_back(q2);
            wld_pts.push_back(wld_2d_pts[j]);
        }
        
        // downsample image and find matches
        vil_image_pyramid<vxl_byte> warpedImagePyramid(warpedTopviews[ite], params);
        vil_image_view<vxl_byte> half_warpedImage = warpedImagePyramid[1];
        for (int j = 0; j< pts1.size(); j++) {
            pts1[j].set(pts1[j].x()/2, pts1[j].y()/2);
            pts2[j].set(pts2[j].x()/2, pts2[j].y()/2);
        }
        vcl_vector<vgl_point_2d<double> > correspondence;
        bool isRefine = VilPlusExtra::vil_refine_patch_position(half_warpedImage, pts1, half_queryImage, pts2, patchSize/2, searchSize/2, correspondence);
        assert(isRefine);
        
        // test patch match result
        if(isTest)
        {
            vil_image_view<vxl_byte> matchImage;
            VilPlus::draw_match(half_warpedImage, half_queryImage, pts1, correspondence, matchImage);
            char buf[1024] = {NULL};
            sprintf(buf, "%d.jpg", ite);
            VilPlus::vil_save(matchImage, buf);
        }
        
        
        for (int j = 0; j<correspondence.size(); j++) {
            correspondence[j].set(correspondence[j].x() * 2, correspondence[j].y() * 2);
            pts1[j].set(pts1[j].x() * 2, pts1[j].y() * 2);
            assert(vgl_inside_image(correspondence[j], width, height));
        }
        
        // filter outliers by position displacement
        vcl_vector<double> x_dif;
        vcl_vector<double> y_dif;
        vcl_vector<bool> x_isInlier;
        vcl_vector<bool> y_isInlier;
        for (int j = 0; j<correspondence.size(); j++) {
            x_dif.push_back(correspondence[j].x() - pts1[j].x());
            y_dif.push_back(correspondence[j].y() - pts1[j].y());
        }
        
      //  for (int j = 0; j<x_dif.size(); j++) {
      //      printf("x y dif is %f %f\n", x_dif[j], y_dif[j]);
      //  }
        
        filterOutlierFromMedian(x_dif, x_isInlier, threshold);
        filterOutlierFromMedian(y_dif, y_isInlier, threshold);
        
        int inlier_num = 0;
        for (int j = 0; j<x_isInlier.size(); j++) {
            if (x_isInlier[j] && y_isInlier[j]) {
                wld_pts_sampled.push_back(wld_pts[j]);
                img_pts_sampled.push_back(correspondence[j]);
                inlier_num++;
            }
        }
        if (inlier_num < 5) {
            printf("warning: image correspondence number %d less than 5\n", inlier_num);
        }
    }
    
    // homography from world to image
    vcl_vector<bool> inlier;
    vgl_h_matrix_2d< double > H = VrelPlus::homography_RANSAC(wld_pts_sampled, img_pts_sampled, inlier, 1.0);
    
    // camera from H
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    vcl_vector<vgl_point_2d<double> > wld_2d_pts = DisneyWorldBasketballCourt::getCourtKeyPoints();
    
    for (int i = 0; i<wld_2d_pts.size(); i++) {
        vgl_point_2d<double> p = H(vgl_homg_point_2d<double>(wld_2d_pts[i]));
        if (vgl_inside_image(p, width, height, 10)) {
            wld_pts.push_back(wld_2d_pts[i]);
            img_pts.push_back(p);
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    if (wld_pts.size() < 5) {
        printf("Warning: matching number less 5, can't get optimized camera\n");
        return false;
    }
    
    //
    vpgl_perspective_camera<double> curInitCamera;
    bool isInit = VpglPlusExtra::init_calib(wld_pts, img_pts, vgl_point_2d<double>(width/2, height/2), curInitCamera);
    if (!isInit) {
        return false;
    }
    bool isOptimize = VpglPlus::optimize_perspective_camera(wld_pts, img_pts, curInitCamera, finalCamera);
    
    if (isOptimize && isTest) {
        double fl = 0;
        double pan = 0;
        double tilt = 0;
        VxlPTZCamera::CameraToPTZ(finalCamera, pan, tilt, fl);
        printf("dif in pan, tilt, fl: %f %f %f\n", pan - init_pan, tilt - init_tilt, fl - init_fl);
        
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(queryImage);
        court.overlayAllLines(initCamera, showImage, VilPlus::green());
        VilPlus::vil_save(showImage, "camera_before_patch_match.jpg");
        
        showImage.deep_copy(queryImage);
        court.overlayAllLines(finalCamera, showImage, VilPlus::green());
        VilPlus::vil_save(showImage, "camera_after_patch_match.jpg");
    }
    return isOptimize;
}


bool VxlVirtualKeyframe::refinePTZCameraByVirtualKeyFramesFullResolution(const vpgl_perspective_camera<double> & initCamera, const vil_image_view<vxl_byte> & queryImage,
                                                                         const vil_image_view<vxl_byte> & topviewImage, const double threshold,
                                                                         const int patchSize, const int searchSize, vpgl_perspective_camera<double> & finalCamera)
{
    assert(queryImage.nplanes() == 3);
    
    const bool isTest = false;
    
    int width  = queryImage.ni();
    int height = queryImage.nj();
    assert(width == 1280);
    assert(height == 720);
    
    double init_fl = 0;
    double init_pan = 0;
    double init_tilt = 0;
    
    bool isPTZ = VxlPTZCamera::CameraToPTZ(initCamera, init_pan, init_tilt, init_fl);
    if (!isPTZ) {
        return false;
    }
    
    // generate nearby key frames
    vcl_vector<vil_image_view<vxl_byte> > warpedTopviews;
    vcl_vector<vpgl_perspective_camera<double> > cameras;
    DisneyWorldBasketballCourt court;
    for (double pan = init_pan - 1; pan <= init_pan + 1; pan += 1.0)
    {
      //  for (double tilt = init_tilt - 1; tilt <= init_tilt + 1; tilt += 1.0)
        double tilt = init_tilt;
        {
            vil_image_view<vxl_byte> warpedImage;
            vpgl_perspective_camera<double> camera;
            bool isCamera = VxlPTZCamera::PTZToCamera(init_fl, pan, tilt, camera);
            if (isCamera) {
                bool isWarped = court.projectTopviewImage(topviewImage, camera, width, height, warpedImage, 20);
                if (isWarped) {
                    warpedTopviews.push_back(warpedImage);
                    cameras.push_back(camera);
                }
            }
            else
            {
                printf("Warning: PTZ to camera failed.\n");
            }
        }
    }
    assert(warpedTopviews.size() == cameras.size());
    
    
    // matching between image and warped key frames by pre-defined positions
    vcl_vector<vgl_point_2d<double> > wld_pts_sampled;
    vcl_vector<vgl_point_2d<double> > img_pts_sampled;
    for (int ite = 0; ite<warpedTopviews.size(); ite++) {
        vpgl_perspective_camera<double> curCamera = cameras[ite];
        vcl_vector<vgl_point_2d<double> > wld_2d_pts = DisneyWorldBasketballCourt::getPatchMatchingPoints();
        // project to query image and warped topview image and match them
        
        vcl_vector<vgl_point_2d<double> > pts1;
        vcl_vector<vgl_point_2d<double> > pts2;
        vcl_vector<vgl_point_2d<double> > wld_pts;
        for (int j = 0; j<wld_2d_pts.size(); j++) {
            vgl_point_3d<double> p(wld_2d_pts[j].x(), wld_2d_pts[j].y(), 0.0);
            vgl_point_2d<double> q1 = (vgl_point_2d<double>)curCamera.project(p);
            vgl_point_2d<double> q2 = (vgl_point_2d<double>)initCamera.project(p);
            
            // patch corners have to be insider image
            if (!vgl_inside_image(q1, width, height, -patchSize/2 - 1)) {
                continue;
            }
            if (!vgl_inside_image(q2, width, height, -patchSize/2 - 1)) {
                continue;
            }
            pts1.push_back(q1);
            pts2.push_back(q2);
            wld_pts.push_back(wld_2d_pts[j]);
        }
       
        vcl_vector<vgl_point_2d<double> > correspondence;
        bool isRefine = VilPlusExtra::vil_refine_patch_position(warpedTopviews[ite], pts1, queryImage, pts2, patchSize, searchSize, correspondence);
        assert(isRefine);
        
        // test patch match result
        if(isTest)
        {
            vil_image_view<vxl_byte> matchImage;
            VilPlus::draw_match(warpedTopviews[ite], queryImage, pts1, correspondence, matchImage);
            char buf[1024] = {NULL};
            sprintf(buf, "%d.jpg", ite);
            VilPlus::vil_save(matchImage, buf);
        }        
        
        
        
        // filter outliers by position displacement
        vcl_vector<double> x_dif;
        vcl_vector<double> y_dif;
        vcl_vector<bool> x_isInlier;
        vcl_vector<bool> y_isInlier;
        for (int j = 0; j<correspondence.size(); j++) {
            x_dif.push_back(correspondence[j].x() - pts1[j].x());
            y_dif.push_back(correspondence[j].y() - pts1[j].y());
        }
        
        //  for (int j = 0; j<x_dif.size(); j++) {
        //      printf("x y dif is %f %f\n", x_dif[j], y_dif[j]);
        //  }
        
        filterOutlierFromMedian(x_dif, x_isInlier, threshold);
        filterOutlierFromMedian(y_dif, y_isInlier, threshold);
        
        int inlier_num = 0;
        for (int j = 0; j<x_isInlier.size(); j++) {
            if (x_isInlier[j] || y_isInlier[j]) {
                wld_pts_sampled.push_back(wld_pts[j]);
                img_pts_sampled.push_back(correspondence[j]);
                inlier_num++;
            }
        }
        if (inlier_num < 5) {
            printf("warning: image correspondence number %d less than 5\n", inlier_num);
        }
    }
    
    // homography from world to image
    vcl_vector<bool> inlier;
    vgl_h_matrix_2d< double > H = VrelPlus::homography_RANSAC(wld_pts_sampled, img_pts_sampled, inlier, 1.0);
    
    // camera from H
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    vcl_vector<vgl_point_2d<double> > wld_2d_pts = DisneyWorldBasketballCourt::getCourtKeyPoints();
    
    for (int i = 0; i<wld_2d_pts.size(); i++) {
        vgl_point_2d<double> p = H(vgl_homg_point_2d<double>(wld_2d_pts[i]));
        if (vgl_inside_image(p, width, height, 10)) {
            wld_pts.push_back(wld_2d_pts[i]);
            img_pts.push_back(p);
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    if (wld_pts.size() < 5) {
        printf("Warning: matching number less 5, can't get optimized camera\n");
        return false;
    }
    
    //
    vpgl_perspective_camera<double> curInitCamera;
    bool isInit = VpglPlusExtra::init_calib(wld_pts, img_pts, vgl_point_2d<double>(width/2, height/2), curInitCamera);
    if (!isInit) {
        return false;
    }
    bool isOptimize = VpglPlus::optimize_perspective_camera(wld_pts, img_pts, curInitCamera, finalCamera);
    
    if (isOptimize && isTest) {
        double fl = 0;
        double pan = 0;
        double tilt = 0;
        VxlPTZCamera::CameraToPTZ(finalCamera, pan, tilt, fl);
        printf("dif in pan, tilt, fl: %f %f %f\n", pan - init_pan, tilt - init_tilt, fl - init_fl);
        
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(queryImage);
        court.overlayAllLines(initCamera, showImage, VilPlus::green());
        VilPlus::vil_save(showImage, "camera_before_patch_match.jpg");
        
        showImage.deep_copy(queryImage);
        court.overlayAllLines(finalCamera, showImage, VilPlus::green());
        VilPlus::vil_save(showImage, "camera_after_patch_match.jpg");
    }
    return isOptimize;
}

bool VxlVirtualKeyframe::optimizePatchSearchSize(const vpgl_perspective_camera<double> & initCamera, const vil_image_view<vxl_byte> & queryImage,
                             const vil_image_view<vxl_byte> & topviewImage, const double threshold, int & patchSize, int & searchSize)
{
    
    vcl_vector<vcl_pair<int, int> > patchsize_searchsize;
    patchsize_searchsize.push_back(vcl_pair<int, int> (30, 100));
    patchsize_searchsize.push_back(vcl_pair<int, int> (30, 150));
    patchsize_searchsize.push_back(vcl_pair<int, int> (40, 200));
    patchsize_searchsize.push_back(vcl_pair<int, int> (40, 250));
    patchsize_searchsize.push_back(vcl_pair<int, int> (50, 300));
    
    //get optimized patch seach size from down sample matching
    DisneyWorldBasketballCourt court;
    patchSize =  -1;
    searchSize = -1;
    double ssd_min = INT_MAX;
    
    for (int i = 0; i<patchsize_searchsize.size(); i++) {
        int sz1  = patchsize_searchsize[i].first;
        int sz2 = patchsize_searchsize[i].second;
        vpgl_perspective_camera<double> curCamera;
        bool isRefine = VxlVirtualKeyframe::refinePTZCameraByVirtualKeyFrames(initCamera, queryImage, topviewImage, threshold, sz1, sz2, curCamera);
        if (!isRefine) {
            printf("Warning: can't refine camera with patch, search size %d %d\n", patchSize, searchSize);
            continue;
        }
        
        double ssd = 0;
        double sad = 0;
        court.alignmentQuality(topviewImage, queryImage, curCamera, ssd, sad, true);
        if (ssd < ssd_min) {
            ssd_min = ssd;
            patchSize  = sz1;
            searchSize = sz2;
        }
    }
    
    return patchSize != -1;
}



