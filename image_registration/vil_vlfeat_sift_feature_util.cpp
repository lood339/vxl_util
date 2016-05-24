//
//  vil_vlfeat_sift_feature_util.cpp
//  VideoCalibVXL
//
//  Created by jimmy on 7/18/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_vlfeat_sift_feature_util.h"
#include "basketballCourt.h"
#include <vgl/vgl_transform_2d.h>
#include <bapl/bapl_lowe_keypoint.h>
#include <vil/vil_crop.h>


#include <bapl/bapl_keypoint_extractor.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_dense_sift_sptr.h>
#include <bapl/bapl_lowe_keypoint_sptr.h>
#include <bapl/bapl_bbf_tree.h>
#include "vxl_plus.h"
#include "vxl_vrel_plus.h"

void VilVlFeatSIFTFetureUtil::getSiftFromWarpedTopview(const vil_image_view<vxl_byte> &topviewImage,
                                                   int destWidth, int destHeight,
                                                   const vpgl_perspective_camera<double> & keyframeCamera,
                                                   const vl_feat_sift_parameter &parameter,
                                                   vcl_vector<bapl_keypoint_sptr> & keyframeSift)
{
    assert(topviewImage.nplanes() == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    assert(keyframeSift.size() == 0);
    
    vil_image_view<vxl_byte> warpedTopview;
    vil_image_view<vxl_byte> alphaMap;    //court area
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    vgl_point_2d<int> starP(30, 30);
    vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
    court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap, starP, endP);
    
    // get features from warpted topview
    vcl_vector<bapl_keypoint_sptr> keyframeSiftAll;
    VilVlFeatSIFTFeture::vl_keypoint_extractor( warpedTopview, parameter, keyframeSiftAll, true);
    
    // filter out keypoints near boundary area
    for (int i = 0; i<keyframeSiftAll.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(keyframeSiftAll[i].as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        
        if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
            keyframeSift.push_back(keyframeSiftAll[i]);
        }
    }
    keyframeSiftAll.clear();
}

bool VilVlFeatSIFTFetureUtil::calib_sequential_frames(const vil_image_view<vxl_byte> & imageA, const vpgl_perspective_camera<double> & cameraA,
                                                  const vil_image_view<vxl_byte> & imageB, vpgl_perspective_camera<double> & cameraB)
{
    assert(imageA.ni() == imageB.ni());
    assert(imageA.nj() == imageB.nj());
    
    vl_feat_sift_parameter param;
    param.edge_thresh = 10;
    param.peak_thresh = 1.0;
    param.magnif = 3.0;
    
    vcl_vector<bapl_keypoint_sptr> sift1;
    vcl_vector<bapl_keypoint_sptr> sift2;
    VilVlFeatSIFTFeture::vl_keypoint_extractor(imageA, param, sift1, false);
    VilVlFeatSIFTFeture::vl_keypoint_extractor(imageB, param, sift2, false);
    
    // filter keypoint out of court boundary
    DisneyWorldBasketballCourt court;
    vil_image_view<vxl_byte> alphaImage;
    court.getProjectedCourtArea(cameraA, imageA.ni(), imageA.nj(), alphaImage);
    vcl_vector<bapl_keypoint_sptr> court_sift1;
    for (int i = 0; i<sift1.size(); i++) {
        bapl_lowe_keypoint_sptr s1 = dynamic_cast<bapl_lowe_keypoint *>(sift1[i].as_pointer());
        vgl_point_2d<double> p(s1->location_i(), s1->location_j());
        int x = p.x();
        int y = p.y();
        if (alphaImage(x, y) != 0) {
            court_sift1.push_back(sift1[i]);
        }
    }
    
    bapl_bbf_tree tree(sift2, 16);
    double ratio = 1.5;
    
    vcl_vector<bapl_key_match> matches;
    for (unsigned int i = 0; i<court_sift1.size(); i++) {
        bapl_keypoint_sptr      query = court_sift1[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        if (ssd0 * ratio < ssd1) {
            matches.push_back(k_p);
        }
    }
    
    // initial match points
    vcl_vector<vgl_point_2d<double> > pts1;
    vcl_vector<vgl_point_2d<double> > pts2;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr s1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr s2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(s1->location_i(), s1->location_j());
        vgl_point_2d<double> p2(s2->location_i(), s2->location_j());
        
        pts1.push_back(p1);
        pts2.push_back(p2);
    }
    
    vcl_vector<bool> isInliner;
    vgl_h_matrix_2d<double> H = VrelPlus::homography_RANSAC(pts1, pts2, isInliner, 1.0);
    bool isCalib = DisneyWorldBasketballCourt::getCameraFromSequentialHomo(cameraA, H, cameraB);
    return isCalib;
}

bool VilVlFeatSIFTFetureUtil::calibSequentialFrames(const vcl_vector<bapl_keypoint_sptr> & siftA, const vpgl_perspective_camera<double> & cameraA,
                                                const vil_image_view<vxl_byte> & imageB, vpgl_perspective_camera<double> & cameraB)
{
    vl_feat_sift_parameter param;
    param.edge_thresh = 10;
    param.peak_thresh = 1.0;
    param.magnif = 3.0;
    
    
    vcl_vector<bapl_keypoint_sptr> sift2;
    VilVlFeatSIFTFeture::vl_keypoint_extractor(imageB, param, sift2, false);
    
    // filter keypoint out of court boundary
    DisneyWorldBasketballCourt court;
    vil_image_view<vxl_byte> alphaImage;
    court.getProjectedCourtArea(cameraA, imageB.ni(), imageB.nj(), alphaImage);
    vcl_vector<bapl_keypoint_sptr> court_sift1;
    for (int i = 0; i<siftA.size(); i++) {
        bapl_lowe_keypoint_sptr s1 = dynamic_cast<bapl_lowe_keypoint *>(siftA[i].as_pointer());
        vgl_point_2d<double> p(s1->location_i(), s1->location_j());
        int x = p.x();
        int y = p.y();
        if (alphaImage(x, y) != 0) {
            court_sift1.push_back(siftA[i]);
        }
    }
    
    bapl_bbf_tree tree(sift2, 16);
    double ratio = 1.5;
    
    vcl_vector<bapl_key_match> matches;
    for (unsigned int i = 0; i<court_sift1.size(); i++) {
        bapl_keypoint_sptr      query = court_sift1[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        if (ssd0 * ratio < ssd1) {
            matches.push_back(k_p);
        }
    }
    
    // initial match points
    vcl_vector<vgl_point_2d<double> > pts1;
    vcl_vector<vgl_point_2d<double> > pts2;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr s1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr s2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(s1->location_i(), s1->location_j());
        vgl_point_2d<double> p2(s2->location_i(), s2->location_j());
        
        pts1.push_back(p1);
        pts2.push_back(p2);
    }
    
    vcl_vector<bool> isInliner;
    vgl_h_matrix_2d<double> H = VrelPlus::homography_RANSAC(pts1, pts2, isInliner, 1.0);
    bool isCalib = DisneyWorldBasketballCourt::getCameraFromSequentialHomo(cameraA, H, cameraB);
    return isCalib;
}
