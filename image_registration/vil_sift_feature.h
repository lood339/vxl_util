//
//  vil_sift_feature.h
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VpglPtzOpt__vil_sift_feature__
#define __VpglPtzOpt__vil_sift_feature__

#include <iostream>
#include <vil/vil_image_view.h>
#include <vgl/vgl_point_2d.h>
#include <vcl_vector.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <bapl/bapl_lowe_keypoint.h>
#include <bapl/bapl_bbf_tree.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_keypoint_set.h>

//wrap sift feature in bapl

class VilSIFT
{
public:
    static void getSiftPositions(const vil_image_view<vxl_byte> &image, vcl_vector<vgl_point_2d<double> > &points, double curve_ratio = 10.0);
    static void getSiftDescription(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_lowe_keypoint_sptr> & descriptions);
    
    static void getSiftDescription(const vil_image_view<vxl_byte> &image, const vcl_vector<vgl_point_2d<double> > &pts, vcl_vector<bapl_lowe_keypoint_sptr> & sifts);
    
    static void getSIFT(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_keypoint_sptr> & features);
    
    static void getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints, vcl_vector<vgl_point_2d<double> > & pts);
    static void getMatchLocations(const vcl_vector<bapl_key_match> & matches, vcl_vector<vgl_point_2d<double> > & pts1, vcl_vector<vgl_point_2d<double> > & pts2);
    
    static vcl_vector<vgl_point_2d<double> > getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints);
    static vcl_vector<vgl_point_2d<double> > getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints, const vcl_vector<bool> & inlier);
    
    static vgl_point_2d<double> sift_location(const bapl_keypoint_sptr & keypoint);
    static void set_location(bapl_keypoint_sptr & keypoint, const vgl_point_2d<double> & p);
    
    static void getSiftFromWarpedTopview(const vil_image_view<vxl_byte> &topviewImage, int destWidth, int destHeight,
                                         const vpgl_perspective_camera<double> &keyframeCamera, vcl_vector<bapl_keypoint_sptr> & keyframeSift);
    
    // generate SIFT from pre-defined location, scale and orientations
    // location_i_(i), location_j_(j), scale_(s), orientation_(o)
    static void getSiftFromDesignatedPositions(const vil_image_view<vxl_byte> & image, vcl_vector<vnl_vector_fixed<double, 4>> & loc_sca_ori,
                                               vcl_vector<bapl_keypoint_sptr> & sifts);
    
    
    // curve_ratio: the larger, the more features numbers
    static void getSiftFromWarpedTopview(const vil_image_view<vxl_byte> &topviewImage, int destWidth, int destHeight,
                                         const vpgl_perspective_camera<double> &keyframeCamera, vcl_vector<bapl_keypoint_sptr> & keyframeSift,
                                         vil_image_view<vxl_byte> & warpedTopview, double curve_ratio = 10.0);
    
    // warped topview image tends to be blured, so it dones not have sufficient featues, so the location of feature is extract from the original image
    // but the description of feature is from warped key frame
    // curve_ratio: the larger, the more features, default 10.0
    // patch_size: ncc value of patch matching to filter foreground (players), default 10
    // NCC: normalized crossvalidation threshold, default 0.8
    static void geSIFTFromWarpedTopview(const vil_image_view<vxl_byte> &topviewImage,
                                        const vil_image_view<vxl_byte> &keyImage,
                                        const vpgl_perspective_camera<double> &keyframeCamera,
                                        vcl_vector<bapl_keypoint_sptr> & warpedTopviewSift,
                                        vcl_vector<bapl_keypoint_sptr> & keyImageSift,                                        
                                        vil_image_view<vxl_byte> & warpedTopview,
                                        double curve_ratio, int patch_size, double ncc);
    
    // sift feature in blured image, filter out features in the palyer's area with SSD in a window
    // window_size: larger, fewer features
    // threshold: normalized (0-1.0) average color difference (SSD) in window, smaller , fewer features
    static void getSiftFromBluredKeyframe(const vil_image_view<vxl_byte> &topviewImage, const vil_image_view<vxl_byte> &bluredImage,
                                          const vpgl_perspective_camera<double> & keyframeCamera, vcl_vector<bapl_keypoint_sptr> & keyframeSift,
                                          const int window_size = 10, const double & threshold = 0.15);
    
    // pts1: feature position in firstImage
    // pts2: feature position in secndImage
    static void getSiftMatching(const vil_image_view<vxl_byte> & firstImage, const vil_image_view<vxl_byte> & secondImage,
                                vcl_vector<vgl_point_2d<double> > & pts1, vcl_vector<vgl_point_2d<double> > & pts2, double ratio = 0.7);
    
   
    
    // tree: sift feaure tree from query image
    // ratio: ssd ratio between first matched and second matched features, 0.6, 0.7 ... less than 1.0
    // destWidth, destHeight, projected image size (1280, 720) in WDW video
    // features1: feature positions in the projected topview image
    // features2: feature positions in query image
    // use RANSAC
    // assume: WDW basketball court
    static void matchSIFTfromProjectedTopview(const vil_image_view<vxl_byte> &topviewImage, const vpgl_perspective_camera<double> & keyFrameCamera,
                                              bapl_bbf_tree & tree, double ratio, int destWidth, int destHeight,
                                              vcl_vector<vgl_point_2d<double> > & features1, vcl_vector<vgl_point_2d<double> > & features2);
    
    // ratio constraint and ransac
    static void matchSIFTfromKdtree(const vil_image_view<vxl_byte> &image, bapl_bbf_tree & tree, double ratio,
                                    vcl_vector<vgl_point_2d<double> > & features1, vcl_vector<vgl_point_2d<double> > & features2);
    
    // adaptive ratio between 0.6, 0.7
    static vgl_h_matrix_2d<double> adativeMatchSIFT(const vcl_vector<bapl_keypoint_sptr> & keyframeFeatures,
                                 const vcl_vector<bapl_keypoint_sptr> & queryImageFeatures,
                                 vcl_vector<vgl_point_2d<double> > & keyframe_pts,
                                 vcl_vector<vgl_point_2d<double> > & query_image_pts);
    
    // get inlier feature by Homography
    // return: homogray marix from keyframe to query frame
    static void inlierByAdativeMatch(const vcl_vector<bapl_keypoint_sptr> & keyframeFeatures,
                                                        const vcl_vector<bapl_keypoint_sptr> & queryImageFeatures,
                                                        vcl_vector<bool> & keyFrameFeatureInlier);
                                     
    
    
    
    // calibrate camera from keyframe matching to query image
    // matching from warped keyframe to image is RANSAC, so the result changes from time to time
    static bool calibFromKeyframe(const vil_image_view<vxl_byte> &topviewImage, const vpgl_perspective_camera<double> &keyframeCamera,
                                  const vil_image_view<vxl_byte> &queryImage, vpgl_perspective_camera<double> &queryCamera,
                                  int featureMatchingNumThreshold = 20, double projectionErrorThreshold = 1.0);
    
    // only used in testing, as comparison
    // queryCamera frome pre-defiend key points
    // queryCameraFromSIFT from sift
    // sift_pts for the optimal correspondence
    static bool calibFromKeyframe_keypoint_vs_SIFT(const vil_image_view<vxl_byte> &topviewImage, const vpgl_perspective_camera<double> &keyframeCamera,
                                                   const vil_image_view<vxl_byte> &queryImage, vpgl_perspective_camera<double> &queryCamera,
                                                   vpgl_perspective_camera<double> & queryCameraFromSIFT,
                                                   vcl_vector<vgl_point_2d<double> > & sift_pts,
                                                   int featureMatchingNumThreshold = 20, double projectionErrorThreshold = 1.0);
    
    // calibrate camera from keyframe matching to query iamge
    // if keyframeFeatures = NULL, features are calculated from topview image
    // RANSAC is used to get the best camera, so result change from time to time
    // SAD: sum of absolute difference of projected topview image and image is used to select the best camera
    static bool calibFromKeyframeCached(const vil_image_view<vxl_byte> &topviewImage, const vpgl_perspective_camera<double> &keyframeCamera,
                                        vcl_vector<bapl_keypoint_sptr> * keyframeFeatures,
                                        const vil_image_view<vxl_byte> &queryImage, vpgl_perspective_camera<double> &queryCamera,
                                        int featureMatchingNumThreshold = 20, double projectionErrorThreshold = 1.0);
    
    // calibrate camera from keyframe matching to query iamge
    // if keyframeFeatures = NULL, features are calculated from topview image
    // if queryImageFeatures = NULL, features are calculated from queryImage
    // RANSAC is used to get the best camera, so result change from time to time
    // SAD: sum of absolute difference of projected topview image and image is used to select the best camera
    // camera position threshold: x [10, 20] y[-20, -10] , z [5 10]    
    static bool calibFromKeyframeCachedSIFT(const vil_image_view<vxl_byte> &topviewImage,
                                            const vpgl_perspective_camera<double> &keyframeCamera,
                                            vcl_vector<bapl_keypoint_sptr> * keyframeFeatures,
                                            const vil_image_view<vxl_byte> & queryImage,
                                            vcl_vector<bapl_keypoint_sptr> * queryImageFeatures,
                                            vpgl_perspective_camera<double> &queryCamera,
                                            int featureMatchingNumThreshold = 20);
    
    static bool writeSIFT(const char *file, const vcl_vector< bapl_keypoint_sptr > & keypoints);
    static bool readSIFT(const char *file, vcl_vector< bapl_keypoint_sptr > & keypoints);
    
};

#endif /* defined(__VpglPtzOpt__vil_sift_feature__) */
