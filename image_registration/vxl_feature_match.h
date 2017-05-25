//
//  vxl_feature_match.h
//  OnlineStereo
//
//  Created by jimmy on 1/3/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_feature_match__
#define __OnlineStereo__vxl_feature_match__

#include <vil/vil_image_view.h>
#include <vgl/vgl_point_2d.h>
#include <vcl_vector.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <bapl/bapl_lowe_keypoint.h>
#include <bapl/bapl_bbf_tree.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_keypoint_set.h>
#include <vcl_utility.h>
#include <vgl/algo/vgl_h_matrix_2d.h>

using namespace std;

class VxlFeatureMatch
{
public:
    // match from A to B
    // ratio: small --> fewer matches
    static void siftMatchByRatio(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                 const vcl_vector<bapl_keypoint_sptr> & keypointsB,
                                 vcl_vector<bapl_key_match> & matches,
                                 vcl_vector<vcl_pair<int, int> > & matchedIndices,
                                 double ratio = 0.7,
                                 double feature_distance_threshold = 0.5);
    // return inlier number
    static int siftMatchAndHomography(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                      const vcl_vector<bapl_keypoint_sptr> & keypointsB,
                                      vcl_vector<vgl_point_2d<double> > & inlierPtsA,
                                      vcl_vector<vgl_point_2d<double> > & inlierPtsB,
                                      double homography_inlier_distance = 3.0,
                                      double ratio = 0.7,
                                      double feature_distance_threshold = 0.5);
    
    // matching from keypointsA to tree
    static int siftMatchAndHomography(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                      bapl_bbf_tree *tree,
                                      vcl_vector<vgl_point_2d<double> > & inlierPtsA,
                                      vcl_vector<vgl_point_2d<double> > & inlierPtsB,
                                      double homography_inlier_distance = 3.0,
                                      double ratio = 0.7,
                                      double feature_distance_threshold = 0.5);
    
    static bool filterMatchingByHomography(const vcl_vector<bapl_key_match> & initMatches,
                                           vcl_vector<bapl_key_match> & finalMatches,
                                           double homography_inlier_distance = 3.0);
    
    // best match from keypointsA to keypointsB, and vise verse
    // very slow
    static void mutualBestMatching(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                   const vcl_vector<bapl_keypoint_sptr> & keypointsB,
                                   vcl_vector<vgl_point_2d<double> > & ptsA,
                                   vcl_vector<vgl_point_2d<double> > & ptsB, double ratio = 0.7);
    
    /*
    // matching points constrained by epipolar line
    static bool epipolarMatch(const vcl_vector<bapl_keypoint_sptr> & leftKeypoints,
                              const vcl_vector<bapl_keypoint_sptr> & rightKeypoints,
                              const vpgl_perspective_camera<double> & cl,
                              const vpgl_perspective_camera<double> & cr,
                              vcl_vector<vcl_pair<int, int> > & matches,
                              double min_pts_line_dis = 2.0,
                              double sift_ratio = 0.5);
    
    // filter matches by fundamental matrix constraint
    static bool epipolarMatch(const vcl_vector<vgl_point_2d<double> > & ptsl,
                              const vcl_vector<vgl_point_2d<double> > & ptsr,
                              const vpgl_perspective_camera<double> & cl,
                              const vpgl_perspective_camera<double> & cr,
                              vcl_vector<bool> & isMatch,
                              double min_pts_line_dis = 2.0);
     */
    
    // match sift feature in local area
    // feature_distance_threshold: experimental value, 0.5 for vl feat
    // may failed if ransac fails
    static vgl_h_matrix_2d<double> geometryAwareSIFTMatch(const vcl_vector<bapl_keypoint_sptr> & leftKeypoints,
                                                          const vcl_vector<bapl_keypoint_sptr> & rightKeypoints,
                                                          const vgl_h_matrix_2d<double> & initH, int imageW, int imageH,
                                                          double feature_distance_threshold,
                                                          vcl_vector<bapl_key_match> & matches, bool ransac = true);
    //
    static bool geometryAwareSIFTMatch(const vcl_vector<bapl_keypoint_sptr> & leftKeypoints,
                                       const vcl_vector<bapl_keypoint_sptr> & rightKeypoints,
                                       const vgl_h_matrix_2d<double> & initH, int imageW, int imageH,
                                       double feature_distance_threshold,
                                       vgl_h_matrix_2d<double> & finalH,
                                       vcl_vector<bapl_key_match> & matches, bool ransac = true);
    
    // nK: the number of K nearest neighbor
    // find K nearest neightbor of query feature to filter outlier
    static bool KNNBagOfFeatureMatch(const vcl_vector<bapl_keypoint_sptr> & bagOfFeatures,
                                     const vcl_vector<bapl_keypoint_sptr> & queryFeatures,
                                     int nK, vgl_h_matrix_2d<double> & H,
                                     vcl_vector<bapl_key_match> & matches, bool ransac = true);

    
};



#endif /* defined(__OnlineStereo__vxl_feature_match__) */
