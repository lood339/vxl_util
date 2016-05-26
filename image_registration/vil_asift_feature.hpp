//
//  vil_asift_feature.hpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-30.
//  Copyright © 2015 jimmy. All rights reserved.
//

#ifndef vil_asift_feature_cpp
#define vil_asift_feature_cpp

// affine sift feature extrating and matching
#include "vl_sift_feature.h"
#include <vnl/vnl_matrix_fixed.h>

class vil_asift_feature
{
public:
    // rotation, tilts: degrees
    // all SIFT features are together
    static bool asift_keypoints_extractor(const vil_image_view<vxl_byte> & image,
                                          const vcl_vector<double> & rotations,
                                          const vcl_vector<double> & tilts,
                                          const vl_feat_sift_parameter & parameter,
                                          vcl_vector<bapl_keypoint_sptr> & keypoints,
                                          bool verbose = true);
    
    // rotation, tilts: degrees
    // all sift feature are separately
    static bool asift_keypoints_extractor(const vil_image_view<vxl_byte> & image,
                                          const vcl_vector<double> & rotations,
                                          const vcl_vector<double> & tilts,
                                          const vl_feat_sift_parameter & parameter,
                                          vcl_vector< vcl_vector<bapl_keypoint_sptr> > & keypoints,
                                          bool verbose = true);
    
    
    // extract sift separately from a sequence of images
    static bool asift_keypoint_extractor(const vil_image_view<vxl_byte> & original_image,
                                         const vcl_vector<vil_image_view<vxl_byte> > & affine_warped_images,
                                         const vcl_vector<vnl_matrix_fixed<double, 2, 3> > & affines,
                                         const vl_feat_sift_parameter & parameter,
                                         vcl_vector< vcl_vector<bapl_keypoint_sptr> > & keypoints,
                                         bool verbose = true);
    
    // extract all ASIFT feature fram a sequence of iamges
    static bool asift_keypoint_extractor(const vil_image_view<vxl_byte> & original_image,
                                         const vcl_vector<vil_image_view<vxl_byte> > & affine_warped_images,
                                         const vcl_vector<vnl_matrix_fixed<double, 2, 3> > & affines,
                                         const vl_feat_sift_parameter & parameter,
                                         vcl_vector<bapl_keypoint_sptr> & keypoints,
                                         bool verbose = true);
    
    
};

#endif /* vil_asift_feature_cpp */
