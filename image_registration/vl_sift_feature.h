//
//  vl_sift_feature.h
//  QuadCopter
//
//  Created by jimmy on 3/24/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vl_sift_feature__
#define __QuadCopter__vl_sift_feature__

// wrap sift feature in vl feat

#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vl/sift.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vnl/vnl_vector_fixed.h>
#include <vcl_utility.h>
#include <vil/vil_fwd.h>

#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_keypoint_set.h>
#include <vector>
#include "vl_sift_param.h"

using std::vector;

// use Vl feat libraty generate SIFT feature in the vxl format
// can't compare with the SIFT generate from vxl bapl_keypoint_extractor

class VlSIFTFeature
{
public:
    // extract SIFT with method from vl_feat, to the format of bapl_keypoint_sptr
    static bool vl_keypoint_extractor(const vil_image_view<vxl_byte> & image,
                                      const vl_feat_sift_parameter &parameter,
                                      vcl_vector<bapl_keypoint_sptr> & keypoints,
                                      bool verbose = true);
    // extract SIFT locations
    static bool vl_sift_keypoint(const vil_image_view<vxl_byte> & image,
                                 const vl_feat_sift_parameter & param,
                                 vector<vgl_point_2d<double> > & locations,
                                 bool verbose = true);
    
    
    // match from A to B
    // ratio: small --> fewer matches
    static void sift_match_by_ratio(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                    const vcl_vector<bapl_keypoint_sptr> & keypointsB,
                                    vcl_vector<bapl_key_match> & matches,
                                    vcl_vector<vcl_pair<int, int> > & matchedIndices,
                                    vcl_vector<vgl_point_2d<double> > & pts1,
                                    vcl_vector<vgl_point_2d<double> > & pts2,
                                    double ratio = 0.7,
                                    double feature_distance_threshold = 0.5,
                                    bool verbose = true);
    
    
    
};



#endif /* defined(__QuadCopter__vl_sift_feature__) */
