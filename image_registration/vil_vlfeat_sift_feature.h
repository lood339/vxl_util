//
//  vil_vlfeat_sift_feature.h
//  CalibFromScene
//
//  Created by Jimmy Chen LOCAL on 6/30/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CalibFromScene__vil_vlfeat_sift_feature__
#define __CalibFromScene__vil_vlfeat_sift_feature__

#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vl/sift.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_vector_fixed.h>
#include <vcl_utility.h>
#include <vil/vil_fwd.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_keypoint_set.h>

#include "vl_sift_feature.h"


// use Vl feat libraty generate SIFT feature in the vxl format
// can't compare with the SIFT generate from vxl bapl_keypoint_extractor


/*
 %   Octaves:: maximum possible
 %     Set the number of octave of the DoG scale space.
 %
 %   Levels:: 3
 %     Set the number of levels per octave of the DoG scale space.
 %
 %   FirstOctave:: 0
 %     Set the index of the first octave of the DoG scale space.
 %
 %   PeakThresh:: 0
 %     Set the peak selection threshold.
 %
 %   EdgeThresh:: 10
 %     Set the non-edge selection threshold.
 %
 %   NormThresh:: -inf
 %     Set the minimum l2-norm of the descriptors before
 %     normalization. Descriptors below the threshold are set to zero.
 %
 %   Magnif:: 3
 %     Set the descriptor magnification factor. The scale of the
 %     keypoint is multiplied by this factor to obtain the width (in
 %     pixels) of the spatial bins. For instance, if there are there
 %     are 4 spatial bins along each spatial direction, the
 %     ``side'' of the descriptor is approximatively 4 * MAGNIF.
 %
 %   WindowSize:: 2
 %     Set the variance of the Gaussian window that determines the
 %     descriptor support. It is expressend in units of spatial
 %     bins.
 */
/*
struct vl_feat_sift_parameter
{
    double   edge_thresh;   // larger --> more feature
    double   peak_thresh ;  // smaller--> more feature
    double   magnif     ;
    double   norm_thresh;
    double   window_size;
    
    vl_feat_sift_parameter()
    {
        edge_thresh  = 10 ;
        peak_thresh  = 0 ;
        magnif       = 3 ;
        norm_thresh  = -1;
        window_size  = 2;
    }
};
 */

class VilVlFeatSIFTFeture
{
public:
    // extract SIFT with method from vl_feat, to the format of bapl_keypoint_sptr
    static bool vl_keypoint_extractor( const vil_image_view<vxl_byte> & image, const vl_feat_sift_parameter &parameter,
                                      vcl_vector<bapl_keypoint_sptr> & keypoints, bool verbose = true);
    
    // extract SIFT at locations
    static bool vl_keypoint_custom_extractor(const vil_image_view<vxl_byte> & image,
                                             const vl_feat_sift_parameter &parameter,
                                             const vcl_vector<vgl_point_2d<double> > & locatioins,
                                             vcl_vector<bapl_keypoint_sptr> & keypoints,
                                             bool verbose = true);
    
    
    
          
};


#endif /* defined(__CalibFromScene__vil_vlfeat_sift_feature__) */
