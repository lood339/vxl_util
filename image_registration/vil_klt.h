//
//  vil_klt.h
//  QuadCopter
//
//  Created by jimmy on 7/1/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_klt__
#define __QuadCopter__vil_klt__

#include "klt.h"
#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>

// wrap KLT feature by vxl
struct VilKLTFeature
{
    vgl_point_2d<double> loc_;
    int value_;    // the larger the better, in tracking 0 is tracked, negative value is lost
};

class VilKLT
{
    KLT_TrackingContext tracking_context_;
    KLT_FeatureTable feature_table_;
    
    vil_image_view<vxl_byte> preFrame_;  // current frame
    KLT_FeatureList prevFeatureList_;    // stored for feature history    
    
    int frame_num_;   // total frame number
    int feature_num_;
    int cur_frame_num_;
    
public:
    VilKLT(int nFeatures, int nFrames);
    ~VilKLT();
    
    // features: inputout with empty
    bool initFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features);
    // features: inputout with empty
    // tracking with replace
    int trackingFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features);
    int trackingAndReplaceFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features);
    
    // randomly sample points in the image to initial the feature
    bool randomlyInitFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features);
    int trackingAndRandomlyReplaceFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features);
    
    vil_image_view<vxl_byte> deep_copy_previous_frame(void);
    vcl_vector<VilKLTFeature> previous_features(void);
    
    // randomly tracking features from image1 to image2
    static bool randomTracking(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2,
                               int nFeature,
                               vcl_vector<VilKLTFeature> & feature1, vcl_vector<VilKLTFeature> & feature2);
    
    
public:
    // select nFeatures from the image
    static bool selectGoodFeatures(const vil_image_view<vxl_byte> & image,
                                   const KLT_TrackingContext tc,
                                   const int nFeatures,
                                   vcl_vector<VilKLTFeature> & features,
                                   KLT_FeatureList & featureList); //
    
    // return tracked feature number
    static int trackFeatures(const vil_image_view<vxl_byte> & image1,
                              const vil_image_view<vxl_byte> & image2,
                              const KLT_TrackingContext tc,
                              vcl_vector<VilKLTFeature> & features, // features in image 2
                              KLT_FeatureList & featureList);       // updated feature list
    
    static int trackAndReplaceLostFeatures(const vil_image_view<vxl_byte> & image1,
                                           const vil_image_view<vxl_byte> & image2,
                                           const KLT_TrackingContext tc,
                                           vcl_vector<VilKLTFeature> & features, // features in image 2
                                           KLT_FeatureList & featureList);
    
    // replace the features that lost in the tracking
    // using most distinguishable features in the image
    static bool replaceLostFeatures(const vil_image_view<vxl_byte> & image,
                                    const KLT_TrackingContext tc,
                                    vcl_vector<VilKLTFeature> & features,
                                    KLT_FeatureList & featureList);
    // store features into feature table.
    static bool storeFeatureList(const vcl_vector<VilKLTFeature> & features,
                                 vcl_vector<vcl_vector<VilKLTFeature> > & featureTables,
                                 const KLT_FeatureList fl,
                                 KLT_FeatureTable ft,
                                 const int frame);
    
    // feature history: value = 0 --> successfully tracked, != 0 lost in this frame
    static bool extractFeatureHistory(const KLT_FeatureTable ft,
                                      int feat_idx,
                                      vcl_vector<VilKLTFeature> & featureHistory);

    
};


#endif /* defined(__QuadCopter__vil_klt__) */
