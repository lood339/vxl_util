//
//  vil_klt.cpp
//  QuadCopter
//
//  Created by jimmy on 7/1/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_klt.h"
#include "vil_plus.h"
#include <vil/vil_convert.h>


VilKLT::VilKLT(int nFeatures, int nFrames)
{
    assert(nFeatures > 0);
    assert(nFrames > 0);
    
    prevFeatureList_ = KLTCreateFeatureList(nFeatures);
    tracking_context_ = KLTCreateTrackingContext();
    feature_table_ = KLTCreateFeatureTable(nFrames, nFeatures);
    frame_num_ = nFrames;
    feature_num_= nFeatures;
    cur_frame_num_ = 0;
    
    // set parameters for tracking context
    tracking_context_->sequentialMode = TRUE;
    tracking_context_->writeInternalImages = FALSE;
    tracking_context_->affineConsistencyCheck = 2;
}

VilKLT::~VilKLT()
{
    if (prevFeatureList_) {
        KLTFreeFeatureList(prevFeatureList_);
    }
    if (tracking_context_) {
        KLTFreeTrackingContext(tracking_context_);
    }
    if (feature_table_) {
        KLTFreeFeatureTable(feature_table_);
    }
}

bool VilKLT::initFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features)
{
    assert(cur_frame_num_ == 0);
    
    bool isSelect = VilKLT::selectGoodFeatures(image, tracking_context_, feature_num_, features, prevFeatureList_);
    KLTStoreFeatureList(prevFeatureList_, feature_table_, cur_frame_num_); // prevFeatureList store the curent KLT features
    preFrame_.deep_copy(image);
    cur_frame_num_++;
    return isSelect;
}

bool VilKLT::randomlyInitFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features)
{
    assert(cur_frame_num_ == 0);
    
    bool isSelect = VilKLT::selectGoodFeatures(image, tracking_context_, feature_num_, features, prevFeatureList_);
    
    // randomly change feature locations
    int borderx = tracking_context_->borderx * 1.5;
    int bordery = tracking_context_->bordery * 1.5;
    const int w = image.ni();
    const int h = image.nj();
    for (int i =0; i<features.size(); i++) {
        int x = rand()%(w-2*borderx) + borderx;
        int y = rand()%(h-2*bordery) + bordery;
        features[i].loc_ = vgl_point_2d<double>(x, y);
        features[i].value_ = 0;
        prevFeatureList_->feature[i]->x = x;
        prevFeatureList_->feature[i]->y = y;
        prevFeatureList_->feature[i]->val = 0;
    }
    
    KLTStoreFeatureList(prevFeatureList_, feature_table_, cur_frame_num_); // prevFeatureList store the curent KLT features
    preFrame_.deep_copy(image);
    cur_frame_num_++;
    return isSelect;
}

int VilKLT::trackingFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features)
{
    assert(prevFeatureList_);
    assert(cur_frame_num_ < frame_num_);
    
    int trackingNum = VilKLT::trackFeatures(preFrame_, image, tracking_context_, features, prevFeatureList_);
    KLTStoreFeatureList(prevFeatureList_, feature_table_, cur_frame_num_);
    preFrame_.deep_copy(image);
    cur_frame_num_++;
    return trackingNum;
}

bool VilKLT::randomTracking(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2,
                            int nFeature,
                            vcl_vector<VilKLTFeature> & feature1, vcl_vector<VilKLTFeature> & feature2)
{
    VilKLT klt(nFeature, 2);
    klt.randomlyInitFeatures(image1, feature1);
    klt.trackingFeatures(image2, feature2);
    
    return true;
}




int VilKLT::trackingAndReplaceFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features)
{
    assert(prevFeatureList_);
    assert(cur_frame_num_ < frame_num_);
    
    int trackingNum = VilKLT::trackAndReplaceLostFeatures(preFrame_, image, tracking_context_, features, prevFeatureList_);
    KLTStoreFeatureList(prevFeatureList_, feature_table_, cur_frame_num_);
    preFrame_.deep_copy(image);
    cur_frame_num_++;
    return trackingNum;
}

int VilKLT::trackingAndRandomlyReplaceFeatures(const vil_image_view<vxl_byte> & image, vcl_vector<VilKLTFeature> & features)
{
    assert(prevFeatureList_);
    assert(cur_frame_num_ < frame_num_);
    
    int trackingNum = VilKLT::trackFeatures(preFrame_, image, tracking_context_, features, prevFeatureList_);
    
    // randomly replace feature locations that are NOT tracked
    int borderx = tracking_context_->borderx * 1.5;
    int bordery = tracking_context_->bordery * 1.5;
    const int w = image.ni();
    const int h = image.nj();
    for (int i =0; i<features.size(); i++) {
        if (features[i].value_ != 0) {
            int x = rand()%(w-2*borderx) + borderx;
            int y = rand()%(h-2*bordery) + bordery;
            features[i].loc_ = vgl_point_2d<double>(x, y);
            features[i].value_ = 10;  // @todo this value has no meaning
            prevFeatureList_->feature[i]->x = x;
            prevFeatureList_->feature[i]->y = y;
            prevFeatureList_->feature[i]->val = 10; // @todo this value has no meaning
        }
    }    
    
    KLTStoreFeatureList(prevFeatureList_, feature_table_, cur_frame_num_);
    preFrame_.deep_copy(image);
    cur_frame_num_++;
    return trackingNum;
}


vil_image_view<vxl_byte> VilKLT::deep_copy_previous_frame(void)
{
    assert(cur_frame_num_ > 0);
    vil_image_view<vxl_byte> image;
    image.deep_copy(preFrame_);
    return image;
}

vcl_vector<VilKLTFeature> VilKLT::previous_features(void)
{
    assert(cur_frame_num_ > 0);
    vcl_vector<VilKLTFeature> features;
    
    for (int i = 0 ; i < prevFeatureList_->nFeatures ; i++)  {
        VilKLTFeature feat;
        feat.loc_ = vgl_point_2d<double>(prevFeatureList_->feature[i]->x, prevFeatureList_->feature[i]->y);
        feat.value_ = prevFeatureList_->feature[i]->val;
        features.push_back(feat);
    }
    return features;
}



/*****************************   static functions   ********************/
bool VilKLT::selectGoodFeatures(const vil_image_view<vxl_byte> & img, const KLT_TrackingContext tc,
                                const int nFeatures,
                                vcl_vector<VilKLTFeature> & features, KLT_FeatureList & fl)
{
    vil_image_view<vxl_byte> grayImage;
    if (img.nplanes() == 1) {
        grayImage = img;
    }
    else if(img.nplanes() == 3)
    {
        vil_convert_planes_to_grey(img, grayImage);
    }
    const int w = grayImage.ni();
    const int h = grayImage.nj();
    unsigned char * data = VilPlus::vil_malloc(grayImage);
    fl = KLTCreateFeatureList(nFeatures);
    KLTSelectGoodFeatures(tc, data, w, h, fl);
    
    for (int i = 0 ; i < fl->nFeatures ; i++)  {
        VilKLTFeature feat;
        feat.loc_ = vgl_point_2d<double>(fl->feature[i]->x, fl->feature[i]->y);
        feat.value_ = fl->feature[i]->val;
        features.push_back(feat);
    }
    if (data) {
        delete []data;
    }
    return true;
}

int VilKLT::trackFeatures(const vil_image_view<vxl_byte> & image1,
                           const vil_image_view<vxl_byte> & image2,
                           const KLT_TrackingContext tc,
                           vcl_vector<VilKLTFeature> & features,
                           KLT_FeatureList & featureList)
{
    assert(image1.ni() == image2.ni());
    assert(image1.nj() == image2.nj());
    
    // change to gray images
    vil_image_view<vxl_byte> grayImage1;
    vil_image_view<vxl_byte> grayImage2;
    if (image1.nplanes() == 1) {
        grayImage1 = image1;
    }
    else if(image1.nplanes() == 3)
    {
        vil_convert_planes_to_grey(image1, grayImage1);
    }
    
    if (image2.nplanes() == 1) {
        grayImage2 = image2;
    }
    else if(image2.nplanes() == 3)
    {
        vil_convert_planes_to_grey(image2, grayImage2);
    }
    
    //
    const int w = grayImage1.ni();
    const int h = grayImage1.nj();
    unsigned char * data1 = VilPlus::vil_malloc(grayImage1);
    unsigned char * data2 = VilPlus::vil_malloc(grayImage2);    
  
    KLTTrackFeatures(tc, data1, data2, w, h, featureList);
    int num = 0;  // successfully tracked feature number
    for (int i = 0 ; i < featureList->nFeatures ; i++)  {
        VilKLTFeature feat;
        feat.loc_ = vgl_point_2d<double>(featureList->feature[i]->x, featureList->feature[i]->y);
        feat.value_ = featureList->feature[i]->val;
        features.push_back(feat);
        
        if (feat.value_ == 0) {
            num++;
        }
    }
    
    if (data1) {
        delete []data1;
        data1 = NULL;
    }
    if (data2) {
        delete []data2;
        data2 = NULL;
    }
    return num;
}

int VilKLT::trackAndReplaceLostFeatures(const vil_image_view<vxl_byte> & image1,
                                        const vil_image_view<vxl_byte> & image2,
                                        const KLT_TrackingContext tc,
                                        vcl_vector<VilKLTFeature> & features, // features in image 2
                                        KLT_FeatureList & featureList)
{
    assert(image1.ni() == image2.ni());
    assert(image1.nj() == image2.nj());
    
    // change to gray images
    vil_image_view<vxl_byte> grayImage1;
    vil_image_view<vxl_byte> grayImage2;
    if (image1.nplanes() == 1) {
        grayImage1 = image1;
    }
    else if(image1.nplanes() == 3)
    {
        vil_convert_planes_to_grey(image1, grayImage1);
    }
    
    if (image2.nplanes() == 1) {
        grayImage2 = image2;
    }
    else if(image2.nplanes() == 3)
    {
        vil_convert_planes_to_grey(image2, grayImage2);
    }
    
    //
    const int w = grayImage1.ni();
    const int h = grayImage1.nj();
    unsigned char * data1 = VilPlus::vil_malloc(grayImage1);
    unsigned char * data2 = VilPlus::vil_malloc(grayImage2);
    
    KLTTrackFeatures(tc, data1, data2, w, h, featureList);
    KLTReplaceLostFeatures(tc, data2, w, h, featureList);
    
    int num = 0;  // successfully tracked feature number
    for (int i = 0 ; i < featureList->nFeatures ; i++)  {
        VilKLTFeature feat;
        feat.loc_ = vgl_point_2d<double>(featureList->feature[i]->x, featureList->feature[i]->y);
        feat.value_ = featureList->feature[i]->val;
        features.push_back(feat);
        
        if (feat.value_ == 0) {
            num++;
        }
    }
    
    if (data1) {
        delete []data1;
        data1 = NULL;
    }
    if (data2) {
        delete []data2;
        data2 = NULL;
    }
    return num;
}

bool VilKLT::replaceLostFeatures(const vil_image_view<vxl_byte> & image,
                                 const KLT_TrackingContext tc,
                                 vcl_vector<VilKLTFeature> & features,
                                 KLT_FeatureList & featureList)
{
    assert(features.size() == 0);
    
    vil_image_view<vxl_byte> grayImage;
    if (image.nplanes() == 1) {
        grayImage = image;
    }
    else if(image.nplanes() == 3)
    {
        vil_convert_planes_to_grey(image, grayImage);
    }
    const int w = grayImage.ni();
    const int h = grayImage.nj();
    unsigned char * data = VilPlus::vil_malloc(grayImage);
    KLTReplaceLostFeatures(tc, data, w, h, featureList);
    
    // updata featuers
    for (int i = 0 ; i < featureList->nFeatures ; i++)  {
        VilKLTFeature feat;
        feat.loc_ = vgl_point_2d<double>(featureList->feature[i]->x, featureList->feature[i]->y);
        feat.value_ = featureList->feature[i]->val;
        features.push_back(feat);
    }
    if (data) {
        delete []data;
    }
    return true;
}


bool VilKLT::storeFeatureList(const vcl_vector<VilKLTFeature> & features,
                              vcl_vector<vcl_vector<VilKLTFeature> > & featureTables,
                              const KLT_FeatureList fl,
                              KLT_FeatureTable ft,
                              const int frame)
{
    KLTStoreFeatureList(fl, ft, frame);
    featureTables.push_back(features);
    return true;
}

bool VilKLT::extractFeatureHistory(const KLT_FeatureTable ft,
                                   int feat_idx,
                                   vcl_vector<VilKLTFeature> & featureHistory)
{
    KLT_FeatureHistory fh = KLTCreateFeatureHistory(ft->nFrames);
    if (feat_idx < 0 && feat_idx >= ft->nFeatures) {
        printf("Error: feature index out of scope.\n");
        return false;
    }
    
    KLTExtractFeatureHistory(fh, ft, feat_idx);
    
    for (int i = 0 ; i < fh->nFrames ; i++)  {
        VilKLTFeature feat;
        feat.loc_ = vgl_point_2d<double>(fh->feature[i]->x, fh->feature[i]->y);
        feat.value_ = fh->feature[i]->val;
        featureHistory.push_back(feat);
    }
    
    if (fh) {
        delete []fh;
        fh = NULL;
    }
    return true;    
}




