//
//  vxlTimeSequentialFeature.h
//  CameraPlaning
//
//  Created by jimmy on 10/11/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CameraPlaning__vxlTimeSequentialFeature__
#define __CameraPlaning__vxlTimeSequentialFeature__

// generate time sequential features by given feature
// generate continuous time sequential data from discrete data
#include <vcl_vector.h>
#include <vnl/vnl_vector.h>


class VxlTimeSequentialFeature
{
public:
    // observation,   --> features
    // ground truth   --> labels
    // windowSize is non-decrease order
    static void generateMultiScaleFeature(const vcl_vector<double> & observation,
                                   const vcl_vector<double> & groundTruth,
                                   const vcl_vector<unsigned int> & windowSize,
                                   const int step,
                                   //output
                                   vcl_vector< vcl_vector<vnl_vector<double> > > & features,
                                   vcl_vector<double> & labels); // labels can be
    
    static void generateSingleScaleFeature(const vcl_vector<double> & observation,
                                           const vcl_vector<double> & groundTruth,
                                           const unsigned int bandwidth,
                                           const int step,
                                           //output
                                           vcl_vector< vnl_vector<double> > & features,
                                           vcl_vector<double> & labels);
    
    // get multiple label feature
    static void getMultipleLabelFeatures(const vcl_vector<double> & observation,
                                         const vcl_vector<double> & groundTruth,
                                         const vcl_vector<unsigned int> & windowSize,
                                         const int step,
                                         vcl_vector<vnl_vector<double> > & features,
                                         vcl_vector<vnl_vector<double> > & labels);
    
    // fill gap in data by threshold by randomly pick segment from orignal signals
    // gap_fns: new fns has gap
    // no_gap_data: is continuous
    static void fill_gaps(const vcl_vector< vcl_vector<int> > & fn_segments,
                          const vcl_vector< vcl_vector<double> > & data_segments,
                          const double threshold,
                          // output
                          vcl_vector<int> & gap_fns,
                          vcl_vector<double> & no_gap_data);
    
    static void forward_gradient(const vcl_vector<double> & data, vcl_vector<double> & gradient);
    static void forward_gradient(const vcl_vector<vnl_vector<double> > & data, vcl_vector<vnl_vector<double> > & gradient);
    
    // assume: featureStep and labelStep is sorted
    static void stackFeatures(const vcl_vector<int> & fns, const vcl_vector<vnl_vector<double> > & features, const vcl_vector<double> & labels,
                              const vcl_vector<int> & featureStep, const vcl_vector<int> & labelStep,
                              vcl_vector<int> & outFns, vcl_vector<vnl_vector<double> > & outFeatures, vcl_vector<double> & outLabels);
    
    // stack feature and predictions of previous frames to get new feature
    // idx: index in of current feature
    static void stackLastFeature(const vcl_vector<vnl_vector<double> > & features, const vcl_vector<double> & previousPredictions,
                                 const vcl_vector<int> & featureStep, const vcl_vector<int> & labelStep, int idx,
                                 vnl_vector<double> & feature_new);
    
};

// segment frames (fns) to multiple segments, in each segment, frame number is continuous
template <class T>
void VxlTimeSequentialFeature_segment(const vcl_vector<int> & fns, // frame numbers,
                                      const vcl_vector<T> & data,  // data, can be features or labels
                                      vcl_vector< vcl_vector<int> > & fn_segments,  // segmented frame numbers
                                      vcl_vector< vcl_vector<T> > & data_segments)  // segmented data
{
    assert(fns.size() == data.size());
    
    // start index in the fns
    vcl_vector<int> startIndex;
    startIndex.push_back(0);
    for (int i = 1; i<fns.size(); i++) {
        if (fns[i-1] + 1 != fns[i]) {
            startIndex.push_back(i);
        }
    }
    startIndex.push_back((int)fns.size());
    
    // separate data and fns
    for (int i = 0; i<startIndex.size()-1; i++) {
        int sIdx = startIndex[i];
        int eIdx = startIndex[i+1];
        
        vcl_vector<int> cur_fns;
        vcl_vector<T> cur_data;
        
        for (int j = sIdx; j < eIdx; j++) {
            cur_fns.push_back(fns[j]);
            cur_data.push_back(data[j]);
        }
        fn_segments.push_back(cur_fns);
        data_segments.push_back(cur_data);
    }
    assert(fn_segments.size() == data_segments.size());
}



#endif /* defined(__CameraPlaning__vxlTimeSequentialFeature__) */
