//
//  vxlTimeSequentialFeature.cpp
//  CameraPlaning
//
//  Created by jimmy on 10/11/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxlTimeSequentialFeature.h"
#include <vnl/vnl_vector_fixed.h>
#include <vcl_numeric.h>
#include <vcl_algorithm.h>


void VxlTimeSequentialFeature::generateMultiScaleFeature(const vcl_vector<double> & data,
                                                         const vcl_vector<double> & groundTruth,
                                                         const vcl_vector<unsigned int> & windowSize,
                                                         const int step,
                                                         vcl_vector< vcl_vector<vnl_vector<double> > > & features,
                                                         vcl_vector<double> & labels)
{
    assert(step >= 1);
    assert(data.size() == groundTruth.size());
    
    const int beingInd = windowSize.back();    
    
    for (int i = beingInd; i<data.size(); i++) {
        
        vcl_vector<vnl_vector<double> > multi_scale_feature;
        // loop all windowSize
        for (int j = 0; j<windowSize.size(); j++) {
            const int feat_length = windowSize[j]/step;
            assert(feat_length > 0);
            vnl_vector<double> feat(feat_length, 0);
            for (int k = 0; k<feat_length; k += 1) {
                feat[k] = data[i - k * step];
            }
            multi_scale_feature.push_back(feat);
        }
        features.push_back(multi_scale_feature);
        labels.push_back(groundTruth[i]);
    }
    
    assert(features.size() == labels.size());
}

void VxlTimeSequentialFeature::generateSingleScaleFeature(const vcl_vector<double> & observation,
                                                          const vcl_vector<double> & groundTruth,
                                                          const unsigned int bandwidth,
                                                          const int step,
                                                          //output
                                                          vcl_vector< vnl_vector<double> > & features,
                                                          vcl_vector<double> & labels)
{
    assert(bandwidth > 0);
    
    vcl_vector<unsigned int> windowSizes;
    windowSizes.push_back(bandwidth);
    
    vcl_vector< vcl_vector<vnl_vector<double> > > multiScaleFeatures;
    
    VxlTimeSequentialFeature::generateMultiScaleFeature(observation, groundTruth, windowSizes, step, multiScaleFeatures, labels);
    for (int i = 0; i<multiScaleFeatures.size(); i++) {
        features.push_back(multiScaleFeatures[i][0]);
    }
    assert(features.size() == labels.size());
}



void VxlTimeSequentialFeature::getMultipleLabelFeatures(const vcl_vector<double> & observation,
                                                        const vcl_vector<double> & groundTruth,
                                                        const vcl_vector<unsigned int> & windowSize,
                                                        const int step,
                                                        //output
                                                        vcl_vector<vnl_vector<double> > & features,
                                                        vcl_vector<vnl_vector<double> > & labels)
{
    assert(step >= 1);
    assert(observation.size() == groundTruth.size());
    assert(0);  // @todo
    
    
    assert(features.size() == labels.size());
}

// assume there is only one candindate in the data
static bool find_start_end_index(const vcl_vector<double> & data, const double threshold,
                                 const double startVal,
                                 const double endVal,
                                 int & startInd, int & endInd)
{
    startInd = -1;
    endInd   = -1;
    for (int i = 0; i<data.size(); i++) {
        if (fabs(data[i] - startVal) < threshold) {
            // loop over the rest value
            for (int j = i; j<data.size(); j++) {
                if (fabs(data[j] - endVal) < threshold) {
                    startInd = i;
                    endInd = j;
                    break;
                }
            }
        }
        if (startInd != -1 && endInd != -1) {
            break;
        }
    }
    return startInd != -1 && endInd != -1;
}

void VxlTimeSequentialFeature::fill_gaps(const vcl_vector< vcl_vector<int> > & fn_segments,
                                         const vcl_vector< vcl_vector<double> > & data_segments,
                                         const double threshold,
                                         vcl_vector<int> & gap_fns,
                                         vcl_vector<double> & no_gap_data)
{
    assert(fn_segments.size() == data_segments.size());
    
    for (int i = 1; i<fn_segments.size(); i++) {
        // read previous (fn, data)
        for (int j = 0; j<fn_segments[i-1].size(); j++) {
            gap_fns.push_back(fn_segments[i-1][j]);
            no_gap_data.push_back(data_segments[i-1][j]);
        }
        
        // fill gap in all segments      
        double startVal = data_segments[i-1].back();
        double endVal = data_segments[i].front();
        vcl_vector<vnl_vector_fixed<int, 3> > candindates;
        for (int j = 0; j<data_segments.size(); j++) {
            int startInd = -1;
            int endInd = -1;
            vnl_vector_fixed<int, 3> ind_start_end(-1);
            // find in each segments
            bool isFind = find_start_end_index(data_segments[j], threshold, startVal, endVal, startInd, endInd);
            if (isFind) {
                ind_start_end[0] = j;
                ind_start_end[1] = startInd;
                ind_start_end[2] = endInd;
                candindates.push_back(ind_start_end);
            }
        }
        if (candindates.size() == 0) {
            printf("Error: can not find candindates to fill the gap!");
        }
        else
        {
            // randomly pick a cadinate
            int rnd = rand()%candindates.size();
            int seg_ind   = candindates[rnd][0];
            int start_ind = candindates[rnd][1];
            int end_ind   = candindates[rnd][2];
            // fill gap
            for (int j = start_ind; j<end_ind; j++) {
                gap_fns.push_back(fn_segments[seg_ind][j]);
                no_gap_data.push_back(data_segments[seg_ind][j]);
            }
        }
    }
    // cat the last segment
    for (int j = 0; j<fn_segments.back().size(); j++) {
        gap_fns.push_back(fn_segments.back()[j]);
        no_gap_data.push_back(data_segments.back()[j]);
    }
    
    assert(gap_fns.size() == no_gap_data.size());
    
    // info,
    unsigned int size_before = 0;
    for (int i = 0; i<fn_segments.size(); i++) {
        size_before += fn_segments[i].size();
    }
    printf("data length before vc after fill gap: %u %lu\n", size_before, gap_fns.size());
}

void VxlTimeSequentialFeature::forward_gradient(const vcl_vector<double> & data, vcl_vector<double> & gradient)
{
    assert(data.size() >= 2);
    
    gradient.resize(data.size());
    vcl_adjacent_difference(data.begin(), data.end(), gradient.begin());
    gradient[0] = data[1] - data[0];
    
    assert(gradient.size() == data.size());
}

vnl_vector<double> op(const vnl_vector<double> & d1, const vnl_vector<double> & d2)
{
    return d1 - d2;
}

void VxlTimeSequentialFeature::forward_gradient(const vcl_vector<vnl_vector<double> > & data, vcl_vector<vnl_vector<double> > & gradient)
{
    assert(data.size() >= 2);
    
    gradient.resize(data.size());
    vcl_adjacent_difference(data.begin(), data.end(), gradient.begin(), op);
    gradient[0] = gradient[1];
    /*
    gradient.clear();
    vnl_vector<double> feat;
    
    for (int i = 0; i<data.size(); i++) {
        if (i == 0) {
            feat = data[1] - data[0];
        }
        else
        {
            feat = data[i] - data[i-1];
        }
        gradient.push_back(feat);
    }
     */
    
    assert(gradient.size() == data.size());
}

void VxlTimeSequentialFeature::stackFeatures(const vcl_vector<int> & fns, const vcl_vector<vnl_vector<double> > & features, const vcl_vector<double> & labels,
                                             const vcl_vector<int> & featureStep, const vcl_vector<int> & labelStep,
                                             vcl_vector<int> & outFns, vcl_vector<vnl_vector<double> > & outFeatures, vcl_vector<double> & outLabels)
{
    assert(featureStep.size() > 0);
    const int dim = (int)features[0].size();
    const int feature_dim = dim * (int)featureStep.size() + (int)labelStep.size(); // always add current feature
    int lookbackFrameNum = featureStep.back();
    if (!labelStep.empty()) {
        lookbackFrameNum = vcl_max(featureStep.back(), labelStep.back());
    }
    
    for (int i = lookbackFrameNum; i<fns.size(); i++) {
        vnl_vector<double> feat(feature_dim, 0);
        for (int j = 0; j<featureStep.size(); j++) {
            int step = featureStep[j];
            feat.update(features[i-step], j * dim);
        }
        for (int j = 0; j<labelStep.size(); j++) {
            int step = labelStep[j];
            feat[dim * (int)featureStep.size() + j] = labels[i - step];
        }
        
        outFeatures.push_back(feat);
        outFns.push_back(fns[i]);
        outLabels.push_back(labels[i]);
    }
    
    assert(outFns.size() == outFeatures.size());
    assert(outFeatures.size() == outLabels.size());
}

void VxlTimeSequentialFeature::stackLastFeature(const vcl_vector<vnl_vector<double> > & features, const vcl_vector<double> & previousPredictions,
                                                const vcl_vector<int> & featureStep, const vcl_vector<int> & labelStep,
                                                int idx, vnl_vector<double> & feature_new)
{
    assert(previousPredictions.size() < features.size());
    
    const int dim = (int)features[0].size();
    const int feature_dim = dim * (int)featureStep.size() + (int)labelStep.size(); // always add current feature
    int lookbackFrameNum = featureStep.back();
    if (!labelStep.empty()) {
        lookbackFrameNum = vcl_max(featureStep.back(), labelStep.back());
    }
    
    assert(idx >= lookbackFrameNum);    
    
    feature_new = vnl_vector<double>(feature_dim, 0);
    for (int j = 0; j<featureStep.size(); j++) {
        int step = featureStep[j];
        assert(idx - step >= 0);
        feature_new.update(features[idx-step], j * dim);
    }
    for (int j = 0; j<labelStep.size(); j++) {
        int step = labelStep[j];
        assert(idx - step >= 0);
        feature_new[dim * (int)featureStep.size() + j] = previousPredictions[idx - step];
    }
}










