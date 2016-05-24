//
//  rgrsn_ldp.h
//  CameraPlaning
//
//  Created by jimmy on 8/6/15.
//  Copyright (c) 2015 Disney Reasearch Pittsburgh. All rights reserved.
//

#ifndef __CameraPlaning__rgrsn_ldp__
#define __CameraPlaning__rgrsn_ldp__

// local dynamic programming
#include <vnl/vnl_matrix.h>
#include <vcl_vector.h>

class rgrsn_ldp
{
public:
    // average of multiple local dynamic programming
    // data:         raw prediction. Each row is the data for a time instance
    // v_min, v_max: minimum and maximum value of data (i.e. prior knowledge from train data)
    // nBin:         number of bins in the data
    // nJumpBin:     maximum number of bins the path can jump
    // windowSize:   sliding window in time
    static bool dynamic_programming(const vnl_matrix<double> & data,
                                    double v_min, double v_max,
                                    unsigned int nBin,
                                    int nJumpBin,
                                    unsigned int windowSize,
                                    vnl_vector<double> & optimalSignal);
    
    // signal_variance: standard deviation
    static bool dynamic_programming(const vnl_matrix<double> & data,
                                    double v_min, double v_max,
                                    unsigned int nBin,
                                    int nJumpBin,
                                    unsigned int windowSize,
                                    vnl_vector<double> & optimalSignal,
                                    vnl_vector<double> & signal_variance);
    
    // median instead of average
    static bool dynamic_programming_median(const vnl_matrix<double> & data,
                                           double v_min, double v_max,
                                           unsigned int nBin,
                                           int nJumpBin,
                                           unsigned int windowSize,
                                           vnl_vector<double> & optimalSignal,
                                           vnl_vector<double> & medianSignal);
    
    // local viterbi algorithm
    // resolution: continous -- > discrete
    // transition: transition probability. Its resolution should be the same as in "resolution". from -2 1 0 1 2
    // windowSize: sliding window size
    // optimal_signal: output
    static bool local_viterbi(const vnl_matrix<double> & data,
                              double resolution,
                              const vnl_vector<double> & transition,
                              unsigned int window_size,
                              vnl_vector<double> & optimal_signal);
    
    // overlapping_ratio: 0 - w-1/w, like: 0, 0.2, ... 0.8
    static bool local_viterbi_overlapping_ratio(const vnl_matrix<double> & data,
                                                double resolution,
                                                const vnl_vector<double> & transition,
                                                unsigned int window_size,
                                                const double overlapping_ratio,
                                                vnl_vector<double> & optimal_signal);
    
    // signal_variance: standard deviation
    static bool local_viterbi(const vnl_matrix<double> & data,
                              double resolution,
                              const vnl_vector<double> & transition,
                              unsigned int window_size,
                              vnl_vector<double> & optimal_signal,
                              vnl_vector<double> & signal_variance);
    
    // transition matrix between frames
    static bool transition_matrix(const vcl_vector<int> & fns,
                                  const vcl_vector<double> & values,
                                  vnl_matrix<double> & transition,
                                  const double resolution);
    // transition matrix assume transition only happens on near by area
    static bool compact_transition_matrix(const vcl_vector<int> & fns,
                                          const vcl_vector<double> & values,
                                          vnl_matrix<double> & transition,
                                          const double resolution);
    
    // quantilization method
    // interval: resolution, the width of bin
    // nBin: tobal number of bins
    static unsigned value_to_bin_number(double v_min, double interval, double value, const unsigned nBin);
    
    // bin: bin index
    static double bin_number_to_value(double v_min, double interval, int bin);    

    // prob_map: confusion matrix, p(x | state)
    // transition: state trainsition vector (one dimension)
    // optimal_bins: bin numbers from backtrack
    static bool viterbi(const vnl_matrix<double> & prob_map,
                        const vnl_vector<double> & transition,
                        vcl_vector<int> & optimal_bins);
    
private:
    // dynamic programming inside a window
    static bool local_dynamic_programming(const vnl_matrix<double> & probMap, int nNeighborBin,
                                          vcl_vector<int> & optimalBins);
    
    static bool local_dynamic_programming_log(const vnl_matrix<double> & probMap, int nNeighborBin,
                                              vcl_vector<int> & optimalBins);
    
    
};

#endif /* defined(__CameraPlaning__rgrsn_ldp__) */
