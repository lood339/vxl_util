//
//  rgrsn_viterbi.h
//  CameraPlaning
//
//  Created by jimmy on 9/16/15.
//  Copyright (c) 2015 Disney Reasearch Pittsburgh. All rights reserved.
//

#ifndef __CameraPlaning__rgrsn_viterbi__
#define __CameraPlaning__rgrsn_viterbi__

#include <vnl/vnl_matrix.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector.h>

class rgrns_viterbi
{
public:
    // local viterbi algorithm
    // resolution: continous -- > discrete
    // transition: transition probability. Its resolution should be the same as in "resolution". from -2 1 0 1 2
    // windowSize: sliding window size
    // num_path: number of path tracked in the forward
    // optimal_signal: output
    static bool local_viterbi(const vnl_matrix<double> & data,
                              double resolution,
                              const vnl_vector<double> & transition,
                              unsigned int window_size,
                              int num_path,
                              vnl_vector<double> & optimal_signal);
    
private:
    
    // prob_map: confusion matrix, p(x | state)
    // valid_map: 0 -- > cells are ignored
    // transition: state trainsition vector (one dimension)
    // optimal_bins: bin numbers from backtrack
    static bool viterbi(const vnl_matrix<double> & prob_map,
                        const vnl_matrix<int> & valid_map,
                        const vnl_vector<double> & transition,
                        vcl_vector<int> & optimal_bins);
    
};

#endif /* defined(__CameraPlaning__rgrsn_viterbi__) */
