//
//  rgrsn_viterbi.cpp
//  CameraPlaning
//
//  Created by jimmy on 9/16/15.
//  Copyright (c) 2015 Disney Reasearch Pittsburgh. All rights reserved.
//

#include "rgrsn_viterbi.h"
#include "rgrsn_ldp.h"
#include <vcl_limits.h>
#include <vcl_algorithm.h>


bool rgrns_viterbi::local_viterbi(const vnl_matrix<double> & data,
                                  double resolution,
                                  const vnl_vector<double> & transition,
                                  unsigned int window_size,
                                  int num_path,
                                  vnl_vector<double> & optimal_signal)
{
    assert(resolution > 0.0);
    assert(transition.size()%2 == 1);
    
    const double min_v = data.min_value();
    const double max_v = data.max_value();
    const int nBin = (max_v - min_v)/resolution;
    const int path_width_ratio = 20;
    
    // raw data to probability map
    // quantilization
    const int N = data.rows();
    vnl_matrix<double> probMap = vnl_matrix<double>(N, nBin);
    for (int r = 0; r<N; r++) {
        for (int c = 0; c<data.cols(); c++) {
            int num = rgrsn_ldp::value_to_bin_number(min_v, resolution, data[r][c], nBin);
            probMap[r][num] += 1.0;
        }
    }
    probMap /= data.cols(); // normalization
    
    vcl_vector<double> optimalValues(N, 0);
    vcl_vector<int> numValues(N, 0);      // multiple values from local dynamic programming
    vnl_matrix<int> valid_map(window_size, nBin);  //
    valid_map.fill(0);
    
    for (int i = 0; i <= N - window_size; i++) {
        // get a local probMap;
        vnl_matrix<double> localProbMap = probMap.extract(window_size, probMap.cols(), i, 0);
        vcl_vector<int> localOptimalBins;
        if (i < window_size) {
            rgrsn_ldp::viterbi(localProbMap, transition, localOptimalBins);
        }
        else
        {
            // only check the path along with the previous optimal path
            rgrns_viterbi::viterbi(localProbMap, valid_map, transition, localOptimalBins);
        }
        
        assert(localOptimalBins.size() == window_size);
        for (int j = 0; j < localOptimalBins.size(); j++) {
            double value = rgrsn_ldp::bin_number_to_value(min_v, resolution, localOptimalBins[j]);
            numValues[j + i]     += 1;
            optimalValues[j + i] += value;
        }
        
        // set valid_map for next iteration
        valid_map.fill(0);
        for (int j = 0; j<localOptimalBins.size(); j++) {
            int start = localOptimalBins[j] - transition.size() * path_width_ratio;
            int end = localOptimalBins[j]   + transition.size() * path_width_ratio;
            start = (start >= 0) ? start : 0;
            end = (end < nBin) ? end : nBin - 1;
            for (int k = start; k <= end; k++) {
                valid_map[j][k] = 1;
            }
        }
    }
    
    // average all optimal path as final result
    for (int i = 0; i<optimalValues.size(); i++) {
        optimalValues[i] /= numValues[i];
    }
    optimal_signal = vnl_vector<double>(&optimalValues[0], (int)optimalValues.size());
    
    return true;
}


bool rgrns_viterbi::viterbi(const vnl_matrix<double> & prob_map,
                            const vnl_matrix<int> & valid_map,
                            const vnl_vector<double> & transition,
                            vcl_vector<int> & optimal_bins)
{
    printf("this method is not accurate\n");
    assert(0);
    const int N    = prob_map.rows();
    const int nBin = prob_map.cols();
    const int nNeighborBin = transition.size()/2;
    const double epsilon = 0.01;
    const double small_value = -10000.0;  // small value to prevent a cell be used in the optimization
    
    // dynamic programming
    vnl_matrix<double> log_accumulatedProbMap = vnl_matrix<double>(N, nBin);
    log_accumulatedProbMap.fill(0.0);
    vnl_matrix<int> lookbackTable = vnl_matrix<int>(N, nBin);
    lookbackTable.fill(0);
    // copy first row
    for (int c = 0; c<prob_map.cols(); c++) {
        log_accumulatedProbMap[0][c] = log(prob_map[0][c] + epsilon);
        lookbackTable[0][c] = c;
    }
    
    // transition probability to log space
    vnl_vector<double> log_transition = vnl_vector<double>(transition.size(), 0);
    
    for (int i = 0; i<transition.size(); i++) {
        log_transition[i] = log(transition[i] + epsilon);
    }
    
    for (int r = 1; r <N; r++) {
        for (int c = 0; c<prob_map.cols(); c++) {
            // only
            if (valid_map(r, c) == 1) {
                // lookup all possible place in the window
                double max_val = vcl_numeric_limits<int>::min();
                int max_index  = -1;
                for (int w = -nNeighborBin; w <= nNeighborBin; w++) {
                    if (c + w < 0 || c + w >= prob_map.cols()) {
                        continue;
                    }
                    assert(w + nNeighborBin >= 0 && w + nNeighborBin < transition.size());
                    if (valid_map[r-1][c+w] == 0) {
                        continue;
                    }
                    double val = log_accumulatedProbMap[r-1][c+w] + log_transition[w + nNeighborBin];
                    if (val > max_val) {
                        max_val = val;
                        max_index = c + w; // most probable path from the [r-1] row, in column c + w
                    }
                }
                assert(max_index != -1);
                log_accumulatedProbMap[r][c] = max_val + log(prob_map[r][c] + epsilon);
                lookbackTable[r][c]          = max_index;
            }
            else
            {
                log_accumulatedProbMap[r][c] = small_value;
                lookbackTable[r][c] = 0;
            }
            
        }
    }
    
    // lookback the table
    double max_prob    = vcl_numeric_limits<int>::min();
    int max_prob_index = -1;
    for (int c = 0; c<log_accumulatedProbMap.cols(); c++) {
        if (log_accumulatedProbMap[N-1][c] > max_prob) {
            max_prob = log_accumulatedProbMap[N-1][c];
            max_prob_index = c;
        }
    }
    
    // back track
    optimal_bins.push_back(max_prob_index);
    for (int r = N-1; r > 0; r--) {
        int bin = lookbackTable[r][optimal_bins.back()];
        optimal_bins.push_back(bin);
    }
    assert(optimal_bins.size() == N);
    vcl_reverse(optimal_bins.begin(), optimal_bins.end());
    return true;
}
