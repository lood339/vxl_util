//
//  rgrsn_ldp.cpp
//  CameraPlaning
//
//  Created by jimmy on 8/6/15.
//  Copyright (c) 2015 Disney Reasearch Pittsburgh. All rights reserved.
//

#include "rgrsn_ldp.h"
#include <vcl_algorithm.h>
#include <vnl/vnl_matlab_filewrite.h>
#include "vnl_plus.h"
#include <vcl_limits.h>
#include <vcl_algorithm.h>

unsigned rgrsn_ldp::value_to_bin_number(double v_min, double interval, double value, const unsigned nBin)
{
    int num = (value - v_min)/interval;
    if (num < 0) {
        return 0;
    }
    if (num >= nBin) {
        return nBin - 1;
    }
    return (unsigned)num;
}

double rgrsn_ldp::bin_number_to_value(double v_min, double interval, int bin)
{
    return v_min + bin * interval;
}

bool rgrsn_ldp::dynamic_programming(const vnl_matrix<double> & data, double v_min, double v_max,
                                    unsigned int nBin, int nJumpBin, unsigned int windowSize,
                                    vnl_vector<double> & optimalSignal)
{
    assert(v_min < v_max);
    // raw data to probability map
    const int N = data.rows();
    vnl_matrix<double> probMap = vnl_matrix<double>(N, nBin);
    double interval = (v_max - v_min)/nBin;
    for (int r = 0; r<N; r++) {
        for (int c = 0; c<data.cols(); c++) {
            int num = value_to_bin_number(v_min, interval, data[r][c], nBin);
            probMap[r][num] += 1.0;
        }
    }
    probMap /= data.cols(); // normalize
    
    // save prob
    {
      //  vnl_matlab_filewrite awriter("prob.mat");
      //  awriter.write(probMap, "prob");
     //   printf("save to prob.mat.\n");
    }
  
    vcl_vector<double> optimalValues(N, 0);
    vcl_vector<int> numValues(N, 0);      // multiple values from local dynamic programming
    for (int i = 0; i<=N - windowSize; i++) {
        // get a local probMap;
        vnl_matrix<double> localProbMap = probMap.extract(windowSize, probMap.cols(), i, 0);
        vcl_vector<int> localOptimalBins;
        rgrsn_ldp::local_dynamic_programming(localProbMap, nJumpBin, localOptimalBins);
        assert(localOptimalBins.size() == windowSize);
        for (int j = 0; j < localOptimalBins.size(); j++) {
            numValues[j + i]     += 1;
            optimalValues[j + i] += bin_number_to_value(v_min, interval, localOptimalBins[j]);
        }
        
        // test
        if (0 && i == 0)
        {
            printf("test first window output\n");
            for (int j = 0; j<optimalValues.size() && j<windowSize; j++) {
                printf("%f ", optimalValues[j]);
            }
            printf("\n");
        }
    }
    //
    for (int i = 0; i<optimalValues.size(); i++) {
        optimalValues[i] /= numValues[i];
    }
    optimalSignal = vnl_vector<double>(&optimalValues[0], (int)optimalValues.size());
    return true;
}

bool rgrsn_ldp::dynamic_programming(const vnl_matrix<double> & data,
                                    double v_min, double v_max,
                                    unsigned int nBin,
                                    int nJumpBin,
                                    unsigned int windowSize,
                                    vnl_vector<double> & optimalSignal,
                                    vnl_vector<double> & signal_variance)
{
    assert(v_min < v_max);
    // raw data to probability map
    // quantilization
    const int N = data.rows();
    vnl_matrix<double> probMap = vnl_matrix<double>(N, nBin);
    double interval = (v_max - v_min)/nBin;
    for (int r = 0; r<N; r++) {
        for (int c = 0; c<data.cols(); c++) {
            int num = value_to_bin_number(v_min, interval, data[r][c], nBin);
            probMap[r][num] += 1.0;
        }
    }
    probMap /= data.cols(); // normalization
    
    
    vcl_vector<double> optimalValues(N, 0);
    vcl_vector<int> numValues(N, 0);      // multiple values from local dynamic programming
    vcl_vector<vcl_vector<double> > all_values(N);
    for (int i = 0; i<=N - windowSize; i++) {
        // get a local probMap;
        vnl_matrix<double> localProbMap = probMap.extract(windowSize, probMap.cols(), i, 0);
        vcl_vector<int> localOptimalBins;
        rgrsn_ldp::local_dynamic_programming(localProbMap, nJumpBin, localOptimalBins);
        assert(localOptimalBins.size() == windowSize);
        for (int j = 0; j < localOptimalBins.size(); j++) {
            double value = bin_number_to_value(v_min, interval, localOptimalBins[j]);
            assert(j + i < N);
            all_values[j + i].push_back(value);
            numValues[j + i]     += 1;
            optimalValues[j + i] += value;
        }
    }
    // mean value
    for (int i = 0; i<optimalValues.size(); i++) {
        optimalValues[i] /= numValues[i];
    }
    
    // variance
    signal_variance = vnl_vector<double>(N, 0);
    for (int i = 0; i<optimalValues.size(); i++) {
        assert(all_values[i].size() > 0);
        if (all_values[i].size() == 1) {
            signal_variance[i] = 0.0001;
        }
        else
        {
            double dump_mean = 0.0;
            double sigma = 0.0;
            VnlPlus::mean_std(&all_values[i][0], (int)all_values[i].size(), dump_mean, sigma);
            signal_variance[i] = sigma + 0.0001; // avoid zero
        }
    }
    optimalSignal = vnl_vector<double>(&optimalValues[0], (int)optimalValues.size());
    
    // save variance with the size of window size, for test purpose
    if(0)
    {
        vcl_vector<vnl_vector<double> > all_value_vecs;
        for (int i = 0; i<all_values.size(); i++) {
            if (all_values[i].size() == windowSize) {
                all_value_vecs.push_back(VnlPlus::vector_2_vec(all_values[i]));
            }
        }
        
        vcl_string save_file("ldp_all_prediction.mat");
        vnl_matlab_filewrite awriter(save_file.c_str());
        awriter.write(VnlPlus::vector_2_mat(all_value_vecs), "ldp_all_opt_path");
        printf("save to %s\n", save_file.c_str());
    }
    return true;
}

bool rgrsn_ldp::dynamic_programming_median(const vnl_matrix<double> & data,
                                           double v_min, double v_max,
                                           unsigned int nBin,
                                           int nJumpBin,
                                           unsigned int windowSize,
                                           vnl_vector<double> & optimalSignal,
                                           vnl_vector<double> & medianSignal)
{
    assert(v_min < v_max);
    // raw data to probability map
    // quantilization
    const int N = data.rows();
    vnl_matrix<double> probMap = vnl_matrix<double>(N, nBin);
    double interval = (v_max - v_min)/nBin;
    for (int r = 0; r<N; r++) {
        for (int c = 0; c<data.cols(); c++) {
            int num = value_to_bin_number(v_min, interval, data[r][c], nBin);
            probMap[r][num] += 1.0;
        }
    }
    probMap /= data.cols(); // normalization
    
    vcl_vector<double> optimalValues(N, 0);
    vcl_vector<int> numValues(N, 0);         // multiple values from local dynamic programming
    vcl_vector<vcl_vector<double> > all_values(N);
    for (int i = 0; i<=N - windowSize; i++) {
        // get a local probMap;
        vnl_matrix<double> localProbMap = probMap.extract(windowSize, probMap.cols(), i, 0);
        vcl_vector<int> localOptimalBins;
        rgrsn_ldp::local_dynamic_programming(localProbMap, nJumpBin, localOptimalBins);
        assert(localOptimalBins.size() == windowSize);
        for (int j = 0; j < localOptimalBins.size(); j++) {
            double value = bin_number_to_value(v_min, interval, localOptimalBins[j]);
            assert(j + i < N);
            all_values[j + i].push_back(value);
            numValues[j + i]     += 1;
            optimalValues[j + i] += value;
        }
    }
    // mean value
    for (int i = 0; i<optimalValues.size(); i++) {
        optimalValues[i] /= numValues[i];
    }
    
    // variance
    medianSignal = vnl_vector<double>(N, 0);
    for (int i = 0; i<optimalValues.size(); i++) {
        assert(all_values[i].size() > 0);
        if (all_values[i].size() == 1) {
            medianSignal[i] = all_values[i][0];
        }
        else
        {
            size_t n = all_values[i].size() / 2;
            vcl_nth_element(all_values[i].begin(), all_values[i].begin() + n, all_values[i].end());
            medianSignal[i] = all_values[i][n];
        }
    }
    optimalSignal = vnl_vector<double>(&optimalValues[0], (int)optimalValues.size());
    
    return true;
}

bool rgrsn_ldp::local_viterbi(const vnl_matrix<double> & data,
                              double resolution,
                              const vnl_vector<double> & transition,
                              unsigned int window_size,
                              vnl_vector<double> & optimal_signal)
{
    assert(resolution > 0.0);
    assert(transition.size()%2 == 1);
    
    const double min_v = data.min_value();
    const double max_v = data.max_value();
    const int nBin = (max_v - min_v)/resolution;
    
    // raw data to probability map
    // quantilization
    const int N = data.rows();
    vnl_matrix<double> probMap = vnl_matrix<double>(N, nBin);
    for (int r = 0; r<N; r++) {
        for (int c = 0; c<data.cols(); c++) {
            int num = value_to_bin_number(min_v, resolution, data[r][c], nBin);
            probMap[r][num] += 1.0;
        }
    }
    probMap /= data.cols(); // normalization
    
    vcl_vector<double> optimalValues(N, 0);
    vcl_vector<int> numValues(N, 0);      // multiple values from local dynamic programming
    
    for (int i = 0; i <= N - window_size; i++) {
        // get a local probMap;
        vnl_matrix<double> localProbMap = probMap.extract(window_size, probMap.cols(), i, 0);
        vcl_vector<int> localOptimalBins;
        rgrsn_ldp::viterbi(localProbMap, transition, localOptimalBins);
        assert(localOptimalBins.size() == window_size);
        for (int j = 0; j < localOptimalBins.size(); j++) {
            double value = bin_number_to_value(min_v, resolution, localOptimalBins[j]);
            numValues[j + i]     += 1;
            optimalValues[j + i] += value;
        }
    }
    
    // average all optimal path as final result
    for (int i = 0; i<optimalValues.size(); i++) {
        optimalValues[i] /= numValues[i];
    }
    optimal_signal = vnl_vector<double>(&optimalValues[0], (int)optimalValues.size());
    
    return true;
}

bool rgrsn_ldp::local_viterbi_overlapping_ratio(const vnl_matrix<double> & data,
                                                double resolution,
                                                const vnl_vector<double> & transition,
                                                unsigned int window_size,
                                                const double overlapping_ratio,
                                                vnl_vector<double> & optimal_signal)
{
    assert(resolution > 0.0);
    assert(transition.size()%2 == 1);
    assert(overlapping_ratio >=0 && overlapping_ratio < 1.0);
    
    const double min_v = data.min_value();
    const double max_v = data.max_value();
    const int nBin = (max_v - min_v)/resolution;
    const int moving_step = window_size * (1.0 - overlapping_ratio);  // window move forward step
    assert(moving_step >= 1);
    
    // raw data to probability map
    // quantilization
    const int N = data.rows();
    vnl_matrix<double> probMap = vnl_matrix<double>(N, nBin);
    for (int r = 0; r<N; r++) {
        for (int c = 0; c<data.cols(); c++) {
            int num = value_to_bin_number(min_v, resolution, data[r][c], nBin);
            probMap[r][num] += 1.0;
        }
    }
    probMap /= data.cols(); // normalization
    
    vcl_vector<double> optimalValues(N, 0);
    vcl_vector<int> numValues(N, 0);      // multiple values from local dynamic programming
    
    int last_index = 0;
    for (int i = 0; i <= N - window_size; i += moving_step) {
        // get a local probMap;
        vnl_matrix<double> localProbMap = probMap.extract(window_size, probMap.cols(), i, 0);
        vcl_vector<int> localOptimalBins;
        rgrsn_ldp::viterbi(localProbMap, transition, localOptimalBins);
        assert(localOptimalBins.size() == window_size);
        for (int j = 0; j < localOptimalBins.size(); j++) {
            double value = bin_number_to_value(min_v, resolution, localOptimalBins[j]);
            numValues[j + i]     += 1;
            optimalValues[j + i] += value;
        }
        last_index = i;
    }
    
    // with fully overlapping for last several numbers
    for (int i = last_index; i <= N - window_size; i++) {
        vnl_matrix<double> localProbMap = probMap.extract(window_size, probMap.cols(), i, 0);
        vcl_vector<int> localOptimalBins;
        rgrsn_ldp::viterbi(localProbMap, transition, localOptimalBins);
        assert(localOptimalBins.size() == window_size);
        for (int j = 0; j < localOptimalBins.size(); j++) {
            double value = bin_number_to_value(min_v, resolution, localOptimalBins[j]);
            numValues[j + i]     += 1;
            optimalValues[j + i] += value;
        }
    }
    // average all optimal path as final result
    for (int i = 0; i<optimalValues.size(); i++) {
        assert(numValues[i] != 0);
        optimalValues[i] /= numValues[i];
    }
    optimal_signal = vnl_vector<double>(&optimalValues[0], (int)optimalValues.size());
    return true;
}


bool rgrsn_ldp::local_viterbi(const vnl_matrix<double> & data,
                              double resolution,
                              const vnl_vector<double> & transition,
                              unsigned int window_size,
                              vnl_vector<double> & optimal_signal,
                              vnl_vector<double> & signal_variance)
{
    assert(resolution > 0.0);
    assert(transition.size()%2 == 1);
    
    const double min_v = data.min_value();
    const double max_v = data.max_value();
    const int nBin = (max_v - min_v)/resolution;
    
    // raw data to probability map
    // quantilization
    const int N = data.rows();
    vnl_matrix<double> probMap = vnl_matrix<double>(N, nBin);
    for (int r = 0; r<N; r++) {
        for (int c = 0; c<data.cols(); c++) {
            int num = value_to_bin_number(min_v, resolution, data[r][c], nBin);
            probMap[r][num] += 1.0;
        }
    }
    probMap /= data.cols(); // normalization
    
    vcl_vector<double> optimalValues(N, 0);
    vcl_vector<int> numValues(N, 0);      // multiple values from local dynamic programming
    vcl_vector<vcl_vector<double> > all_values(N);   // for calculate variance
    for (int i = 0; i<=N - window_size; i++) {
        // get a local probMap;
        vnl_matrix<double> localProbMap = probMap.extract(window_size, probMap.cols(), i, 0);
        vcl_vector<int> localOptimalBins;
        rgrsn_ldp::viterbi(localProbMap, transition, localOptimalBins);
        assert(localOptimalBins.size() == window_size);
        for (int j = 0; j < localOptimalBins.size(); j++) {
            double value = bin_number_to_value(min_v, resolution, localOptimalBins[j]);
            numValues[j + i]     += 1;
            optimalValues[j + i] += value;
            all_values[j + i].push_back(value);
        }
    }
    
    //
    for (int i = 0; i<optimalValues.size(); i++) {
        optimalValues[i] /= numValues[i];
    }
    optimal_signal = vnl_vector<double>(&optimalValues[0], (int)optimalValues.size());
    
    if(1)
    {
        vcl_vector<vnl_vector<double> > all_value_vecs;
        for (int i = 0; i<all_values.size(); i++) {
            if (all_values[i].size() == window_size) {
                all_value_vecs.push_back(VnlPlus::vector_2_vec(all_values[i]));
            }
        }
        
        vcl_string save_file("lv_all_prediction.mat");
        vnl_matlab_filewrite awriter(save_file.c_str());
        awriter.write(VnlPlus::vector_2_mat(all_value_vecs), "lv_all_opt_path");
        printf("save to %s\n", save_file.c_str());
    }

    
    return true;
}

bool rgrsn_ldp::transition_matrix(const vcl_vector<int> & fns,
                                  const vcl_vector<double> & values,
                                  vnl_matrix<double> & transition,
                                  const double resolution)
{
    assert(fns.size() == values.size());
    
    double min_v = *vcl_min_element(values.begin(), values.end());
    double max_v = *vcl_max_element(values.begin(), values.end());
    
    unsigned num_bin = (max_v - min_v)/resolution;
    transition = vnl_matrix<double>(num_bin, num_bin, 0.0);
    vnl_vector<double> column(num_bin, 0.0);
    for (int i = 0; i<fns.size()-1; i++) {
        if (fns[i] + 1 == fns[i+1]) {
            double cur_v = values[i];
            double next_v = values[i+1];
            unsigned cur_bin = value_to_bin_number(min_v, resolution, cur_v, num_bin);
            unsigned next_bin = value_to_bin_number(min_v, resolution, next_v, num_bin);
            transition[next_bin][cur_bin] += 1.0;
            column[cur_bin] += 1.0;
        }
    }
    
    // normalize each column
    for (int r = 0; r < transition.rows(); r++) {
        for (int c = 0; c < transition.cols(); c++) {
            transition[r][c] /= column[c];
        }
    }
    return true;
}

bool rgrsn_ldp::compact_transition_matrix(const vcl_vector<int> & fns,
                                          const vcl_vector<double> & values,
                                          vnl_matrix<double> & transition,
                                          const double resolution)
{
    assert(fns.size() == values.size());
    
    double min_v = *vcl_min_element(values.begin(), values.end());
    double max_v = *vcl_max_element(values.begin(), values.end());
    unsigned num_bin = (max_v - min_v)/resolution;
    
    unsigned max_bin_transition = 0;  // biggest transition between frames quantized in bin
    for (int i = 0; i<fns.size(); i++) {
        if (fns[i] + 1 == fns[i+1]) {
            double cur_v  = values[i];
            double next_v = values[i+1];
            int cur_bin  = value_to_bin_number(min_v, resolution, cur_v, num_bin);
            int next_bin = value_to_bin_number(min_v, resolution, next_v, num_bin);
            int dif = abs(next_bin - cur_bin);
            if (dif > max_bin_transition) {
                max_bin_transition = dif;
            }
        }
    }
    transition = vnl_matrix<double>(max_bin_transition * 2 + 1, num_bin, 0.0);
    vnl_vector<double> column(num_bin, 0.0);
    for (int i = 0; i<fns.size()-1; i++) {
        if (fns[i] + 1 == fns[i+1]) {
            double cur_v = values[i];
            double next_v = values[i+1];
            int cur_bin  = value_to_bin_number(min_v, resolution, cur_v, num_bin);
            int next_bin = value_to_bin_number(min_v, resolution, next_v, num_bin);
            int row = max_bin_transition + (next_bin - cur_bin);
            transition[row][cur_bin] += 1.0;
            column[cur_bin] += 1.0;
        }
    }
    
    // normalize each column
    for (int r = 0; r < transition.rows(); r++) {
        for (int c = 0; c < transition.cols(); c++) {
    //        transition[r][c] /= column[c];
        }
    }
    
    return true;
}


bool rgrsn_ldp::local_dynamic_programming(const vnl_matrix<double> & probMap, int nNeighborBin,
                                          vcl_vector<int> & optimalBins)
{
    const int N    = probMap.rows();
    const int nBin = probMap.cols();
    
    // dynamic programming
    vnl_matrix<double> accumulatedProbMap = vnl_matrix<double>(N, nBin);
    accumulatedProbMap.fill(0.0);
    vnl_matrix<int> lookbackTable = vnl_matrix<int>(N, nBin);
    lookbackTable.fill(0);
    // copy first row
    for (int c = 0; c<probMap.cols(); c++) {
        accumulatedProbMap[0][c] = probMap[0][c];
        lookbackTable[0][c] = c;
    }
    
    for (int r = 1; r <N; r++) {
        for (int c = 0; c<probMap.cols(); c++) {
            // lookup all possible place in the window
            double max_val = -1;
            int max_index  = -1;
            for (int w = -nNeighborBin; w <= nNeighborBin; w++) {
                if (c + w <0 || c + w >= probMap.cols()) {
                    continue;
                }
                double val = probMap[r][c] + accumulatedProbMap[r-1][c+w];
                if (val > max_val) {
                    max_val = val;
                    max_index = c + w; // most probable path from the [r-1] row, in column c + w
                }
            }
            assert(max_index != -1);
            accumulatedProbMap[r][c] = max_val;
            lookbackTable[r][c]      = max_index;
        }
    }
    
    // lookback the table
    double max_prob    = -1.0;
    int max_prob_index = -1;
    for (int c = 0; c<accumulatedProbMap.cols(); c++) {
        if (accumulatedProbMap[N-1][c] > max_prob) {
            max_prob = accumulatedProbMap[N-1][c];
            max_prob_index = c;
        }
    }
    
    // back track
    optimalBins.push_back(max_prob_index);
    for (int r = N-1; r > 0; r--) {
        int bin = lookbackTable[r][optimalBins.back()];
        optimalBins.push_back(bin);
    }
    assert(optimalBins.size() == N);
    
  //  vcl_reverse(optimalBins.begin(), optimalBins.end());
    return true;
}

bool rgrsn_ldp::viterbi(const vnl_matrix<double> & prob_map, const vnl_vector<double> & transition,
                        vcl_vector<int> & optimal_bins)
{
    
    const int N    = prob_map.rows();
    const int nBin = prob_map.cols();
    const int nNeighborBin = transition.size()/2;
    const double epsilon = 0.01;
    
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
    vnl_vector<double> log_transition = vnl_vector<double>(transition.size(), 0);
    
    for (int i = 0; i<transition.size(); i++) {
        log_transition[i] = log(transition[i] + epsilon);
    }
    
    for (int r = 1; r <N; r++) {
        for (int c = 0; c<prob_map.cols(); c++) {
            // lookup all possible place in the window
            double max_val = vcl_numeric_limits<int>::min();
            int max_index  = -1;
            for (int w = -nNeighborBin; w <= nNeighborBin; w++) {
                if (c + w < 0 || c + w >= prob_map.cols()) {
                    continue;
                }
                assert(w + nNeighborBin >= 0 && w + nNeighborBin < transition.size());
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


bool rgrsn_ldp::local_dynamic_programming_log(const vnl_matrix<double> & probMap, int nNeighborBin,
                                              vcl_vector<int> & optimalBins)
{
    // find minimum path
    const int N    = probMap.rows();
    const int nBin = probMap.cols();
    const double epsilon = 0.01;
    
    vnl_matrix<double> negLogProbMap(N, nBin);
    for (int r = 0; r<N; r++) {
        for (int c = 0; c <nBin; c++) {
            negLogProbMap(r, c) = -log(probMap(r, c) + epsilon);
        }
    }
    
    // dynamic programming
    vnl_matrix<double> accumulatedMap = vnl_matrix<double>(N, nBin);
    accumulatedMap.fill(0.0);
    vnl_matrix<int> lookbackTable = vnl_matrix<int>(N, nBin);
    lookbackTable.fill(0);
    // copy first row
    for (int c = 0; c<negLogProbMap.cols(); c++) {
        accumulatedMap[0][c] = negLogProbMap[0][c];
    }
    
    for (int r = 1; r <N; r++) {
        for (int c = 0; c<negLogProbMap.cols(); c++) {
            // lookup all possible place in the window
            double min_val = INT_MAX;
            int index      = -1;
            for (int w = -nNeighborBin; w <= nNeighborBin; w++) {
                if (c + w <0 || c + w >= negLogProbMap.cols()) {
                    continue;
                }
                double val = negLogProbMap[r][c] + accumulatedMap[r-1][c+w];
                if (val < min_val) {
                    min_val = val;
                    index = c + w;
                }
            }
            assert(index != -1);
            accumulatedMap[r][c] = min_val;
            lookbackTable[r][c]  = index;
        }
    }
    
    // lookback the table
    double min_val = INT_MAX;
    int initIndex  = -1;
    for (int c = 0; c<accumulatedMap.cols(); c++) {
        if (accumulatedMap[N-1][c] < min_val) {
            min_val = accumulatedMap[N-1][c];
            initIndex = c;
        }
    }
    
    // back track
    optimalBins.push_back(initIndex);
    for (int r = N-1; r > 0; r--) {
        int bin = lookbackTable[r][optimalBins.back()];
        optimalBins.push_back(bin);
    }
    assert(optimalBins.size() == N);
    
    vcl_reverse(optimalBins.begin(), optimalBins.end());
    return true;
}
