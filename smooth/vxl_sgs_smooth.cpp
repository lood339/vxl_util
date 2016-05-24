//
//  vxl_sgs_smooth.cpp
//  FinalCalib
//
//  Created by Jimmy Chen LOCAL on 7/20/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_sgs_smooth.h"
#include <assert.h>

void VxlSgsSmooth::smooth(vcl_vector<double> & data, int window_size, int order)
{
    double *pSmoothed = calc_sgsmooth((int)data.size(), &data[0], window_size, order);
    assert(pSmoothed == &data[0]);
}

bool VxlSgsSmooth::smooth(const vcl_vector<vnl_vector<double> > & data,
                          vcl_vector<vnl_vector<double> > & smoothedData, int window_size, int order)
{
    smoothedData = data;
    
    // smooth in each dimension
    const int dimNum = (int)data[0].size();
    const int sz = (int)data.size();
    for (int i = 0; i<dimNum; i++) {
        vcl_vector<double> orgData(sz);
        
        for (int j = 0; j<data.size(); j++) {
            orgData[j] = data[j][i];
        }
        
        VxlSgsSmooth::smooth(orgData, window_size, order);
        for (int j = 0; j<smoothedData.size(); j++) {
            smoothedData[j][i] = orgData[j];
        }
    }
    return true;
}
