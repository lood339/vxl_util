//
//  vxl_sgs_smooth.h
//  FinalCalib
//
//  Created by Jimmy Chen LOCAL on 7/20/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __FinalCalib__vxl_sgs_smooth__
#define __FinalCalib__vxl_sgs_smooth__

#include "sgsmooth.h"
#include <vector>
#include <vnl/vnl_vector.h>

class VxlSgsSmooth
{
public:
    // window_size: 16
    // order: 1
    static void smooth(std::vector<double> & data_in_out, int window_size, int order);
    static bool smooth(const std::vector<vnl_vector<double> > & data,
                       std::vector<vnl_vector<double> > & smoothedData, int window_size, int order);
};

#endif /* defined(__FinalCalib__vxl_sgs_smooth__) */
