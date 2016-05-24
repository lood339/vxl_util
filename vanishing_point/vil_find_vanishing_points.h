//
//  vil_find_vanishing_points.h
//  QuadCopter
//
//  Created by jimmy on 7/7/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_find_vanishing_points__
#define __QuadCopter__vil_find_vanishing_points__

// wrap code from http://medusa.fit.vutbr.cz/pclines/

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_vector_fixed.h>

struct VanishingPointParameter
{
    int nVP_;  // number of vanishing point
    int imageW_;
    int imageH_;
    int spaceSize_;
    
    VanishingPointParameter()
    {
        nVP_ = 3;
        spaceSize_ = 320;
    }
};

class VilFindVanishingPoints
{
public:
    // works but not stable, it depends on the the quality of lines from canny detection
    static vcl_vector<vgl_point_2d<double> > findVanP(const vcl_vector<vnl_vector_fixed<double, 4> > & lines,
                                                      const VanishingPointParameter & para);
};

#endif /* defined(__QuadCopter__vil_find_vanishing_points__) */
