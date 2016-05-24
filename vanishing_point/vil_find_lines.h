//
//  vil_find_lines.h
//  QuadCopter
//
//  Created by jimmy on 7/6/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_find_lines__
#define __QuadCopter__vil_find_lines__

// wrap code from http://medusa.fit.vutbr.cz/pclines/
// Parallel Coordinates
#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector_fixed.h>
#include <vgl/vgl_point_2d.h>

struct PCLineParameters
{
    double min_canny_gradient_magnitude2_;
    int patchSize;
    
    PCLineParameters()
    {
        min_canny_gradient_magnitude2_ = 100.0;
        patchSize = 32;
    }
};

class VilFindLines
{
public:
    // find lines for VilFindVanishingPoints to detect vanishing point
    static void findLines(const vil_image_view<vxl_byte> & image,
                          const PCLineParameters & para,
                          vcl_vector<vnl_vector_fixed<double, 4> > & outLines);
    
    static void findLinesFromEdges(const vil_image_view<vxl_byte> & edges, const PCLineParameters &  para,
                                   vcl_vector<vnl_vector_fixed<double, 4> > & outLines);
    
};



#endif /* defined(__QuadCopter__vil_find_lines__) */
