//
//  vil_algo_plus.h
//  QuadCopter
//
//  Created by jimmy on 7/9/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_algo_plus__
#define __QuadCopter__vil_algo_plus__

#include <vgl/vgl_point_2d.h>
#include <vcl_vector.h>
#include <vil/vil_image_view.h>

class VilAlgoPlus
{
public:
    // 0, 1, 2, 3 of canny direction
    //       2
    //    3     1
    //       0
    static int cannyDirection(double dx, double dy);    
    
    //   3 2 1
    //   4 * 0
    //   5 6 7
    // discrete direction from p1 to p2
    static int octaveIndex(double dx, double dy);
    
    // scan direction from octative Index
    static vgl_vector_2d<int> scanDirectionFromOctaveIndex(int oct_index);
    
    // discrete direction into 8 directions. The direction is from p1 to p2
    static vgl_vector_2d<int> scanDirection(const vgl_point_2d<double> & fromPt, const vgl_point_2d<double> & toPt);
    
    // pixels locate on the line
    static bool linePixels(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts, int w, int h);
    
    // linear interpolate from center (1.0) to edge (0.0)
    static bool linearInterpolateFromCenter(int imageW, int imageH, vil_image_view<double> & wtImage);   
    
    
    // points located in the line p0 -- p1
    static bool linePixels(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts);
};


#endif /* defined(__QuadCopter__vil_algo_plus__) */
