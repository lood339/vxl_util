//
//  vil_bay_line_matching.h
//  QuadCopter
//
//  Created by jimmy on 6/16/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_bay_line_matching__
#define __QuadCopter__vil_bay_line_matching__

// implement Herbert Bay's paper "wide-baseline stereo matching with line segments" cvpr 2005

#include "cvx_LSD.h"
#include <vcl_utility.h>
#include <vcl_algorithm.h>


// from "wide-baseline stereo matching with line segments" cvpr 2015
// histogram is 18 * 3 * 3 in HSV color space + 4 grey scale
struct LineAppearance
{
    vgl_line_segment_2d<double> seg_; // line segment in image
    vnl_vector<int> left_bin_;     // histogram bin in left area of the segment
    vnl_vector<int> right_bin_;    // histogram bin in right area of the segment
    int left_pixel_num_;
    int right_pixel_num_;
    
    vnl_vector<double> norm_bin_left_;
    vnl_vector<double> norm_bin_right_;    
    
    double distance(const LineAppearance & other) const
    {
        return (vnl_vector_ssd(norm_bin_left_, other.norm_bin_left_) + vnl_vector_ssd(norm_bin_right_, other.norm_bin_right_))/2.0;
    }
    
    void getNormBin(void)
    {
        assert(left_bin_.size() == right_bin_.size());
        
        norm_bin_left_  = vnl_vector<double>(left_bin_.size());
        norm_bin_right_ = vnl_vector<double>(right_bin_.size());
        double inv1 = 1.0/left_pixel_num_;
        double inv2 = 1.0/right_pixel_num_;
        for(int i = 0; i<norm_bin_left_.size(); i++)
        {
            norm_bin_left_[i]  = inv1 * left_bin_[i];
            norm_bin_right_[i] = inv2 * right_bin_[i];
        }
    }
    
    vgl_point_2d<double> center(void) const
    {
        return centre(seg_.point1(), seg_.point2());
    }
    
};

struct BayLineMatchingParameter
{
    double distance_threshold_;
    double violation_threshold_;
    bool find_more_mathes_;  // cost more time and may add mis-matchings
    
    BayLineMatchingParameter()
    {
        distance_threshold_  = 0.25;
        violation_threshold_ = 0.15;
        find_more_mathes_ = false;
    }
    
};


class VilBayLineMatching
{
public:
    // calcuate hisrogram along the line in HSV color space, line segment direction from p1 -- > p2
    static void generateLineAppearance(const vil_image_view<vxl_byte> & image,
                                       const vcl_vector<LSDLineSegment> & line_segments,
                                       double threshold_length, int color_profile_width,
                                       vcl_vector<LineAppearance> & line_seg_appearances);
    
    // match line segments by section 3. Topological filter
    static void match_lines(const vcl_vector<LineAppearance> & line_seg1, const vcl_vector<LineAppearance> & line_seg2,
                            vcl_vector<vcl_pair<int, int> > & matchIndices, const BayLineMatchingParameter & para);
    
};




#endif /* defined(__QuadCopter__vil_bay_line_matching__) */
