//
//  WWoSCourtLine.h
//  OnlineStereo
//
//  Created by jimmy on 2/17/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__WWoSCourtLine__
#define __OnlineStereo__WWoSCourtLine__

#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_2d.h>

// line patch sampler
// a patch along the soccer field line
struct LinePatchSampler
{
    int line_id_;
    vgl_point_3d<double> center_pt_;  // unit meter
    
    // image space
    int patchSize_;
    vgl_point_2d<double> im_loc_;    // image location
};

struct WWOSSoccerLinePair
{
    int line_id_;
    vgl_line_segment_2d<double> world_;    // unit meter, fixed value
    vgl_line_segment_2d<double> image_;    // image space
    
    vgl_line_2d<double> world_line_;       // fixed value
    vgl_line_2d<double> image_line_;       // infinite line
    
    WWOSSoccerLinePair()
    {
        line_id_ = -1;
    }
    
    WWOSSoccerLinePair(int lineId, const vgl_point_2d<double> & wldP1, const vgl_point_2d<double> & wldP2)
    {
        line_id_ = lineId;
        world_ = vgl_line_segment_2d<double>(wldP1, wldP2);
        world_line_ = vgl_line_2d<double>(wldP1, wldP2);
    }
};












#endif /* defined(__OnlineStereo__WWoSCourtLine__) */
