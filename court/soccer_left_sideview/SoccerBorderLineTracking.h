//
//  SoccerBorderLineTracking.h
//  QuadCopter
//
//  Created by jimmy on 7/7/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__SoccerBorderLineTracking__
#define __QuadCopter__SoccerBorderLineTracking__

#include "vil_gmm.h"
#include "SoccerLineParameters.h"
#include <vnl/vnl_vector_fixed.h>
#include "vil_line_tracking.h"
#include "vil_weak_line_tracking.h"

// for WWoS soccer video left side view only

class SoccerBorderLineTracking
{
public:
    // border line: four border lines in the soccer field
    // scan from grass land area to nun-grassland area
    // far touch line
    // adjust parameters to fit the border line
    static bool borderLineDetection(const VilGMM & green_gmm, const vil_image_view<vxl_byte> & image,
                                    const vil_image_view<double> & magnitude,
                                    const vgl_line_segment_2d<double> & initLineSegment,
                                    const LeftsideViewBorderLineParameter & para,
                                    vgl_line_2d<double> & line);
    // cb_gmm: commercial board gaussian
    // pixelTypes:
    static bool commercialBoardDetectionByColorClassifer(const vil_image_view<vxl_byte> & image,  // for test only
                                                         const vil_image_view<vxl_byte> & pixelTypes,
                                                         const vgl_line_segment_2d<double> & initLineSegment,
                                                         const FarCommerticalBoarderLineParameter & para,
                                                         vgl_line_2d<double> & line);
    // right border: border line behind right goal
    static bool rightBorderDetectionByColorClassifer(const vil_image_view<vxl_byte> & image,  // for test only
                                                     const vil_image_view<vxl_byte> & pixelTypes,
                                                     const vgl_line_segment_2d<double> & initLineSegment,
                                                     const RightBorderLine & para,
                                                     vgl_line_2d<double> & line);
    
    // scan line and adaptively collect parameters    
    static vcl_vector<vgl_point_2d<double> > scan_line(const vil_image_view<double> & magnitude,
                                                                   const vcl_vector<vgl_point_2d<double> > & scanLinePts,
                                                                   ScanLineParameter & para);
       
    
private:
    
    
    
    
    
};

#endif /* defined(__QuadCopter__SoccerBorderLineTracking__) */
