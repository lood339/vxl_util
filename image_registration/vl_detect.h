//
//  vl_detect.h
//  QuadCopter
//
//  Created by jimmy on 7/18/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef QuadCopter_vl_detect_h
#define QuadCopter_vl_detect_h

#ifdef __cplusplus
extern "C" {
#endif
    
// generate keypoint at cumstomed location
// only check smallest sigma
// do not check the keypoint quality
// locations: input location in original image
void vl_sift_detect_custom(VlSiftFilt * f, double *locations, int nLoc);
    
#ifdef __cplusplus
}
#endif

#endif
