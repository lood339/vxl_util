//
//  WWoSSoccerCourtSideViewUtil.h
//  QuadCopter
//
//  Created by jimmy on 8/4/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__WWoSSoccerCourtSideViewUtil__
#define __QuadCopter__WWoSSoccerCourtSideViewUtil__

#include "vpgl_ptz_camera.h"

// only for side view camera
class WWoSSoccerCourtSideViewUtil
{
public:
    static bool cameraToPTZ(const vpgl_perspective_camera<double> & camera, vnl_vector_fixed<double, 3> & ptz);
    static bool PTZToCamera(const vnl_vector_fixed<double, 3> & ptz, vpgl_perspective_camera<double> & camera);
};


#endif /* defined(__QuadCopter__WWoSSoccerCourtSideViewUtil__) */
