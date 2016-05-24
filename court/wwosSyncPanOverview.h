//
//  wwosSyncPanOverview.h
//  PlayerTracking
//
//  Created by jimmy on 7/21/15.
//  Copyright (c) 2015 Disney Reasearch. All rights reserved.
//

#ifndef __PlayerTracking__wwosSyncPanOverview__
#define __PlayerTracking__wwosSyncPanOverview__

// syncize pan angle and stationary overview image
#include <vil/vil_image_view.h>
#include <vcl_string.h>
#include "cameraCalibPlayerDetectionData.h"
#include <vnl/vnl_vector_fixed.h>

struct ObservationData
{
    unsigned int fn_;   // 25 fps
    vcl_string image_name_;
};

struct ObservationPTZData
{
    unsigned int fn_;         // frame number in stationary camera, 25 fps
    vcl_string image_name_;
    double pan_;
};

class WWoSSyncPanOverview
{
    int ptz1_;
    int ptz2_;
    int stationary1_;
    int stationary2_;
    
    vcl_vector<PTZData> ptzs_;
    vcl_vector<ObservationData> observations_;
public:
    WWoSSyncPanOverview();
    WWoSSyncPanOverview(const vnl_vector_fixed<int, 4> & syncFrames);
    ~WWoSSyncPanOverview();
    
    // from stationary camera
    bool readObservation(const char *folder, int start_fn, int step);
    // from ptz camera
    bool readPTZCameras(const char *ptzFilename, int dump_lines_num = 1);
    bool syncObservationPTZ(vcl_vector<ObservationPTZData> & observation_ptzs);
    
    static vnl_vector_fixed<int, 4> MarkinCameraSyncFrames()
    {
        vnl_vector_fixed<int, 4> sync;
        sync[0] = 5410;
        sync[1] = 411842;
        sync[2] = 2254;
        sync[3] = 171597;
        return sync;
    }
    
    static bool writeObservationPTZ(const char *fileName, const vcl_vector<ObservationPTZData> & observation_ptzs);
    static bool readObservationPTZ(const char *fileName, vcl_vector<ObservationPTZData> & observation_ptzs);
    
private:
    int getFnPTZ(int fn25);
};

#endif /* defined(__PlayerTracking__wwosSyncPanOverview__) */
