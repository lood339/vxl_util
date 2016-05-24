//
//  SoccerImageSeqCalib.h
//  UltimateCalib
//
//  Created by jimmy on 3/5/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __UltimateCalib__SoccerImageSeqCalib__
#define __UltimateCalib__SoccerImageSeqCalib__

#include "ImageSequenceData.h"

class WWoSSoccerReadWriteCamera
{
public:
    static bool read_cameras_from_file(const char *file_name, ImageSequenceCameras & cameraSequence);
    static bool write_cameras(const char *file, const ImageSequenceCameras & cameraSequence);
    
    static bool read_ptz_from_file(const char *file_name, ImageSequenceCameras & sequence);
    static bool write_ptzs(const char *file, const ImageSequenceCameras & cameraSquence);
    
};

#endif /* defined(__UltimateCalib__SoccerImageSeqCalib__) */
