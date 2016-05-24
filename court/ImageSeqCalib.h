//
//  ImageSeqCalib.h
//  FinalCalib
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __FinalCalib__ImageSeqCalib__
#define __FinalCalib__ImageSeqCalib__

// calibration data in an image sequence
#include <vcl_vector.h>
#include <vpgl/vpgl_perspective_camera.h>
#include "imageSequenceData.h"


class WWoSBasketballReadWriteCamera  // Disney basketball
{
public:
    static bool read_cameras_from_file(const char *file_name, ImageSequenceCameras & cameraSequence);
    static bool read_ptz_from_file(const char *file_name, ImageSequenceCameras & sequence);
    static bool write_cameras(const char *file, const ImageSequenceCameras & cameraSequence);
    static bool write_ptzs(const char *file, const ImageSequenceCameras & cameraSquence);
    
    // read all ptz, both basketball and soccer
    static bool read_ptz(const char *file_name, ImageSequencePTZs &imageSeqPTZ);
};




#endif /* defined(__FinalCalib__ImageSeqCalib__) */
