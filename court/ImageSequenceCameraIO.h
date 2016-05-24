//
//  ImageSequenceCameraIO.h
//  VideoCalibVXL
//
//  Created by jimmy on 7/18/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __VideoCalibVXL__ImageSequenceCameraIO__
#define __VideoCalibVXL__ImageSequenceCameraIO__
// image sequence camera read and write

#include "ImageSequenceData.h"
#include "cameraCalibPlayerDetectionData.h"


class ImageSequenceCameraIO
{
public:
    static bool read_cameras(const char *file_name, ImageSequenceCameras & cameraSequence);
    static bool write_cameras(const char *file, const ImageSequenceCameras & cameraSequence);
    
    static bool read_ptzFrames(const char *file, ImageSequencePTZs & ptzs);
    static bool write_ptzFrames(const char *file, const ImageSequencePTZs & ptzs);
    static bool interpolate_ptzFrames(const ImageSequencePTZs & originalPTZs, ImageSequencePTZs & interpolatedPTZs);
    
    
    static bool write_ptzVideoParameter(const char *file_name, const PTZVideoParameter &para);
    static bool read_ptzVideoParameter(const char *file, PTZVideoParameter &para);
};

class PlayerPositionIO
{
public:
    // read part of frames
    static bool read_players(const char *file, PTZPlayerSet & players, int stare_frame, int end_frame);
    static bool write_players(const char *file, const PTZPlayerSet & players);
    // write to json file for cpsc 547
    static bool write_json(const char *file, const PTZPlayerSet & players);
    
};

#endif /* defined(__VideoCalibVXL__ImageSequenceCameraIO__) */
