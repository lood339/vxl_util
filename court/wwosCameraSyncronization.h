//
//  wwosCameraSyncronization.h
//  OnlineStereo
//
//  Created by jimmy on 3/13/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__wwosCameraSyncronization__
#define __OnlineStereo__wwosCameraSyncronization__

// synchronization between stationary camera and PTZ camera
#include "cameraCalibPlayerDetectionData.h"
#include <vnl/vnl_vector_fixed.h>


// for PTZ camera
class WWoSCameraSyncronization
{
    int ptz1_;
    int ptz2_;
    int stationary1_;
    int stationary2_;
public:
    vcl_vector<DetectionResult>    players_;
    vcl_vector<PTZData>            ptzs_;
    
public:
    // [first last] frame numbers in PTZ camera and stationary camera
    WWoSCameraSyncronization();
    WWoSCameraSyncronization(int p1, int p2, int s1, int s2);
    ~WWoSCameraSyncronization();
    
    // // tracking data with velocity
    bool readPlayerPositionFromTrackingFile(const char *file, int max_fn_num = INT_MAX);
    bool readPTZCameras(const char *ptzFilename, int dump_lines_num = 1);
    bool syncPTZPlayerPosition(vcl_vector<PlayerPTZData> & player_cameras);    // sync PTZ
    bool syncPTZPlayerPosition(vcl_vector<CourtObeservation> & player_ptzs);
    bool syncPTZPlayerPositionWithoutPTZ(vcl_vector<PlayerPTZData> & ptz_player_positions); // do not have pan, tilt, zoom parameters
    
    // right side looking stationary camera
    static vcl_vector<int> MarkinCameraSyncFrames()
    {
        vcl_vector<int> frameNums;
        frameNums.push_back(5410);
        frameNums.push_back(411842);
        frameNums.push_back(2254);
        frameNums.push_back(171597);
        assert(frameNums.size() == 4);
        
        return frameNums;
    }
    
public:
    int PTZframe2StatioanryFrame(int ptzFrame) const;
    int stationayFrame2PTZFrame(const int stationaryFrame)const;
    
public:
    // merge player position in each frame
    static void detectionToFramePlayerPosition(const vcl_vector<DetectionResult> & detections, vcl_vector<FramePlayerPositions> & frames);
    
public:
    // discarded
    bool readCameras(const char *cameraFileName, int dump_lines_num = 1);
    
    vcl_vector<CalibrationResult> cameras_;
    vcl_vector<CameraPlayerData>   cameraAndPlayerPosition_;
};

// for general camera
class CameraSyncronization
{
    int ptz1_;
    int ptz2_;
    int stationary1_;
    int stationary2_;
public:
    vcl_vector<DetectionResult>    players_;
    vcl_vector<CalibrationResult> cameras_;
    
public:
    // [first last] frame numbers in PTZ camera and stationary camera
    CameraSyncronization(int p1, int p2, int s1, int s2);
    ~CameraSyncronization();
    
    // // tracking data with velocity
    bool readPlayerPositionFromTrackingFile(const char *file, int max_fn_num = INT_MAX);
    bool readCameras(const char *cameraFileName, int dummy_lines_num = 1);
    bool syncCameraPlayerPosition(vcl_vector<CameraPlayerData> & player_cameras);
    
private:
    int PTZframe2StatioanryFrame(int ptzFrame);
    
};
   


#endif /* defined(__OnlineStereo__wwosCameraSyncronization__) */
