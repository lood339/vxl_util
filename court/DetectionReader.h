//
//  DetectionReader.h
//  PlayerTracking
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __PlayerTracking__DetectionReader__
#define __PlayerTracking__DetectionReader__

#include <iostream>
#include <stdio.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vgl/algo/vgl_h_matrix_2d.h>
#include "cameraCalibPlayerDetectionData.h"



// read result from TVProductionGameObserver
bool readDetectionFile(const char *fileName, vcl_vector<DetectionResult> & detections);

// tracking data with velocity
bool readTrackingFile(const char *fileName, vcl_vector<DetectionResult> & detections);

// merge two tracking data to one tracking
// both tracking data are sorted by frame number
void mergeTrackingData(const vcl_vector<DetectionResult> & tracking1, const vcl_vector<DetectionResult> & tracking2, vcl_vector<DetectionResult> & traking12);

// merge tracking data and remove same positions in range[minX maxX], with mininum distance
void mergeTrackingDataRemoveSamePosition(const vcl_vector<DetectionResult> & tracking1, const vcl_vector<DetectionResult> & tracking2, vcl_vector<DetectionResult> & traking12,
                                         double minX = 50.0, double maxX = 58.0, double minDistance = 0.3);
// write traking data to a .txt file
bool writeTrackgingData(const char *fileName, const vcl_vector<DetectionResult> & tracking);

// detection result version 1
bool readDetectionFileVersionOne(const char *fileName, vcl_vector<FramePlayerPositions> & frames);

// read result from VideoCalibration
// default number must be 3 since it was used in the old code
bool readPTZCameraFile(const char *fileName, vcl_vector<CalibrationResult> & cameras, const int dump_line = 3);

// read fl, pan, tilt from smoothed camera data
bool readPTZCameraFile(const char *fileName, vcl_vector<PTZData> & ptzs, const int dump_line = 1);

// write ptz to file
bool writePTZCameraFile(const char *fileName, const vcl_vector<PTZData> & ptzs);

// change formate from DetectionResult to FramePlayerPositions
void detectionToFramePlayerPosition(const vcl_vector<DetectionResult> & detections, vcl_vector<FramePlayerPositions> & frames);

// y_threshold: 14.3, filter players' too close to board line
void detectionToFramePlayerPosition(const vcl_vector<DetectionResult> & detections, vcl_vector<FramePlayerPositions> & frames, double y_threshold);

inline int frameNumberFromPTZTTOStationaryCamera(int frame60)
{
    //only work for disney-research-2 - MAN - 2013-07-17-08-04-26 and isney-research-2 - SMV - 2013-07-17-08-13-42.mov
    if (frame60 < 33551 || frame60 > 282120) {
        return -1;
    }
    return 1.0 * (frame60 - 33551)/(282120 - 33551)*(103987 - 415) + 415;
    // 1.0 * (f60 - 33551) * 0.41667303646 + 415
    // (fn25 - 415)/(103987 - 415) * (282120 - 33551) + 33551
}

inline int frameNumberFromPTZT2StationaryCamera_14_34_33(int frame60)
{
    //only work for disney-research-2-MAN-2013-07-17-14-34-33.mov
    const int p1 = 3468;
    const int p2 = 238796;
    const int h1 = 1555;
    const int h2 = 99609;
    if (frame60 < p1 || frame60 > p2) {
        return -1;
    }
    return 1.0 * (frame60 - p1)/(p2 - p1)*(h2 - h1) + h1;
}

// ptz start frame, ptz end frame, sensor camera start frame, sensor camera end frame
inline int frameNumberFromPTZTTOStationaryCamera(const int p1, const int p2, const int h1, const int h2, int frame60)
{
    assert(p1 < p2);
    assert(h1 < h2);
    if (frame60 < p1 || frame60 > p2) {
        return -1;
    }
    return 1.0 * (frame60 - p1)/(p2 - p1)*(h2 - h1) + h1;
}


// sync ptz camera and sensor camera (stationary camera)
void combinePTZdataAndSensorCameraData(const vcl_vector<DetectionResult> & sensorData, const vcl_vector<PTZData> &ptzs, vcl_vector<PlayerPTZData> & play_cameras);


// get player positions (world coordinate in meters) in frame range [startFrameNum, endFrameNum)
void grabPositionsInFrames(const vcl_vector<DetectionResult> & detectionRecords, int startFrameNum, int endFrameNum,
                           vcl_vector<vcl_vector<vgl_point_2d<double> > > & positions);

// positions: index is frame number in the detectionRecords
void grabPositionsInAllFrames(const vcl_vector<DetectionResult> & detectionRecords,
                              vcl_vector<vcl_vector<vgl_point_2d<double> > > & positions);


// combine player position data and ptz camera data
// detection_start_frame and ptz_start_frame used to sync the image sequence
void getCameraPlayerData(const vcl_vector<DetectionResult> & players, const vcl_vector<CalibrationResult> & ptzCameras,
                         const int detection_sync_start_frame, const int ptz_sync_start_frame, const int ptz_record_start_frame,
                         vcl_vector<CameraPlayerData> & player_cameras,
                         const double detection_frame_rate = 25.0, const double ptz_frame_rate = 60.0); // 58.513514



// save synced camera player data
void saveCameraPlayerData(const char *file, const char *videoName, double frame_rate, const vcl_vector<CameraPlayerData> & camera_players);
void loadCameraPlayerData(const char *file, vcl_vector<CameraPlayerData> & camera_players);

struct LocalHomography
{
    vcl_string videoName_;
    double frameRate_;
    int    frequency_;   // sampling key frame frequency
    
    int imageW_;
    int imageH_;
    
    int startFrame_;
    
    vcl_vector<vgl_h_matrix_2d<double> > Hs_;
};


void writeLocalHomographyMatrix(const char *file, const LocalHomography & Hs);
void readLocalHomographyMatrix(const char *file, LocalHomography & Hs);



#endif /* defined(__PlayerTracking__DetectionReader__) */
