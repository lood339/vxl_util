//
//  DetectionReader.cpp
//  PlayerTracking
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "DetectionReader.h"
#include <assert.h>
#include <vcl_iostream.h>
#include <stdlib.h>
#include <vcl_algorithm.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vgl/vgl_distance.h>


bool readDetectionFile(const char *fileName, vcl_vector<DetectionResult> & detections)
{
    assert(fileName);
    
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<fileName<<vcl_endl;
        return false;
    }
    vcl_cout<<"read from "<<fileName<<vcl_endl;
    
    for (int i = 0; i<3; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    while (1) {
        DetectionResult dr;
        int ret = fscanf(pf, "%lu %lu %lf %lf %lf %lf %lf", &dr.idNumber, &dr.frameNumber, &dr.x, &dr.y, &dr.teamPro[0], &dr.teamPro[1], &dr.teamPro[2]);
        if (ret != 7) {  //end of file
            break;
        }        
        detections.push_back(dr);
    }
    fclose(pf);
    return true;
}

bool readTrackingFile(const char *fileName, vcl_vector<DetectionResult> & detections)
{
    assert(fileName);
    
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<fileName<<vcl_endl;
        return false;
    }
    vcl_cout<<"read from "<<fileName<<vcl_endl;
    
    for (int i = 0; i<3; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    while (1) {
        DetectionResult dr;
        int ret = fscanf(pf, "%lu %lu %lf %lf %lf %lf %lf %lf %lf", &dr.idNumber, &dr.frameNumber, &dr.x, &dr.y, &dr.teamPro[0], &dr.teamPro[1], &dr.teamPro[2],
                         &dr.dx, &dr.dy);
        if (ret != 9) {  //end of file
            break;
        }
        detections.push_back(dr);
        //if (dr.frameNumber > 8000) {
        //    printf("dr.frameNumber > 6003 only for test\n");
        //    break;
        //}
    }
    fclose(pf);
    return true;
}

// merge too tracking data to one tracking
// both tracking data are sorted by frame number
void mergeTrackingData(const vcl_vector<DetectionResult> & tracking1, const vcl_vector<DetectionResult> & tracking2, vcl_vector<DetectionResult> & traking12)
{
    int i = 0;
    int j = 0;
    while (i < tracking1.size() && j < tracking2.size()) {
        while (i < tracking1.size() && j < tracking2.size() && tracking1[i].frameNumber <= tracking2[j].frameNumber) {
            traking12.push_back(tracking1[i]);
            i++;
        }
        while (i < tracking1.size() && j < tracking2.size() && tracking2[j].frameNumber < tracking1[i].frameNumber ) {
            traking12.push_back(tracking2[j]);
            j++;
        }
    }
    while (i<tracking1.size()) {
        traking12.push_back(tracking1[i]);
        i++;
    }
    while (j<tracking2.size()) {
        traking12.push_back(tracking2[j]);
        j++;
    }
    assert(tracking1.size() + tracking2.size() == traking12.size());
    
    // reassign detection identiy
    for (int k = 0; k<traking12.size(); k++) {
        traking12[k].idNumber = k + 1;
    }
}

static void getTrackingDataByFrameNumber(const vcl_vector<DetectionResult> & tracking1, const vcl_vector<DetectionResult> & tracking2,
                                         int & startIndex1, int & startIndex2, int fn,
                                         vcl_vector<DetectionResult> & fromTracking1, vcl_vector<DetectionResult> & fromTracking2)
{
    while (startIndex1 < tracking1.size() && tracking1[startIndex1].frameNumber == fn) {
        fromTracking1.push_back(tracking1[startIndex1]);
        startIndex1++;
    }
    while (startIndex2 < tracking2.size() && tracking2[startIndex2].frameNumber == fn) {
        fromTracking2.push_back(tracking2[startIndex2]);
        startIndex2++;
    }
}

void mergeTrackingDataRemoveSamePosition(const vcl_vector<DetectionResult> & tracking1, const vcl_vector<DetectionResult> & tracking2, vcl_vector<DetectionResult> & tracking12,
                                         double minX, double maxX, double minDistance)
{
    int minFn = (int)vcl_min(tracking1[0].frameNumber, tracking2[0].frameNumber);
    int maxFn = (int)vcl_max(tracking1.back().frameNumber, tracking2.back().frameNumber);
    
    int ind1 = 0;
    int ind2 = 0;
    for (int fn = minFn; fn <= maxFn; fn++) {
    //    printf("fn is %d\n", fn);
        
        vcl_vector<DetectionResult> from1;
        vcl_vector<DetectionResult> from2;
        
        // get players that at the same frame number
        getTrackingDataByFrameNumber(tracking1, tracking2, ind1, ind2, fn, from1, from2);
        
        // filter very similar positions in trajectory2
        for (int i = 0; i<from2.size(); i++) {
            double x = from2[i].x;
            double y = from2[i].y;
            bool isSimilar = false;
            
            if (x >= minX && x <= maxX) {
                vgl_point_2d<double> p(x, y);
                for (int j = 0; j<from1.size(); j++) {
                    if (from1[j].x >= minX && from1[j].x <= maxX) {
                        vgl_point_2d<double> q(from1[j].x, from1[j].y);
                        if (vgl_distance(p, q) <= minDistance) {
                            isSimilar = true;
                            break;
                        }
                    }
                }
            }
            if (!isSimilar) {
                tracking12.push_back(from2[i]);
            }
        }
        
        // all all players in trajectory 1
        tracking12.insert(tracking12.end(), from1.begin(), from1.end());
    }
    
    // reassign detection identiy
    for (int k = 0; k<tracking12.size(); k++) {
        tracking12[k].idNumber = k + 1;
    }
    int dif = (int)tracking1.size() + (int)tracking2.size() - (int)tracking12.size();
    
    printf("remove %d %f players\n", dif, 1.0*dif/(tracking1.size() + tracking2.size()));
}

// write traking data to a .txt file
bool writeTrackgingData(const char *fileName, const vcl_vector<DetectionResult> & tracking)
{
    assert(fileName);
    
    FILE *pf = fopen(fileName, "w");
    if (!pf) {
        vcl_cout<<"can not open file "<<fileName<<vcl_endl;
        return false;
    }
    vcl_cout<<"write to from "<<fileName<<vcl_endl;
    
    for (int i = 0; i<tracking.size(); i++) {
        DetectionResult dr = tracking[i];
        fprintf(pf, "%lu\t %lu\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\n", dr.idNumber, dr.frameNumber, dr.x, dr.y, dr.teamPro[0], dr.teamPro[1], dr.teamPro[2],
                         dr.dx, dr.dy);
    }
    
    fclose(pf);
    return true;
}


bool readDetectionFileVersionOne(const char *fileName, vcl_vector<FramePlayerPositions> & positions)
{
    assert(fileName);
    
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<fileName<<vcl_endl;
        return false;
    }
    vcl_cout<<"read from "<<fileName<<vcl_endl;
    vcl_cout<<"assume maximum detected player number is 10 "<<vcl_endl;
    
    while (1) {
        //read a aline
        char lineBuf[BUFSIZ] = {NULL};
        char *ret = fgets(lineBuf, sizeof(lineBuf), pf);
        // end of file
        if (!ret) {
            break;
        }
        //read frame index and position from this line
        FramePlayerPositions fpp;
        double pos[10][2];// assume the maximum detected player number is 20
        int num = sscanf(lineBuf, "%ld %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &(fpp.frameNumber),
                         &pos[0][0], &pos[0][1],
                         &pos[1][0], &pos[1][1],
                         &pos[2][0], &pos[2][1],
                         &pos[3][0], &pos[3][1],
                         &pos[4][0], &pos[4][1],
                         &pos[5][0], &pos[5][1],
                         &pos[6][0], &pos[6][1],
                         &pos[7][0], &pos[7][1],
                         &pos[8][0], &pos[8][1],
                         &pos[9][0], &pos[9][1]
                         );
        if (num >1)
        {
            for (int i = 0; i<(num-1)/2; i++) {
                fpp.positions.push_back(vgl_point_2d<double>(pos[i][0], pos[i][1]));
            }
            positions.push_back(fpp);
        }
        else
        {
            break;
        }
    }
    
    fclose(pf);
    return true;
}


bool readPTZCameraFile(const char *fileName, vcl_vector<CalibrationResult> & cameras, const int dump_lines)
{
    assert(fileName);
    
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<fileName<<vcl_endl;
        return false;
    }
    
    for (int i = 0; i<dump_lines; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    
    vgl_point_2d<double> principlePoint(1280/2, 720/2);
    vcl_cout<<"principle point is "<<principlePoint<<vcl_endl;
    
    //read cameras
    while (1) {
        int frame_num = 0;
        double fl, rx, ry, rz, cx, cy, cz;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf %lf %lf %lf", &frame_num, &fl, &rx, &ry, &rz, &cx, &cy, &cz);
        if (ret == EOF) {
            break;
        }
        
        CalibrationResult frame;
        frame.frame_num = frame_num;
        
        vpgl_calibration_matrix<double> K(fl, principlePoint);
        vnl_vector_fixed<double, 3> rod(rx, ry, rz);
        vgl_rotation_3d<double> R(rod);
        vgl_point_3d<double> cc(cx, cy, cz);
        
        frame.camera.set_calibration(K);
        frame.camera.set_rotation(R);
        frame.camera.set_camera_center(cc);
        
        cameras.push_back(frame);
    }
    
    fclose(pf);
    return true;
}

bool readPTZCameraFile(const char *fileName, vcl_vector<PTZData> & ptzs, const int dump_line)
{
    assert(fileName);
    
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<fileName<<vcl_endl;
        return false;
    }
    
    for (int i = 0; i<dump_line; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    
    while (1) {
        PTZData ptz;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf", &(ptz.frame_num), &(ptz.fl), &(ptz.pan), &(ptz.tilt), &(ptz.ssd));
        if (ret == EOF) {
            break;
        }
        ptzs.push_back(ptz);
    }
    fclose(pf);
    printf("read %lu ptz data\n", ptzs.size());
    return true;
}

bool writePTZCameraFile(const char *fileName, const vcl_vector<PTZData> & ptzs)
{
    assert(fileName);
    
    FILE *pf = fopen(fileName, "w");
    if (!pf) {
        vcl_cout<<"can not open file "<<fileName<<vcl_endl;
        return false;
    }
    
    fprintf(pf, "frame_number	 focal_length	 pan	 tilt	 ssd\n");
    
    for (int i = 0; i<ptzs.size(); i++) {
        PTZData ptz = ptzs[i];
        fprintf(pf, "%d\t %f\t %f\t %f\t %f\n", ptz.frame_num, ptz.fl, ptz.pan, ptz.tilt, ptz.ssd);
    }
    fclose(pf);
    return true;
}

void detectionToFramePlayerPosition(const vcl_vector<DetectionResult> & detections, vcl_vector<FramePlayerPositions> & frames)
{
    int idx = 0;
    while (idx < detections.size()) {
        //collect positions in the same frame number
        FramePlayerPositions fpp;
        fpp.frameNumber = detections[idx].frameNumber;
        while (idx <detections.size() && detections[idx].frameNumber == fpp.frameNumber) {
            fpp.positions.push_back(vgl_point_2d<double>(detections[idx].x, detections[idx].y));
            fpp.velocities_.push_back(vnl_vector_fixed<double, 2>(detections[idx].dx, detections[idx].dy));
            fpp.teamProbability_.push_back(vnl_vector_fixed<double, 3>(detections[idx].teamPro[0], detections[idx].teamPro[1], detections[idx].teamPro[2]));
            idx++;
        }
        frames.push_back(fpp);
    }
}

void detectionToFramePlayerPosition(const vcl_vector<DetectionResult> & detections, vcl_vector<FramePlayerPositions> & frames, double y_threshold)
{
    int idx = 0;
    while (idx < detections.size()) {
        //collect positions in the same frame number
        FramePlayerPositions fpp;
        fpp.frameNumber = detections[idx].frameNumber;
        while (idx <detections.size() && detections[idx].frameNumber == fpp.frameNumber) {
            if (detections[idx].y < y_threshold) {
                fpp.positions.push_back(vgl_point_2d<double>(detections[idx].x, detections[idx].y));
                fpp.velocities_.push_back(vnl_vector_fixed<double, 2>(detections[idx].dx, detections[idx].dy));
                fpp.teamProbability_.push_back(vnl_vector_fixed<double, 3>(detections[idx].teamPro[0], detections[idx].teamPro[1], detections[idx].teamPro[2]));
            }
            idx++;
        }
        frames.push_back(fpp);
    }

    
}

void combinePTZdataAndSensorCameraData(const vcl_vector<DetectionResult> & sensorData, const vcl_vector<PTZData> &ptzs, vcl_vector<PlayerPTZData> & play_cameras)
{
    vcl_vector<FramePlayerPositions> frames;
    detectionToFramePlayerPosition(sensorData, frames);
    
    //sync
    FrameHashTable fhTable(frames);
    for (int i = 0; i<ptzs.size(); i++) {
        unsigned int fn_60 = ptzs[i].frame_num;
        unsigned long fn_25 = frameNumberFromPTZTTOStationaryCamera(fn_60);
        vcl_vector<vgl_point_2d<double> > pts = fhTable.find(fn_25);
        if (pts.size() > 0) {
            PlayerPTZData ptd;
            ptd.frame_num = fn_60;
            ptd.fl  = ptzs[i].fl;
            ptd.pan = ptzs[i].pan;
            ptd.tilt = ptzs[i].tilt;
            ptd.pts = pts;
            play_cameras.push_back(ptd);
        }
    }
    vcl_cout<<"collect "<<play_cameras.size()<<" player-camera data.\n";
}


void grabPositionsInFrames(const vcl_vector<DetectionResult> & detectionRecords, int startFrameNum, int endFrameNum,
                           vcl_vector<vcl_vector<vgl_point_2d<double> > > & positions)
{
    assert(detectionRecords.size() > 0);
    assert(startFrameNum < endFrameNum);
    
    positions.resize(endFrameNum - startFrameNum);
    for (int i = 0; i<detectionRecords.size(); i++) {
        unsigned long frameNum = detectionRecords[i].frameNumber;
        if (frameNum >= startFrameNum && frameNum < endFrameNum) {
            int idx = (int)frameNum - startFrameNum;
            double x = detectionRecords[i].x;
            double y = detectionRecords[i].y;
            vgl_point_2d<double> pos(x, y);
            positions[idx].push_back(pos);
        }
        if (frameNum > endFrameNum) {   // assume frameNumber non-decrease
            break;
        }
    }
}

void grabPositionsInAllFrames(const vcl_vector<DetectionResult> & detectionRecords,
                              vcl_vector<vcl_vector<vgl_point_2d<double> > > & positions)
{
    assert(detectionRecords.size() > 0);
    
    long int startFrame = detectionRecords[0].frameNumber;
    long int endFrame = detectionRecords.back().frameNumber;
    long int frameNum = vcl_max(startFrame, endFrame) + 1;
    positions.resize(frameNum);
    for (int i = 0; i<detectionRecords.size(); i++) {
        unsigned long fn = detectionRecords[i].frameNumber;
        double x = detectionRecords[i].x;
        double y = detectionRecords[i].y;
        assert(fn < frameNum);
        positions[fn].push_back(vgl_point_2d<double>(x, y));
    } 
}

void getCameraPlayerData(const vcl_vector<DetectionResult> & players, const vcl_vector<CalibrationResult> & ptzCameras,
                         const int detection_sync_start_frame, const int ptz_sync_start_frame, const int ptz_record_start_frame,
                         vcl_vector<CameraPlayerData> &player_cameras,
                         const double detection_frame_rate, const double ptz_frame_rate)
{
    // filter and sort players
    vcl_vector<DetectionResult> sortedPlayers;
    for (int i = 0; i<players.size(); i++) {
        DetectionResult dr = players[i];
        if (dr.frameNumber >= detection_sync_start_frame) {
            dr.frameNumber -= detection_sync_start_frame;
            sortedPlayers.push_back(dr);
        }
    }
    vcl_sort(sortedPlayers.begin(), sortedPlayers.end());
    
    // filter and sort cameras
    vcl_vector<CalibrationResult> sortedCameras;
    for (int i = 0; i<ptzCameras.size(); i++) {
        CalibrationResult camera = ptzCameras[i];
        if (camera.frame_num >= ptz_sync_start_frame) {
            camera.frame_num -= ptz_sync_start_frame;
            sortedCameras.push_back(camera);
        }
    }
    vcl_sort(sortedCameras.begin(), sortedCameras.end());
    
    // player buffer
    vcl_vector< vcl_vector < vgl_point_2d<double> > > playerBuffer;
    playerBuffer.resize(sortedPlayers.back().frameNumber + 1);
    for (int i = 0; i<sortedPlayers.size(); i++) {
        unsigned long frameIdx = sortedPlayers[i].frameNumber;
        double x = sortedPlayers[i].x;
        double y = sortedPlayers[i].y;
        assert(frameIdx < playerBuffer.size());
        playerBuffer[frameIdx].push_back(vgl_point_2d<double>(x, y));
    }
    
    player_cameras.clear();
    // suppose camera frame is unique
    for (int i = 0; i<sortedCameras.size(); i++) {
        // original frame  number
        int frameNum = sortedCameras[i].frame_num + ptz_sync_start_frame;
        if (frameNum < ptz_record_start_frame) {
            continue;
        }
        //change frame number in ptz camera to stationary camera
        double time_interval = (double)(sortedCameras[i].frame_num)/ptz_frame_rate;
        int idx = time_interval * detection_frame_rate;
        
        CameraPlayerData cpd;
        cpd.frame_num = frameNum;
        cpd.camera = sortedCameras[i].camera;
        cpd.player_positions = playerBuffer[idx];
        
        player_cameras.push_back(cpd);
    }
}


void saveCameraPlayerData(const char *file, const char *videoName, double frame_rate, const vcl_vector<CameraPlayerData> & camera_players)
{
    assert(file);
    assert(videoName);
    assert(frame_rate > 0);
    
    FILE *pf = fopen(file, "w");
    assert(pf);
    fprintf(pf, "video name: %s, frame_rate: %lf\n", videoName, frame_rate);
    fprintf(pf, "frame idx\t focal length\t Rx\t Ry\t Rz\t Cx\t Cy\t Cz\n");
    fprintf(pf, "player position number p1 p2 ...\n");
    for (int i = 0; i<camera_players.size(); i++) {
        CameraPlayerData data = camera_players[i];
        vpgl_perspective_camera<double> camera = data.camera;
        unsigned int idx = data.frame_num;
        double fl = camera.get_calibration().get_matrix()[0][0];
        double Rx = camera.get_rotation().as_rodrigues()[0];
        double Ry = camera.get_rotation().as_rodrigues()[1];
        double Rz = camera.get_rotation().as_rodrigues()[2];
        double Cx = camera.get_camera_center().x();
        double Cy = camera.get_camera_center().y();
        double Cz = camera.get_camera_center().z();
        fprintf(pf, "%u\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", idx, fl, Rx, Ry, Rz, Cx, Cy, Cz);
        
        fprintf(pf, "%d\t", (int)data.player_positions.size());
        for (int j = 0; j<data.player_positions.size(); j++) {
            vgl_point_2d<double> p = data.player_positions[j];
            fprintf(pf, "%f %f ", p.x(), p.y());
        }
        if (i != camera_players.size() -1) {
            fprintf(pf, "\n");
        }
    }
    fclose(pf);
    vcl_cout<<"save to : "<<file<<vcl_endl;
}

void loadCameraPlayerData(const char *file, vcl_vector<CameraPlayerData> & camera_players)
{
    assert(file);
    
    FILE *pf = fopen(file, "r");
    assert(pf);
    for (int i = 0; i<3; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    vgl_point_2d<double> principlePoint(1280/2, 720/2);
    vcl_cout<<"principle point is "<<principlePoint<<vcl_endl;
    
    //read cameras
    while (1) {
        int frame_num = 0;
        double fl, rx, ry, rz, cx, cy, cz;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf %lf %lf %lf", &frame_num, &fl, &rx, &ry, &rz, &cx, &cy, &cz);
        if (ret == EOF) {
            break;
        }
        
        CameraPlayerData data;
        
        vpgl_calibration_matrix<double> K(fl, principlePoint);
        vnl_vector_fixed<double, 3> rod(rx, ry, rz);
        vgl_rotation_3d<double> R(rod);
        vgl_point_3d<double> cc(cx, cy, cz);
        
        data.frame_num = frame_num;
        data.camera.set_calibration(K);
        data.camera.set_rotation(R);
        data.camera.set_camera_center(cc);
        
        int player_num = 0;
        fscanf(pf, "%d", &player_num);
        for (int i = 0; i<player_num; i++) {
            double x = 0;
            double y = 0;
            fscanf(pf, "%lf %lf ", &x, &y);
            data.player_positions.push_back(vgl_point_2d<double>(x, y));
        }
        camera_players.push_back(data);
    }
    fclose(pf);
    vcl_cout<<"load from: "<<file<<vcl_endl;
}


// homography matrix
void writeLocalHomographyMatrix(const char *file, const LocalHomography & Hs)
{
    assert(file);
    FILE *pf = fopen(file, "w");
    assert(pf);
    fprintf(pf, "%s\t", Hs.videoName_.c_str());
    fprintf(pf, "%f\t %d\t %d\t %d %d\n", Hs.frameRate_, Hs.frequency_, Hs.startFrame_, Hs.imageW_, Hs.imageH_);
    int startFrame = Hs.startFrame_;
    for (int i = 0; i<Hs.Hs_.size(); i++) {
        int frameN = startFrame + i;
        vnl_matrix_fixed<double, 3, 3> H = Hs.Hs_[i].get_matrix();
        fprintf(pf, "%d\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\n", frameN,
                H(0, 0), H(0, 1), H(0, 2),
                H(1, 0), H(1, 1), H(1, 2),
                H(2, 0), H(2, 1), H(2, 2));
    }
    
    fclose(pf);
}

void readLocalHomographyMatrix(const char *file, LocalHomography & Hs)
{
    assert(file);
    
    FILE *pf = fopen(file, "r");
    assert(pf);
    
    char buf[1024] = {NULL};
    fscanf(pf, "%s", buf);
    Hs.videoName_ = vcl_string(buf);
    fscanf(pf, "%lf %d %d %d %d", &(Hs.frameRate_), &(Hs.frequency_), &(Hs.startFrame_), &(Hs.imageW_), &(Hs.imageH_));
    
    Hs.Hs_.clear();
    while (1) {
        int frameNum = 0;
        double data[3][3] = {0};
        int ret = fscanf(pf, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf", &frameNum,
                         &data[0][0], &data[0][1], &data[0][2],
                         &data[1][0], &data[1][1], &data[1][2],
                         &data[2][0], &data[2][1], &data[2][2]);
        if (ret != 10) {
            break;
        }
        vgl_h_matrix_2d<double> H(&data[0][0]);
        Hs.Hs_.push_back(H);
    }
    fclose(pf);
}












