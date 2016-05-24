//
//  wwosCameraSyncronization.cpp
//  OnlineStereo
//
//  Created by jimmy on 3/13/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "wwosCameraSyncronization.h"

WWoSCameraSyncronization::WWoSCameraSyncronization()
{
    vcl_vector<int> syncFrames = WWoSCameraSyncronization::MarkinCameraSyncFrames();
    ptz1_ = syncFrames[0];
    ptz2_ = syncFrames[1];
    stationary1_ = syncFrames[2];
    stationary2_ = syncFrames[3];
}
WWoSCameraSyncronization::WWoSCameraSyncronization(int p1, int p2, int s1, int s2)
{
    ptz1_ = p1;
    ptz2_ = p2;
    stationary1_ = s1;
    stationary2_ = s2;
}
WWoSCameraSyncronization::~WWoSCameraSyncronization()
{
    
}

bool WWoSCameraSyncronization::readCameras(const char *fileName, int dump_lines_num)
{
    assert(fileName);
    
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<fileName<<vcl_endl;
        return false;
    }
    
    for (int i = 0; i<dump_lines_num; i++) {
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
        if (ret != 8) {
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
        
        cameras_.push_back(frame);
    }
    fclose(pf);
    printf("read %lu camera data.\n", cameras_.size());
    return true;
}

bool WWoSCameraSyncronization::syncPTZPlayerPosition(vcl_vector<PlayerPTZData> & player_cameras)
{
    vcl_vector<FramePlayerPositions> frames;
    WWoSCameraSyncronization::detectionToFramePlayerPosition(players_, frames);
    
    //sync
    FrameHashTable fhTable(frames);
    for (int i = 0; i<ptzs_.size(); i++) {
        unsigned int fn_60 = ptzs_[i].frame_num;
        unsigned long fn_25 = this->PTZframe2StatioanryFrame(fn_60);
        vcl_vector<vgl_point_2d<double> > pts = fhTable.find(fn_25);
        if (pts.size() > 0) {
            PlayerPTZData ptd;
            ptd.frame_num = fn_60;
            ptd.fl  = ptzs_[i].fl;
            ptd.pan = ptzs_[i].pan;
            ptd.tilt = ptzs_[i].tilt;
            ptd.pts = pts;
            player_cameras.push_back(ptd);
        }
    }
    vcl_cout<<"collect "<<player_cameras.size()<<" player camera data.\n";
    return true;    
}

bool WWoSCameraSyncronization::syncPTZPlayerPosition(vcl_vector<CourtObeservation> & player_ptzs)
{
    vcl_vector<FramePlayerPositions> frames;
    WWoSCameraSyncronization::detectionToFramePlayerPosition(players_, frames);
    
    //sync
    FrameHashTable fhTable(frames);
    for (int i = 0; i<ptzs_.size(); i++) {
        unsigned int fn_60 = ptzs_[i].frame_num;
        unsigned long fn_25 = this->PTZframe2StatioanryFrame(fn_60);
        vcl_vector<vgl_point_2d<double> > pts = fhTable.find(fn_25);
        if (pts.size() > 0) {
            CourtObeservation cob;
            cob.frame_num = fn_60;
            cob.fl  = ptzs_[i].fl;
            cob.pan = ptzs_[i].pan;
            cob.tilt = ptzs_[i].tilt;
            cob.pts = pts;
            player_ptzs.push_back(cob);
        }
    }
    vcl_cout<<"collect "<<player_ptzs.size()<<" player camera data.\n";
    return true;
}

bool WWoSCameraSyncronization::syncPTZPlayerPositionWithoutPTZ(vcl_vector<PlayerPTZData> & ptz_player_positions)
{
    vcl_vector<FramePlayerPositions> frames;
    WWoSCameraSyncronization::detectionToFramePlayerPosition(players_, frames);
    
    //sync
    FrameHashTable fhTable(frames);
    for (int i = ptz1_; i < ptz2_; i++) {
        unsigned int fn_60 = i;
        unsigned long fn_25 = this->PTZframe2StatioanryFrame(fn_60);
        vcl_vector<vgl_point_2d<double> > pts = fhTable.find(fn_25);
        if (pts.size() > 0) {
            PlayerPTZData ppd;
            ppd.frame_num = fn_60;
            ppd.pts = pts;
            ptz_player_positions.push_back(ppd);
        }
    }
    return true;
}

bool WWoSCameraSyncronization::readPTZCameras(const char *ptzFilename, int dump_lines_num)
{
    assert(ptzFilename);
    
    FILE *pf = fopen(ptzFilename, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<ptzFilename<<vcl_endl;
        return false;
    }
    
    for (int i = 0; i<dump_lines_num; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    
    ptzs_.clear();
    while (1) {
        PTZData ptz;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf", &ptz.frame_num, &ptz.fl, &ptz.pan, &ptz.tilt, &ptz.ssd);
        if (ret != 5) {
            break;
        }
        ptzs_.push_back(ptz);
    }
    fclose(pf);
    printf("read %lu ptz data.\n", ptzs_.size());
    return true;
}

bool WWoSCameraSyncronization::WWoSCameraSyncronization::readPlayerPositionFromTrackingFile(const char *fileName, int max_fn_num)
{
    assert(fileName);
    players_.clear();
    
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
        players_.push_back(dr);
        if (dr.frameNumber == max_fn_num) {
            break;
        }
    }
    fclose(pf);
    return true;    
}


int WWoSCameraSyncronization::PTZframe2StatioanryFrame(int ptzFrame) const
{
    assert(ptz1_ < ptz2_);
    assert(stationary1_ < stationary2_);
    if (ptzFrame < ptz1_ || ptzFrame > ptz2_) {
        return -1;
    }
    double fn25 = 1.0 * (ptzFrame - ptz1_)/(ptz2_ - ptz1_)*(stationary2_ - stationary1_) + stationary1_;
    return int(fn25);
}

int WWoSCameraSyncronization::stationayFrame2PTZFrame(const int stationaryFrame)const
{
    assert(ptz1_ < ptz2_);
    assert(stationary1_ < stationary2_);
    
    if (stationaryFrame < stationary1_ || stationaryFrame > stationary2_) {
        return -1;
    }
    
    double fn60 = 1.0 * (stationaryFrame - stationary1_)/(stationary2_ - stationary1_)*(ptz2_ - ptz1_) + ptz1_;
    return int(fn60);
}


void WWoSCameraSyncronization::detectionToFramePlayerPosition(const vcl_vector<DetectionResult> & detections, vcl_vector<FramePlayerPositions> & frames)
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


CameraSyncronization::CameraSyncronization(int p1, int p2, int s1, int s2)
{
    ptz1_ = p1;
    ptz2_ = p2;
    stationary1_ = s1;
    stationary2_ = s2;
    
}
CameraSyncronization::~CameraSyncronization()
{
    
}

// // tracking data with velocity
bool CameraSyncronization::readPlayerPositionFromTrackingFile(const char *fileName, int max_fn_num)
{
    assert(fileName);
    players_.clear();
    
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
        players_.push_back(dr);
        if (dr.frameNumber == max_fn_num) {
            break;
        }
    }
    fclose(pf);
    return true;
}
bool CameraSyncronization::readCameras(const char *cameraFileName, int dummy_lines_num)
{
    assert(cameraFileName);
    
    FILE *pf = fopen(cameraFileName, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<cameraFileName<<vcl_endl;
        return false;
    }
    
    for (int i = 0; i<dummy_lines_num; i++) {
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
        if (ret != 8) {
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
        
        cameras_.push_back(frame);
    }
    fclose(pf);
    printf("read %lu camera data.\n", cameras_.size());
    return true;
}

bool CameraSyncronization::syncCameraPlayerPosition(vcl_vector<CameraPlayerData> & player_cameras)
{
    vcl_vector<FramePlayerPositions> frames;
    WWoSCameraSyncronization::detectionToFramePlayerPosition(players_, frames);
    
    //sync
    FrameHashTable fhTable(frames);
    for (int i = 0; i<cameras_.size(); i++) {
        unsigned int fn_60 = cameras_[i].frame_num;
        unsigned long fn_25 = this->PTZframe2StatioanryFrame(fn_60);
        vcl_vector<vgl_point_2d<double> > pts = fhTable.find(fn_25);
        if (pts.size() > 0) {
            CameraPlayerData cpd;
            cpd.frame_num = fn_60;
            cpd.camera = cameras_[i].camera;
            cpd.player_positions = pts;
            player_cameras.push_back(cpd);
        }
    }
    vcl_cout<<"collect "<<player_cameras.size()<<" player camera data.\n";
    return true;
}

int CameraSyncronization::PTZframe2StatioanryFrame(int ptzFrame)
{
    assert(ptz1_ < ptz2_);
    assert(stationary1_ < stationary2_);
    if (ptzFrame < ptz1_ || ptzFrame > ptz2_) {
        return -1;
    }
    return 1.0 * (ptzFrame - ptz1_)/(ptz2_ - ptz1_)*(stationary2_ - stationary1_) + stationary1_;
    
}















