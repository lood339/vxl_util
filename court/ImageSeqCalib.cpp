//
//  ImageSeqCalib.cpp
//  FinalCalib
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "ImageSeqCalib.h"
#include "vxl_ptz_camera.h"
#include "basketballCourt.h"
#include "vxl_plus.h"


bool WWoSBasketballReadWriteCamera::read_cameras_from_file(const char *file_name, ImageSequenceCameras & cameraSequence)
{
    assert(file_name);
    
    FILE *pf = fopen(file_name, "r");
    if (!pf) {
        printf("can not open %s\n", file_name);
        return false;
    }
    char buf[1024] = {NULL};
    int ret = fscanf(pf, "%s", buf);
    if (ret != 1) {
        return false;
    }
    cameraSequence.video_name_ = vcl_string(buf);
    
    ret = fscanf(pf, "%lf %d %d", &(cameraSequence.frame_rate_), &(cameraSequence.width), &(cameraSequence.height));
    if (ret != 3) {
        return false;
    }
    vgl_point_2d<double> pp(cameraSequence.width/2, cameraSequence.height/2);
    while (1) {
        int fn = 0;
        double fl, rx, ry, rz, cx, cy, cz;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf %lf %lf %lf", &fn, &fl, &rx, &ry, &rz, &cx, &cy, &cz);
        if (ret != 8) {
            break;
        }
        
        assert(fn >= 0);
        
        FrameData data;
        
        vpgl_calibration_matrix<double> K(fl, pp);
        vnl_vector_fixed<double, 3> rod(rx, ry, rz);
        vgl_rotation_3d<double> R(rod);
        vgl_point_3d<double> cc(cx, cy, cz);
        
        data.fn_ = fn;
        data.camera_.set_calibration(K);
        data.camera_.set_rotation(R);
        data.camera_.set_camera_center(cc);
        
        cameraSequence.frames_.push_back(data);
    }
    
    fclose(pf);
    
    //check frame number
    for (int i = 0; i<cameraSequence.frames_.size() - 1; i++) {
        int fn = cameraSequence.frames_[i].fn_;
        int fn_next = cameraSequence.frames_[i+1].fn_;
        if (fn >= fn_next) {
            printf("frame number is not mono-increase %d %d\n", fn, fn_next);
        }
    }
    
    return true;
}

bool WWoSBasketballReadWriteCamera::read_ptz_from_file(const char *file_name, ImageSequenceCameras & sequence)
{
    FILE *pf = fopen(file_name, "r");
    if (!pf) {
        printf("can not open %s\n", file_name);
        return false;
    }
    char buf[1024] = {NULL};
    int ret = fscanf(pf, "%s", buf);
    if (ret != 1) {
        return false;
    }
    sequence.video_name_ = vcl_string(buf);
    
    ret = fscanf(pf, "%lf %d %d", &(sequence.frame_rate_), &(sequence.width), &(sequence.height));
    if (ret != 3) {
        return false;
    }
    
    sequence.frames_.clear();
    while (1) {
        int fn = -1;
        double fl = 0, pan = 0, tilt = 0, ssd = 0;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf", &fn, &fl, &pan, &tilt, &ssd);
        if (ret != 5) {
            break;
        }
        
        vpgl_perspective_camera<double> camera;
        bool isCamera =  VxlPTZCamera::PTZToCamera(fl, pan, tilt, camera);
        if (isCamera) {
            FrameData data;
            data.fn_ = fn;
            data.camera_ = camera;
            
            sequence.frames_.push_back(data);
        }
        else {
            printf("%d can't change ptz to camera\n", fn);
        }
    }
    fclose(pf);
    
    //check frame number
    for (int i = 0; i<sequence.frames_.size() - 1; i++) {
        int fn = sequence.frames_[i].fn_;
        int fn_next = sequence.frames_[i+1].fn_;
        if (fn >= fn_next) {
            printf("frame number is not mono-increase %d %d\n", fn, fn_next);
        }
    }
    return true;
}

bool WWoSBasketballReadWriteCamera::read_ptz(const char *file_name, ImageSequencePTZs & imageSeqPTZ)
{
    FILE *pf = fopen(file_name, "r");
    if (!pf) {
        printf("can not open %s\n", file_name);
        return false;
    }
    char buf[1024] = {NULL};
    int ret = fscanf(pf, "%s", buf);
    if (ret != 1) {
        return false;
    }
    imageSeqPTZ.video_name_ = vcl_string(buf);
    
    ret = fscanf(pf, "%lf %d %d", &(imageSeqPTZ.frame_rate_), &(imageSeqPTZ.width), &(imageSeqPTZ.height));
    if (ret != 3) {
        return false;
    }
    
    imageSeqPTZ.frames_.clear();
    while (1) {
        int fn = -1;
        double fl = 0, pan = 0, tilt = 0, ssd = 0;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf", &fn, &fl, &pan, &tilt, &ssd);
        if (ret != 5) {
            break;
        }
        FramePTZData data;
        data.fn_ = fn;
        data.pan_ = pan;
        data.tilt_ = tilt;
        data.fl_ = fl;
        imageSeqPTZ.frames_.push_back(data);
    }
    fclose(pf);
    
    //check frame number
    for (int i = 0; i<imageSeqPTZ.frames_.size() - 1; i++) {
        int fn = imageSeqPTZ.frames_[i].fn_;
        int fn_next = imageSeqPTZ.frames_[i+1].fn_;
        if (fn >= fn_next) {
            printf("frame number is not mono-increase %d %d\n", fn, fn_next);
        }
    }
    return true;
}

bool WWoSBasketballReadWriteCamera::write_cameras(const char *file, const ImageSequenceCameras & cameras)
{
    assert(file);
    FILE *pf = fopen(file, "w");
    if (!pf) {
        printf("can not open %s\n", file);
        return false;
    }
    fprintf(pf, "%s\t", cameras.video_name_.c_str());
    fprintf(pf, "%f\t", cameras.frame_rate_);
    fprintf(pf, "%d %d\n", cameras.width, cameras.height);
    for (int i = 0; i<cameras.frames_.size(); i++) {
        vpgl_perspective_camera<double> camera = cameras.frames_[i].camera_;
        unsigned int idx = cameras.frames_[i].fn_;
        double fl = camera.get_calibration().get_matrix()[0][0];
        double Rx = camera.get_rotation().as_rodrigues()[0];
        double Ry = camera.get_rotation().as_rodrigues()[1];
        double Rz = camera.get_rotation().as_rodrigues()[2];
        double Cx = camera.get_camera_center().x();
        double Cy = camera.get_camera_center().y();
        double Cz = camera.get_camera_center().z();
        fprintf(pf, "%u\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", idx, fl, Rx, Ry, Rz, Cx, Cy, Cz);
    }
    fclose(pf);
    return true;
}

bool WWoSBasketballReadWriteCamera::write_ptzs(const char *file, const ImageSequenceCameras & cameras)
{
    assert(file);
    FILE *pf = fopen(file, "w");
    if (!pf) {
        printf("can not open %s\n", file);
        return false;
    }
    fprintf(pf, "%s\t", cameras.video_name_.c_str());
    fprintf(pf, "%f\t", cameras.frame_rate_);
    fprintf(pf, "%d %d\n", cameras.width, cameras.height);
    for (int i = 0; i<cameras.frames_.size(); i++) {
        unsigned int fn = cameras.frames_[i].fn_;
        double fl = 0;
        double pan = 0;
        double tilt = 0;
        double ssd = 10000;
        bool isPTZ = VxlPTZCamera::CameraToPTZ(cameras.frames_[i].camera_, pan, tilt, fl);
        if (!isPTZ) {
            printf("Warning: can not convert to ptz. Write  0 0 0 as focal length, pan, tilt\n");
          //  continue;
        }
        fprintf(pf, "%d\t %f\t %f\t %f\t %f\n", fn, fl, pan, tilt, ssd);
    }
    fclose(pf);
    return true;
}








