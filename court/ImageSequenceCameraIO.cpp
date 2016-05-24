//
//  ImageSequenceCameraIO.cpp
//  VideoCalibVXL
//
//  Created by jimmy on 7/18/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "ImageSequenceCameraIO.h"
#include "vnl_plus.h"

bool ImageSequenceCameraIO::read_cameras(const char *file_name, ImageSequenceCameras & cameraSequence)
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
bool ImageSequenceCameraIO::write_cameras(const char *file, const ImageSequenceCameras & cameras)
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

bool ImageSequenceCameraIO::read_ptzFrames(const char *file, ImageSequencePTZs & ptzs)
{
    FILE *pf = fopen(file, "r");
    if (!pf) {
        printf("can not open %s\n", file);
        return false;
    }
    char buf[1024] = {NULL};
    int ret = fscanf(pf, "%s", buf);
    if (ret != 1) {
        return false;
    }
    ptzs.video_name_ = vcl_string(buf);
    
    ret = fscanf(pf, "%lf %d %d", &(ptzs.frame_rate_), &(ptzs.width), &(ptzs.height));
    if (ret != 3) {
        return false;
    }
    
    ptzs.frames_.clear();
    while (1) {
        int fn = -1;
        double fl = 0, pan = 0, tilt = 0, ssd = 0;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf", &fn, &fl, &pan, &tilt, &ssd);
        if (ret != 5) {
            break;
        }
        FramePTZData ptz;
        ptz.fn_ = fn;
        ptz.pan_ = pan;
        ptz.tilt_ = tilt;
        ptz.fl_ = fl;
        ptzs.frames_.push_back(ptz);
    }
    fclose(pf);
    printf("load %lu ptz data\n", ptzs.frames_.size());
    return true;
}

bool ImageSequenceCameraIO::write_ptzFrames(const char *file, const ImageSequencePTZs & ptzs)
{
    assert(file);
    FILE *pf = fopen(file, "w");
    if (!pf) {
        printf("can not open %s\n", file);
        return false;
    }
    fprintf(pf, "%s\t", ptzs.video_name_.c_str());
    fprintf(pf, "%f\t", ptzs.frame_rate_);
    fprintf(pf, "%d %d\n", ptzs.width, ptzs.height);
   
    for (int i = 0; i<ptzs.frames_.size(); i++) {
        unsigned int fn = ptzs.frames_[i].fn_;
        double fl = ptzs.frames_[i].fl_;;
        double pan = ptzs.frames_[i].pan_;;
        double tilt = ptzs.frames_[i].tilt_;;
        double ssd = 10000;
        fprintf(pf, "%d\t %f\t %f\t %f\t %f\n", fn, fl, pan, tilt, ssd);
    }
    fclose(pf);
    printf("save to: %s\n", file);
    return true;
}

bool ImageSequenceCameraIO::interpolate_ptzFrames(const ImageSequencePTZs & originalPTZs, ImageSequencePTZs & interpolatedPTZs)
{
    interpolatedPTZs.video_name_ = originalPTZs.video_name_;
    interpolatedPTZs.frame_rate_ = originalPTZs.frame_rate_;
    interpolatedPTZs.width = originalPTZs.width;
    interpolatedPTZs.height = originalPTZs.height;
    interpolatedPTZs.frames_.clear();
    
    vcl_vector<FramePTZData> frames = originalPTZs.frames_;
    for (int i = 0; i<frames.size()-1; i++) {
        int startFn = frames[i].fn_;
        double startPan = frames[i].pan_;
        double startTilt = frames[i].tilt_;
        double startFl = frames[i].fl_;
        
        int endFn = frames[i+1].fn_;
        double endPan = frames[i+1].pan_;
        double endTilt = frames[i+1].tilt_;
        double endFl = frames[i+1].fl_;
        
        assert(startFn < endFn);
        for (int j = startFn; j<endFn; j++) {
            double pan  = VnlPlus::linearInterpolate(startFn, endFn, startPan, endPan, j);
            double tilt = VnlPlus::linearInterpolate(startFn, endFn, startTilt, endTilt, j);
            double fl   = VnlPlus::linearInterpolate(startFn, endFn, startFl, endFl, j);
            
            FramePTZData fpd;
            fpd.pan_ = pan;
            fpd.tilt_ = tilt;
            fpd.fl_ = fl;
            fpd.fn_ = j;
            interpolatedPTZs.frames_.push_back(fpd);
        }
    }
    interpolatedPTZs.frames_.push_back(originalPTZs.frames_.back());
    
    printf("interpolated %lu ptz frames\n", interpolatedPTZs.frames_.size());
    return true;
}

bool ImageSequenceCameraIO::write_ptzVideoParameter(const char *file, const PTZVideoParameter &para)
{
    FILE *pf = fopen(file, "w");
    assert(pf);
    fprintf(pf, "#videoName frameRate resolution distortion; camera center, Rodrigues vector\n");
    fprintf(pf, "%s\t", para.video_name_.c_str());
    fprintf(pf, "%f\t", para.frame_rate_);
    fprintf(pf, "%d %d\t %f\n", para.width_, para.height_, para.undistortion_);
    fprintf(pf, "%f %f %f\t %f %f %f\n", para.camera_center_.x(), para.camera_center_.y(), para.camera_center_.z(),
            para.rodrigues_[0], para.rodrigues_[1], para.rodrigues_[2]);
    fclose(pf);
    return true;
}

bool ImageSequenceCameraIO::read_ptzVideoParameter(const char *file, PTZVideoParameter &para)
{
    FILE *pf = fopen(file, "r");
    if (!pf) {
        printf("can not open %s\n", file);
        return false;
    }
    char buf[1024] = {NULL};
    fgets(buf, sizeof(buf), pf);
    printf("%s\n", buf);
    
    char video_name_buf[1024] = {NULL};
    int ret = fscanf(pf, "%s", video_name_buf);
    if (ret != 1) {
        return false;
    }
    para.video_name_ = vcl_string(video_name_buf);
    
    ret = fscanf(pf, "%lf %d %d %lf", &(para.frame_rate_), &(para.width_), &(para.height_), &(para.undistortion_));
    if (ret != 4) {
        return false;
    }
    double x = 0;
    double y = 0;
    double z = 0;
    double r1 = 0;
    double r2 = 0;
    double r3 = 0;
    ret = fscanf(pf, "%lf %lf %lf %lf %lf %lf", &x, &y, &z, &r1, &r2, &r3);
    assert(ret == 6);
    para.camera_center_ = vgl_point_3d<double>(x, y, z);
    para.rodrigues_     = vnl_vector_fixed<double, 3>(r1, r2, r3);
    fclose(pf);
    return true;
}

/****************          PlayerPositionIO        **************/
bool PlayerPositionIO::read_players(const char *file, PTZPlayerSet & players, int start_frame, int end_frame)
{
    FILE * pf = fopen(file, "r");
    if (!pf) {
        printf("Error: can not read from %s\n", file);
        return false;
    }
    char buf[1024] = {NULL};
    int num = fscanf(pf, "%s %lf %d %d", buf, &(players.frame_rate_), &(players.frame_width_), &(players.frame_height_));
    assert(num == 4);
    players.video_name_ = vcl_string(buf);
    
    int first_fn = 0;
    int last_fn = 0;
    num = fscanf(pf, "%d %d", &first_fn, &last_fn);
    assert(num == 2);
    for (int i = first_fn; i <= last_fn; i++) {
        int fn = 0;
        int num_players = 0;
        num = fscanf(pf, "%d %d", &fn, &num_players);
        assert(num == 2);
        if (fn >= start_frame && fn <= end_frame) {
            PTZFramePlayerPosition frame;
            frame.fn_ = fn;
            for (int j = 0; j<num_players; j++) {
                double x = 0;
                double y = 0;
                num = fscanf(pf, "%lf %lf", &x, &y);
                assert(num);
                frame.player_positions_.push_back(vgl_point_2d<double>(x, y));
            }
            players.frames_.push_back(frame);
        }
        else
        {
            //just skip
            for (int j = 0; j<num_players; j++) {
                double x = 0;
                double y = 0;
                num = fscanf(pf, "%lf %lf", &x, &y);
                assert(num);
            }
        }
        // jump to the end of file
        if (fn > end_frame) {
            break;
        }        
    }
    //
    if (players.frames_.size() > 0) {
        players.start_frame_ = players.frames_.front().fn_;
        players.end_frame_ = players.frames_.back().fn_;
    }
    else
    {
        players.start_frame_ = 0;
        players.end_frame_ = 0;
    }
    
    // check continus
    for (int i = 0; i<players.frames_.size()-1; i++) {
        if (players.frames_[i].fn_ + 1 != players.frames_[i+1].fn_) {
            printf("Warning: frame number is not continuous in %d\n", players.frames_[i].fn_);
        }
    }
    
    fclose(pf);
    return true;
}

bool PlayerPositionIO::write_players(const char *file, const PTZPlayerSet & players)
{
    FILE * pf = fopen(file, "w");
    assert(pf);
    
    fprintf(pf, "%s\t %f\t %d\t %d\n", players.video_name_.c_str(), players.frame_rate_, players.frame_width_, players.frame_height_);
    fprintf(pf, "%d\t %d\n", players.frames_[0].fn_, players.frames_.back().fn_);
    vcl_vector<PTZFramePlayerPosition> frames = players.frames_;
    for (int i = 0; i<frames.size(); i++) {
        fprintf(pf, "%d\t %d\n", frames[i].fn_, (int)frames[i].player_positions_.size());
        for (int j = 0; j<frames[i].player_positions_.size(); j++) {
            fprintf(pf, "%f %f", frames[i].player_positions_[j].x(), frames[i].player_positions_[j].y());
            if (j%5 == 4) {
                fprintf(pf, "\n");
            }
            else
            {
                fprintf(pf, "\t");
            }
        }
        if (frames[i].player_positions_.size()%5 != 0) {
            fprintf(pf, "\n");
        }
    }    
    fclose(pf);
    printf("write to: %s\n", file);
    return true;
}

bool PlayerPositionIO::write_json(const char *file, const PTZPlayerSet & players)
{
    FILE *pf = fopen(file, "w");
    assert(pf);
    fprintf(pf, "[\n");
    vcl_vector<PTZFramePlayerPosition> frames = players.frames_;
    for (int i = 0; i<frames.size(); i++) {
        fprintf(pf, "{\n");
        fprintf(pf, "\"fn\":\"%d\",\n", frames[i].fn_);
        fprintf(pf, "\"location\":[");
        for (int j = 0; j<frames[i].player_positions_.size(); j++) {
            fprintf(pf, "%.1f, %.1f", frames[i].player_positions_[j].x(), frames[i].player_positions_[j].y());
            if (j != frames[i].player_positions_.size() - 1) {
                fprintf(pf, ", ");
            }
            else
            {
                fprintf(pf, "]\n");
            }
        }
        // end }
        if (i != frames.size() -1) {
            fprintf(pf, "},\n");
        }
        else
        {
            fprintf(pf, "}\n");
        }
    }
    
    fprintf(pf, "]\n");
    fclose(pf);
    printf("write to: %s\n", file);
    return true;
}














