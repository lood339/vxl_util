//
//  cameraCalibPlayerDetectionData.h
//  OnlineStereo
//
//  Created by jimmy on 3/13/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef OnlineStereo_cameraCalibPlayerDetectionData_h
#define OnlineStereo_cameraCalibPlayerDetectionData_h

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_vector_fixed.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vnl/vnl_vector_fixed.h>

// read and write raw data from .txt file
// raw data from detection result
// detection result: no velocity
// tracking result: velocity
struct DetectionResult
{
    unsigned long idNumber;
    unsigned long frameNumber;   // frame number in stationary camera
    double x;
    double y;
    double teamPro[3]; //team probabilities
    double dx;         //velocity
    double dy;
    
    DetectionResult()
    {
        dx = 0;
        dy = 0;
    }
    
    bool operator < (const DetectionResult & other) const
    {
        if(frameNumber != other.frameNumber)
        {
            return frameNumber < other.frameNumber;
        }
        else
        {
            return idNumber < other.idNumber;
        }
    }
};

struct FramePlayerPositions {
    unsigned long frameNumber;      // topview camera frame, stationary camera
    vcl_vector<vgl_point_2d<double> > positions;
    vcl_vector<vnl_vector_fixed<double, 2> > velocities_;
    vcl_vector<vnl_vector_fixed<double, 3> > teamProbability_;
};

struct PTZFramePlayerPosition
{
    int fn_;                    // PTZ camera frame
    vcl_vector<vgl_point_2d<double> > player_positions_;
};

class PTZPlayerSet
{
public:
    vcl_string video_name_;
    double frame_rate_;
    int frame_width_;
    int frame_height_;
    int start_frame_;
    int end_frame_; // include
    vcl_vector<PTZFramePlayerPosition> frames_;
    
public:
    vcl_vector<vgl_point_2d<double> > get_players(int fn)
    {
        vcl_vector<vgl_point_2d<double> > pts;
        if (fn >= start_frame_ && fn <= end_frame_) {
            pts = frames_[fn - start_frame_].player_positions_;
        }
        return pts;
    }
};


class FrameHashTable
{
    vcl_vector<FramePlayerPositions> frames_;
    vcl_vector<long int> hashtable_;          // index --> frame number, -1 for no detection
    
public:
    FrameHashTable(const vcl_vector<FramePlayerPositions> & frames)
    {
        frames_ = frames;
        for (int i = 0; i<frames_.size()-1; i++) {
            assert(frames_[i].frameNumber < frames_[i+1].frameNumber);
        }
        hashtable_.resize(frames_.back().frameNumber + 1, -1);
        for (int i = 0; i<frames_.size(); i++) {   /// one to one hash
            hashtable_[frames_[i].frameNumber] = i;
        }
    }
    ~FrameHashTable()
    {
        
    }
    
    // input is stationary frame number
    vcl_vector<vgl_point_2d<double> > find(unsigned long frameNumber)
    {
        vcl_vector<vgl_point_2d<double> > ret;
        if(frameNumber >= hashtable_.size() || hashtable_[frameNumber] == -1){
            return ret;
        }
        else
        {
            return frames_[hashtable_[frameNumber]].positions;
        }
    }
    
    vcl_vector<vnl_vector_fixed<double, 2> > findVelocity(unsigned long frameNumber)
    {
        vcl_vector<vnl_vector_fixed<double, 2> > ret;
        if(frameNumber >= hashtable_.size() || hashtable_[frameNumber] == -1){
            return ret;
        }
        else
        {
            return frames_[hashtable_[frameNumber]].velocities_;
        }
    }
    
};


class CalibrationResult
{
public:
    unsigned int frame_num;    // frame number in ptz camera
    vpgl_perspective_camera<double> camera;
    
    CalibrationResult()
    {
        frame_num = -1;
    }
    
    bool operator <(const CalibrationResult & other) const
    {
        return frame_num < other.frame_num;
    }
    
    bool operator == (const CalibrationResult & other) const
    {
        return frame_num == other.frame_num;
    }
};

class CameraPlayerData
{
public:
    unsigned int frame_num;        //  frame number from ptz video
    vpgl_perspective_camera<double> camera;
    vcl_vector<vgl_point_2d<double> > player_positions;
    
    bool operator == (const CameraPlayerData & other) const
    {
        if (frame_num != other.frame_num) {
            return false;
        }
        for (int i = 0; i<player_positions.size(); i++) {
            if (player_positions[i] != other.player_positions[i]) {
                return false;
            }
        }
        return true;
    }
};

class PTZData
{
public:
    unsigned int frame_num;
    double fl;
    double pan;
    double tilt;
    double ssd;
    
    bool operator < (const PTZData & other)const
    {
        return frame_num < other.frame_num;
    }
    bool operator == (const PTZData & other) const
    {
        return frame_num == other.frame_num;
    }
};

class PlayerPTZData
{
public:
    unsigned int frame_num;    // ptz camera frame number
    double fl;
    double pan;
    double tilt;
    vcl_vector<vgl_point_2d< double> > pts;   // player positions
};


// used to generate feature
struct CourtObeservation
{
    int frame_num;            // ptz camera frame number, synced, 60 fps
    int frame_num_top_view;   // top view camera frame numbe, 25 fps
    double fl;
    double pan;
    double tilt;
    vcl_vector<vgl_point_2d< double> > pts;        // all positions
    vcl_vector<vnl_vector_fixed<double, 2> > velocities; // velocity of each players opitional
  //  vcl_vector<vgl_point_2d< double> > player_pts; // player positions @todo
  //  vgl_point_2d<double> ball_pt;                  // ball position @todo
};


#endif
