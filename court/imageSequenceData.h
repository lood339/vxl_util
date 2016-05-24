//
//  imageSequenceData.h
//  QuadCopter
//
//  Created by jimmy on 7/4/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef QuadCopter_imageSequenceData_h
#define QuadCopter_imageSequenceData_h

#include <vpgl/vpgl_perspective_camera.h>
#include <vcl_string.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector_fixed.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_point_2d.h>

class FrameData
{
public:
    int fn_;
    vpgl_perspective_camera<double> camera_;
    
    bool operator < (const FrameData & other) const
    {
        return this->fn_ < other.fn_;
    }
};

struct ImageSequenceCameras
{
    vcl_string video_name_;
    double frame_rate_;
    int width;
    int height;
    vcl_vector<FrameData> frames_;
};

class FramePTZData
{
public:
    int fn_;
    double pan_;
    double tilt_;
    double fl_;
    
    bool operator < (const FrameData & other) const
    {
        return this->fn_ < other.fn_;
    }
};

struct ImageSequencePTZs
{
    vcl_string video_name_;
    double frame_rate_;
    int width;
    int height;    
    vcl_vector<FramePTZData> frames_;
};

// video parameter, image have radial distortion
struct PTZVideoParameter
{
    vcl_string video_name_;
    double frame_rate_;
    int width_;
    int height_;
    double undistortion_;
    vgl_point_3d<double> camera_center_;
    vnl_vector_fixed<double, 3> rodrigues_;
    
    // principle point
    vgl_point_2d<double> pp(void)
    {
        return vgl_point_2d<double>(width_/2.0, height_/2.0);
    }
    
    vnl_vector_fixed<double, 6> coeff(void)
    {
        vnl_vector_fixed<double, 6> default_coeff;
        default_coeff.fill(0.0);
        return default_coeff;
    }
};









#endif
