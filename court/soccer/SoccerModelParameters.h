//
//  SoccerModelParameters.h
//  OnlineStereo
//
//  Created by jimmy on 9/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef OnlineStereo_SoccerModelParameters_h
#define OnlineStereo_SoccerModelParameters_h

// WWoS soccer model parameters
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_2d.h>
#include <vgl/vgl_ellipse_2d.h>
#include <vcl_vector.h>

// soccer graph cut calibration line
// graph cut is not used :)
struct SGCCalibLine
{
private:
    // do not change over time
    int node_;
    vgl_line_segment_2d<double> world_line_segment_;
    vgl_line_2d<double> world_line_;
    
    // updating overtime
    vgl_line_2d<double> projected_image_line_;
    vgl_line_2d<double> detected_image_line_;
    
    vgl_line_segment_2d<double> projected_image_line_segment_;
    
public:
    vgl_line_segment_2d<double> detected_image_line_segment_;
    
public:
    SGCCalibLine(){node_ = -1;}
    
    SGCCalibLine(int n, const vgl_line_segment_2d<double> & seg):node_(n), world_line_segment_(seg), world_line_(seg.point1(), seg.point2())
    {
    }
    
    void setProjectedLine(const vgl_point_2d<double> & p1, const vgl_point_2d<double> & p2)
    {
        projected_image_line_ = vgl_line_2d<double>(p1, p2);
        projected_image_line_segment_ = vgl_line_segment_2d<double>(p1, p2);
    }
    void setDetectedLine(const vgl_line_2d<double> & line)
    {
        detected_image_line_ = line;
    }
    
    vgl_point_3d<double> point1_3d()
    {
        vgl_point_3d<double> p(world_line_segment_.point1().x(), world_line_segment_.point1().y(), 0.0);
        return p;
    }
    
    vgl_point_3d<double> point2_3d()
    {
        vgl_point_3d<double> p(world_line_segment_.point2().x(), world_line_segment_.point2().y(), 0.0);
        return p;
    }
    
    vgl_point_2d<double> point1()
    {
        return projected_image_line_segment_.point1();
    }
    
    vgl_point_2d<double> point2()
    {
        return projected_image_line_segment_.point2();
    }
    int node()const {return node_;}
    
    vgl_line_2d<double> projectedLine() const
    {
        return projected_image_line_;
    }
    vgl_line_2d<double> detectedLine() const
    {
        return detected_image_line_;
    }
    
    // 0: horizontal 1: vertical
    int scan_direction()
    {
        vgl_point_2d<double> p1 = world_line_segment_.point1();
        vgl_point_2d<double> p2 = world_line_segment_.point2();
        double dx = fabs(p1.x() - p2.x());
        double dy = fabs(p1.y() - p2.y());
        if(dx > dy)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    // assume two lines have intersection in world coordinate
    bool intersection(const SGCCalibLine & other, vgl_point_2d<double> & wld, vgl_point_2d<double> & img_detected) const
    {
        bool isOk = vgl_intersection(this->world_line_, other.world_line_, wld);
        if(!isOk){
            return false;
        }
        isOk = vgl_intersection(this->detected_image_line_, other.detected_image_line_, img_detected);
        return isOk;
    }
};

struct LineTrackingParameter
{
    double circle_ratio_;
    double line_thick_ness_;
    double ransac_H_threshold_;
    LineTrackingParameter()
    {
        circle_ratio_ = 0.3;
        line_thick_ness_ = 20;
        ransac_H_threshold_ = 3.0;
    }
};

struct NodeIntersection
{
    vgl_point_2d<double> wld_pt_;
    vgl_point_2d<double> img_pt_;
    int node_1_;  // node index in model, 0 for center line and 100 for center circle
    int node_2_;
    NodeIntersection()
    {
        node_1_ = -1;
        node_2_ = -2;
    }
};

/*
 struct PatchMatchParameter
 {
 int patchSize_;
 int gradientRadius_;
 int iternationNum_;
 double distance_difference_threshold_;   // distance difference between detected location and projected location
 
 PatchMatchParameter()
 {
 patchSize_ = 64;
 gradientRadius_ = 20;
 iternationNum_  = 10;
 distance_difference_threshold_ = 50;
 }
 };
 */

struct SoccerModelTrackingResult
{
    vcl_vector<SGCCalibLine> detected_calib_Lines_;   // long lines in the model
    
    vcl_vector<vgl_point_3d<double> > wld_pts_;  // line intersection
    vcl_vector<vgl_point_2d<double> > img_pts_;
    
    vcl_vector<int> detected_node_id_;     // SoccerShortLineModel
    
    bool is_centercircle_tracked_;
    vgl_ellipse_2d<double> center_ellipse_; // center circle in image
    
    bool is_left_ellipe_detected_;
    vgl_ellipse_2d<double> left_ellipse_;
    vcl_vector<vgl_point_2d<double> > left_ellipse_points_;   // point in the image space
};





#endif
