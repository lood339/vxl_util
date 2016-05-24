//
//  SoccerCourt.h
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VpglPtzOpt__SoccerCourt__
#define __VpglPtzOpt__SoccerCourt__

#include <vcl_vector.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_transform_2d.h>

class SoccerCourt
{
public:
    SoccerCourt();
    virtual ~SoccerCourt();
    
    virtual void courtImage(vil_image_view<vxl_byte> &image) = 0;
    virtual void courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image) = 0;
    virtual void getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt) = 0;
    virtual vgl_transform_2d< double > imageToWorld() = 0;
    virtual vcl_string name() = 0;
    
    // width, height in yard
    // output in meter
    // only has 2D lines in the court
    static vcl_vector< vgl_line_segment_2d< double > > getAllLineSegments(const double width = 115.0, const double height = 74.0);    
    
    // world (meter) to image (pixel)
    static vgl_point_2d<double> World2Image(const vgl_point_2d<double> &p);
    
    // points that locate in the intersection of field lines
    static vcl_vector<vgl_point_2d<double> > getCalibratePoints(const double width = 115.0, const double height = 74.0);
    
    // calibration points:
    // segment points in the middle of line segment (end points are excluded)
    static vcl_vector< vgl_point_3d<double> > getCalibrationPoints(double segmentLength, const double width = 115.0, const double height = 74.0);
};

// barcelona court
// width 115 yard, height 74 yard
class BarcelonaCourt :public SoccerCourt
{
    public:
    BarcelonaCourt();
    ~BarcelonaCourt();
    
    virtual void courtImage(vil_image_view<vxl_byte> &image);
    virtual void courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image);
    // lineWidth  : 5-10 * 2
    // gauss_sigma: around 100
    virtual void getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt);
    virtual vgl_transform_2d< double > imageToWorld();
    virtual vcl_string name(){return vcl_string("BarcelonaCourt");}
    
    
    static void overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image);
    static void overlayPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image);
};


#endif /* defined(__VpglPtzOpt__SoccerCourt__) */
