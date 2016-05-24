//
//  SoccerModelLeftSideView.h
//  QuadCopter
//
//  Created by jimmy on 6/30/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__SoccerModelLeftSideView__
#define __QuadCopter__SoccerModelLeftSideView__

// WWoS soccer field model from left side view camera
#include <vcl_vector.h>
#include <vcl_utility.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_segment_3d.h>
#include <vgl/vgl_line_3d_2_points.h>
#include <vpgl/vpgl_perspective_camera.h>
#include "SoccerLineParameters.h"
#include <vcl_utility.h>


class SoccerModelLeftSideView
{
private:
    double field_width_;     // unit is yard
    double field_heigh_;
    
    vcl_vector<vgl_line_segment_2d<double> > short_segments_;  // depends on the camera pose
    vcl_vector<vgl_line_segment_2d<double> > long_segments_;   // not depends on the camera pose
    
public:
    SoccerModelLeftSideView(double w = 118, double h = 70);
    ~SoccerModelLeftSideView();
    
    void calculate_short_segment(double segment_length = 2.0); // meter
    void calculate_short_segment(const vpgl_perspective_camera<double> & camera, double segment_image_length = 100.0); // pixel
    // segment in the image space
    bool projectNode(const vpgl_perspective_camera<double> & camera, int node,
                     vgl_line_segment_2d<double> & wld_seg,
                     vgl_line_segment_2d<double> & img_seg);
    bool projectShortLineSegment(const vpgl_perspective_camera<double> & camera, int node, vgl_line_segment_2d<double> & segment);
    bool getShortlineSegment(int node, vgl_line_segment_2d<double> & segment); // line segment in world coordinate
    
    //
    int shortLineSegmentNum(void){return (int)short_segments_.size();}
    int longLineSegmentNum(void);
    // far touch line from the view of main PTZ camera
    bool getFarTouchLinesegment(const vpgl_perspective_camera<double> & camera, vgl_line_segment_2d<double> & segment);
    bool getNearTouchLinesegment(const vpgl_perspective_camera<double> & camera, vgl_line_segment_2d<double> & segment);
    bool getRightBorderLinesegment(const vpgl_perspective_camera<double> & camera, vgl_line_segment_2d<double> & segment);
    bool getCommercialboardLinesegment(const vpgl_perspective_camera<double> & camera, vgl_line_segment_2d<double> & segment);
    
    // displacement from commercial board to far touch line
    // it assume the two projected lines are parallel
    bool commercialBoardFartouchLineDisplacement(const vpgl_perspective_camera<double> & camera, vnl_vector_fixed<double, 2> & displacement);
    
    
    // retrieve edge width by edge node
    // every node is a line or circle. It has properties. Most of them are hard coded.
    // search_distance: distance in world coordinate, meter
    int nodeScanWidth(const vpgl_perspective_camera<double> & camera, int node, double search_distance = 1.0) const;
    double nodeMagnitude(int node) const;  //
    double nodeLambda(int node) const;
    SoccerLineType nodetype(int node) const;
    bool hasDoubleEdge(const vpgl_perspective_camera<double> & camera, int node) const;
    bool isValidNodePair(int n1, int n2) const;
    

    private:
    static vcl_vector<vgl_line_segment_2d<double> > get2DWorldLinesegment(double width, double height);
    // divide long segment into short segments, so that thay have more chance be seen inside the image
    // segmentLegnth: length of line segment in the image
    static vcl_vector<vgl_line_segment_2d<double> > get2DShortLinesegment(double width, double height, double segmentLegnth);
       
    
    static vcl_vector<vgl_line_3d_2_points<double> > getWorldLines(double width, double height);
    static vcl_vector<vcl_pair<int, int> > nodePairs();
    
    // border lines
    static vcl_vector< vgl_line_segment_2d<double> > borderLines(double width, double height);   
   
};









#endif /* defined(__QuadCopter__SoccerModelLeftSideView__) */
