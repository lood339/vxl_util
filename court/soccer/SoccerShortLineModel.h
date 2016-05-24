//
//  SoccerShortLineModel.h
//  OnlineStereo
//
//  Created by jimmy on 3/5/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__SoccerShortLineModel__
#define __OnlineStereo__SoccerShortLineModel__

// describe the soccer model using short line and its intersection
#include <vcl_vector.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vcl_utility.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_line_segment_2d.h>
#include "SoccerGraphCut.h"
#include "SoccerGraphCutUtil.h"

// for main PTZ camera
class SoccerShortLineModel
{
private:
    double field_width_;     // unit is yard
    double field_heigh_;
    
public:
    SoccerShortLineModel(double w = 118, double h = 70);
    ~SoccerShortLineModel();
    
    vgl_point_2d<double> intersectionInWorld(int node1, int node2);
    bool projectNode(const vpgl_perspective_camera<double> & camera, int node, vgl_line_segment_2d<double> & segment);
    
    // sample line segment by camera
    bool sampleCalibLine(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, int nodeNum,
                         double sampleUnit, SGCCalibLine & calibLine);
    
    
    // retrieve edge width by edge node
    // every node is a line or circle. It has properties. Most of them are hard coded.
    int nodeWidth(int node) const;
    double nodeMagnitude(int node) const;  //
    double nodeLambda(int node) const;
    double nodeOnlineRatio(int node) const;
    bool hasDoubleEdge(int node) const;
    bool isValidNodePair(int n1, int n2) const;
       
    
private:
    static vcl_vector<vgl_line_segment_2d<double> > getWorldLineSegment(double width, double height);
    static vcl_vector<vgl_line_2d<double> > getWorldLines(double width, double height);
    static vcl_vector<vcl_pair<int, int> > nodePairs();
    
};

#endif /* defined(__OnlineStereo__SoccerShortLineModel__) */
