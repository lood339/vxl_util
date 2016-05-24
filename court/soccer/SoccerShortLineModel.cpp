//
//  SoccerShortLineModel.cpp
//  OnlineStereo
//
//  Created by jimmy on 3/5/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "SoccerShortLineModel.h"
#include <vgl/vgl_intersection.h>
#include <vcl_algorithm.h>
#include "vil_plus.h"
#include <vgl/vgl_distance.h>
#include "vxl_plus.h"
#include "vgl_plus.h"

SoccerShortLineModel::SoccerShortLineModel(double w, double h)
{
    field_width_ = w;
    field_heigh_ = h;
}

SoccerShortLineModel::~SoccerShortLineModel()
{
    
}

vgl_point_2d<double> SoccerShortLineModel::intersectionInWorld(int node1, int node2)
{
    vcl_vector<vgl_line_2d<double> > lines = SoccerShortLineModel::getWorldLines(field_width_, field_heigh_);
    assert(node1 < lines.size());
    assert(node2 < lines.size());
    
    vgl_point_2d<double> p;
    bool isIntersect = vgl_intersection(lines[node1], lines[node2], p);
    assert(isIntersect);
    return p;
}

bool SoccerShortLineModel::projectNode(const vpgl_perspective_camera<double> & camera, int node, vgl_line_segment_2d<double> & segment)
{
    vcl_vector<vgl_line_segment_2d<double> > lineSegment = SoccerShortLineModel::getWorldLineSegment(field_width_, field_heigh_);
    assert(node >= 0 && node < lineSegment.size());
    
    vgl_point_2d<double> p1 = lineSegment[node].point1();
    vgl_point_2d<double> p2 = lineSegment[node].point2();
    
    vgl_homg_point_3d<double> ph1(p1.x(), p1.y(), 0.0, 1.0);
    vgl_homg_point_3d<double> ph2(p2.x(), p2.y(), 0.0, 1.0);
    if (!camera.is_behind_camera(ph1) && !camera.is_behind_camera(ph2)) {
        vgl_point_2d<double> q1 = camera.project(ph1);
        vgl_point_2d<double> q2 = camera.project(ph2);
        
        segment = vgl_line_segment_2d<double>(q1, q2);
        return true;
    }
    return false;
}

bool SoccerShortLineModel::sampleCalibLine(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, int nodeNum,
                                         double sampleUnit, SGCCalibLine & calibLine)
{
    vcl_vector<vgl_line_segment_2d<double> > wLines = SoccerShortLineModel::getWorldLineSegment(field_width_, field_heigh_);
    
    if (nodeNum >= wLines.size()) {
        printf("Error: node number should be smaller than line set size\n");
        return false;
    }
    
    vgl_point_3d<double> p1(wLines[nodeNum].point1().x(), wLines[nodeNum].point1().y(), 0);
    vgl_point_3d<double> p2(wLines[nodeNum].point2().x(), wLines[nodeNum].point2().y(), 0);
    
    vgl_point_2d<double> q1(-1, -1); // (-1, -1) is the mask for unfined end point
    vgl_point_2d<double> q2(-1, -1);
    vgl_vector_2d<double> dir = wLines[nodeNum].direction();
    const double length = vgl_distance(p1, p2);
    
    // sample first point from p1 to p2 in the image
    bool isFound = false;
    for (double j = 0; j <= length; j += sampleUnit) {
        double x = p1.x() + dir.x() * j;
        double y = p1.y() + dir.y() * j;
        
        vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, 0.0));
        if (VglPlus::vgl_inside_image(q, imageW, imageH)) {
            q1 = q;
            isFound = true;
            break;
        }
    }
    if (!isFound) {
        return false;
    }
    isFound = false;
    
    // sample second point from p2 to p1 in the image
    for (double j = length; j >=  0; j -= sampleUnit) {
        double x = p1.x() + dir.x() * j;
        double y = p1.y() + dir.y() * j;
        
        vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, 0.0));
        if (VglPlus::vgl_inside_image(q, imageW, imageH)) {
            q2 = q;
            isFound = true;
            break;
        }
    }
    if (!isFound) {
        return false;
    }
    
    double dis = vgl_distance(q1, q2);
    if (dis > 5 * sampleUnit) {
        calibLine = SGCCalibLine(nodeNum, wLines[nodeNum]);
        calibLine.setProjectedLine(q1, q2);
        return true;
    }
    return false;
}


int SoccerShortLineModel::nodeWidth(int node) const
{
    assert(node >= 0 && node < 20);
    
    int nodeWidth[] = {30, 30, 30, 30, 30,
                       15, 15, 20, 20, 20,
                       15, 20, 15, 30, 30,
                       30, 30, 10, 50, 60};
    assert(sizeof(nodeWidth)/sizeof(nodeWidth[0]) == 20);
    return nodeWidth[node];
}

double SoccerShortLineModel::nodeMagnitude(int node) const
{
    assert(node >= 0 && node < 20);
    
    double nodeMagnitude[] = {0.02, 0.02, 0.02, 0.02, 0.02,
                              0.02, 0.02, 0.02, 0.02, 0.02,
                              0.02, 0.02, 0.02, 0.015, 0.015,
                              0.015, 0.015, 0.02, 0.02, 0.02};
    assert(sizeof(nodeMagnitude)/sizeof(nodeMagnitude[0]) == 20);
    return nodeMagnitude[node];
}

double SoccerShortLineModel::nodeLambda(int node) const
{
    assert(node >= 0 && node < 20);
    
    double nodeLambda[] = {2.0, 2.0, 2.0, 2.0, 2.0,
                           1.5, 1.5, 1.5, 1.5, 1.5,
                           1.5, 1.5, 1.5, 1.5, 1.5,
                           2.0, 2.0, 2.0, 2.0, 2.0};
    assert(sizeof(nodeLambda)/sizeof(nodeLambda[0]) == 20);
    return nodeLambda[node];
}

double SoccerShortLineModel::nodeOnlineRatio(int node) const
{
    assert(node >= 0 && node < 20);
    double nodeOnlineRatio[] = {0.5, 0.5, 0.5, 0.5, 0.5,
                                0.5, 0.5, 0.4, 0.4, 0.5,
                                0.5, 0.5, 0.5, 0.5, 0.5,
                                0.5, 0.5, 0.5, 0.5, 0.5
                                };
    
    assert(sizeof(nodeOnlineRatio)/sizeof(nodeOnlineRatio[0]) == 20);
    return nodeOnlineRatio[node];
}

bool SoccerShortLineModel::hasDoubleEdge(int node) const
{
    assert(node >= 0 && node < 20);
    
    bool hasDoubleEdge[] = {true, true, true, true, true,
                            true, true, false, false, true,
                            false, true, false, false, false,
                            false, false, false, true, true};
    assert(sizeof(hasDoubleEdge)/sizeof(hasDoubleEdge[0]) == 20);
    return hasDoubleEdge[node];
}
bool SoccerShortLineModel::isValidNodePair(int n1, int n2) const
{
    vcl_vector<vcl_pair<int, int> > nodePairs = SoccerShortLineModel::nodePairs();
    for (int i = 0; i<nodePairs.size(); i++) {
        if ((n1 == nodePairs[i].first && n2 ==  nodePairs[i].second) ||
            (n1 == nodePairs[i].second && n2 == nodePairs[i].first)) {
            return true;
        }
    }
    return false;
}


vcl_vector<vgl_line_segment_2d<double> > SoccerShortLineModel::getWorldLineSegment(double width, double height)
{
    
    vcl_vector< vgl_line_segment_2d< double > > markings(20);
    
    // one center line
    markings[0] = vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(0, height/2));
    
    // 18 yard line
    markings[1] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) );
    markings[2] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) );
    
    // 18 yard line short, horizontal, near end
    markings[3] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ) );
    markings[4] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ) );
    
    // 6 yard
    markings[5] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * (-1), -10 ), vgl_point_2d< double >( (width/2 - 6) * (-1), +10 ) );
    markings[6] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * (+1), -10 ), vgl_point_2d< double >( (width/2 - 6) * (+1), +10 ) );
    
    // 18 yard line short, horizontal, far end
    markings[7] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) );
    markings[8] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) );
    
    // part of goal line (left)
    markings[9]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-width/2, -12));
    markings[10] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, 12), vgl_point_2d< double >(-width/2, height/2));
    
    // part of goal line (right)
    // a bug
    markings[11]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, -12));
    markings[12] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, 12), vgl_point_2d< double >(width/2, height/2));
    
    // a bug, fortunatelly harmless
 //   markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, height/2)));
    
    // two field hockey line horizontal
    markings[13] = vgl_line_segment_2d<double>(vgl_point_2d<double>(-width/2 + 18, -height/2+11.4), vgl_point_2d<double>(- 10, -height/2+11.4));
    markings[14] = vgl_line_segment_2d<double>(vgl_point_2d<double>(10, -height/2+11.4), vgl_point_2d<double>(width/2 - 18, -height/2+11.4));
    
    // two field hockey line vertical
    markings[15] = vgl_line_segment_2d<double>(vgl_point_2d<double>(-10, -height/2+11.4), vgl_point_2d<double>(-10, -10));
    markings[16] = vgl_line_segment_2d<double>(vgl_point_2d<double>(10,  -height/2+11.4), vgl_point_2d<double>(10, -10));
    
    // part of far touch line
    markings[17] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, height/2), vgl_point_2d< double >(20, height/2));
    
    // part of near touch line
    markings[18] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-20, -height/2));
    markings[19] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, -height/2), vgl_point_2d< double >(width/2, -height/2));
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 );
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    return markings;    
}

vcl_vector<vgl_line_2d<double> > SoccerShortLineModel::getWorldLines(double width, double height)
{
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerShortLineModel::getWorldLineSegment(width, height);
    
    // end points to lines
    vcl_vector<vgl_line_2d<double> >  lines;
    for (int i = 0; i<markings.size(); i++) {
        vgl_line_2d<double> line(markings[i].point1(), markings[i].point2());
        lines.push_back(line);
    }
    return lines;
}
vcl_vector<vcl_pair<int, int> > SoccerShortLineModel::nodePairs()
{
    vcl_vector<vcl_pair<int, int> > pairs;
    
    int data[][2] = { 0,3, 0,7, 0,13, 0, 17,
                      0,4, 0,8, 0,14,
                      1,3, 1,7, 1,13, 1,18,
                      2,4, 2,8, 2,14, 2,19,
                      3,5, 3,9, 4,6, 4,11,
                      5,7, 6,8, 7,10, 8,12,
                      9,13, 9,18, 11,14, 11,19,
                      13,15, 14,16};
    
    int dataNum = sizeof(data)/(sizeof(data[0][0]) * 2);
    for (int i = 0; i< dataNum; i++) {
        pairs.push_back(vcl_pair<int, int> (data[i][0], data[i][1]));
    }
    
    return pairs;    
}




