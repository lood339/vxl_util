//
//  SoccerModelLeftSideView.cpp
//  QuadCopter
//
//  Created by jimmy on 6/30/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "SoccerModelLeftSideView.h"
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_distance.h>
#include "vgl_plus.h"
#include "vpgl_plus.h"
#include <vgl/vgl_closest_point.h>

SoccerModelLeftSideView::SoccerModelLeftSideView(double w, double h)
{
    field_width_ = w;
    field_heigh_ = h;
    calculate_short_segment();
    long_segments_ = SoccerModelLeftSideView::get2DWorldLinesegment(field_width_, field_heigh_);
}

SoccerModelLeftSideView::~SoccerModelLeftSideView()
{
    
}

void SoccerModelLeftSideView::calculate_short_segment(double segment_length)
{
    short_segments_.clear();
    short_segments_ = SoccerModelLeftSideView::get2DShortLinesegment(field_width_, field_heigh_, segment_length);
}

void SoccerModelLeftSideView::calculate_short_segment(const vpgl_perspective_camera<double> & camera, double segment_image_length)
{
    short_segments_.clear();
    vcl_vector<vgl_line_segment_2d<double> > longSegments = SoccerModelLeftSideView::get2DWorldLinesegment(field_width_, field_heigh_);
    
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    for (int i = 0; i<longSegments.size(); i++) {       
    
        vgl_line_segment_2d<double> sampledSegment;  // sampled line segment must be inside image
        bool isSampled = VpglPlus::sampleLineSegment(camera, w, h, longSegments[i], 0.5, sampledSegment);
        if (!isSampled) {
            continue;
        }
        vgl_point_3d<double> p1 = vgl_point_3d<double>(sampledSegment.point1().x(), sampledSegment.point1().y(), 0.0);
        vgl_point_3d<double> p2 = vgl_point_3d<double>(sampledSegment.point2().x(), sampledSegment.point2().y(), 0.0);
        vgl_point_2d<double> q1 = camera.project(p1);
        vgl_point_2d<double> q2 = camera.project(p2);
        
        // one of the end inside image view
        if (VglPlus::vgl_inside_image(q1, w, h) || VglPlus::vgl_inside_image(q2, w, h)) {
            double distance = vgl_distance(q1, q2);
            int num = distance/segment_image_length;  // the number is decided by its length in image space
            // @todo num may be zero
            for (int j = 0; j<num; j++) {
                double t1 = 1.0 * j / num;
                double t2 = 1.0 * (j+1)/num;
                vgl_point_2d<double> p3 = sampledSegment.point_t(t1);
                vgl_point_2d<double> p4 = sampledSegment.point_t(t2);
                short_segments_.push_back(vgl_line_segment_2d<double>(p3, p4));
            }
        }
    }    
    return;
}

bool SoccerModelLeftSideView::projectNode(const vpgl_perspective_camera<double> & camera, int node,
                                          vgl_line_segment_2d<double> & wld_seg,
                                          vgl_line_segment_2d<double> & img_seg)
{
    vcl_vector<vgl_line_segment_2d<double> > lineSegments = SoccerModelLeftSideView::get2DWorldLinesegment(field_width_, field_heigh_);
    if (node < 0 || node >= lineSegments.size()) {
        return false;
    }
    wld_seg = lineSegments[node];
    
    vgl_homg_point_3d<double> p1 = vgl_homg_point_3d<double>(wld_seg.point1().x(), wld_seg.point1().y(), 0.0, 1.0);
    vgl_homg_point_3d<double> p2 = vgl_homg_point_3d<double>(wld_seg.point2().x(), wld_seg.point2().y(), 0.0, 1.0);
    
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    
    if (!camera.is_behind_camera(p1) && !camera.is_behind_camera(p2))
    {
        // make sure the sampled segment is inside image
        vgl_line_segment_2d<double> in_image_segment;
        bool isInImage = VpglPlus::sampleLineSegment(camera, w, h, wld_seg, 0.5, in_image_segment);
        if (!isInImage) {
            return false;
        }

        vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(in_image_segment.point1().x(), in_image_segment.point1().y(), 0.0));
        vgl_point_2d<double> q2 = camera.project(vgl_point_3d<double>(in_image_segment.point2().x(), in_image_segment.point2().y(), 0.0));
        
        img_seg = vgl_line_segment_2d<double>(q1, q2);
        return true;
    }
    return false;
}

bool SoccerModelLeftSideView::projectShortLineSegment(const vpgl_perspective_camera<double> & camera, int node, vgl_line_segment_2d<double> & segment)
{     
    if (node < 0 || node >= short_segments_.size()) {
        return false;
    }
    
    vgl_homg_point_3d<double> p1 = vgl_homg_point_3d<double>(short_segments_[node].point1().x(), short_segments_[node].point1().y(), 0.0);
    vgl_homg_point_3d<double> p2 = vgl_homg_point_3d<double>(short_segments_[node].point2().x(), short_segments_[node].point2().y(), 0.0);
    
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    if (!camera.is_behind_camera(p1) && !camera.is_behind_camera(p2)) {
        vgl_point_2d<double> q1 = camera.project(p1);
        vgl_point_2d<double> q2 = camera.project(p2);
        
        if (VglPlus::vgl_inside_image(q1, w, h, 20) && VglPlus::vgl_inside_image(q2, w, h, 20)) {
            segment = vgl_line_segment_2d<double>(q1, q2);
            return true;
        }
    }
    return false;
}

bool SoccerModelLeftSideView::getShortlineSegment(int node, vgl_line_segment_2d<double> & segment)
{
    if (node < 0 || node >= short_segments_.size()) {
        return false;
    }
    segment = short_segments_[node];
    return true;
}

int SoccerModelLeftSideView::longLineSegmentNum(void)
{
    return (int)long_segments_.size();
}

bool SoccerModelLeftSideView::getFarTouchLinesegment(const vpgl_perspective_camera<double> & camera, vgl_line_segment_2d<double> & segment)
{
    vcl_vector<vgl_line_segment_2d<double> > longLines = SoccerModelLeftSideView::borderLines(field_width_, field_heigh_);
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    vgl_line_segment_2d<double> farTouchLine = longLines[0];
    bool isSampled = VpglPlus::sampleLineSegment(camera, w, h, farTouchLine, 0.5, segment);
    if (!isSampled) {
        return false;
    }
    
    vgl_point_2d<double> p1 = camera.project(vgl_point_3d<double>(segment.point1().x(), segment.point1().y(), 0.0 ));
    vgl_point_2d<double> p2 = camera.project(vgl_point_3d<double>(segment.point2().x(), segment.point2().y(), 0.0 ));
    segment = vgl_line_segment_2d<double>(p1, p2);
    return isSampled;
}

bool SoccerModelLeftSideView::getNearTouchLinesegment(const vpgl_perspective_camera<double> & camera, vgl_line_segment_2d<double> & segment)
{
    vcl_vector<vgl_line_segment_2d<double> > longLines = SoccerModelLeftSideView::borderLines(field_width_, field_heigh_);
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    vgl_line_segment_2d<double> farTouchLine = longLines[1];
    bool isSampled = VpglPlus::sampleLineSegment(camera, w, h, farTouchLine, 0.5, segment);
    if (!isSampled) {
        return false;
    }
    
    vgl_point_2d<double> p1 = camera.project(vgl_point_3d<double>(segment.point1().x(), segment.point1().y(), 0.0 ));
    vgl_point_2d<double> p2 = camera.project(vgl_point_3d<double>(segment.point2().x(), segment.point2().y(), 0.0 ));
    segment = vgl_line_segment_2d<double>(p1, p2);
    return isSampled;    
}

bool SoccerModelLeftSideView::getRightBorderLinesegment(const vpgl_perspective_camera<double> & camera, vgl_line_segment_2d<double> & segment)
{
    vcl_vector<vgl_line_segment_2d<double> > longLines = SoccerModelLeftSideView::borderLines(field_width_, field_heigh_);
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    vgl_line_segment_2d<double> farTouchLine = longLines[2];
    bool isSampled = VpglPlus::sampleLineSegment(camera, w, h, farTouchLine, 0.5, segment);
    if (!isSampled) {
        return false;
    }
    
    vgl_point_2d<double> p1 = camera.project(vgl_point_3d<double>(segment.point1().x(), segment.point1().y(), 0.0 ));
    vgl_point_2d<double> p2 = camera.project(vgl_point_3d<double>(segment.point2().x(), segment.point2().y(), 0.0 ));
    segment = vgl_line_segment_2d<double>(p1, p2);
    return isSampled;
}

bool SoccerModelLeftSideView::getCommercialboardLinesegment(const vpgl_perspective_camera<double> & camera, vgl_line_segment_2d<double> & segment)
{
    vcl_vector<vgl_line_segment_2d<double> > longLines = SoccerModelLeftSideView::borderLines(field_width_, field_heigh_);
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    vgl_line_segment_2d<double> farTouchLine = longLines[3];
    bool isSampled = VpglPlus::sampleLineSegment(camera, w, h, farTouchLine, 0.5, segment);
    if (!isSampled) {
        return false;
    }
    
    vgl_point_2d<double> p1 = camera.project(vgl_point_3d<double>(segment.point1().x(), segment.point1().y(), 0.0 ));
    vgl_point_2d<double> p2 = camera.project(vgl_point_3d<double>(segment.point2().x(), segment.point2().y(), 0.0 ));
    segment = vgl_line_segment_2d<double>(p1, p2);
    return isSampled;
}

bool SoccerModelLeftSideView::commercialBoardFartouchLineDisplacement(const vpgl_perspective_camera<double> & camera, vnl_vector_fixed<double, 2> & displacement)
{
    vgl_line_segment_2d<double> seg1;
    vgl_line_segment_2d<double> seg2;
    bool isCommercial = this->getCommercialboardLinesegment(camera, seg1);
    bool isFartouch = this->getFarTouchLinesegment(camera, seg2);
    if (!isCommercial || !isFartouch) {
        return false;
    }
    
    vgl_point_2d<double> ct1 = seg1.point_t(0.5);
    vgl_point_2d<double> p = vgl_closest_point(seg2, ct1);
    
    displacement[0] = p.x() - ct1.x();
    displacement[1] = p.y() - ct1.y();
    return true;
}

int SoccerModelLeftSideView::nodeScanWidth(const vpgl_perspective_camera<double> & camera, int node, double search_distance) const
{
    assert(node >= 0 && node < long_segments_.size());
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    
    // 1 meter distance in world coordinate,
    vgl_line_segment_2d<double> inImageSegment;  // sampled line segment must be inside image, world coordinate
    bool isSampled = VpglPlus::sampleLineSegment(camera, w, h, long_segments_[node], 0.5, inImageSegment);
    if (!isSampled) {
        return false;
    }
    
    vgl_point_2d<double> ct = inImageSegment.point_t(0.5);
    // displace center point in the normal direction of the segment
    vgl_vector_2d<double> d1 = inImageSegment.normal() * search_distance;
    vgl_point_2d<double> ct1 = ct + d1;
    vgl_point_2d<double> proj_ct1 = camera.project(vgl_point_3d<double>(ct1.x(), ct1.y(), 0.0));
    // project the line segment
    vgl_point_2d<double> p1 = camera.project(vgl_point_3d<double>(inImageSegment.point1().x(), inImageSegment.point1().y(), 0.0));
    vgl_point_2d<double> p2 = camera.project(vgl_point_3d<double>(inImageSegment.point2().x(), inImageSegment.point2().y(), 0.0));
    vgl_line_segment_2d<double> segment(p1, p2);
    double distance = vgl_distance(proj_ct1, segment);
    if (distance < 5.0) {
        distance = 5.0;
    }
    distance *= 2.0;
    if (distance > 30.0) {
        distance = 30.0;
    }
    
    printf("search distance is %f\n", distance);
        
    return distance;
}

double SoccerModelLeftSideView::nodeMagnitude(int node) const
{
    assert(node >= 0 && node < 28);
    
    double nodeMagnitude[] = {0.05, 0.05, 0.05, 0.05, 0.05,
        0.05, 0.05, 0.05, 0.05, 0.05,
        0.05, 0.05, 0.05, 0.05, 0.05,
        0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
    assert(sizeof(nodeMagnitude)/sizeof(nodeMagnitude[0]) == 28);
    return nodeMagnitude[node];
}

double SoccerModelLeftSideView::nodeLambda(int node) const
{
    assert(node >= 0 && node < 28);
    
    double nodeLambda[] = {2.0, 2.0, 2.0, 2.0, 2.0,
        1.5, 1.5, 1.5, 1.5, 1.5,
        1.5, 1.5, 1.5, 1.5, 1.5,
        2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};
    assert(sizeof(nodeLambda)/sizeof(nodeLambda[0]) == 28);
    return nodeLambda[node];
}

SoccerLineType SoccerModelLeftSideView::nodetype(int node) const
{
    if (node == 21 || node == 22 || node == 23)
    {
        return FarTouchBorder;
    }
    if (node == 24 || node == 25) {
        return RightGoalBorder;
    }
    if (node == 18 || node == 19 || node == 20) {
        return NearTouchBorder;
    }
    return Internal;
}

bool SoccerModelLeftSideView::hasDoubleEdge(const vpgl_perspective_camera<double> & camera, int node) const
{
    assert(node >= 0 && node < long_segments_.size());
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    // line is 5 inch, 0.12 meter width
    vgl_line_segment_2d<double> inImageSegment;  // sampled line segment must be inside image
    bool isSampled = VpglPlus::sampleLineSegment(camera, w, h, long_segments_[node], 0.5, inImageSegment);
    if (!isSampled) {
        return false;
    }
    vgl_point_2d<double> ct = inImageSegment.point_t(0.5);
    // displace center point in the normal direction of the segment
    vgl_vector_2d<double> d1 = inImageSegment.normal() * 0.06;
    vgl_vector_2d<double> d2 = inImageSegment.normal() * (-0.06);
    vgl_point_2d<double> ct1 = ct + d1;
    vgl_point_2d<double> ct2 = ct + d2;
    vgl_point_2d<double> proj_ct1 = camera.project(vgl_point_3d<double>(ct1.x(), ct1.y(), 0.0));
    vgl_point_2d<double> proj_ct2 = camera.project(vgl_point_3d<double>(ct2.x(), ct2.y(), 0.0));
    
    double distance = vgl_distance(proj_ct1, proj_ct2);
    printf("node, half edge line width is %d %f\n", node, distance);
    // considering motion blur
    if (distance >= 2.0) {  // @ hard code parameter
        return true;
    }
    else
    {
        return false;
    }
}

bool SoccerModelLeftSideView::isValidNodePair(int n1, int n2) const
{
    vcl_vector<vcl_pair<int, int> > nodePairs = SoccerModelLeftSideView::nodePairs();
    for (int i = 0; i<nodePairs.size(); i++) {
        if ((n1 == nodePairs[i].first && n2 ==  nodePairs[i].second) ||
            (n1 == nodePairs[i].second && n2 == nodePairs[i].first)) {
            return true;
        }
    }
    return false;
}



/*
vcl_vector<vgl_line_segment_3d<double> > SoccerModelLeftSideView::getWorldLineSegment(double width, double height)
{
    vcl_vector< vgl_line_segment_2d< double > > markings2d(26);
 
    // one center line
    markings2d[0] = vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(0, height/2));
    
    // 18 yard line
    markings2d[1] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) );
    markings2d[2] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) );
    
    // 18 yard line short, horizontal, near end
    markings2d[3] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ) );
    markings2d[4] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ) );
    
    // 6 yard
    markings2d[5] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * (-1), -10 ), vgl_point_2d< double >( (width/2 - 6) * (-1), +10 ) );
    markings2d[6] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * (+1), -10 ), vgl_point_2d< double >( (width/2 - 6) * (+1), +10 ) );
    
    // 18 yard line short, horizontal, far end
    markings2d[7] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) );
    markings2d[8] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) );
    
    // part of goal line (left)
    markings2d[9]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-width/2, -12));
    markings2d[10] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, 12), vgl_point_2d< double >(-width/2, height/2));
    
    // part of goal line (right)
    markings2d[11]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, -12));
    markings2d[12] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, 12), vgl_point_2d< double >(width/2, height/2));
    
    // two field hockey line horizontal
    markings2d[13] = vgl_line_segment_2d<double>(vgl_point_2d<double>(-width/2 + 18, -height/2+11.4), vgl_point_2d<double>(- 10, -height/2+11.4));
    markings2d[14] = vgl_line_segment_2d<double>(vgl_point_2d<double>(10, -height/2+11.4), vgl_point_2d<double>(width/2 - 18, -height/2+11.4));
    
    // part of far touch line
    markings2d[15] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, height/2), vgl_point_2d< double >(-20, height/2));
    markings2d[16] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, height/2), vgl_point_2d< double >(20, height/2));
    markings2d[17] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, height/2), vgl_point_2d< double >(width/2, height/2));
    
    // part of near touch line
    markings2d[18] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-20, -height/2));
    markings2d[19] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, -height/2), vgl_point_2d< double >(20, -height/2));
    markings2d[20] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, -height/2), vgl_point_2d< double >(width/2, -height/2));
    
    // commercial board near far touch line
    const double delta_y = 3.5; // ? estimated may not accurate
    markings2d[21] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2+6, height/2 + delta_y), vgl_point_2d< double >(-20, height/2 + delta_y));
    markings2d[22] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, height/2 + delta_y), vgl_point_2d< double >(20, height/2 + delta_y));
    markings2d[23] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, height/2 + delta_y), vgl_point_2d< double >(width/2-6, height/2 + delta_y));
    
    // line back of right goal, disable them now
    const double delta_x = 6.0;
    markings2d[24]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2 + delta_x, -height/2+10), vgl_point_2d< double >(width/2 + delta_x, -8));
    markings2d[25] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2 + delta_x, 8), vgl_point_2d< double >(width/2 + delta_x, height/2-10));
    
    assert(markings2d.size() == 26);
    vcl_vector<vgl_line_segment_3d<double> > markings3d(29);
    for (int i = 0; i<markings2d.size(); i++) {
        vgl_point_2d<double> p1 = markings2d[i].point1();
        vgl_point_2d<double> p2 = markings2d[i].point2();
        markings3d[i] = vgl_line_segment_3d<double>(vgl_point_3d<double>(p1.x(), p1.y(), 0.0), vgl_point_3d<double>(p2.x(), p2.y(), 0.0));
    }
    
    // right side goal poal
    //
    //    p3----p4
    //----p2    p1----
    vgl_point_3d<double> p1(+width/2.0, -4, 0);
    vgl_point_3d<double> p2(+width/2.0, 4, 0);
    vgl_point_3d<double> p3(+width/2.0,  4, 8.0/3); // 8 foot to yard
    vgl_point_3d<double> p4(+width/2.0, -4, 8.0/3);
    markings3d[26] = vgl_line_segment_3d<double>(p2, p3);
    markings3d[27] = vgl_line_segment_3d<double>(p3, p4);
    markings3d[28] = vgl_line_segment_3d<double>(p4, p1);
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings3d.size(); ++i )
    {
        double x1 = ( (markings3d[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings3d[i].point1().y() + height/2) * 0.9144 );
        double z1 = ( (markings3d[i].point1().z() + 0) * 0.9144 );
        double x2 = ( (markings3d[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings3d[i].point2().y() + height/2) * 0.9144 );
        double z2 = ( (markings3d[i].point2().z() + 0) * 0.9144 );
        
        markings3d[i] = vgl_line_segment_3d< double >( vgl_point_3d< double >( x1, y1, z1 ), vgl_point_3d< double >( x2, y2, z2 ) );
    }

    return markings3d;
}
 */

vcl_vector<vgl_line_segment_2d<double> > SoccerModelLeftSideView::get2DWorldLinesegment(double width, double height)
{
    vcl_vector< vgl_line_segment_2d< double > > markings2d(28);
    
    // one center line
    markings2d[0] = vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(0, height/2));
    
    // 18 yard line
    markings2d[1] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) );
    markings2d[2] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) );
    
    // 18 yard line short, horizontal, near end
    markings2d[3] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ) );
    markings2d[4] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ) );
    
    // 6 yard
    markings2d[5] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * (-1), -10 ), vgl_point_2d< double >( (width/2 - 6) * (-1), +10 ) );
    markings2d[6] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * (+1), -10 ), vgl_point_2d< double >( (width/2 - 6) * (+1), +10 ) );
    
    // 18 yard line short, horizontal, far end
    markings2d[7] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) );
    markings2d[8] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) );
    
    // part of goal line (left)
    markings2d[9]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-width/2, -12));
    markings2d[10] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, 12), vgl_point_2d< double >(-width/2, height/2));
    
    // part of goal line (right)
    markings2d[11]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, -12));
    markings2d[12] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, 12), vgl_point_2d< double >(width/2, height/2));
    
    // two field hockey line horizontal
    markings2d[13] = vgl_line_segment_2d<double>(vgl_point_2d<double>(-width/2 + 18, -height/2+11.4), vgl_point_2d<double>(- 10, -height/2+11.4));
    markings2d[14] = vgl_line_segment_2d<double>(vgl_point_2d<double>(10, -height/2+11.4), vgl_point_2d<double>(width/2 - 18, -height/2+11.4));
    
    // part of far touch line
    markings2d[15] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, height/2), vgl_point_2d< double >(-20, height/2));
    markings2d[16] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, height/2), vgl_point_2d< double >(20, height/2));
    markings2d[17] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, height/2), vgl_point_2d< double >(width/2, height/2));
    
    // part of near touch line
    markings2d[18] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-20, -height/2));
    markings2d[19] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, -height/2), vgl_point_2d< double >(20, -height/2));
    markings2d[20] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, -height/2), vgl_point_2d< double >(width/2, -height/2));
    
    // commercial board near far touch line
    const double delta_y = 3.5; // ? estimated may not accurate
    markings2d[21] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2+6, height/2 + delta_y), vgl_point_2d< double >(-20, height/2 + delta_y));
    markings2d[22] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, height/2 + delta_y), vgl_point_2d< double >(20, height/2 + delta_y));
    markings2d[23] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, height/2 + delta_y), vgl_point_2d< double >(width/2-6, height/2 + delta_y));
    
    // line back of right goal, disable them now
    const double delta_x = 6.0; // 6.0
    markings2d[24] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2 + delta_x, -height/2+10), vgl_point_2d< double >(width/2 + delta_x, -8));
    markings2d[25] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2 + delta_x, 8), vgl_point_2d< double >(width/2 + delta_x, height/2-10));
    
    
    // left 6 yard
    markings2d[26] = vgl_line_segment_2d< double >( vgl_point_2d< double >( -width/2, -10 ), vgl_point_2d< double >( -width/2 + 6, -10 ) );
    markings2d[27] = vgl_line_segment_2d< double >( vgl_point_2d< double >( -width/2, 10 ), vgl_point_2d< double >( -width/2 + 6, 10 ) );
    
    // only use standard lines, and commercial board
    markings2d.resize(28);
    
    /*
    // commercial board near far touch line
    const double delta_y = 3.5; // ? estimated may not accurate
    markings2d[21] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2+6, height/2 + delta_y), vgl_point_2d< double >(-20, height/2 + delta_y));
    markings2d[22] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, height/2 + delta_y), vgl_point_2d< double >(20, height/2 + delta_y));
    markings2d[23] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, height/2 + delta_y), vgl_point_2d< double >(width/2-6, height/2 + delta_y));
    
    // line back of right goal, disable them now
    const double delta_x = 6.0;
    markings2d[24]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2 + delta_x, -height/2+10), vgl_point_2d< double >(width/2 + delta_x, -8));
    markings2d[25] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2 + delta_x, 8), vgl_point_2d< double >(width/2 + delta_x, height/2-10));
     */

    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings2d.size(); ++i )
    {
        double x1 = ( (markings2d[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings2d[i].point1().y() + height/2) * 0.9144 );
        double x2 = ( (markings2d[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings2d[i].point2().y() + height/2) * 0.9144 );
        
        markings2d[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1), vgl_point_2d< double >( x2, y2) );
    }
    
    return markings2d;
}

vcl_vector< vgl_line_segment_2d<double> > SoccerModelLeftSideView::borderLines(double width, double height)
{
    vcl_vector<vgl_line_segment_2d<double> > markings(4);
    
    markings[0] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, height/2), vgl_point_2d< double >(width/2, height/2));   // far touch line
    markings[1] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(width/2, -height/2)); // near touch line
    markings[2] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, height/2));  // right board
    const double delta_y = 3.5; // ? estimated may not accurate
    markings[3] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2+6, height/2 + delta_y), vgl_point_2d< double >(width/2-6, height/2 + delta_y));  // commercial board
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 );
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1), vgl_point_2d< double >( x2, y2) );
    }
    return markings;
}


vcl_vector<vgl_line_segment_2d<double> > SoccerModelLeftSideView::get2DShortLinesegment(double width, double height, double segmentLegnth)
{
    vcl_vector<vgl_line_segment_2d<double> > longSegments = SoccerModelLeftSideView::get2DWorldLinesegment(width, height);
    vcl_vector<vgl_line_segment_2d<double> > shortSegments;
    
    for (int i = 0; i<longSegments.size(); i++) {
        double length = vgl_distance(longSegments[i].point1(), longSegments[i].point2());
        int num = length/segmentLegnth;
        for (int j = 0; j<num; j++) {
            double t1 = 1.0 * j / num;
            double t2 = 1.0 * (j+1)/num;
            vgl_point_2d<double> p1 = longSegments[i].point_t(t1);
            vgl_point_2d<double> p2 = longSegments[i].point_t(t2);
            shortSegments.push_back(vgl_line_segment_2d<double>(p1, p2));
        }
    }
    
    return shortSegments;
}

vcl_vector<vgl_line_3d_2_points<double> > SoccerModelLeftSideView::getWorldLines(double width, double height)
{
    vcl_vector<vgl_line_3d_2_points<double> > lines;
    
    return lines;
}
vcl_vector<vcl_pair<int, int> > SoccerModelLeftSideView::nodePairs()
{
    vcl_vector<vcl_pair<int, int> > pairs;
    int data[][2] = {
        0,16, 0,19, 0,22,
        1,3, 1,13, 1,18, 1,7, 1,15, 1,21, 1,26, 1,27,
        2,4, 2,14, 2,20, 2,8, 2,17, 2,23,
        3,5, 3,9,
        4,6, 4,11,
        5,7, 5,15, 5,21, 5,26,
        7,10,
        8,12, 8,25,
        9,13, 9,18, 9,26,
        10,15, 10,21, 10,27,
        11,14, 11,20,
        12,17, 12,23,
        14,24,
        15,27,
        17,25,
        20,24,
        23,25
        };
    
    int dataNum = sizeof(data)/(sizeof(data[0][0]) * 2);
    for (int i = 0; i< dataNum; i++) {
        pairs.push_back(vcl_pair<int, int> (data[i][0], data[i][1]));
    }   
    
    return pairs;
}
