//
//  SoccerGraphCutUtil.cpp
//  QuadCopter
//
//  Created by jimmy on 7/4/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "SoccerGraphCutUtil.h"
#include "vxl_plus.h"
#include <vil/vil_crop.h>
#include <vgl/vgl_distance.h>
#include "vil_plus.h"
//#include "vil_gmm_util.h"
#include "vxl_half_edge_line_tracking.h"


/*************************        SoccerGraphCutUtil            *****************/
SoccerGraphCutUtil::SoccerGraphCutUtil(double w, double h)
{
    field_width_ = w;
    field_heigh_ = h;
}

SoccerGraphCutUtil::~SoccerGraphCutUtil()
{
    
}

vgl_point_2d<double> SoccerGraphCutUtil::intersectionInWorld(int node1, int node2)
{
    vcl_vector<vgl_line_2d<double> > lines = SoccerGraphCutUtil::getWorldLines(field_width_, field_heigh_);
    assert(node1 < lines.size());
    assert(node2 < lines.size());
    
    vgl_point_2d<double> p;
    bool isIntersect = vgl_intersection(lines[node1], lines[node2], p);
    assert(isIntersect);
    return p;
}

bool SoccerGraphCutUtil::intersectionInImage(const vpgl_perspective_camera<double> & camera, int node1, int node2, vgl_point_2d<double> & imgPt)
{
    assert(node1 != node2);
    vgl_point_2d<double> p = this->intersectionInWorld(node1, node2);
    if (camera.is_behind_camera(vgl_homg_point_3d<double>(p.x(), p.y(), 0.0, 1.0))) {
        return false;
    }
    imgPt = camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0));
    return true;
}

bool SoccerGraphCutUtil::intersectionPatch(const vpgl_perspective_camera<double> & camera, int node1, int node2, int patchSize,
                                           const vil_image_view<vxl_byte> & image, vil_image_view<vxl_byte> & patch)
{
    assert(node1 != node2);
    
    vgl_point_2d<double> img_pt;
    bool isOk = this->intersectionInImage(camera, node1, node2, img_pt);
    // patch center in the image
    if (isOk && vgl_inside_image(img_pt, image.ni(), image.nj())) {
        int i0 = img_pt.x() - patchSize/2;
        int j0 = img_pt.y() - patchSize/2;
        // whole patch in the image
        if (i0 >= 0 && j0 >= 0 &&
            i0 + patchSize < image.ni() &&
            j0 + patchSize < image.nj())
        {
            patch = vil_crop(image, i0, patchSize, j0, patchSize);
            return true;
        }
    }
    return false;
}

void SoccerGraphCutUtil::removeWhitePixelAwayFromLines(const vpgl_perspective_camera<double> & camera,
                                                       const vcl_vector<vgl_point_2d<double> > & initialPixels,
                                                       int threshold, vcl_vector<vgl_point_2d<double> > & refinedPixels)
{
    vcl_vector<vgl_line_2d<double> > lines = SoccerGraphCutUtil::getImagelines(camera, field_width_, field_heigh_);
    
    for (int i = 0; i<initialPixels.size(); i++) {
        // loop all lines
        for (int j = 0; j<lines.size(); j++) {
            double dis = vgl_distance(lines[j], initialPixels[i]);
            if (dis < threshold) {
                refinedPixels.push_back(initialPixels[i]);
                break;
            }
        }
    }
}

void SoccerGraphCutUtil::sampleCalibLines(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                          double sampleUnit, vcl_vector<SGCCalibLine> & calibLines)
{
    vcl_vector<vgl_line_segment_2d<double> > wLines = SoccerGraphCutUtil::getWorldLineSegment(field_width_, field_heigh_);
    
    for (int i = 0; i<wLines.size(); i++) {
        vgl_point_3d<double> p1(wLines[i].point1().x(), wLines[i].point1().y(), 0);
        vgl_point_3d<double> p2(wLines[i].point2().x(), wLines[i].point2().y(), 0);
        
        vgl_point_2d<double> q1(-1, -1); // (-1, -1) is the mask for unfined end point
        vgl_point_2d<double> q2(-1, -1);
        vgl_vector_2d<double> dir = wLines[i].direction();
        const double length = vgl_distance(p1, p2);
        
        // sample first point from p1 to p2 in the image
        bool isFound = false;
        for (double j = 0; j <= length; j += sampleUnit) {
            double x = p1.x() + dir.x() * j;
            double y = p1.y() + dir.y() * j;
            
            vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, 0.0));
            if (vgl_inside_image(q, imageW, imageH)) {
                q1 = q;
                isFound = true;
                break;
            }
        }
        if (!isFound) {
            continue;
        }
        isFound = false;
        
        // sample second point from p2 to p1 in the image
        for (double j = length; j >=  0; j -= sampleUnit) {
            double x = p1.x() + dir.x() * j;
            double y = p1.y() + dir.y() * j;
            
            vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, 0.0));
            if (vgl_inside_image(q, imageW, imageH)) {
                q2 = q;
                isFound = true;
                break;
            }
        }
        if (!isFound) {
            continue;
        }
        
        double dis = vgl_distance(q1, q2);
        if (dis > 5 * sampleUnit) {
            SGCCalibLine cLine(i, wLines[i]);
            cLine.setProjectedLine(q1, q2);
            calibLines.push_back(cLine);
        }
    }
}

bool SoccerGraphCutUtil::sampleCalibLine(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, int nodeNum,
                                         double sampleUnit, SGCCalibLine & calibLine)
{
    vcl_vector<vgl_line_segment_2d<double> > wLines = SoccerGraphCutUtil::getWorldLineSegment(field_width_, field_heigh_);
    
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
        if (vgl_inside_image(q, imageW, imageH)) {
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
        if (vgl_inside_image(q, imageW, imageH)) {
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

bool SoccerGraphCutUtil::lineCircleIntersectionPairs(const vpgl_perspective_camera<double> & camera,
                                                     const vgl_point_2d<double> & p1,
                                                     const vgl_point_2d<double> & p2,
                                                     vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > >  & pairs)
{
    vgl_line_segment_2d<double> seg = SoccerGraphCutUtil::getCenterLineCircleIntersection(field_width_, field_heigh_);
    vgl_point_2d<double> p3 = camera.project(vgl_point_3d<double>(seg.point1().x(), seg.point1().y(), 0.0));
    vgl_point_2d<double> p4 = camera.project(vgl_point_3d<double>(seg.point2().x(), seg.point2().y(), 0.0));
    
    double d1 = vgl_distance(p1, p3);
    double d2 = vgl_distance(p2, p4);
    double d3 = vgl_distance(p1, p4);
    double d4 = vgl_distance(p2, p3);
    if (d1 + d2 < d3 + d4) {
        pairs.push_back(vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> >(seg.point1(), p1));
        pairs.push_back(vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> >(seg.point2(), p2));
    }
    else
    {
        pairs.push_back(vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> >(seg.point2(), p1));
        pairs.push_back(vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> >(seg.point1(), p2));
    }
    return true;
}



vcl_vector<vgl_point_2d<double> > SoccerGraphCutUtil::getAllIntersectionWorld()
{
    vcl_vector<vgl_point_2d<double> > pts;
    vcl_vector<vcl_pair<int, int> > nodePairs = SoccerGraphCutUtil::nodePairs();
    for (int i = 0; i<nodePairs.size(); i++) {
        printf("node1 node2 is %d %d\n", nodePairs[i].first, nodePairs[i].second);
        
        vgl_point_2d<double> p;
        p = this->intersectionInWorld(nodePairs[i].first, nodePairs[i].second);
        pts.push_back(p);
    }
    return pts;
}

vcl_vector<vgl_point_2d<double> > SoccerGraphCutUtil::penaltyArcIntersectionWorld()
{
    vcl_vector<vgl_point_2d<double> > pts;
    pts.push_back(vgl_point_2d<double>(18, field_heigh_/2.0 - 8.0));
    pts.push_back(vgl_point_2d<double>(18, field_heigh_/2.0 + 8.0));
    
    pts.push_back(vgl_point_2d<double>(field_width_ - 18, field_heigh_/2.0 - 8.0));
    pts.push_back(vgl_point_2d<double>(field_width_ - 18, field_heigh_/2.0 + 8.0));
    
    // yard to meter
    for ( unsigned int i = 0; i < pts.size(); ++i )
    {
        double x = pts[i].x() * 0.9144;
        double y = pts[i].y() * 0.9144;
        
        pts[i].set(x, y);
    }
    
    
    return pts;
}

bool SoccerGraphCutUtil::isValidNodePair(int n1, int n2) const
{
    vcl_vector<vcl_pair<int, int> > nodePairs = SoccerGraphCutUtil::nodePairs();
    for (int i = 0; i<nodePairs.size(); i++) {
        if ((n1 == nodePairs[i].first && n2 ==  nodePairs[i].second) ||
            (n1 == nodePairs[i].second && n2 == nodePairs[i].first)) {
            return true;
        }
    }
    return false;
}

int SoccerGraphCutUtil::nodeNumber()
{
    vcl_vector<vgl_line_segment_2d<double> > markings = SoccerGraphCutUtil::getWorldLineSegment(field_width_, field_heigh_);
    
    return (int)markings.size();
}

vcl_vector<vcl_pair<int, int> > SoccerGraphCutUtil::nodePairs()
{
    vcl_vector<vcl_pair<int, int> > pairs;
    /* no hockey lines
     int data[][2] = {0,3,0,7,0,4,0,8,
     1,3,1,7,1,9,1,10,
     2,4,2,8,2,9,2,10,
     3,5,4,6,
     5,7,5,9,5,10,
     6,8,6,9,6,10};
     */
    int data[][2] = {0,3,0,7,0,4,0,8,0,11,0,12,
        1,3,1,7,1,9,1,10,
        2,4,2,8,2,9,2,10,
        3,5,4,6,3,13,4,14,
        5,7,5,9,5,10,
        6,8,6,9,6,10,
        11,13,12,14};
    
    int dataNum = sizeof(data)/(sizeof(data[0][0]) * 2);
    for (int i = 0; i< dataNum; i++) {
        pairs.push_back(vcl_pair<int, int> (data[i][0], data[i][1]));
    }
    return pairs;
}

bool SoccerGraphCutUtil::detectLine(const vpgl_perspective_camera<double> & camera, const vil_image_view<vxl_byte> & image,
                                    int node, const AverageMagnitudeParameter & para, const VxlHoughParameter & houghPara,
                                    vgl_line_2d<double> & line)
{
    assert(image.nplanes() == 3);
    SoccerGraphCutUtil sgcUtil;
    if (node < 0 || node >= sgcUtil.nodeNumber()) {
        return false;
    }
    
    const int w = image.ni();
    const int h = image.nj();
    
    SGCCalibLine calibLine;
    bool isSampled = sgcUtil.sampleCalibLine(camera, w, h, node, 0.5, calibLine);  // sample every 0.5 meter
    if (!isSampled) {
        return false;
    }
    
    vil_image_view<double> magnitude;
    VilPlus::vil_magnitude(image, magnitude);
    
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    maskImage.fill(0);
    VilPlus::draw_segment(maskImage, calibLine.point1(), calibLine.point2(), VilPlus::white(), 10);
    
    
    // detect edge point candidate
    vcl_vector<vgl_point_2d<double> > side_edge_pts1;
    vcl_vector<vgl_point_2d<double> > side_edge_pts2;
    AverageMagnitudeParameter avgMag = para;
    avgMag.direction_ = calibLine.scan_direction();
    vxl_half_edge_line_tracking::detectLinePixelByAverageGradient(magnitude, maskImage, avgMag, side_edge_pts1, side_edge_pts2);
    
    // hough line detection for two edge line
    vgl_line_2d<double> dumpLine;
    vcl_vector<vgl_point_2d<double> > inlier_pts1;
    vcl_vector<vgl_point_2d<double> > inlier_pts2;
    VxlHoughLine::detectOneLine(side_edge_pts1, w, h, houghPara, dumpLine, inlier_pts1);
    VxlHoughLine::detectOneLine(side_edge_pts2, w, h, houghPara, dumpLine, inlier_pts2);
    
    // estimate center line from edge line
    vcl_vector<vgl_point_2d<double> > center_pts;
    bool isFindCenterLine = false;
    if (inlier_pts1.size() > inlier_pts2.size()) {
        isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts1, inlier_pts2, center_pts, line);
    }
    else
    {
        isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts2, inlier_pts1, center_pts, line);
    }
    
    return isFindCenterLine;
}

vcl_vector<vgl_line_segment_2d<double> > SoccerGraphCutUtil::getWorldLineSegment(double width, double height)
{
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // one center line
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(0, height/2)));
    
    // 18 yard line
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) ) );
    
    // 18 yard line short, horizontal, near end
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ) ) );
    
    // two goal lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-width/2, height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, height/2)));
    
    // 18 yard line short, horizontal, far end
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) ) );
    
    // two touch lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, height/2), vgl_point_2d< double >(width/2, height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(width/2, -height/2)));
    
    // two field hockey line horizontal
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(-width/2, -height/2+11.4), vgl_point_2d<double>(- 10, -height/2+11.4)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(10, -height/2+11.4), vgl_point_2d<double>(width/2, -height/2+11.4)));
    
    // two field hockey line vertical
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(-10, -height/2+11.4), vgl_point_2d<double>(-10, -10)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(10,  -height/2+11.4), vgl_point_2d<double>(10, -10)));
    
    // field hockey points
    //    pts.push_back(vgl_point_2d<double>(width/2 - 10, 11.4));
    //    pts.push_back(vgl_point_2d<double>(width/2 + 10, 11.4));
    
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

vcl_vector<vgl_line_2d<double> > SoccerGraphCutUtil::getWorldLines(double width, double height)
{
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerGraphCutUtil::getWorldLineSegment(width, height);
    
    // end points to lines
    vcl_vector<vgl_line_2d<double> >  lines;
    for (int i = 0; i<markings.size(); i++) {
        vgl_line_2d<double> line(markings[i].point1(), markings[i].point2());
        lines.push_back(line);
    }
    return lines;
}

vcl_vector<vgl_line_2d<double> > SoccerGraphCutUtil::getImagelines(const vpgl_perspective_camera<double> & camera, double width, double height)
{
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerGraphCutUtil::getWorldLineSegment(width, height);
    
    vcl_vector<vgl_line_2d<double> >  lines;
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> p1 = markings[i].point1();
        vgl_point_2d<double> p2 = markings[i].point2();
        // @todo, may miss near end board line
        if (!camera.is_behind_camera(vgl_homg_point_3d<double>(p1.x(), p1.y(), 0.0, 1)) &&
            !camera.is_behind_camera(vgl_homg_point_3d<double>(p2.x(), p2.y(), 0.0, 1))) {
            vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(p1.x(), p1.y(), 0.0));
            vgl_point_2d<double> q2 = camera.project(vgl_point_3d<double>(p2.x(), p2.y(), 0.0));
            
            vgl_line_2d<double> line(q1, q2);
            lines.push_back(line);
        }
    }
    return lines;
}

vgl_line_segment_2d<double> SoccerGraphCutUtil::getCenterLineCircleIntersection(double width, double height)
{
    double yard_to_meter = 0.9144;
    double x = width/2.0;
    double y = height/2.0 + 10;
    x *= yard_to_meter;
    y *= yard_to_meter;
    
    vgl_point_2d<double> p1(x, y);
    
    x = width/2.0;
    y = height/2.0 - 10;
    x *= yard_to_meter;
    y *= yard_to_meter;
    
    vgl_point_2d<double> p2(x, y);
    
    return vgl_line_segment_2d<double>(p1, p2);
}
