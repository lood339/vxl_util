//
//  SoccerGraphCutUtil.h
//  QuadCopter
//
//  Created by jimmy on 7/4/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__SoccerGraphCutUtil__
#define __QuadCopter__SoccerGraphCutUtil__

#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_2d.h>
#include <vcl_vector.h>
#include <vcl_map.h>
#include <vcl_utility.h>

#include <vil/vil_image_view.h>
#include <vgl/vgl_line_2d.h>
#include <vgl/vgl_point_2d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vgl/vgl_intersection.h>
#include "vxl_hough_line.h"
#include "SoccerLineParameters.h"
#include "SoccerModelParameters.h"




// generate edge and intersections
// node: line or circle in image
// intersection: intersection between node (lines or circles)
class SoccerGraphCutUtil
{
private:
    double field_width_;     // unit is yard
    double field_heigh_;
public:
    SoccerGraphCutUtil(double w = 118, double h = 70);
    ~SoccerGraphCutUtil();
    
    vgl_point_2d<double> intersectionInWorld(int node1, int node2);
    // imgPt may out of image
    bool intersectionInImage(const vpgl_perspective_camera<double> & camera, int node1, int node2, vgl_point_2d<double> & imgPt);
    bool intersectionPatch(const vpgl_perspective_camera<double> & camera, int node1, int node2, int patchSize,
                           const vil_image_view<vxl_byte> & image, vil_image_view<vxl_byte> & patch);
    // threshold: 5-10
    // pixels that are away from the line sets will be removed
    void removeWhitePixelAwayFromLines(const vpgl_perspective_camera<double> & camera, const vcl_vector<vgl_point_2d<double> > & initialPixels,
                                       int threshold, vcl_vector<vgl_point_2d<double> > & refinedPixels);
    // sample white line from the court
    // sampleUnit: meter
    // dense sample: sampleUnit 0.5 meter
    void sampleCalibLines(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                          double sampleUnit, vcl_vector<SGCCalibLine> & calibLines);
    
    // sample one calib line in the image
    bool sampleCalibLine(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, int nodeNum,
                         double sampleUnit, SGCCalibLine & calibLine);
    
    // pairs: <world_pt, image_pt>
    // p1, p2: two intersections from center line to circle, but do not know which one is left/right
    // output: paired line-circle intersection
    bool lineCircleIntersectionPairs(const vpgl_perspective_camera<double> & camera,
                                     const vgl_point_2d<double> & p1,
                                     const vgl_point_2d<double> & p2,
                                     vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > >  & pairs);
    
    
    // all intersection point in world coordinate
    vcl_vector<vgl_point_2d<double> > getAllIntersectionWorld();
    // penalty arc with line
    vcl_vector<vgl_point_2d<double> > penaltyArcIntersectionWorld();
    // node pairs that intersect
    bool isValidNodePair(int node1, int node2) const;
    int nodeNumber();
    static vcl_vector<vcl_pair<int, int> > nodePairs();
    
    // detect a center by node number
    static bool detectLine(const vpgl_perspective_camera<double> & camera, const vil_image_view<vxl_byte> & image,
                           int node, const AverageMagnitudeParameter & edgePointPara, const VxlHoughParameter & houghPara,
                           vgl_line_2d<double> & line);
private:
    static vcl_vector<vgl_line_segment_2d<double> > getWorldLineSegment(double width, double height);
    static vcl_vector<vgl_line_2d<double> > getWorldLines(double width, double height);
    static vcl_vector<vgl_line_2d<double> > getImagelines(const vpgl_perspective_camera<double> & camera, double width, double height);
    static vgl_line_segment_2d<double> getCenterLineCircleIntersection(double width, double height);
};


#endif /* defined(__QuadCopter__SoccerGraphCutUtil__) */
