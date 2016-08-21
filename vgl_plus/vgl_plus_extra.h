//
//  vgl_plus_extra.h
//  Relocalization
//
//  Created by jimmy on 2016-08-14.
//  Copyright (c) 2016 Nowhere Planet. All rights reserved.
//

#ifndef __Relocalization__vgl_plus_extra__
#define __Relocalization__vgl_plus_extra__

#include <stdio.h>
#include <vgl/vgl_ellipse_2d.h>

class VglPlusExtra
{
public:
    // 0, 1, 2 for 0, 1, 2 intersection point
    static int lineEllipseIntersection(const vgl_line_2d<double> & line, const vgl_ellipse_2d<double> & ellipse,
                                       vgl_point_2d<double> & pt1, vgl_point_2d<double> & pt2, bool isMajorAxis = true);
    
    // sample point in an ellipse
    static int sampleElliseInImage(const vgl_ellipse_2d<double> & ellipse, int sampleNum, int imageW, int imageH, vcl_vector<vgl_point_2d<double> > & pts);
    
    // sample points in the ellipse, chose the one closes to the line
    static bool lineElliseTangency(const vgl_line_2d<double> & line, const vgl_ellipse_2d<double> & ellipse, vgl_point_2d<double> & pt,
                                   double disThreshold = 0.5, int sampleNum = 400);
    
    static bool lineEllipseTangencyByProjection(int imageW, int imageH,
                                                const vgl_line_2d<double> & line,
                                                const vgl_ellipse_2d<double> & ellipseInImage,
                                                vgl_point_2d<double> & pt, int sampleNum = 400);
    
    // distanceThreshold: minimum distance a pixel to an ellipse
    static bool mergeEllipse(const vcl_vector<vgl_ellipse_2d<double> > & ellipse, const vcl_vector< vcl_vector<vgl_point_2d<double> > > & ellipsePts,
                             const double distanceThreshold,
                             vcl_vector<vgl_ellipse_2d<double> > & merged_ellipse, vcl_vector<vcl_vector<vgl_point_2d<double> > > & merged_points);
    
    
    // 0, 1, 2 intersections
    // points is in pixel. distance < 1.0 ---> on the line
    static vcl_vector<vgl_point_2d<double> > intersection(const vgl_ellipse_2d<double> & ellipse, const vgl_line_segment_2d<double> & seg);
    
    // distance_threshold: minimum distance a pixel to a line
    static bool mergeLinesegments(const vcl_vector<vgl_line_segment_2d<double> > & segments,
                                  const vcl_vector<vcl_vector<vgl_point_2d<double> > > & segment_pts,
                                  const double distance_threshold,
                                  vcl_vector<vgl_line_segment_2d<double> > & merged_segments,
                                  vcl_vector<vcl_vector<vgl_point_2d<double> > > & merged_pts);

    
};

#endif /* defined(__Relocalization__vgl_plus_extra__) */
