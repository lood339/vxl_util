//
//  vgl_plus.h
//  CameraPlaning
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __CameraPlaning__vgl_plus__
#define __CameraPlaning__vgl_plus__

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>
#include <vcl_string.h>
#include <vgl/algo/vgl_h_matrix_2d.h>


class VglPlus
{
public:
    static inline bool vgl_inside_image(const vgl_point_2d<double> &p, int width, int height)
    {
        return p.x() >= 0 && p.x() < width && p.y() >= 0 && p.y() < height;
    }
    
    static inline bool is_inside_image(const int x, const int y, const int width, const int height)
    {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
    
    
    static inline bool vgl_inside_image(const vgl_point_2d<double> &p, int width, int height, int threshold)
    {
        // assert(threshold > 0);
        return p.x() >= (0 - threshold) && p.x() < (width + threshold) && p.y() >= (0 - threshold) && p.y() < (height + threshold);
    }
    
    static inline bool vgl_inside_rect(const vgl_point_2d<double> &p, int min_x, int min_y, int max_x, int max_y)
    {
        assert(min_x < max_x);
        assert(min_y < max_y);
        
        return p.x() >= min_x && p.x() < max_x && p.y() >= min_y && p.y() < max_y;
    }

    static void savePointVector(const char *fileName, vcl_vector<vgl_point_2d<double>> &pts);
    static void loadPointVector(const char *fileName, vcl_vector<vgl_point_2d<double>> &pts, int pts_num);
    static void saveMatrix(const char *fileName, const vnl_matrix<double> & data);
    static void loadMatrix(const char *fileName, vnl_matrix<double> &data);
    static void saveVector(const char *matName, const vcl_vector<double> & data);
    
    static void savePointVector(const char *fileName, const char *imageName, const vcl_vector<vgl_point_2d<double>> &wldPts, const vcl_vector< vgl_point_2d<double> > &imgPts);
    static void loadPointVector(const char *fileName, vcl_string &imageName, vcl_vector<vgl_point_2d<double>> &wldPts, vcl_vector< vgl_point_2d<double> > &imgPts);
    
    // generate vertex of a simple box, line 0123, 4567 04 ... 37
    static void generateBox(const vgl_point_3d<double> & center, const vnl_vector_fixed<double, 3> & xyz_size, vcl_vector<vgl_point_3d<double> > & vertex);
    
    // fing smallest distance between two group pts
    // matchedIndex: matched index for pts1 from pts2
    static bool uniqueMutualMatch(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2, vcl_vector<int> & matchedIndex);
    
    static vgl_h_matrix_2d<double> homography(const vcl_vector<vgl_point_2d<double> > &from_pts, const vcl_vector<vgl_point_2d<double> > &to_pts);
    
    // orthocenter (average position) of all intersection
    static bool intersectionFromLineSet(const vcl_vector<vgl_line_2d<double> > & lines, vgl_point_2d<double> & orthocenter);
    
    
    // paralle move 2d line segment
    static void parallelMove(const vgl_line_segment_2d<double> & initSeg, double distance,
                             vgl_line_segment_2d<double> & seg1, vgl_line_segment_2d<double> & seg2);
    
    // seg1 and seg2 is exactly on the same line
    static bool mergeTwolineSegmentOnALine(const vgl_line_segment_2d<double> & seg1,
                                           const vgl_line_segment_2d<double> & seg2,
                                           vgl_line_segment_2d<double> & merged_seg);
    //   3 2 1
    //   4 * 0
    //   5 6 7
    // discrete direction from p1 to p2
    static int octaveDirection(double dx, double dy);
    
    // is "p" on the left side of the segment
    static bool isLeftSide(const vgl_line_segment_2d<double> & seg, const vgl_point_2d<double> & p);
    
    
    
        
    // clamp the box in a range
    static void clamp_box(vgl_box_2d<double> & box, double min_x, double min_y, double max_x, double max_y);
    
};


#endif /* defined(__CameraPlaning__vgl_plus__) */
