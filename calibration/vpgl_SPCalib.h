//
//  vpgl_SPCalib.h
//  QuadCopter
//
//  Created by jimmy on 6/23/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vpgl_SPCalib__
#define __QuadCopter__vpgl_SPCalib__

// implemenat paper "Method for pan-tilt camera calibration using single control point" 2015
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_vector_fixed.h>

// single point (SP) calibration
class VpglSPCalib
{
public:
    // return: number of solution, have upto 2 solutions
    // ambiguity in solution: tilt and pan angle may have same tan(angle) value but different sin(angle) value.
    // wld_p: world coordinate point in the camera coordinate. wld_p - camera_center
    // cam_p: camera coordinate = K^-(1) * p_img
    // all the input is in camera coordinate
    static int estimatePanTilt(const vgl_point_3d<double> &wld_p_in_camera_coordinate, const vgl_point_2d<double> & cam_p,
                               double & pan1, double & tilt1, double & pan2, double & tilt2);
    
    // pan respect to Y axis and tilt respect to X axis.
    // both wldPt, projectPt is in camera coordinate.
    // P --> camera coordinate --> only Pan then tilt --> camera coordinate --> projected by camera matrix K
    static int estimateYPanXTilt(const vgl_point_3d<double> &wldPt, const vgl_point_2d<double> & projectPt,
                                 vnl_vector_fixed<double, 4> & panTilts);
    
    // wldPt    : P --> camera coordinate
    // projectPt: its projection in the image space
    // fl: focal length
    static int estimateFocalLength(const vgl_point_3d<double> & wldPt, const vgl_point_2d<double> & projectPt,
                                   const vgl_point_2d<double> & principlePoint, double & fl);
    
    // estimate focal length by multiple points
    static bool estimateFocalLength(const vcl_vector<vgl_point_3d<double> > & wldPts, const vcl_vector<vgl_point_2d<double> > & projectPts,
                                    const vgl_point_2d<double> & pp, const double & init_fl, double & fl);
    
};

#endif /* defined(__QuadCopter__vpgl_SPCalib__) */
