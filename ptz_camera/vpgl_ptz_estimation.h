//
//  vpgl_ptz_estimation.h
//  QuadCopter
//
//  Created by jimmy on 6/28/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vpgl_ptz_estimation__
#define __QuadCopter__vpgl_ptz_estimation__

// estimate fixed position PTZ camera parameters
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vcl_vector.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <vnl/vnl_matrix_fixed.h>
#include <vgl/vgl_conic.h>

struct PointsOnLine
{
    vcl_vector<vgl_point_2d<double> > pts_;
    vgl_line_3d_2_points<double> line_;
};

struct PointsOnCircle
{
    vgl_conic<double> circle_;        // only circle
    vcl_vector<vgl_point_2d<double> > pts_;    
};

struct LinePointsInCameraview
{
    vcl_vector<PointsOnLine> lines_;
    vcl_vector<PointsOnCircle> circles_;
    vcl_vector<vgl_point_3d<double> > wld_pts_;
    vcl_vector<vgl_point_2d<double> > img_pts_;
    vpgl_perspective_camera<double> camera_;
};

class VpglPTZEstimation
{
public:
    // P = K Q S D
    // initSR: initial stationary rotation matrix
    //  1 0 0; 0 0 -1; 0 1 0 for main PTZ
    //  0 -1 0; 0 0 -1; 1 0 0 for left side view PTZ
    static bool estimateCommomCameraCenterAndStationaryRotation(const vcl_vector< vcl_vector<vgl_point_3d<double> > > & wld_pts,
                                                           const vcl_vector< vcl_vector<vgl_point_2d<double> > > & img_pts,
                                                           const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                           const vnl_matrix_fixed<double, 3, 3> & initSR,
                                                           vgl_rotation_3d<double> & estimatedRs,
                                                           vgl_point_3d<double> & estimatedCameraCenter,
                                                           vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras);
    
    // P = K Q S D
    // initSR: initial stationary rotation matrix
    //  1 0 0; 0 0 -1; 0 1 0 for main PTZ
    //  0 -1 0; 0 0 -1; 1 0 0 for left side view PTZ
    // estimate camera center and stationary rotation by point-point correspondences and point on the line
    // one camera has multiple lines, one line has multiple points
    static bool estimateCommomCameraCenterAndStationaryRotationByPointsOnLines(const vcl_vector<LinePointsInCameraview> & corres,
                                                                               const vnl_matrix_fixed<double, 3, 3> & initSR,
                                                                               vgl_rotation_3d<double> & estimatedRs,
                                                                               vgl_point_3d<double> & estimatedCameraCenter,
                                                                               vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras);
    // estimate ptz from point--point and points on lines
    // assume principle point in the image center and is correct
    // SR: stationary rotation
    static bool camera2PTZByPointsOnLines(const LinePointsInCameraview & corre,
                                          const vgl_point_3d<double> & cc,
                                          const vgl_rotation_3d<double> & SR,
                                          vpgl_perspective_camera<double> & estimatedCamera,
                                          vnl_vector_fixed<double, 3> & ptz, bool verbose = true);
    // 7 degree of freedom
    // may have points on circle
    static bool estimageCamera(const LinePointsInCameraview & corre,
                               vpgl_perspective_camera<double> & estimatedCamera,
                               bool verbose = true);
    
    // locate camera center to specified position
    static bool modifyCameraCenter(const LinePointsInCameraview & corre,
                                   const vgl_point_3d<double> & cc,
                                   vpgl_perspective_camera<double> & estimatedCamera,
                                   bool verbose = true);
    
};

#endif /* defined(__QuadCopter__vpgl_ptz_estimation__) */
