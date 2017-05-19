//
//  vpgl_plus.h
//  FinalCalib
//
//  Created by jimmy on 12/31/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __FinalCalib__vpgl_plus__
#define __FinalCalib__vpgl_plus__

#include <vpgl/vpgl_perspective_camera.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_line_2d.h>
#include <vgl/vgl_conic.h>

// 4 * 4 colum major matrix
struct OpenglCamera
{
    vnl_matrix<double> model_view_;
    vnl_matrix<double> proj_;
};

class VpglPlus
{
public:
    //re-calibrate camera by adding lines
    //initCamera: camera model calibrated by other method, points correspondeces
    //
    static vpgl_perspective_camera< double >
    opt_natural( const vpgl_perspective_camera< double >& initCamera,
                const vcl_vector< vgl_infinite_line_3d< double > >& world_lines,
                const vcl_vector< vgl_line_segment_2d< double > >& image_lines);
    
    // area between the line and the link of two points
    static double vpgl_line_linesegment_area(const vgl_line_2d<double> &line, const vgl_point_2d<double> &p1, const vgl_point_2d<double> &p2);
    
    // focal length from two orthogonal vanishing points
    static bool focal_length(const vgl_point_2d<double> vp1, const vgl_point_2d<double> & vp2, double & fl);
    
   
    
    static bool init_calib(const vcl_vector<vgl_point_2d<double> > &wldPts,
                           const vcl_vector<vgl_point_2d<double> > &imgPts,
                           const vgl_point_2d<double> &principlePoint,
                           double focalLength,
                           vpgl_perspective_camera<double> &camera);
    
    
    
    // distance_threshod: projection error in pixel
    static bool optimize_camera_by_inliers(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                           const vcl_vector<vgl_point_2d<double> > & imgPts,
                                           const vpgl_perspective_camera<double> & initCamera,
                                           const double distance_threshod,
                                           vpgl_perspective_camera<double> & finalCamera);
    
    // optimize perspective_camera by minimizing projected distance
    //     wldPts: z = 0, >= 5 points, the more the better
    // initCamera: camera from algebra calibration
    static bool optimize_perspective_camera(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                            const vcl_vector<vgl_point_2d<double> > & imgPts,
                                            const vpgl_perspective_camera<double> & initCamera,
                                            vpgl_perspective_camera<double> & finalCamera);
    
    // iterated closest points (ICP) on the line to optimize the camera parameter
    static bool optimize_perspective_camera_ICP(const vcl_vector<vgl_point_2d<double> > &wldPts,
                                                const vcl_vector<vgl_point_2d<double> > &imgPts,
                                                const vcl_vector<vgl_line_3d_2_points<double> > & wldLines,
                                                const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgLinePts,
                                                const vpgl_perspective_camera<double> & initCamera,
                                                vpgl_perspective_camera<double> &camera);
    
    // iterative closest points on the conic to optimize the camera parameter
    // assume the conic on the z = 0 plane, only for circle
    static bool optimize_perspective_camera_ICP(const vcl_vector<vgl_point_2d<double> > &wldPts,
                                                const vcl_vector<vgl_point_2d<double> > &imgPts,
                                                const vcl_vector<vgl_conic<double> > & wldConics,
                                                const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgConicPts,
                                                const vpgl_perspective_camera<double> & initCamera,
                                                vpgl_perspective_camera<double> &camera);
    
    // iterative cloesest points (ICP) by line segments and conic (only circle)
    static bool optimize_perspective_camera_ICP(const vcl_vector<vgl_point_2d<double> > &wldPts,
                                                const vcl_vector<vgl_point_2d<double> > &imgPts,
                                                const vcl_vector<vgl_line_3d_2_points<double> > & wldLines,
                                                const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgLinePts,
                                                const vcl_vector<vgl_conic<double> > & wldConics,
                                                const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgConicPts,
                                                const vpgl_perspective_camera<double> & initCamera,
                                                vpgl_perspective_camera<double> &camera);
    
    // refine camera by minimize re-projection error
    static bool optimize_perspective_camera(const vcl_vector<vgl_point_3d<double> > & wldPts,
                                            const vcl_vector<vgl_point_2d<double> > & imgPts,
                                            const vpgl_perspective_camera<double> & initCamera,
                                            vpgl_perspective_camera<double> & finalCamera);
    
    // project a conic by H, assume conic is a circle
    static vgl_conic<double> projectConic(const vnl_matrix_fixed<double, 3, 3> & H, const vgl_conic<double> & conic);
    
    // init_segment: world coordinate, meter
    // sampleUnit: meter
    // sampled_segment: meter
    static bool sampleLineSegment(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                  const vgl_line_segment_2d<double> & init_segment, double sampleUnit,
                                  vgl_line_segment_2d<double> & sampled_segment);
    // 3D version
    static bool sampleLineSegment(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                  const vgl_line_segment_3d<double> & init_segment, double sampleUnit,
                                  vgl_line_segment_3d<double> & sampled_segment);
    
    // project line width in world coordinate to image space
    // lineWidth: meter in world coordinate
    static double projectLineWidth(const vpgl_perspective_camera<double> & camera,
                                   const vgl_line_segment_2d<double> & segment,
                                   double lineWidth,
                                   vgl_line_segment_2d<double> & projectedNorm);
    
    static double projectLineWidth(const vpgl_perspective_camera<double> & camera,
                                   const vgl_line_segment_2d<double> & segment,
                                   double lineWidth);    
    
    
    // homography for z = 0;
    static vnl_matrix_fixed<double, 3, 3> homographyFromProjectiveCamera(const vpgl_perspective_camera<double> & camera);
    
    static void matrixFromPanYTiltX(double pan, double tilt, vnl_matrix_fixed<double, 3, 3> &m);
    static void matrixFromPanYTiltX(double pan, double tilt, vnl_matrix_fixed<double, 4, 4> &m);
    static vnl_matrix<double> matrixPanY(double pan);
    static vnl_matrix<double> matrixTiltX(double tilt);
    
    // pp: principle point
    // pp_ptz: image center PTZ,
    // p2    : an image pixel position
    // p2_ptz: PTZ parameter of p2. two points are from the same image
    static void panYtiltXFromPrinciplePoint(const vgl_point_2d<double> & pp,
                                            const vnl_vector_fixed<double, 3> & pp_ptz,
                                            const vgl_point_2d<double> & p2,
                                            vnl_vector_fixed<double, 3> & p2_ptz);
    
    // two points can be from different image
    static void panYtiltXFromPrinciplePointEncodeFocalLength(const vgl_point_2d<double> & pp,
                                                             const vnl_vector_fixed<double, 3> & pp_ptz,
                                                             const vgl_point_2d<double> & p2,
                                                             vnl_vector_fixed<double, 3> & p2_ptz);
    
    // pan, tilt, focal length from reference point
    // two points are from the same image
    static void panTiltFromReferencePoint(const vgl_point_2d<double> & reference_point,
                                          const vnl_vector_fixed<double, 3> & reference_ptz,
                                          const vgl_point_2d<double> & principle_point,
                                          vnl_vector_fixed<double, 3> & ptz);
    
    // two points can be from different images
    static void panTiltFromReferencePointDecodeFocalLength(const vgl_point_2d<double> & reference_point,
                                                           const vnl_vector_fixed<double, 3> & reference_ptz,
                                                           const vgl_point_2d<double> & principle_point,
                                                           vnl_vector_fixed<double, 3> & ptz);
    
    
    static void matrixFromRotationCameraCenter(const vnl_matrix_fixed<double, 3, 3> &rot,
                                               const vgl_point_3d<double> &cameraCenter,
                                               vnl_matrix_fixed<double, 4, 4> &outMatrix);
    
    // euler angle --> rotation matrix, y -> z -> x
    // pan, rotate, tilt
    static vnl_matrix<double> eularToRotation(const double y_angle, const double z_angle, const double x_angle);
    
    // P = K*C*Q*S*D -> perspectice camera
    // cameraCenter: rotation center
    // rod:
    static bool decomposeCameraPTZ(double fl, const vnl_vector_fixed<double, 6> & coeff,
                                   const vgl_point_2d<double> & principlePoint,
                                   double pan, double tilt,
                                   const vnl_vector_fixed<double, 3> & rod,
                                   const vgl_point_3d<double> & cameraCenter,
                                   vpgl_perspective_camera<double> & camera);
    
    // compose PTZ camera by default parameters
    static bool decomposeCameraPTZ(double fl, double pan, double tilt, vpgl_perspective_camera<double> & camera);
    
       
    
        
    // vxl camera to opengl Camera
    // scale = display image size /original image size, like 0.5
    static void cameraToOpenGLCamera(const vpgl_perspective_camera<double> & camera, OpenglCamera & glCamera, double scale);
    
    // assume the point is in z = 0 planar
    static vgl_h_matrix_2d<double> cameraViewToWorld(const vpgl_perspective_camera<double> & camera);
    
    // assume two camera are located in the same place
    // return: map from image point in cameraA to image point im camera B
    // assume two camera have same location (only rotate)
    static vgl_h_matrix_2d<double> homographyFromCameraToCamera(const vpgl_perspective_camera<double> & cameraA,
                                                                const vpgl_perspective_camera<double> & cameraB);
    
    // pure rotation camera calibration
    // assume camera1 and camera2 has similar pose
    // x2 = K_2 * R_2 * R_1^{-1} * K_1^{-1} x1
    static bool calibrate_pure_rotate_camera(const vcl_vector<vgl_point_2d<double> > & pts1,
                                             const vcl_vector<vgl_point_2d<double> > & pts2,
                                             const vpgl_perspective_camera<double> & camera1,
                                             vpgl_perspective_camera<double> & camera2);
    
    // intrinsic parameter K from projection matrix
    // the result is initial
    static bool KfromP(const vnl_matrix_fixed<double, 3, 4> & P, const vgl_point_2d<double> & pp, vnl_matrix_fixed<double, 3, 3> & K);
    
    // fsign: focal length sign
    
    static bool KRTfromP(const vnl_matrix_fixed<double, 3, 4> &P,
                         vnl_matrix_fixed<double, 3, 3> & outK,
                         vnl_matrix_fixed<double, 3, 3> & outR,
                         vnl_vector_fixed<double, 3> & outT,
                         double fsign = 1.0);
    
    // rectify image from left, right projection matrix
    static bool rectifyStereoImage(const vpgl_perspective_camera<double> & leftCamera, const vpgl_perspective_camera<double> & rightCamera,
                                   const vgl_point_2d<double> & d1, const vgl_point_2d<double> & d2,
                                   vnl_matrix_fixed<double, 3, 4> & leftProj, vnl_matrix_fixed<double, 3, 4> & rightProj,
                                   vnl_matrix_fixed<double, 3, 3> & leftTransform, vnl_matrix_fixed<double, 3, 3> & rightTransform);
    
    // maximum number of projected "wld_pts" to img_pts
    // 
    static int unique_match_number(const vpgl_perspective_camera<double> & camera,
                                   const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                   const vcl_vector<vgl_point_2d<double> > & img_pts,
                                   double inlier_distance);
    
    
    static bool writeCamera(const char *fileName, const char *imageName, const vpgl_perspective_camera<double> & camera);
    static bool readCamera(const char *fileName, vcl_string & imageName, vpgl_perspective_camera<double> & camera);
    
    static bool readPTZCamera(const char *fileName, vcl_string &imageName, vnl_vector_fixed<double, 3> &ptz,
                              vgl_point_2d<double> & pp,
                              vgl_point_3d<double> & cc,
                              vnl_vector_fixed<double, 3> & rod);
    
    static bool writePTZCamera(const char *fileName, const char *imageName, const vnl_vector_fixed<double, 3> &ptz,
                               const vgl_point_2d<double> & pp,
                               const vgl_point_3d<double> & cc,
                               const vnl_vector_fixed<double, 3> & rod);
    
private:
    // alpha, beta: fl_x, fl_y
    // u0, v0     : principle point
    // near_clip < 0.0 (?)
    // projection: column major matrix
    static vnl_matrix<double> opengl_projection_from_intrinsics(
                                                  double alpha, double beta,
                                                  double skew,
                                                  double u0, double v0,
                                                  double near_clip, double far_clip );
    
    
};


#endif /* defined(__FinalCalib__vpgl_plus__) */
