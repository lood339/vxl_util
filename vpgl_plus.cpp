//
//  vpgl_plus.cpp
//  FinalCalib
//
//  Created by jimmy on 12/31/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vpgl_plus.h"
#include <vgl/vgl_intersection.h>
#include <vgl/vgl_distance.h>

#include <vpgl/algo/vpgl_camera_compute.h>
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>

#include <vgl/algo/vgl_homg_operators_2d.h>
#include <vcl_algorithm.h>
#include <vgl/algo/vgl_h_matrix_2d_compute_linear.h>
#include <vgl/algo/vgl_h_matrix_3d.h>
#include <vnl/vnl_double_3.h>
#include <vnl/vnl_det.h>
#include <vnl/vnl_inverse.h>
#include <vpgl/algo/vpgl_optimize_camera.h>
#include <vnl/algo/vnl_cholesky.h>
#include <vnl/algo/vnl_matrix_inverse.h>
#include <vnl/algo/vnl_qr.h>
#include <vnl/algo/vnl_determinant.h>
#include <vnl/vnl_rank.h>



// minimize area between line in image and the projection of "ideal line" in the image
// from paper "Using line and ellipse features for rectification of broadcase hockey video"
class vpgl_line_area_residual: public vnl_least_squares_function
{
    protected:
    const vcl_vector< vgl_infinite_line_3d< double > > world_lines_;
    const vcl_vector< vgl_line_segment_2d< double > > image_lines_;
    const vgl_point_2d<double> principle_point_;
    
    public:
    vpgl_line_area_residual(const vcl_vector< vgl_infinite_line_3d< double > > &world_lines, const vcl_vector< vgl_line_segment_2d< double > > &image_lines,
                            const vgl_point_2d<double> &pp):
    vnl_least_squares_function(7, (unsigned int)world_lines.size(), no_gradient),
    world_lines_(world_lines),
    image_lines_(image_lines),
    principle_point_(pp)
    {
        assert(world_lines.size() == image_lines.size());
        assert(world_lines.size() >= 7);
    }
    
    double area(const vgl_line_2d<double> &line, const vgl_point_2d<double> &p1, const vgl_point_2d<double> &p2) const
    {
        return  VpglPlus::vpgl_line_linesegment_area(line, p1, p2);
    }
    
    void f(const vnl_vector<double> &x, vnl_vector<double> &fx)
    {
        //construct camera
        vpgl_perspective_camera<double> camera;
        camera.set_calibration(vpgl_calibration_matrix<double>(x[0], principle_point_));
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        camera.set_rotation(vgl_rotation_3d<double>(rod));
        camera.set_camera_center(vgl_point_3d<double>(x[4], x[5], x[6]));
        
        //project world_lines to the image
        for (int i = 0; i<world_lines_.size(); i++) {
            vgl_line_2d<double> projLine = camera.project(world_lines_[i]);
            
            //calculate areas between lines
            fx[i] = this->area(projLine, image_lines_[i].point1(), image_lines_[i].point2());
        }
    }
};


vpgl_perspective_camera< double >
VpglPlus::opt_natural( const vpgl_perspective_camera< double >& initCamera,
                      const vcl_vector< vgl_infinite_line_3d< double > >& world_lines,
                      const vcl_vector< vgl_line_segment_2d< double > >& image_lines)

{
    vpgl_perspective_camera< double > opt_camera = initCamera;
    
    vgl_point_2d<double> principlePoint = initCamera.get_calibration().principal_point();
    
    vpgl_line_area_residual residual(world_lines, image_lines, principlePoint);
    
    //init parameters
    vnl_vector<double> x(7, 0.0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    x[1] = initCamera.get_rotation().as_rodrigues()[0];
    x[2] = initCamera.get_rotation().as_rodrigues()[1];
    x[3] = initCamera.get_rotation().as_rodrigues()[2];
    x[4] = initCamera.get_camera_center().x();
    x[5] = initCamera.get_camera_center().y();
    x[6] = initCamera.get_camera_center().z();
    
    vnl_levenberg_marquardt lmq(residual);
    
    vcl_cout<<"before opt...\n"<<x<<vcl_endl;
    bool isMinimizeOk = lmq.minimize(x);
    vcl_cout<<"after opt...\n"<<x<<vcl_endl;
    lmq.diagnose_outcome();
    
    vpgl_calibration_matrix<double> K(x[0], principlePoint);
    opt_camera.set_calibration(K);
    
    vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
    opt_camera.set_rotation(vgl_rotation_3d<double>(rod));
    opt_camera.set_camera_center(vgl_point_3d<double>(x[4], x[5], x[6]));
    
    if (!isMinimizeOk) {
        vcl_cout<<"Warning: camera optimization failed.\n";
    }
    
    return opt_camera;
}


double VpglPlus::vpgl_line_linesegment_area(const vgl_line_2d<double> &line, const vgl_point_2d<double> &p1, const vgl_point_2d<double> &p2)
{
    double ret = 0.0;
    //check if two points are on the same side
    double a = line.a();
    double b = line.b();
    double c = line.c();
    
    //construct two lines perpendicular with line and cross p1, p2
    double a1 = b;
    double b1 = -a;
    double c1 = - (a1 * p1.x() + b1 * p1.y());
    double c2 = - (a1 * p2.x() + b1 * p2.y());
    
    vgl_line_2d<double> line1(a1, b1, c1);
    vgl_line_2d<double> line2(a1, b1, c2);
    
    vgl_point_2d<double> perpendicular1;
    vgl_point_2d<double> perpendicular2;
    
    //intersect with line
    bool isIntersect1 = vgl_intersection(line, line1, perpendicular1);
    bool isIntersect2 = vgl_intersection(line, line2, perpendicular2);
    
    if (isIntersect1 && isIntersect2) {
        double s1 = a * p1.x() + b * p1.y() + c;
        double s2 = a * p2.x() + b * p2.y() + c;
        
        if (s1 * s2 >= 0)
        {
            //same side
            double d1 = vgl_distance(perpendicular1, p1);
            double d2 = vgl_distance(perpendicular2, p2);
            double d3 = vgl_distance(perpendicular1, perpendicular2);
            
            ret = 0.5 * d3 * (d1 + d2);
        }
        else
        {
            //p1, p2 are in different side of line
            //intersection point:line and line(p1, p2);
            vgl_line_2d<double> line_p1p1(p1, p2);
            vgl_point_2d<double> p_o;
            bool isIntersect = vgl_intersection(line, line_p1p1, p_o);
            if (isIntersect) {
                double d1 = vgl_distance(perpendicular1, p1);
                double d2 = vgl_distance(perpendicular2, p2);
                double d3 = vgl_distance(perpendicular1, p_o);
                double d4 = vgl_distance(perpendicular2, p_o);
                ret = 0.5 * (d1 * d3 + d2 * d4);
            }
            else
            {
                vcl_cout<<"Error: vpgl_line_lineseg_area not find line intersection 1 \n";
            }
        }
    }
    else
    {
        vcl_cout<<"Error: vpgl_line_lineseg_area not find line intersection 2 \n";
    }
    return ret;
}

bool VpglPlus::focal_length(const vgl_point_2d<double> vp1, const vgl_point_2d<double> & vp2, double & fl)
{
    double x1 = vp1.x();
    double y1 = vp1.y();
    double x2 = vp2.x();
    double y2 = vp2.y();
    double ff = - (x1 * x2 + y1 * y2);
    if (ff < 0) {
        return false;
    }
    fl = sqrt(ff);
    return true;
}



bool VpglPlus::init_calib(const vcl_vector<vgl_point_2d<double> > &wldPts, const vcl_vector<vgl_point_2d<double> > &imgPts,
                          const vgl_point_2d<double> &principlePoint, double focalLength, vpgl_perspective_camera<double> &camera)
{
    if (wldPts.size() < 4 && imgPts.size() < 4) {
        return false;
    }
    if (wldPts.size() != imgPts.size()) {
        return false;
    }
    assert(wldPts.size() >= 4 && imgPts.size() >= 4);
    assert(wldPts.size() == imgPts.size());
    
    vpgl_calibration_matrix<double> K(focalLength, principlePoint);
    camera.set_calibration(K);
    
    // vpgl_perspective_camera_compute_positiveZ
    if (vpgl_perspective_camera_compute::compute(imgPts, wldPts, camera) == false) {
        vcl_cerr<<"Failed to computer R, C"<<vcl_endl;
        return false;
    }
    return true;
}


static bool vpgl_perspective_camera_compute_positiveZ( const vcl_vector< vgl_point_2d<double> >& image_pts,
                                                      const vcl_vector< vgl_point_2d<double> >& ground_pts,
                                                      vpgl_perspective_camera<double>& camera )
{
    unsigned num_pts = ground_pts.size();
    if (image_pts.size()!=num_pts)
    {
        vcl_cout << "Unequal points sets in"
        << " vpgl_perspective_camera_compute::compute()\n";
        return false;
    }
    if (num_pts<4)
    {
        vcl_cout << "Need at least 4 points for"
        << " vpgl_perspective_camera_compute::compute()\n";
        return false;
    }
    
    vcl_vector<vgl_homg_point_2d<double> > pi, pg;
    for (unsigned i=0; i<num_pts; ++i) {
#ifdef CAMERA_DEBUG
        vcl_cout << '('<<image_pts[i].x()<<", "<<image_pts[i].y()<<") -> "
        << '('<<ground_pts[i].x()<<", "<<ground_pts[i].y()<<')'<<vcl_endl;
#endif
        pi.push_back(vgl_homg_point_2d<double>(image_pts[i].x(),image_pts[i].y()));
        pg.push_back(vgl_homg_point_2d<double>(ground_pts[i].x(),ground_pts[i].y()));
    }
    
    // compute a homography from the ground plane to image plane
    vgl_h_matrix_2d_compute_linear est_H;
    vnl_double_3x3 H = est_H.compute(pg,pi).get_matrix();
    if (vnl_det(H) > 0)
    H *= -1.0;
    
    // invert the effects of intrinsic parameters
    vnl_double_3x3 Kinv = vnl_inverse(camera.get_calibration().get_matrix());
    vnl_double_3x3 A(Kinv*H);
    // get the translation vector (up to a scale)
    vnl_vector_fixed<double,3> t = A.get_column(2);
    t.normalize();
    
    // compute the closest rotation matrix
    A.set_column(2, vnl_cross_3d(A.get_column(0), A.get_column(1)));
    vnl_svd<double> svdA(A.as_ref());
    vnl_double_3x3 R = svdA.U()*svdA.V().conjugate_transpose();
    
    // find the point farthest from the origin
    int max_idx = 0;
    double max_dist = 0.0;
    for (unsigned int i=0; i < ground_pts.size(); ++i) {
        double d = (ground_pts[i]-vgl_point_2d<double>(0,0)).length();
        if (d >= max_dist) {
            max_dist = d;
            max_idx = i;
        }
    }
    
    // compute the unknown scale
    vnl_vector_fixed<double,3> i1 = Kinv*vnl_double_3(image_pts[max_idx].x(),image_pts[max_idx].y(),1.0);
    vnl_vector_fixed<double,3> t1 = vnl_cross_3d(i1, t);
    vnl_vector_fixed<double,3> p1 = vnl_cross_3d(i1, R*vnl_double_3(ground_pts[max_idx].x(),ground_pts[max_idx].y(),1.0));
    double s = p1.magnitude()/t1.magnitude();
    
    // compute the camera center
    t *= s;
    t = -R.transpose()*t;
    
    camera.set_rotation(vgl_rotation_3d<double>(R));
    camera.set_camera_center(vgl_point_3d<double>(t[0],t[1],t[2]));
    
    //perform a final non-linear optimization
    vcl_vector<vgl_homg_point_3d<double> > h_world_pts;
    for (unsigned i = 0; i<num_pts; ++i) {
        h_world_pts.push_back(vgl_homg_point_3d<double>(ground_pts[i].x(),ground_pts[i].y(),0,1));
        if (camera.is_behind_camera(h_world_pts.back())) {
            //    vcl_cout << "vpgl_perspective_camera_compute_compute behind camera" << vcl_endl;
            //    return false;
        }
    }
    camera = vpgl_optimize_camera::opt_orient_pos(camera, h_world_pts, image_pts);
    
    return camera.get_camera_center().z() > 0;
}


bool VpglPlus::optimize_camera_by_inliers(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                          const vcl_vector<vgl_point_2d<double> > & imgPts,
                                          const vpgl_perspective_camera<double> & initCamera,
                                          const double distance_threshod,
                                          vpgl_perspective_camera<double> & finalCamera)
{
    assert(wldPts.size() == imgPts.size());
    
    vcl_vector<vgl_point_2d<double> > wld_inliers;
    vcl_vector<vgl_point_2d<double> > img_inliers;
    for (int i = 0; i<wldPts.size(); i++) {
        vgl_homg_point_3d<double> p(wldPts[i].x(), wldPts[i].y(), 0.0, 1.0);
        if (initCamera.is_behind_camera(p)) {
            continue;
        }
        vgl_point_2d<double> q = initCamera.project(p);
        double dis = vgl_distance(imgPts[i], q);
        if (dis < distance_threshod) {
            wld_inliers.push_back(wldPts[i]);
            img_inliers.push_back(imgPts[i]);
        }
    }
    if (wld_inliers.size() < 4) {
        return false;
    }
    return VpglPlus::optimize_perspective_camera(wld_inliers, img_inliers, initCamera, finalCamera);
}

class optimize_perspective_camera_residual:public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_point_2d<double> > wldPts_;
    const vcl_vector<vgl_point_2d<double> > imgPts_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    optimize_perspective_camera_residual(const vcl_vector<vgl_point_2d<double> > & wldPts, const vcl_vector<vgl_point_2d<double> > & imgPts,
                                         const vgl_point_2d<double> & pp):
    vnl_least_squares_function(7, (unsigned int)(wldPts.size()) * 2, no_gradient),
    wldPts_(wldPts),
    imgPts_(imgPts),
    principlePoint_(pp)
    {
        assert(wldPts.size() == imgPts.size());
        assert(wldPts.size() >= 4);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //focal length, Rxyz, Camera_center_xyz
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> cc(x[4], x[5], x[6]);  //camera center
        
        vpgl_perspective_camera<double> camera;
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(cc);
        
        //loop all points
        int idx = 0;
        for (int i = 0; i<wldPts_.size(); i++) {
            vgl_point_3d<double> p(wldPts_[i].x(), wldPts_[i].y(), 0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)camera.project(p);
            
            fx[idx] = imgPts_[i].x() - proj_p.x();
            idx++;
            fx[idx] = imgPts_[i].y() - proj_p.y();
            idx++;
        }
    }
    
    void getCamera(vnl_vector<double> const &x, vpgl_perspective_camera<double> &camera)
    {
        
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> camera_center(x[4], x[5], x[6]);
        
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(camera_center);
    }
    
};


bool VpglPlus::optimize_perspective_camera(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                           const vcl_vector<vgl_point_2d<double> > & imgPts,
                                           const vpgl_perspective_camera<double> &initCamera,
                                           vpgl_perspective_camera<double> & finalCamera)
{
    assert(wldPts.size() == imgPts.size());
    assert(wldPts.size() >= 4);
    
    optimize_perspective_camera_residual residual(wldPts, imgPts, initCamera.get_calibration().principal_point());
    
    vnl_vector<double> x(7, 0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    x[1] = initCamera.get_rotation().as_rodrigues()[0];
    x[2] = initCamera.get_rotation().as_rodrigues()[1];
    x[3] = initCamera.get_rotation().as_rodrigues()[2];
    x[4] = initCamera.camera_center().x();
    x[5] = initCamera.camera_center().y();
    x[6] = initCamera.camera_center().z();
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimied = lmq.minimize(x);
    if (!isMinimied) {
        vcl_cerr<<"Error: perspective camera optimize not converge.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    //    lmq.diagnose_outcome();
    residual.getCamera(x, finalCamera);
    return true;
}


class optimize_perspective_camera_ICP_residual: public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_point_2d<double> > wldPts_;
    const vcl_vector<vgl_point_2d<double> > imgPts_;
    const vcl_vector<vgl_line_3d_2_points<double> >  wldLines_;
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >  imgLinePts_;
    const vgl_point_2d<double> principlePoint_;
public:
    optimize_perspective_camera_ICP_residual(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                             const vcl_vector<vgl_point_2d<double> > & imgPts,
                                             const vcl_vector<vgl_line_3d_2_points<double> >  & wldLines,
                                             const vcl_vector<vcl_vector<vgl_point_2d<double> > >  & imgLinePts,
                                             const vgl_point_2d<double> & pp,
                                             const int num_line_pts):
    vnl_least_squares_function(7, (unsigned int)(wldPts.size()) * 2 + num_line_pts, no_gradient),
    wldPts_(wldPts),
    imgPts_(imgPts),
    wldLines_(wldLines),
    imgLinePts_(imgLinePts),
    principlePoint_(pp)
    {
        assert(wldPts.size() == imgPts.size());
        assert(wldPts.size() >= 4);
        assert(wldLines.size() == imgLinePts.size());
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //focal length, Rxyz, Camera_center_xyz
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> cc(x[4], x[5], x[6]);  //camera center
        
        vpgl_perspective_camera<double> camera;
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(cc);
        
        //loop all points
        int idx = 0;
        for (int i = 0; i<wldPts_.size(); i++) {
            vgl_point_3d<double> p(wldPts_[i].x(), wldPts_[i].y(), 0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)camera.project(p);
            
            fx[idx] = imgPts_[i].x() - proj_p.x();
            idx++;
            fx[idx] = imgPts_[i].y() - proj_p.y();
            idx++;
        }
        
        // for points locate on the line
        for (int i = 0; i<wldLines_.size(); i++) {
            vgl_point_2d<double> p1 = camera.project(wldLines_[i].point1());
            vgl_point_2d<double> p2 = camera.project(wldLines_[i].point2());
            vgl_line_2d<double> line(p1, p2);
            for (int j = 0; j<imgLinePts_[i].size(); j++) {
                vgl_point_2d<double> p3 = imgLinePts_[i][j];
                fx[idx] = vgl_distance(line, p3);
                idx++;
            }
        }
    }
    
    void getCamera(vnl_vector<double> const &x, vpgl_perspective_camera<double> &camera)
    {
        
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> camera_center(x[4], x[5], x[6]);
        
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(camera_center);
    }
};


bool VpglPlus::optimize_perspective_camera_ICP(const vcl_vector<vgl_point_2d<double> > &wldPts,
                                               const vcl_vector<vgl_point_2d<double> > &imgPts,
                                               const vcl_vector<vgl_line_3d_2_points<double> > & wldLines,
                                               const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgLinePts,
                                               const vpgl_perspective_camera<double> & initCamera,
                                               vpgl_perspective_camera<double> &camera)
{
    assert(wldPts.size() == imgPts.size());
    assert(wldPts.size() >= 4);
    assert(wldLines.size() == imgLinePts.size());
    
    int num_line_pts = 0;
    for (int i = 0; i<imgLinePts.size(); i++) {
        num_line_pts += (int)imgLinePts[i].size();
    }
    optimize_perspective_camera_ICP_residual residual(wldPts, imgPts, wldLines, imgLinePts, initCamera.get_calibration().principal_point(), num_line_pts);
    
    vnl_vector<double> x(7, 0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    x[1] = initCamera.get_rotation().as_rodrigues()[0];
    x[2] = initCamera.get_rotation().as_rodrigues()[1];
    x[3] = initCamera.get_rotation().as_rodrigues()[2];
    x[4] = initCamera.camera_center().x();
    x[5] = initCamera.camera_center().y();
    x[6] = initCamera.camera_center().z();
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimied = lmq.minimize(x);
    if (!isMinimied) {
        vcl_cerr<<"Error: perspective camera optimize not converge.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    residual.getCamera(x, camera);
    return true;
}


class optimize_perspective_camera_conic_ICP_residual: public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_point_2d<double> > wldPts_;
    const vcl_vector<vgl_point_2d<double> > imgPts_;
    const vcl_vector<vgl_conic<double> >  wldConics_;
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >  imgConicPts_;
    const vgl_point_2d<double> principlePoint_;
public:
    optimize_perspective_camera_conic_ICP_residual(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                                   const vcl_vector<vgl_point_2d<double> > & imgPts,
                                                   
                                                   const vcl_vector<vgl_conic<double> >  & wldConics,
                                                   const vcl_vector<vcl_vector<vgl_point_2d<double> > >  & imgConicPts,
                                                   
                                                   const vgl_point_2d<double> & pp,
                                                   const int num_conic_pts):
    vnl_least_squares_function(7, (unsigned int)(wldPts.size()) * 2 + num_conic_pts, no_gradient),
    wldPts_(wldPts),
    imgPts_(imgPts),
    wldConics_(wldConics),
    imgConicPts_(imgConicPts),
    principlePoint_(pp)
    {
        assert(wldPts.size() == imgPts.size());
        assert(wldPts.size() >= 4);
        assert(wldConics.size() == imgConicPts.size());
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //focal length, Rxyz, Camera_center_xyz
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> cc(x[4], x[5], x[6]);  //camera center
        
        vpgl_perspective_camera<double> camera;
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(cc);
        
        //loop all points
        int idx = 0;
        for (int i = 0; i<wldPts_.size(); i++) {
            vgl_point_3d<double> p(wldPts_[i].x(), wldPts_[i].y(), 0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)camera.project(p);
            
            fx[idx] = imgPts_[i].x() - proj_p.x();
            idx++;
            fx[idx] = imgPts_[i].y() - proj_p.y();
            idx++;
        }
        
        vnl_matrix_fixed<double, 3, 3> H = VpglPlus::homographyFromProjectiveCamera(camera);
        
        // for points locate on the conics
        for (int i = 0; i<wldConics_.size(); i++) {
            vgl_conic<double> conic_proj = VpglPlus::projectConic(H, wldConics_[i]);
            for (int j = 0; j<imgConicPts_[i].size(); j++) {
                vgl_point_2d<double> p = imgConicPts_[i][j];
                double dis = vgl_homg_operators_2d<double>::distance_squared(conic_proj, vgl_homg_point_2d<double>(p.x(), p.y(), 1.0));
             //   printf("distance is %f\n", dis);
                dis = sqrt(dis + 0.0000001);
                fx[idx] = dis;
                idx++;
            }
        }
        
    }
    
    void getCamera(vnl_vector<double> const &x, vpgl_perspective_camera<double> &camera)
    {
        
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> camera_center(x[4], x[5], x[6]);
        
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(camera_center);
    }
};


bool VpglPlus::optimize_perspective_camera_ICP(const vcl_vector<vgl_point_2d<double> > &wldPts,
                                               const vcl_vector<vgl_point_2d<double> > &imgPts,
                                               const vcl_vector<vgl_conic<double> > & wldConics,
                                               const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgConicPts,
                                               const vpgl_perspective_camera<double> & initCamera,
                                               vpgl_perspective_camera<double> &camera)
{
    assert(wldPts.size() == imgPts.size());
    assert(wldConics.size() == imgConicPts.size());
    assert(wldPts.size() >= 4);
    
    // only for circle
    for (int i = 0; i<wldConics.size(); i++) {
        vcl_string name = wldConics[i].real_type();
        if(name != "real circle")
        {
            printf("Error: conic ICP only must be a circle!\n");
            return false;
        }
    }
    
    int num_conic_pts = 0;
    for (int i = 0; i<imgConicPts.size(); i++) {
        num_conic_pts += (int)imgConicPts[i].size();
    }
    
    optimize_perspective_camera_conic_ICP_residual residual(wldPts, imgPts, wldConics, imgConicPts, initCamera.get_calibration().principal_point(), num_conic_pts);
    
    vnl_vector<double> x(7, 0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    x[1] = initCamera.get_rotation().as_rodrigues()[0];
    x[2] = initCamera.get_rotation().as_rodrigues()[1];
    x[3] = initCamera.get_rotation().as_rodrigues()[2];
    x[4] = initCamera.camera_center().x();
    x[5] = initCamera.camera_center().y();
    x[6] = initCamera.camera_center().z();
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimied = lmq.minimize(x);
    if (!isMinimied) {
        vcl_cerr<<"Error: perspective camera optimize not converge.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    residual.getCamera(x, camera);
    return true;
}

class optimize_perspective_camera_line_conic_ICP_residual: public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_point_2d<double> > wldPts_;
    const vcl_vector<vgl_point_2d<double> > imgPts_;
    const vcl_vector<vgl_line_3d_2_points<double> >  wldLines_;
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >  imgLinePts_;
    const vcl_vector<vgl_conic<double> >  wldConics_;
    const vcl_vector<vcl_vector<vgl_point_2d<double> > >  imgConicPts_;
    const vgl_point_2d<double> principlePoint_;
public:
    optimize_perspective_camera_line_conic_ICP_residual(const vcl_vector<vgl_point_2d<double> > & wldPts,
                                                        const vcl_vector<vgl_point_2d<double> > & imgPts,
                                                        const vcl_vector<vgl_line_3d_2_points<double> > & wldLines,
                                                        const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgLinePts,
                                                        
                                                        const vcl_vector<vgl_conic<double> >  & wldConics,
                                                        const vcl_vector<vcl_vector<vgl_point_2d<double> > >  & imgConicPts,
                                                   
                                                        const vgl_point_2d<double> & pp,
                                                        const int num_line_pts,
                                                        const int num_conic_pts):
    vnl_least_squares_function(7, (unsigned int)(wldPts.size()) * 2 + num_line_pts + num_conic_pts, no_gradient),
    wldPts_(wldPts),
    imgPts_(imgPts),
    wldLines_(wldLines),
    imgLinePts_(imgLinePts),
    wldConics_(wldConics),
    imgConicPts_(imgConicPts),
    principlePoint_(pp)
    {
        assert(wldPts.size() == imgPts.size());
        assert(wldPts.size() >= 2);
        assert(wldLines.size() == imgLinePts.size());
        assert(wldConics.size() == imgConicPts.size());
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //focal length, Rxyz, Camera_center_xyz
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> cc(x[4], x[5], x[6]);  //camera center
        vpgl_perspective_camera<double> camera;
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(cc);
        
        //loop all points
        int idx = 0;
        for (int i = 0; i<wldPts_.size(); i++) {
            vgl_point_3d<double> p(wldPts_[i].x(), wldPts_[i].y(), 0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)camera.project(p);
            
            fx[idx] = imgPts_[i].x() - proj_p.x();
            idx++;
            fx[idx] = imgPts_[i].y() - proj_p.y();
            idx++;
        }
        
        // for points locate on the line
        for (int i = 0; i<wldLines_.size(); i++) {
            vgl_point_2d<double> p1 = camera.project(wldLines_[i].point1());
            vgl_point_2d<double> p2 = camera.project(wldLines_[i].point2());
            vgl_line_2d<double> line(p1, p2);
            for (int j = 0; j<imgLinePts_[i].size(); j++) {
                vgl_point_2d<double> p3 = imgLinePts_[i][j];
                fx[idx] = vgl_distance(line, p3);
                idx++;
            }
        }
        
        // for points locate on the conics
        vnl_matrix_fixed<double, 3, 3> H = VpglPlus::homographyFromProjectiveCamera(camera);        
        for (int i = 0; i<wldConics_.size(); i++) {
            vgl_conic<double> conic_proj = VpglPlus::projectConic(H, wldConics_[i]);
            for (int j = 0; j<imgConicPts_[i].size(); j++) {
                vgl_point_2d<double> p = imgConicPts_[i][j];
                double dis = vgl_homg_operators_2d<double>::distance_squared(conic_proj, vgl_homg_point_2d<double>(p.x(), p.y(), 1.0));
                dis = sqrt(dis + 0.0000001);
                fx[idx] = dis;
                idx++;
            }
        }
        
    }
    
    void getCamera(vnl_vector<double> const &x, vpgl_perspective_camera<double> &camera)
    {
        
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> camera_center(x[4], x[5], x[6]);
        
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(camera_center);
    }
};


bool VpglPlus::optimize_perspective_camera_ICP(const vcl_vector<vgl_point_2d<double> > &wldPts,
                                               const vcl_vector<vgl_point_2d<double> > &imgPts,
                                               const vcl_vector<vgl_line_3d_2_points<double> > & wldLines,
                                               const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgLinePts,
                                               const vcl_vector<vgl_conic<double> > & wldConics,
                                               const vcl_vector<vcl_vector<vgl_point_2d<double> > > & imgConicPts,
                                               const vpgl_perspective_camera<double> & initCamera,
                                               vpgl_perspective_camera<double> &camera)
{
    assert(wldPts.size() == imgPts.size());
    assert(wldLines.size() == imgLinePts.size());
    assert(wldConics.size() == imgConicPts.size());
    
    int num_line_pts = 0;
    int num_conic_pts = 0;
    for (int i = 0; i<imgLinePts.size(); i++) {
        num_line_pts += imgLinePts[i].size();
    }
    for (int i = 0; i<imgConicPts.size(); i++) {
        num_conic_pts += imgConicPts[i].size();
    }
    assert((unsigned int)(wldPts.size()) * 2 + num_line_pts + num_conic_pts > 7);
    
    optimize_perspective_camera_line_conic_ICP_residual residual(wldPts, imgPts, wldLines, imgLinePts, wldConics, imgConicPts,
                                                                 initCamera.get_calibration().principal_point(),
                                                                 num_line_pts, num_conic_pts);
    vnl_vector<double> x(7, 0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    x[1] = initCamera.get_rotation().as_rodrigues()[0];
    x[2] = initCamera.get_rotation().as_rodrigues()[1];
    x[3] = initCamera.get_rotation().as_rodrigues()[2];
    x[4] = initCamera.camera_center().x();
    x[5] = initCamera.camera_center().y();
    x[6] = initCamera.camera_center().z();
    
    vnl_levenberg_marquardt lmq(residual);
    bool isMinimied = lmq.minimize(x);
    if (!isMinimied) {
        vcl_cerr<<"Error: perspective camera optimize not converge.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    residual.getCamera(x, camera);
    return true;
}


class optimize_perspective_camera_3d_residual :public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_point_3d<double> > wldPts_;
    const vcl_vector<vgl_point_2d<double> > imgPts_;
    const vgl_point_2d<double> principlePoint_;
    
public:
    optimize_perspective_camera_3d_residual(const vcl_vector<vgl_point_3d<double> > & wldPts, const vcl_vector<vgl_point_2d<double> > & imgPts,
                                            const vgl_point_2d<double> & pp):
    vnl_least_squares_function(7, (unsigned int)(wldPts.size()) * 2, no_gradient),
    wldPts_(wldPts),
    imgPts_(imgPts),
    principlePoint_(pp)
    {
        assert(wldPts.size() == imgPts.size());
        assert(wldPts.size() >= 5);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        //focal length, Rxyz, Camera_center_xyz
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> cc(x[4], x[5], x[6]);  //camera center
        
        vpgl_perspective_camera<double> camera;
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(cc);
        
        //loop all points
        int idx = 0;
        for (int i = 0; i<wldPts_.size(); i++) {
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)camera.project(wldPts_[i]);
            
            fx[idx] = imgPts_[i].x() - proj_p.x();
            idx++;
            fx[idx] = imgPts_[i].y() - proj_p.y();
            idx++;
        }
    }
    
    void getCamera(vnl_vector<double> const &x, vpgl_perspective_camera<double> &camera)
    {
        
        vpgl_calibration_matrix<double> K(x[0], principlePoint_);
        
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double>  R(rod);
        vgl_point_3d<double> camera_center(x[4], x[5], x[6]);
        
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(camera_center);
    }
    
};




bool VpglPlus::optimize_perspective_camera(const vcl_vector<vgl_point_3d<double> > & wldPts,
                                           const vcl_vector<vgl_point_2d<double> > & imgPts,
                                           const vpgl_perspective_camera<double> & initCamera,
                                           vpgl_perspective_camera<double> & finalCamera)
{
    assert(wldPts.size() == imgPts.size());
    
    optimize_perspective_camera_3d_residual residual(wldPts, imgPts, initCamera.get_calibration().principal_point());
    
    vnl_vector<double> x(7, 0);
    x[0] = initCamera.get_calibration().get_matrix()[0][0];
    x[1] = initCamera.get_rotation().as_rodrigues()[0];
    x[2] = initCamera.get_rotation().as_rodrigues()[1];
    x[3] = initCamera.get_rotation().as_rodrigues()[2];
    x[4] = initCamera.camera_center().x();
    x[5] = initCamera.camera_center().y();
    x[6] = initCamera.camera_center().z();
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinimied = lmq.minimize(x);
    if (!isMinimied) {
        vcl_cerr<<"Error: perspective camera optimize not converge.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    lmq.diagnose_outcome();
    residual.getCamera(x, finalCamera);
    return true;
}

vgl_conic<double> VpglPlus::projectConic(const vnl_matrix_fixed<double, 3, 3> & H, const vgl_conic<double> & conic)
{
    double a = conic.a();
    double b = conic.b();
    double c = conic.c();
    double d = conic.d();
    double e = conic.e();
    double f = conic.f();
    vnl_matrix_fixed<double, 3, 3> C;
    C(0, 0) = a;     C(0, 1) = b/2.0; C(0, 2) = d/2.0;
    C(1, 0) = b/2.0; C(1, 1) = c;     C(1, 2) = e/2.0;
    C(2, 0) = d/2.0; C(2, 1) = e/2.0; C(2, 2) = f;
    // project conic by H
    vnl_matrix_fixed<double, 3, 3> H_inv = vnl_inverse(H);
    vnl_matrix_fixed<double, 3, 3> C_proj = H_inv.transpose() * C * H_inv;
    
    // approximate a conic from 3*3 matrix
    double aa =  C_proj(0, 0);
    double bb = (C_proj(0, 1) + C_proj(1, 0));
    double cc =  C_proj(1, 1);
    double dd = (C_proj(0, 2) + C_proj(2, 0));
    double ee = (C_proj(1, 2) + C_proj(2, 1));
    double ff =  C_proj(2, 2);
    
    // project conic
    vgl_conic<double> conic_proj(aa, bb, cc, dd, ee, ff);
    return conic_proj;
}
//

static bool vpgl_vgl_inside_image(const vgl_point_2d<double> &p, int width, int height)
{
    return p.x() >= 0 && p.x() < width && p.y() >= 0 && p.y() < height;
}

bool VpglPlus::sampleLineSegment(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                 const vgl_line_segment_2d<double> & init_segment, double sampleUnit,
                                 vgl_line_segment_2d<double> & sampled_segment)
{
    vgl_point_3d<double> p1(init_segment.point1().x(), init_segment.point1().y(), 0);
    vgl_point_3d<double> p2(init_segment.point2().x(), init_segment.point2().y(), 0);
    
    if (camera.is_behind_camera(vgl_homg_point_3d<double>(p1)) ||
        camera.is_behind_camera(vgl_homg_point_3d<double>(p2))) {
        return false;
    }
    
    vgl_point_2d<double> q1(-1, -1); // (-1, -1) is the mask for unfined end point
    vgl_point_2d<double> q2(-1, -1);
    vgl_vector_2d<double> dir = init_segment.direction();
    const double length = vgl_distance(p1, p2);   // length is measured in world coordinate
    vgl_point_2d<double> p3(-1, -1);   // points in world coordinate
    vgl_point_2d<double> p4(-1, -1);
    
    // sample first point from p1 to p2 in the image
    bool isFound = false;
    for (double j = 0; j <= length; j += sampleUnit) {
        double x = p1.x() + dir.x() * j;
        double y = p1.y() + dir.y() * j;
        
        vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, 0.0));
        if (vpgl_vgl_inside_image(q, imageW, imageH)) {
            q1 = q;
            p3 = vgl_point_2d<double>(x, y);
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
        if (vpgl_vgl_inside_image(q, imageW, imageH)) {
            q2 = q;
            p4 = vgl_point_2d<double>(x, y);
            isFound = true;
            break;
        }
    }
    if (!isFound) {
        return false;
    }
    
    double dis = vgl_distance(p3, p4);  // at
    if (dis > 2 * sampleUnit) {
        sampled_segment = vgl_line_segment_2d<double>(p3, p4);
        return true;
    }    
    return false;
}

bool VpglPlus::sampleLineSegment(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                 const vgl_line_segment_3d<double> & init_segment, double sampleUnit,
                                 vgl_line_segment_3d<double> & sampled_segment)
{
    vgl_point_3d<double> p1 = init_segment.point1();
    vgl_point_3d<double> p2 = init_segment.point2();
    
    if (camera.is_behind_camera(vgl_homg_point_3d<double>(p1)) ||
        camera.is_behind_camera(vgl_homg_point_3d<double>(p2))) {
        return false;
    }

    vgl_vector_3d<double> dir = init_segment.direction();
    dir = normalize(dir);
    const double length = vgl_distance(p1, p2);   // length is measured in world coordinate
    vgl_point_3d<double> p3(-1, -1, -1);       // points in world coordinate
    vgl_point_3d<double> p4(-1, -1, -1);
    
    // sample first point from p1 to p2 in the image
    bool isFound = false;
    for (double j = 0; j <= length; j += sampleUnit) {
        double x = p1.x() + dir.x() * j;
        double y = p1.y() + dir.y() * j;
        double z = p1.z() + dir.z() * j;
        
        vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, z));
        if (vpgl_vgl_inside_image(q, imageW, imageH)) {
            p3 = vgl_point_3d<double>(x, y, z);
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
        double z = p1.z() + dir.z() * j;
        
        vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, z));
        if (vpgl_vgl_inside_image(q, imageW, imageH)) {
            p4 = vgl_point_3d<double>(x, y, z);
            isFound = true;
            break;
        }
    }
    if (!isFound) {
        return false;
    }
    
    double dis = vgl_distance(p3, p4);  // at
    if (dis > 2 * sampleUnit) {
        sampled_segment = vgl_line_segment_3d<double>(p3, p4);
        return true;
    }
    return false;
}


double VpglPlus::projectLineWidth(const vpgl_perspective_camera<double> & camera,
                                  const vgl_line_segment_2d<double> & segment,
                                  double lineWidth)
{
    vgl_line_segment_2d<double> dump;
    return VpglPlus::projectLineWidth(camera, segment, lineWidth, dump);
}
                                  


double VpglPlus::projectLineWidth(const vpgl_perspective_camera<double> & camera,
                                  const vgl_line_segment_2d<double> & segment,
                                  double lineWidth,
                                  vgl_line_segment_2d<double> & projectedNorm)
{
    vgl_point_2d<double>  p1 = segment.point_t(0.5);  // center
    vgl_vector_2d<double> norm = segment.normal();
    vgl_point_2d<double> p2 = vgl_point_2d<double>(p1.x() + (lineWidth * norm).x(), p1.y() + (lineWidth * norm).y());  // center width displacment
    vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(p1.x(), p1.y(), 0.0));
    vgl_point_2d<double> q2 = camera.project(vgl_point_3d<double>(p2.x(), p2.y(), 0.0));
    projectedNorm = vgl_line_segment_2d<double>(q1, q2);
    return vgl_distance(q1, q2);
}


vnl_matrix_fixed<double, 3, 3> VpglPlus::homographyFromProjectiveCamera(const vpgl_perspective_camera<double> & camera)
{
    vnl_matrix_fixed<double, 3, 3> H;
    vnl_matrix_fixed<double, 3, 4> P = camera.get_matrix();
    
    H(0, 0) = P(0, 0); H(0, 1) = P(0, 1); H(0, 2) = P(0, 3);
    H(1, 0) = P(1, 0); H(1, 1) = P(1, 1); H(1, 2) = P(1, 3);
    H(2, 0) = P(2, 0); H(2, 1) = P(2, 1); H(2, 2) = P(2, 3);
    
    return H;
}




void VpglPlus::matrixFromPanYTiltX(double pan, double tilt, vnl_matrix_fixed<double, 3, 3> &m)
{
	
    pan *= vnl_math::pi / 180.0;
    tilt *= vnl_math::pi / 180.0;
    
    vnl_matrix_fixed<double, 3, 3> R_tilt;
    R_tilt[0][0] = 1;   R_tilt[0][1] = 0;          R_tilt[0][2] = 0;
    R_tilt[1][0] = 0;   R_tilt[1][1] = cos(tilt);  R_tilt[1][2] = sin(tilt);
    R_tilt[2][0] = 0;   R_tilt[2][1] = -sin(tilt);  R_tilt[2][2] = cos(tilt);
    
    vnl_matrix_fixed<double, 3, 3> R_pan;
    R_pan[0][0] = cos(pan);   R_pan[0][1] = 0;   R_pan[0][2] = -sin(pan);
    R_pan[1][0] = 0;          R_pan[1][1] = 1;   R_pan[1][2] = 0;
    R_pan[2][0] = sin(pan);  R_pan[2][1] = 0;   R_pan[2][2] = cos(pan);
    
    m = R_tilt * R_pan;
}

void VpglPlus::matrixFromPanYTiltX(double pan, double tilt, vnl_matrix_fixed<double, 4, 4> & m)
{
    vnl_matrix_fixed<double, 3, 3> r;
    VpglPlus::matrixFromPanYTiltX(pan, tilt, r);
    
    m.set_identity();
    
    for (int i = 0; i<3; i++) {
        for (int j = 0; j<3; j++) {
            m(i, j) = r(i, j);
        }
    }
}
vnl_matrix<double> VpglPlus::matrixPanY(double pan)
{
    pan *= vnl_math::pi / 180.0;
    vnl_matrix_fixed<double, 3, 3> R_pan;
    R_pan[0][0] = cos(pan);   R_pan[0][1] = 0;   R_pan[0][2] = -sin(pan);
    R_pan[1][0] = 0;          R_pan[1][1] = 1;   R_pan[1][2] = 0;
    R_pan[2][0] = sin(pan);  R_pan[2][1] = 0;   R_pan[2][2] = cos(pan);
    return R_pan;
}

vnl_matrix<double> VpglPlus::matrixTiltX(double tilt)
{
    tilt *= vnl_math::pi / 180.0;
    vnl_matrix_fixed<double, 3, 3> R_tilt;
    R_tilt[0][0] = 1;   R_tilt[0][1] = 0;          R_tilt[0][2] = 0;
    R_tilt[1][0] = 0;   R_tilt[1][1] = cos(tilt);  R_tilt[1][2] = sin(tilt);
    R_tilt[2][0] = 0;   R_tilt[2][1] = -sin(tilt);  R_tilt[2][2] = cos(tilt);
    return R_tilt;
}


void VpglPlus::panYtiltXFromPrinciplePoint(const vgl_point_2d<double> & pp,
                                           const vnl_vector_fixed<double, 3> & pp_ptz,
                                           const vgl_point_2d<double> & p2,
                                           vnl_vector_fixed<double, 3> & p2_ptz)
{
    double dx = p2.x() - pp.x();
    double dy = p2.y() - pp.y();
    
    double pan_pp  = pp_ptz[0];
    double tilt_pp = pp_ptz[1];
    double focal_length = pp_ptz[2];
    double delta_pan  = atan2(dx, focal_length) * 180/vnl_math::pi;
    double delta_tilt = atan2(dy, focal_length) * 180/vnl_math::pi;
    
    double pan2  = pan_pp + delta_pan;
    double tilt2 = tilt_pp - delta_tilt;  // y is upside down
    p2_ptz[0] = pan2;
    p2_ptz[1] = tilt2;
    p2_ptz[2] = focal_length;
}

void VpglPlus::panYtiltXFromPrinciplePointEncodeFocalLength(const vgl_point_2d<double> & pp,
                                                            const vnl_vector_fixed<double, 3> & pp_ptz,
                                                            const vgl_point_2d<double> & p2,
                                                            vnl_vector_fixed<double, 3> & p2_ptz)
{
    double dx = p2.x() - pp.x();
    double dy = p2.y() - pp.y();
    
    double pan_pp  = pp_ptz[0];
    double tilt_pp = pp_ptz[1];
    double focal_length = pp_ptz[2];
    double delta_pan  = atan2(dx, focal_length) * 180/vnl_math::pi;
    double delta_tilt = atan2(dy, focal_length) * 180/vnl_math::pi;
    
    double pan2  = pan_pp + delta_pan;
    double tilt2 = tilt_pp - delta_tilt;  // y is upside down
    p2_ptz[0] = pan2;
    p2_ptz[1] = tilt2;
    p2_ptz[2] = sqrt(focal_length * focal_length + dx * dx + dy * dy);    
}

void VpglPlus::panTiltFromReferencePoint(const vgl_point_2d<double> & reference_point,
                                         const vnl_vector_fixed<double, 3> & reference_ptz,
                                         const vgl_point_2d<double> & principle_point,
                                         vnl_vector_fixed<double, 3> & ptz)
{
    double dx = reference_point.x() - principle_point.x();
    double dy = reference_point.y() - principle_point.y();
    double ref_pan  = reference_ptz[0];
    double ref_tilt = reference_ptz[1];
    double fl = reference_ptz[2];
    
    double delta_pan  = atan2(dx, fl) * 180.0/vnl_math::pi;
    double delta_tilt = atan2(dy, fl) * 180.0/vnl_math::pi;
    
    ptz[0] = ref_pan  - delta_pan;
    ptz[1] = ref_tilt + delta_tilt;
    ptz[2] = fl;
}

void VpglPlus::panTiltFromReferencePointDecodeFocalLength(const vgl_point_2d<double> & reference_point,
                                                          const vnl_vector_fixed<double, 3> & reference_ptz,
                                                          const vgl_point_2d<double> & principle_point,
                                                          vnl_vector_fixed<double, 3> & ptz)
{
    double dx = reference_point.x() - principle_point.x();
    double dy = reference_point.y() - principle_point.y();
    double ref_pan  = reference_ptz[0];
    double ref_tilt = reference_ptz[1];
    double fl = reference_ptz[2];
    
    double delta_pan  = atan2(dx, fl) * 180.0/vnl_math::pi;
    double delta_tilt = atan2(dy, fl) * 180.0/vnl_math::pi;
    
    ptz[0] = ref_pan  - delta_pan;
    ptz[1] = ref_tilt + delta_tilt;
    if (fl*fl - dx*dx - dy*dy > 0) {
        ptz[2] = sqrt(fl*fl - dx*dx - dy*dy);
    }
    else  {
        ptz[2] = fl;
    }
}

void VpglPlus::matrixFromRotationCameraCenter(const vnl_matrix_fixed<double, 3, 3> &rot, const vgl_point_3d<double> &cameraCenter,
                                              vnl_matrix_fixed<double, 4, 4> &outMatrix)
{
    vnl_matrix_fixed<double, 3, 4> translation;
    translation.set_identity();
    translation[0][3] = -cameraCenter.x();
    translation[1][3] = -cameraCenter.y();
    translation[2][3] = -cameraCenter.z();
    
    vnl_matrix_fixed<double, 3, 4> rot_tras_34 = rot * translation;
    
    outMatrix.set_identity();
    for (int i = 0; i<3; i++) {
        for (int j = 0; j<4; j++) {
            outMatrix[i][j] = rot_tras_34[i][j];
        }
    }
}

vnl_matrix<double> VpglPlus::eularToRotation(const double y_angle, const double z_angle, const double x_angle)
{
    // http://www.cprogramming.com/tutorial/3d/rotationMatrices.html
    // right hand
    double pan    = y_angle;
    double tilt   = x_angle;
    double rotate = z_angle;
    
    vnl_matrix<double> R_pan(3, 3, 0);
    R_pan[0][0] = cos(pan);   R_pan[0][1] = 0;   R_pan[0][2] = sin(pan);
    R_pan[1][0] = 0;          R_pan[1][1] = 1;   R_pan[1][2] = 0;
    R_pan[2][0] = -sin(pan);  R_pan[2][1] = 0;   R_pan[2][2] = cos(pan);
    
    vnl_matrix<double> R_tilt(3, 3, 0);
    R_tilt[0][0] = 1;   R_tilt[0][1] = 0;          R_tilt[0][2] = 0;
    R_tilt[1][0] = 0;   R_tilt[1][1] = cos(tilt);  R_tilt[1][2] = -sin(tilt);
    R_tilt[2][0] = 0;   R_tilt[2][1] = sin(tilt);  R_tilt[2][2] = cos(tilt);
    
    vnl_matrix<double> R_rotate(3, 3, 0);
    R_rotate[0][0] = cos(rotate); R_rotate[0][1] = -sin(rotate); R_rotate[0][2] = 0;
    R_rotate[1][0] = sin(rotate); R_rotate[1][1] =  cos(rotate); R_rotate[1][2] = 0;
    R_rotate[2][0] = 0;           R_rotate[2][1] =  0;           R_rotate[2][2] = 1;
    
    return R_tilt*R_rotate*R_pan;
}


bool VpglPlus::decomposeCameraPTZ(double fl, const vnl_vector_fixed<double, 6> & coeff,
                                  const vgl_point_2d<double> & principlePoint,
                                  double pan, double tilt,
                                  const vnl_vector_fixed<double, 3> & rod,
                                  const vgl_point_3d<double> & cameraCenter,
                                  vpgl_perspective_camera<double> & camera)
{
    double c1 = coeff[0];
    double c2 = coeff[1];
    double c3 = coeff[2];
    double c4 = coeff[3];
    double c5 = coeff[4];
    double c6 = coeff[5];
    
    vgl_rotation_3d<double> Rs(rod);  // model rotation
    
    vpgl_calibration_matrix<double> K(fl, principlePoint);
    vnl_matrix_fixed<double, 3, 4> C;
    C.set_identity();
    C(0, 3) = - (c1 + c4 * fl);
    C(1, 3) = - (c2 + c5 * fl);
    C(2, 3) = - (c3 + c6 * fl);
    
    vnl_matrix_fixed<double, 4, 4> Q;  //rotation from pan tilt angle
    VpglPlus::matrixFromPanYTiltX(pan, tilt, Q);
    
    vnl_matrix_fixed<double, 4, 4> RSD;
    VpglPlus::matrixFromRotationCameraCenter(Rs.as_matrix(), cameraCenter,  RSD);
    
    vnl_matrix_fixed<double, 3, 4> P = K.get_matrix() * C * Q * RSD;
    
    return vpgl_perspective_decomposition(P, camera);
}

bool VpglPlus::decomposeCameraPTZ(double fl, double pan, double tilt, vpgl_perspective_camera<double> & camera)
{
    assert(fl > 1000);
    
    //estimte fl, pan, tilt from camera
    vgl_point_3d<double> cc(12.9456, -14.8695, 6.21215); //camera center
    vnl_vector_fixed<double, 3> rod;    // 1.58044 -0.118628 0.124857
    rod[0] =    1.58044;
    rod[1] = -0.118628;
    rod[2] =  0.124857;
    
    vnl_vector_fixed<double, 6> coeff;  // 0.570882 0.0310795 -0.533881 -0.000229727 -5.68634e-06 0.000266362
    coeff[0] =  0.570882;
    coeff[1] =  0.0310795;
    coeff[2] = -0.533881;
    coeff[3] = -0.000229727;
    coeff[4] = -5.68634e-06;
    coeff[5] =  0.000266362;
    
    vgl_point_2d<double> pp(640, 360);  //principle point
    
    return VpglPlus::decomposeCameraPTZ(fl, coeff, pp, pan, tilt, rod, cc, camera);
}


//example:
//glMatrixMode (GL_MODELVIEW);
//glClear (GL_COLOR_BUFFER_BIT);
//glLoadIdentity ();             /* clear the matrix */
//glLoadMatrixd(glCamera.model_view_.data_block());

//glMatrixMode (GL_PROJECTION);
//glLoadIdentity();
//glOrtho(0, width, height, 0, -0.1, 25);
//glMultMatrixd(glCamera.proj_.data_block());

void VpglPlus::cameraToOpenGLCamera(const vpgl_perspective_camera<double> & camera, OpenglCamera & glCamera, double scale)
{
    
    vnl_matrix<double> R = camera.get_rotation().as_matrix();
    vnl_matrix<double> Tmat(3, 4, 0);
    Tmat.set_identity();
    Tmat(0, 3) = -camera.get_camera_center().x();
    Tmat(1, 3) = -camera.get_camera_center().y();
    Tmat(2, 3) = -camera.get_camera_center().z();
    
    
    vnl_matrix<double> RT = R * Tmat;
    
    vnl_matrix<double> m44(4, 4, 0);
    m44.set_identity();
    m44.update(RT, 0, 0);
    m44 = m44.transpose();
    
    glCamera.model_view_ = m44;
    
    
    double alpha = scale * camera.get_calibration().focal_length();
    double beta =  scale * camera.get_calibration().focal_length();
    double u0 = scale * camera.get_calibration().principal_point().x();
    double v0 = scale * camera.get_calibration().principal_point().y();   
    
    glCamera.proj_ = VpglPlus::opengl_projection_from_intrinsics(alpha, beta, 0.0, u0, v0, -0.1, 25);

}

// from http://jamesgregson.blogspot.ca/2011/11/matching-calibrated-cameras-with-opengl.html
//      http://www.songho.ca/opengl/gl_projectionmatrix.html
vnl_matrix<double> VpglPlus::opengl_projection_from_intrinsics(
                                       double alpha, double beta,
                                       double skew,
                                       double u0, double v0,
                                       double near_clip, double far_clip )
{
    assert(near_clip < 0.0);
    double N = near_clip;
    double F = far_clip;
    
    vnl_matrix<double> projection = vnl_matrix<double>(4, 4, 0);
    projection(0,0) = alpha; projection(0,1) = skew; projection(0,2) = u0;
    projection(1,0) = 0.0;   projection(1,1) = beta; projection(1,2) = v0;
    projection(2,0) = 0.0;   projection(2,1) = 0.0;  projection(2,2) = -(N+F); projection(2,3) = -N*F;
    projection(3,0) = 0.0;   projection(3,1) = 0.0;  projection(3,2) = 1.0;    projection(3,3) = 0.0;
    
    projection = projection.transpose();
    return projection;
}

vgl_h_matrix_2d<double> VpglPlus::cameraViewToWorld(const vpgl_perspective_camera<double> & camera)
{
    // homography from P matrix
    vnl_matrix_fixed<double, 3, 4> P = camera.get_matrix();
    double data[3][3] = {0};
    for (int i = 0; i<3; i++) {
        data[i][0] = P(i, 0);
        data[i][1] = P(i, 1);
        data[i][2] = P(i, 3);
    }
    vgl_h_matrix_2d<double> H(&(data[0][0]));
    vgl_h_matrix_2d<double> inV = H.get_inverse();
    return inV;
}

vgl_h_matrix_2d<double> VpglPlus::homographyFromCameraToCamera(const vpgl_perspective_camera<double> & cameraA, const vpgl_perspective_camera<double> & cameraB)
{
    vnl_matrix<double> K1 = cameraA.get_calibration().get_matrix().as_matrix();
    vnl_matrix<double> R1 = cameraA.get_rotation().as_matrix().as_matrix();
    vnl_matrix<double> K2 = cameraB.get_calibration().get_matrix().as_matrix();
    vnl_matrix<double> R2 = cameraB.get_rotation().as_matrix().as_matrix();
    
    // H2 * H1^{-1}
    vnl_matrix<double> H = K2 * R2 * vnl_matrix_inverse<double>(K1*R1);
    H /= H(2, 2);
    assert(H.rows() == 3 && H.cols() == 3);
    return vgl_h_matrix_2d<double>(vnl_matrix_fixed<double,3,3>(H));
}

class calibrate_pure_rotate_camera_residual : public vnl_least_squares_function
{
    protected:
    const vcl_vector<vgl_point_2d<double> > pts1_;
    const vcl_vector<vgl_point_2d<double> > pts2_;
    const vnl_matrix_fixed<double, 3, 3> invR1_;
    const vnl_matrix_fixed<double, 3, 3> invK1_;
    const vgl_point_2d<double> pp_;   // principle point

public:
    calibrate_pure_rotate_camera_residual(const vcl_vector<vgl_point_2d<double> > & pts1,
                                           const vcl_vector<vgl_point_2d<double> > & pts2,
                                           const vnl_matrix_fixed<double, 3, 3> & invR1,
                                           const vnl_matrix_fixed<double, 3, 3> & invK1,
                                           const vgl_point_2d<double> & pp):
    vnl_least_squares_function(4, 2*(int)pts1.size(), no_gradient),
    pts1_(pts1),
    pts2_(pts2),
    invR1_(invR1),
    invK1_(invK1),
    pp_(pp)
    {
        assert(pts1.size() >= 4);
        assert(pts1.size() == pts2.size());
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double fl = x[0];
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double> R2(rod);
        vnl_matrix_fixed<double, 3, 3> R2mat = R2.as_matrix();
        vnl_matrix_fixed<double, 3, 3> K2;
        K2.fill(0);
        K2(0, 0) = K2(1, 1) = fl;
        K2(0, 2) = pp_.x();
        K2(1, 2) = pp_.y();
        K2(2, 2) = 1.0;
        
        // x2 = K_2 * R_2 * R_1^{-1} * K_1^{-1} x1
        int idx = 0;
        for (int i = 0; i<pts1_.size(); i++) {
            vnl_vector_fixed<double, 3> p(pts1_[i].x(), pts1_[i].y(), 1.0);
            vnl_vector_fixed<double, 3> q = K2 * R2mat * invR1_ * invK1_ * p;
            double x = q[0]/q[2];
            double y = q[1]/q[2];
            
            fx[idx] = pts2_[i].x() - x;
            idx++;
            fx[idx] = pts2_[i].y() - y;
            idx++;
        }
    }
    
    void setCameraMatrixRotation(vnl_vector<double> const &x, vpgl_perspective_camera<double> & camera)
    {
        double fl = x[0];
        vnl_vector_fixed<double, 3> rod(x[1], x[2], x[3]);
        vgl_rotation_3d<double> R2(rod);
        
        camera.set_calibration(vpgl_calibration_matrix<double>(fl, pp_));
        camera.set_rotation(R2);
    }
};

bool VpglPlus::calibrate_pure_rotate_camera(const vcl_vector<vgl_point_2d<double> > & pts1,
                                            const vcl_vector<vgl_point_2d<double> > & pts2,
                                            const vpgl_perspective_camera<double> & camera1,
                                            vpgl_perspective_camera<double> & camera2)
{
    assert(pts1.size() == pts2.size());
    assert(pts1.size() >= 4);
    
    vnl_matrix_fixed<double, 3, 3> invR1 = vnl_inverse(camera1.get_rotation().as_matrix());
    vnl_matrix_fixed<double, 3, 3> invK1 = vnl_inverse(camera1.get_calibration().get_matrix());
    vgl_point_2d<double> pp = camera1.get_calibration().principal_point();
    
    calibrate_pure_rotate_camera_residual residual(pts1, pts2, invR1, invK1, pp);
    
    vnl_vector<double> x(4);
    x[0] = camera1.get_calibration().get_matrix()[0][0];
    x[1] = camera1.get_rotation().as_rodrigues()[0];
    x[2] = camera1.get_rotation().as_rodrigues()[1];
    x[3] = camera1.get_rotation().as_rodrigues()[2];
    
    vnl_levenberg_marquardt lmq(residual);
    lmq.set_f_tolerance(0.0001);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    residual.setCameraMatrixRotation(x, camera2);
    camera2.set_camera_center(camera1.get_camera_center());
    return true;
}

class KfromP_residual : public vnl_least_squares_function
{
protected:
    vnl_matrix<double> PSPt_normalized_; // normalized by Frobenius norms
    const double u_;
    const double v_;
    
public:
    KfromP_residual(const vnl_matrix<double> & PSPt, const vgl_point_2d<double> &pp):
    vnl_least_squares_function(1, 9, no_gradient),
    PSPt_normalized_(PSPt),
    u_(pp.x()),
    v_(pp.y())
    {
        
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        double f = x[0];
        vnl_matrix<double> K(3, 3, 0);
        K(0, 0) = K(1, 1) = f;
        K(2, 2) = 1.0;
        K(0, 2) = u_;
        K(1, 2) = v_;
        
        vnl_matrix<double> KKt = K * K.transpose();
        double frob = 0.0;
        for (int r = 0; r<3; r++) {
            for (int c = 0; c<3; c++) {
                frob += KKt(r, c) * KKt(r, c);
            }
        }
        
        KKt = KKt/sqrt(frob);
        int idx = 0;
        for (int r = 0; r<3; r++) {
            for (int c = 0; c<3; c++) {
                fx[idx] = KKt(r, c) - PSPt_normalized_(r, c);
                idx++;
            }
        }
    }
};

// The original node is in matlab
//ART  Factorize camera matrix into intrinsic and extrinsic matrices
//
//[A,R,t] = art(P,fsign)  factorize the projection matrix P
//   as P=A*[R;t] and enforce the sign of the focal lenght to be fsign.
//   By default fsign=1.

// Author: A. Fusiello, 1999
//
// fsign tells the position of the image plane wrt the focal plane. If it is
// negative the image plane is behind the focal plane.

bool VpglPlus::KRTfromP(const vnl_matrix_fixed<double, 3, 4> &P,
                        vnl_matrix_fixed<double, 3, 3> & outK,
                        vnl_matrix_fixed<double, 3, 3> & outR,
                        vnl_vector_fixed<double, 3> & outT,
                        double fsign)
{
    //s = P(1:3,4);
    //Q = inv(P(1:3, 1:3));
    //[U,B] = qr(Q);
    
    vnl_matrix<double> s = P.extract(3, 1, 0, 3);
    vnl_matrix<double> Q = vnl_matrix_inverse<double>(P.extract(3, 3, 0, 0));
    vnl_qr<double> iqr(Q);
    vnl_matrix<double> U = iqr.Q();
    vnl_matrix<double> B = iqr.R();
    
    //% fix the sign of B(3,3). This can possibly change the sign of the resulting matrix,
    //% which is defined up to a scale factor, however.
    //sig = sign(B(3,3));
    //B=B*sig;
    //s=s*sig;
    if (fabs(B(2, 2)) <= 0.0000001) {
        return false;
    }
    double sig = 1.0;
    if (B(2, 2) < 0.0) {
        sig = -1.0;
    }
    B = sig * B;
    s = sig * s;
    
    //% if the sign of the focal lenght is not the required one,
    //    % change it, and change the rotation accordingly.
    //    if fsign*B(1,1) < 0
    //        E= [-1     0     0
    //            0    1     0
    //            0     0     1];
    // B = E*B;
    // U = U*E;
    // end
    
    if (fsign * B(0, 0) < 0.0) {
        vnl_matrix<double> E(3, 3);
        E.fill(0.0);
        E(0, 0) = -1.0;
        E(1, 1) =  1.0;
        E(2, 2) =  1.0;
        B = E * B;
        U = U * E;
    }
    
    //if fsign*B(2,2) < 0
    //    E= [1     0     0
    //        0    -1     0
    //        0     0     1];
    // B = E*B;
    // U = U*E;
    // end
    if (fsign * B(1, 1) < 0.0) {
        vnl_matrix<double> E(3, 3);
        E.fill(0.0);
        E(0, 0) = 1.0;
        E(1, 1) = -1.0;
        E(2, 2) = 1.0;
        B = E * B;
        U = U * E;
    }
    
    //% if U is not a rotation, fix the sign. This can possibly change the sign
    //    % of the resulting matrix, which is defined up to a scale factor, however.
    //    if det(U)< 0
    //        U = -U;
    // s= - s;
    //end
    if (vnl_determinant(U) < 0) {
        U = - U;
        s = - s;
    }
    
    //vcl_cout<<"B is \n"<<B<<vcl_endl;
    
    // % sanity check
    // if (norm(Q-U*B)>1e-10) & (norm(Q+U*B)>1e-10)
    //    error('Something wrong with the QR factorization.'); end
    if ((Q - U*B).array_two_norm() > 1e-10 && (Q + U*B).array_two_norm() > 1e-10)
    {
        printf("Something wrong with the QR factorization.\n");
        return false;
    }
    
    //R = U';
    //t = B*s;
    //A = inv(B);
    //A = A ./A(3,3);
    vnl_matrix<double> R = U.transpose();
    vnl_matrix<double> t = B * s;
    vnl_matrix<double> A = vnl_matrix_inverse<double>(B);
    A /= A(2, 2);
    
    //vcl_cout<<"A is \n"<<A<<vcl_endl;
    
    //  if det(R) < 0 error('R is not a rotation matrix'); end
    //  if A(3,3) < 0 error('Wrong sign of A(3,3)'); end
    if (vnl_determinant(R) < 0.0) {
        printf("error: R is not a rotation matrix\n");
        return false;
    }
    if (A(2, 2) < 0.0) {
        printf("error: Wrong sign of K matirx (2, 2)\n");
        return false;
    }
    
    //% this guarantee that the result *is* a factorization of the given P, up to a scale factor
    //W = A*[R,t];
    //if (rank([P(:), W(:)]) ~= 1 )
    //    error('Something wrong with the ART factorization.'); end

    vnl_matrix<double> Rt(3, 4);
    Rt.update(R, 0, 0);
    assert(t.rows() == 3 && t.cols() == 1);
    Rt.update(t, 0, 3);
    vnl_matrix<double> W = A *Rt;
    
    vnl_vector<double> Pvec(12);
    vnl_vector<double> Wvec(12);
    for (int i = 0; i<12; i++) {
        int r = i/4;
        int c = i%4;
        Pvec[i] = P(r, c);
        Wvec[i] = W(r, c);
    }
    Pvec = Pvec.normalize();
    Wvec = Wvec.normalize();
    double dif_norm = (Pvec-Wvec).two_norm();
    if (dif_norm > 0.000001) {
        printf("Warning: Something wrong with the ART factorization. L2 norm difference is %f\n", dif_norm);
        vcl_cout<<"P   is : \n"<<P<<vcl_endl;
        vcl_cout<<"KRT is : \n"<<W<<vcl_endl;
        return false;
    }
    
    // output
    outK = A;
    outK(1, 0) = outK(2, 0) = 0.0;
    outK(0, 1) = 0.0;
    outR = R;
    outT[0] = t(0, 0);
    outT[1] = t(1, 0);
    outT[2] = t(2, 0);
    return true;
}

bool VpglPlus::KfromP(const vnl_matrix_fixed<double, 3, 4> & P, const vgl_point_2d<double> & pp, vnl_matrix_fixed<double, 3, 3> & K)
{
    vnl_matrix<double> Pmat = P.as_matrix();
    vnl_matrix<double> Simga(4, 4, 0);
    Simga(0, 0) = 1.0;
    Simga(1, 1) = 1.0;
    Simga(2, 2) = 1.0;
    vnl_matrix<double> PSPt = Pmat * Simga * Pmat.transpose();
    // choleskey decompositon as the initial guess of focal lenth
    
    vnl_cholesky chol(PSPt);
    vnl_matrix<double> upTri = chol.upper_triangle();
    assert(upTri.rows() == 3 && upTri.cols() == 3);
    
    double fx = upTri(0, 0)/upTri(2, 2);
    double fy = upTri(1, 1)/upTri(2, 2);
    if (fx < 0 || fy < 0) {
        return false;
    }
    
    // normalization
    double frob = 0.0;
    for (int r = 0; r<3; r++) {
        for (int c = 0; c<3; c++) {
            frob += PSPt(r, c) * PSPt(r, c);
        }
    }
    PSPt = PSPt/sqrt(frob);
    
    //
    KfromP_residual residual(PSPt, pp);
    vnl_vector<double> x(1, 0.0);
    x[0] = (fx + fy)/2.0;                       //constant order first
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        vcl_cerr<<"x = "<<x<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    K.set_identity();
    K(0, 0) = K(1, 1) = x[0];
    K(0, 2) = pp.x();
    K(1, 2) = pp.y();
    return true;
}

bool VpglPlus::rectifyStereoImage(const vpgl_perspective_camera<double> & leftP, const vpgl_perspective_camera<double> & rightP,
                                  const vgl_point_2d<double> & d1, const vgl_point_2d<double> & d2,
                                  vnl_matrix_fixed<double, 3, 4> & leftProj, vnl_matrix_fixed<double, 3, 4> & rightProj,
                                  vnl_matrix_fixed<double, 3, 3> & leftTransform, vnl_matrix_fixed<double, 3, 3> & rightTransform)
{
    // % factorise old PPM
    // [A1,R1,t1] = art(Po1);
    // [A2,R2,t2] = art(Po2);
    vnl_matrix<double> A1 = leftP.get_calibration().get_matrix().as_matrix();
    vnl_matrix<double> R1 = leftP.get_rotation().as_matrix().as_matrix();
    vnl_matrix<double> A2 = rightP.get_calibration().get_matrix().as_matrix();
    vnl_matrix<double> R2 = rightP.get_rotation().as_matrix().as_matrix();
    
    vnl_matrix<double> Po1 = leftP.get_matrix().as_matrix(); // 3 x 4 matrix
    vnl_matrix<double> Po2 = rightP.get_matrix().as_matrix();
    // % optical centers (unchanged)
    // c1 = - R1'*inv(A1)*Po1(:,4);
    // c2 = - R2'*inv(A2)*Po2(:,4);
    vnl_matrix<double> c1 = - R1.transpose()*vnl_matrix_inverse<double>(A1)*Po1.extract(3, 1, 0, 3);
    vnl_matrix<double> c2 = - R2.transpose()*vnl_matrix_inverse<double>(A2)*Po2.extract(3, 1, 0, 3);
    
    //vcl_cout<<"c1 is "<<c1<<vcl_endl;
    //vcl_cout<<"c2 is "<<c2<<vcl_endl;
    
    //% new x axis (baseline, from c1 to c2)
    //v1 = (c2-c1);
    //% new y axes (orthogonal to old z and new x)
    //v2 = cross(R1(3,:)',v1);
    //         % new z axes (no choice, orthogonal to baseline and y)
    //           v3 = cross(v1,v2);
    vnl_vector<double> v1 = (c2 - c1).get_column(0);
    vnl_vector<double> v2 = vnl_cross_3d(R1.get_row(2), v1);
    vnl_vector<double> v3 = vnl_cross_3d(v1, v2);
    
    
    
    //% new extrinsic (translation unchanged)
    //R = [v1'/norm(v1)
    //     v2'/norm(v2)
    //     v3'/norm(v3)];
    //v1 = v1.normalize();
    //v2 = v2.normalize();
    //v3 = v3.normalize();
    //vcl_cout<<"v1 is"<<v1<<vcl_endl;
    //vcl_cout<<"v2 is"<<v2<<vcl_endl;
    //vcl_cout<<"v3 is"<<v3<<vcl_endl;
    
    v1 = v1/v1.two_norm();
    v2 = v2/v2.two_norm();
    v3 = v3/v3.two_norm();
    vnl_matrix<double> R(3, 3);
    R.set_row(0, v1);
    R.set_row(1, v2);
    R.set_row(2, v3);
    
    vcl_cout<<"R is \n"<<R<<vcl_endl;
    //% new intrinsic (arbitrary)
    //An1 = A2;
    //An1(1,2)=0;
    //An2 = A2;
    //An2(1,2)=0;
    vnl_matrix<double> An1 = A2;
    An1(0, 1) = 0.0;
    vnl_matrix<double> An2 = A2;
    An2(0, 1) = 0.0;
    
    //% translate image centers
    //An1(1,3)=An1(1,3)+d1(1);
    //An1(2,3)=An1(2,3)+d1(2);
    //An2(1,3)=An2(1,3)+d2(1);
    //An2(2,3)=An2(2,3)+d2(2);
    
    An1(0,2) += d1.x();
    An1(1,2) += d1.y();
    An2(0,2) += d2.x();
    An2(1,2) += d2.y();
    
    //% new projection matrices
    //Pn1 = An1 * [R -R*c1 ];
    //Pn2 = An2 * [R -R*c2 ];
    vnl_matrix<double> tmp1(3, 4);
    tmp1.update(R, 0, 0);
    tmp1.set_column(3, (-R*c1).get_column(0));
    vnl_matrix<double> Pn1 = An1 * tmp1;
    vnl_matrix<double> tmp2(3, 4);
    tmp2.update(R, 0, 0);
    tmp2.set_column(3, (-R*c2).get_column(0));
    vnl_matrix<double> Pn2 = An2 * tmp2;
    
    //% rectifying image transformation
    //T1 = Pn1(1:3,1:3)* inv(Po1(1:3,1:3));
    //T2 = Pn2(1:3,1:3)* inv(Po2(1:3,1:3));
    vnl_matrix<double> T1 = Pn1.extract(3, 3, 0, 0) * vnl_matrix_inverse<double>(Po1.extract(3, 3, 0, 0));
    vnl_matrix<double> T2 = Pn2.extract(3, 3, 0, 0) * vnl_matrix_inverse<double>(Po2.extract(3, 3, 0, 0));
    
    vcl_cout<<"P1 is:\n "<<Pn1<<vcl_endl;
    vcl_cout<<"P2 is:\n "<<Pn2<<vcl_endl;
    vcl_cout<<"T1 is: \n"<<T1<<vcl_endl;
    vcl_cout<<"T2 is: \n"<<T2<<vcl_endl;
    
    // output
    leftProj  = vnl_matrix_fixed<double, 3, 4>(Pn1);
    rightProj = vnl_matrix_fixed<double, 3, 4>(Pn2);
    
    //
    assert(T1.rows() == 3 && T1.cols() == 3);
    assert(T2.rows() == 3 && T2.cols() == 3);
    
    leftTransform  = T1;
    rightTransform = T2;     
    return true;
}

class DistanceNode
{
public:
    int from_node_id_;
    int to_node_id_;
    double distance_;
    
    bool operator< (const DistanceNode & node) const
    {
        return distance_ < node.distance_;
    }
};

int VpglPlus::unique_match_number(const vpgl_perspective_camera<double> & camera,
                                  const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                  const vcl_vector<vgl_point_2d<double> > & img_pts,
                                  double inlier_distance)
{
    assert(wld_pts.size() > 0 && img_pts.size() > 0);
    
    // project to image
    vcl_vector<vgl_point_2d<double> > proj_pts;
    for (int i = 0; i<wld_pts.size(); i++) {
        vgl_point_2d<double> p = (vgl_point_2d<double>)camera.project(wld_pts[i]);
        proj_pts.push_back(p);
    }
    // distance from img_pts to proj_pts
    vcl_vector<DistanceNode> nodes;
    for (int i = 0; i<img_pts.size(); i++) {
        for (int j = 0; j<proj_pts.size(); j++) {
            double dis = vgl_distance(img_pts[i], proj_pts[j]);
            if (dis < inlier_distance) {
                DistanceNode node;
                node.from_node_id_ = i;
                node.to_node_id_ = j;
                node.distance_ = dis;
                nodes.push_back(node);
            }
        }
    }
    vcl_sort(nodes.begin(), nodes.end());
    
    vcl_vector<bool> used_img_pts(img_pts.size(), false);
    vcl_vector<bool> used_proj_pts(proj_pts.size(), false);
    
    // remove edges from shortest
    int edge_num = 0;
    for (int i = 0; i<nodes.size(); i++) {
        int from = nodes[i].from_node_id_;
        int to   = nodes[i].to_node_id_;
        // both nodes are used
        if (!used_img_pts[from] && !used_proj_pts[to]) {
            edge_num++;
            used_img_pts[from] = true;
            used_proj_pts[to] = true;
        }
        if (edge_num >= img_pts.size() || edge_num >= used_proj_pts.size()) {
            break;
        }
    }
    
    return edge_num;
}


bool VpglPlus::writeCamera(const char *fileName, const char *imageName, const vpgl_perspective_camera<double> & camera)
{
    assert(fileName);
    assert(imageName);
    
    FILE *pf = fopen(fileName, "w");
    if (!pf) {
        printf("can not create file %s\n", fileName);
        return false;
    }
    fprintf(pf, "%s\n", imageName);
    fprintf(pf, "ppx\t ppy\t focal length\t Rx\t Ry\t Rz\t Cx\t Cy\t Cz\n");
    double ppx = camera.get_calibration().principal_point().x();
    double ppy = camera.get_calibration().principal_point().y();
    double fl = camera.get_calibration().get_matrix()[0][0];
    double Rx = camera.get_rotation().as_rodrigues()[0];
    double Ry = camera.get_rotation().as_rodrigues()[1];
    double Rz = camera.get_rotation().as_rodrigues()[2];
    double Cx = camera.get_camera_center().x();
    double Cy = camera.get_camera_center().y();
    double Cz = camera.get_camera_center().z();
    fprintf(pf, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", ppx, ppy, fl, Rx, Ry, Rz, Cx, Cy, Cz);
    fclose(pf);
    return true;
}

bool VpglPlus::readCamera(const char *fileName, vcl_string & imageName, vpgl_perspective_camera<double> & camera)
{
    assert(fileName);
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        printf("can not open file %s\n", fileName);
        return false;
    }
    char buf[1024] = {NULL};
    int num = fscanf(pf, "%s\n", buf);
    assert(num == 1);
    imageName = vcl_string(buf);
    for (int i = 0; i<1; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    double ppx, ppy, fl, rx, ry, rz, cx, cy, cz;
    int ret = fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &ppx, &ppy, &fl, &rx, &ry, &rz, &cx, &cy, &cz);
    if (ret != 9) {
        printf("Error: read camera parameters!\n");
        return false;
    }
    
    vpgl_calibration_matrix<double> K(fl, vgl_point_2d<double>(ppx, ppy));
    vnl_vector_fixed<double, 3> rod(rx, ry, rz);
    vgl_rotation_3d<double> R(rod);
    vgl_point_3d<double> cc(cx, cy, cz);
    
    camera.set_calibration(K);
    camera.set_rotation(R);
    camera.set_camera_center(cc);
    fclose(pf);
    
    return true;
}

bool VpglPlus::readPTZCamera(const char *fileName, vcl_string & imageName,
                             vnl_vector_fixed<double, 3> &ptz,
                             vgl_point_2d<double> & pp,
                             vgl_point_3d<double> & cc,
                             vnl_vector_fixed<double, 3> & rod)
{
    assert(fileName);
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        printf("can not open file %s\n", fileName);
        return false;
    }
    char buf[1024] = {NULL};
    int num = fscanf(pf, "%s\n", buf);
    assert(num == 1);
    imageName = vcl_string(buf);
    for (int i = 0; i<1; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    double ppx, ppy, pan, tilt, fl, ccx, ccy, ccz, rodx, rody, rodz;
    num = fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                 &ppx, &ppy, &pan, &tilt, &fl, &ccx, &ccy, &ccz, &rodx, &rody, &rodz);
    assert(num == 11);
    pp = vgl_point_2d<double>(ppx, ppy);
    ptz[0] = pan;
    ptz[1] = tilt;
    ptz[2] = fl;
    cc = vgl_point_3d<double>(ccx, ccy, ccz);
    rod[0] = rodx;
    rod[1] = rody;
    rod[2] = rodz;
    fclose(pf);    
    return true;    
}

bool VpglPlus::writePTZCamera(const char *fileName, const char *imageName, const vnl_vector_fixed<double, 3> &ptz,
                              const vgl_point_2d<double> & pp,
                              const vgl_point_3d<double> & cc,
                              const vnl_vector_fixed<double, 3> & rod)
{
    FILE *pf = fopen(fileName, "w");
    assert(pf);
    fprintf(pf, "%s\n", imageName);
    fprintf(pf, "ppx	 ppy	 pan	tilt	focal_length	cc_x	cc_y	cc_z	rod_x	 rod_y	 rod_z\n");
    fprintf(pf, "%f %f\t", pp.x(), pp.y());
    fprintf(pf, "%f %f %f\t", ptz[0], ptz[1], ptz[2]);
    fprintf(pf, "%f %f %f\t", cc.x(), cc.y(), cc.z());
    fprintf(pf, "%f %f %f\n", rod[0], rod[1], rod[2]);
    fclose(pf);
    return true;
}




















