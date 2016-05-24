//
//  vxl_mv_line3d.cpp
//  QuadCopter
//
//  Created by jimmy on 6/11/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vxl_mv_line3d.h"
#include <vcl_list.h>
#include <vgl/algo/vgl_intersection.h>
#include <vgl/vgl_plane_3d.h>
#include <vpgl/algo/vpgl_camera_compute.h>
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vgl/vgl_distance.h>
#include <vcl_iostream.h>
#include <vgl/vgl_closest_point.h>


class reconstruct_line_residual: public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_line_segment_2d<double> > line_segments_;
    const vcl_vector<vpgl_perspective_camera<double> > cameras_;
    //vgl_infinite_line_3d<double> line_3d_;  // 5 parameter, over-parameted
    
public:
    reconstruct_line_residual(const vcl_vector<vgl_line_segment_2d<double> > & line_segments,
                              const vcl_vector<vpgl_perspective_camera<double> > & cameras):
    vnl_least_squares_function(5, (int)line_segments.size() * 2, no_gradient),
    line_segments_(line_segments),
    cameras_(cameras)
    {
        assert(line_segments_.size() >= 3);
        assert(line_segments_.size() == cameras_.size());
    }
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        // minimize the distance from the line segment end points to projected (2D) line
        vgl_vector_2d<double> x_0(x[0], x[1]);
        vgl_vector_3d<double> dir(x[2], x[3], x[4]);
        vgl_infinite_line_3d<double> line_3d(x_0, dir);
        int idx = 0;
        for (int i = 0; i<cameras_.size(); i++) {
            // project 3d line to image space
            vgl_line_2d<double> line2d = cameras_[i].project(line_3d);
            vgl_point_2d<double> p1 = line_segments_[i].point1();
            vgl_point_2d<double> p2 = line_segments_[i].point2();
            fx[idx] = vgl_distance(line2d, p1);
            idx++;
            fx[idx] = vgl_distance(line2d, p2);
            idx++;
        }
    }
};


bool VxlMVLine3D::reconstruct_line(const vcl_vector<vgl_line_segment_2d<double> > & line_segments,
                                     const vcl_vector<vpgl_perspective_camera<double> > & cameras,
                                     vgl_infinite_line_3d<double> &line_3d)
{
    assert(line_segments.size() == cameras.size());
    assert(line_segments.size() >= 3);
    
    // initial guess of the 3D line
    vcl_list<vgl_plane_3d<double> > planes;
    for (unsigned int i = 0; i<cameras.size(); i++) {
        vgl_homg_point_2d<double> p1 = vgl_homg_point_2d<double>(line_segments[i].point1());
        vgl_homg_point_2d<double> p2 = vgl_homg_point_2d<double>(line_segments[i].point2());
        vgl_homg_line_2d<double> line(p1, p2);
   
        vgl_homg_plane_3d<double> curPlane = cameras[i].vpgl_proj_camera<double>::backproject(line);
        planes.push_back(vgl_plane_3d<double>(curPlane));
    }   
    
    // common line of multiple plane intersection
    vgl_infinite_line_3d<double> init_line = vgl_intersection<double>(planes);
    reconstruct_line_residual residual(line_segments, cameras);
    // update the 3d line
    vnl_vector<double> x(5);
    x[0] = init_line.x0().x();
    x[1] = init_line.x0().y();
    x[2] = init_line.direction().x();
    x[3] = init_line.direction().y();
    x[4] = init_line.direction().z();
   
    vnl_levenberg_marquardt lmq(residual);
    lmq.set_f_tolerance(0.001);
    bool isMinimized = lmq.minimize(x);
    if (!isMinimized) {
        printf("Error: minimization failed!\n");
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    vgl_vector_2d<double> x_0(x[0], x[1]);
    vgl_vector_3d<double> dir(x[2], x[3], x[4]);
    line_3d = vgl_infinite_line_3d<double>(x_0, dir);
    return true;
}

static bool replaceEndPoint(const vgl_point_3d<double> &p, vgl_line_segment_3d<double> & seg)
{
    double dis  = vgl_distance(seg.point1(), seg.point2());
    double dis1 = vgl_distance(p, seg.point1());
    double dis2 = vgl_distance(p, seg.point2());
    if (dis < dis1 && dis < dis2)
    {
        if (dis1 > dis2) {
            seg = vgl_line_segment_3d<double>(seg.point1(), p);
        }
        else
        {
            seg = vgl_line_segment_3d<double>(p, seg.point2());
        }
    }
    else if (dis1 > dis) {
        seg = vgl_line_segment_3d<double>(seg.point1(), p);
    }
    else if(dis2 > dis)
    {
        seg = vgl_line_segment_3d<double>(p, seg.point2());
    }
    return true;
}

bool VxlMVLine3D::longest_line_segment(const vcl_vector<vpgl_perspective_camera<double> > & cameras,
                                           const vcl_vector<vgl_line_segment_2d<double> > & segments_2d,
                                           const vgl_infinite_line_3d<double> & line_3d,
                                           vgl_line_segment_3d<double> & segment_3d)
{
    assert(cameras.size() > 0);
    assert(cameras.size() == segments_2d.size());
    
    vgl_line_segment_3d<double> seg;
    // 3D line, the segment is supposed to lay in this line
    vgl_line_3d_2_points<double> line(line_3d.point(), line_3d.point_t(1.0));
    for (int i = 0; i<cameras.size(); i++) {
        vgl_line_3d_2_points<double> line1 = cameras[i].backproject(segments_2d[i].point1());
        vgl_line_3d_2_points<double> line2 = cameras[i].backproject(segments_2d[i].point2());
        
        vcl_pair<vgl_point_3d<double>, vgl_point_3d<double> > pt_pair1 = vgl_closest_points(line, line1);
        vcl_pair<vgl_point_3d<double>, vgl_point_3d<double> > pt_pair2 = vgl_closest_points(line, line2);
        if (i == 0) {
            seg = vgl_line_segment_3d<double>(pt_pair1.first, pt_pair2.second);
        }
        else
        {
            vgl_point_3d<double> p1 = pt_pair1.first;
            vgl_point_3d<double> p2 = pt_pair2.first;
            
            replaceEndPoint(p1, seg);
            replaceEndPoint(p2, seg);
        }
    }
    segment_3d = seg;
    return true;
}






