//
//  vgl_plus.cpp
//  CameraPlaning
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vgl_plus.h"

#include <vgl/io/vgl_io_point_2d.h>
#include <vnl/io/vnl_io_matrix.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vgl/vgl_point_3d.h>

#include <vgl/vgl_intersection.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vgl/vgl_line_2d.h>

#include <vgl/vgl_polygon.h>

#include <vgl/vgl_distance.h>
#include <vnl/vnl_math.h>
#include <vil/vil_image_view.h>
#include <vcl_algorithm.h>
#include <vgl/algo/vgl_homg_operators_2d.h>
#include <vgl/vgl_closest_point.h>

#include "vnl_plus.h"
#include "vil_plus.h"

/*
 
 -------------------------------VglPlus-----------------------------------------
 
 */

void VglPlus::savePointVector(const char *fileName, vcl_vector<vgl_point_2d<double>> &pts)
{
    vsl_b_ofstream bfs_out(fileName);
    vcl_cout<<"save point vector, size is "<<pts.size()<<vcl_endl;
    
    for (int i = 0; i<pts.size(); i++) {
        vsl_b_write(bfs_out, pts[i]);
    }
    bfs_out.close();
}

void VglPlus::loadPointVector(const char *fileName, vcl_vector<vgl_point_2d<double>> &pts, int pts_num)
{
    vsl_b_ifstream bfs_in(fileName);
    assert(bfs_in.is().good() == true);
    
    pts.resize(pts_num);
    // && bfs_in.is().eof() != true not work
    for (int i = 0; i < pts_num ; i++) {
        vsl_b_read(bfs_in, pts[i]);
        assert(bfs_in.is().good() == true);
    }
    bfs_in.close();
}

void VglPlus::saveMatrix(const char *fileName, const vnl_matrix<double> & data)
{
    vsl_b_ofstream bfs_out(fileName);
    vcl_cout<<"save to "<<fileName<<vcl_endl;
    
    vsl_b_write(bfs_out, data);
    
    bfs_out.close();
}

void VglPlus::loadMatrix(const char *fileName, vnl_matrix<double> &data)
{
    vsl_b_ifstream bfs_in(fileName);
    vcl_cout<<"load from "<<fileName<<vcl_endl;
    
    vsl_b_read(bfs_in, data);
    
    bfs_in.close();
}

void VglPlus::saveVector(const char *matName, const vcl_vector<double> & data)
{
    assert(data.size() > 0);
    vnl_matlab_filewrite writer(matName);
    vnl_vector<double> dataVec(&data[0], (int)data.size());
    writer.write(dataVec, "data");
    printf("save to file: %s\n", matName);
}

void VglPlus::savePointVector(const char *fileName, const char *imageName, const vcl_vector<vgl_point_2d<double>> &wldPts, const vcl_vector< vgl_point_2d<double> > &imgPts)
{
    assert(wldPts.size() == imgPts.size());
    
    vsl_b_ofstream bfs_out(fileName);
    
    vsl_b_write(bfs_out, imageName);          // image name
    vsl_b_write(bfs_out, (int)wldPts.size()); // pts number
    
    for (int i = 0; i<wldPts.size(); i++) {
        vsl_b_write(bfs_out, wldPts[i]);
    }
    for (int i = 0; i<imgPts.size(); i++) {
        vsl_b_write(bfs_out, imgPts[i]);
    }
    
    bfs_out.close();
}

void VglPlus::loadPointVector(const char *fileName, vcl_string &imageName, vcl_vector<vgl_point_2d<double>> &wldPts, vcl_vector< vgl_point_2d<double> > &imgPts)
{
    assert(wldPts.size() == 0);
    assert(imgPts.size() == 0);
    
    vsl_b_ifstream bfs_in(fileName);
    assert(bfs_in.is().good() == true);
    
    char strName[1024] = {NULL};
    vsl_b_read(bfs_in, strName);
    imageName = vcl_string(strName);
    
    int num = 0;
    vsl_b_read(bfs_in, num);
    wldPts.resize(num);
    imgPts.resize(num);
    
    for (int i = 0; i<num; i++) {
        vsl_b_read(bfs_in, wldPts[i]);
    }
    for (int i = 0; i<num; i++) {
        vsl_b_read(bfs_in, imgPts[i]);
    }
    bfs_in.close();
}

void VglPlus::generateBox(const vgl_point_3d<double> & center, const vnl_vector_fixed<double, 3> & xyz_size, vcl_vector<vgl_point_3d<double> > & vertex)
{
    
    int sign[][3] = {-1, -1, -1,
        1, -1, -1,
        1, 1, -1,
        -1, 1, -1,
        -1, -1, 1,
        1, -1, 1,
        1, 1, 1,
        -1, 1, 1};
    double x_s = xyz_size[0];
    double y_s = xyz_size[1];
    double z_s = xyz_size[2];
    for (int i = 0; i<8; i++) {
        double x = center.x() + sign[i][0] * 0.5 * x_s;
        double y = center.y() + sign[i][1] * 0.5 * y_s;
        double z = center.z() + sign[i][2] * 0.5 * z_s;
        vertex.push_back(vgl_point_3d<double>(x, y, z));
    }
    assert(vertex.size() == 8);
}

bool VglPlus::uniqueMutualMatch(const vcl_vector<vgl_point_2d<double> > & pts1, const vcl_vector<vgl_point_2d<double> > & pts2, vcl_vector<int> & matchedIndex)
{
    assert(pts1.size() == pts2.size());
    
    vcl_vector<int> index1;  // min distance point index in pts2
    vcl_vector<int> index2;
    for (int i = 0; i<pts1.size(); i++) {
        double x = pts1[i].x();
        double y = pts1[i].y();
        int min_idx    = -1;
        double min_dis = INT_MAX;
        for (int j = 0; j<pts2.size(); j++) {
            double dif_x = x - pts2[j].x();
            double dif_y = y - pts2[j].y();
            double dis = dif_x * dif_x + dif_y * dif_y;
            if (dis < min_dis) {
                min_dis = dis;
                min_idx = j;
            }
        }
        if (min_idx != -1) {
            index1.push_back(min_idx);
        }
        else
        {
            return false;
        }
    }
    
    for (int i = 0; i<pts2.size(); i++) {
        double x = pts2[i].x();
        double y = pts2[i].y();
        int min_idx    = -1;
        double min_dis = INT_MAX;
        for (int j = 0; j<pts1.size(); j++) {
            double dif_x = x - pts1[j].x();
            double dif_y = y - pts1[j].y();
            double dis = dif_x * dif_x + dif_y * dif_y;
            if (dis < min_dis) {
                min_dis = dis;
                min_idx = j;
            }
        }
        if (min_idx != -1) {
            index2.push_back(min_idx);
        }
        else
        {
            return false;
        }
    }
    assert(index1.size() == index2.size());
    
    for (int i = 0; i<index1.size(); i++) {
        if (index2[index1[i]] != i) {
            return false;
        }
    }
    matchedIndex = index1;
    return true;
}
vgl_h_matrix_2d<double> VglPlus::homography(const vcl_vector<vgl_point_2d<double> > &from_pts,
                                            const vcl_vector<vgl_point_2d<double> > &to_pts)
{
    assert(from_pts.size() == to_pts.size());
    assert(from_pts.size() >= 4);
    
    vcl_vector<vgl_homg_point_2d<double> > pts1;
    vcl_vector<vgl_homg_point_2d<double> > pts2;
    
    for (int j = 0; j<from_pts.size(); j++) {
        pts1.push_back(vgl_homg_point_2d<double>(from_pts[j].x(), from_pts[j].y(), 1.0));
        pts2.push_back(vgl_homg_point_2d<double> (to_pts[j].x(), to_pts[j].y(), 1.0));
    }
    
    vgl_h_matrix_2d<double> H = vgl_h_matrix_2d<double>(pts1, pts2);
    return H;
}

bool VglPlus::intersectionFromLineSet(const vcl_vector<vgl_line_2d<double> > & lines, vgl_point_2d<double> & orthocenter)
{
    assert(lines.size() >= 2);
    
    int num = 0;
    double px = 0.0;
    double py = 0.0;
    for (int i = 0; i<lines.size(); i++) {
        for (int j = i+1; j<lines.size(); j++) {
            vgl_point_2d<double> p;
            bool isIntersect = vgl_intersection(lines[i], lines[j], p);
            if (isIntersect) {
                px += p.x();
                py += p.y();
                num++;
            }
        }
    }
    
    if (num >= 1) {
        orthocenter = vgl_point_2d<double>(px/num, py/num);
        return true;
    }
    else
    {
        return false;
    }
}

void VglPlus::parallelMove(const vgl_line_segment_2d<double> & initSeg, double distance,
                           vgl_line_segment_2d<double> & seg1, vgl_line_segment_2d<double> & seg2)
{
    // CCW rotated
    vgl_vector_2d<double> orthDir = rotated(initSeg.direction(), vnl_math::pi/2.0);
    orthDir = normalize(orthDir);
    
    vgl_point_2d<double> p1 = initSeg.point1();
    vgl_point_2d<double> p2 = initSeg.point2();
    
    vgl_vector_2d<double> dp = distance * orthDir;
    vgl_point_2d<double> p3(p1.x() + dp.x(), p1.y() + dp.y());
    vgl_point_2d<double> p4(p2.x() + dp.x(), p2.y() + dp.y());
    
    seg1 = vgl_line_segment_2d<double>(p3, p4);
    
    dp = -1.0 * dp;
    vgl_point_2d<double> p5(p1.x() + dp.x(), p1.y() + dp.y());
    vgl_point_2d<double> p6(p2.x() + dp.x(), p2.y() + dp.y());
    seg2 = vgl_line_segment_2d<double>(p5, p6);
}

bool VglPlus::mergeTwolineSegmentOnALine(const vgl_line_segment_2d<double> & seg1,
                                         const vgl_line_segment_2d<double> & seg2,
                                         vgl_line_segment_2d<double> & merged_seg)
{
    double cosVal = fabs(cos_angle(seg1.direction(), seg2.direction()));
    if (cosVal < cos(5.0/180.0*vnl_math::pi)) {
        return false;
    }   
    
    double dis_max = -1.0;
    vcl_vector<vgl_point_2d<double> > pts;
    pts.push_back(seg1.point1());
    pts.push_back(seg1.point2());
    pts.push_back(seg2.point1());
    pts.push_back(seg2.point2());
    // loop all combination and choose the longest one
    for (int i = 0; i<pts.size(); i++) {
        for (int j = i+1; j<pts.size(); j++) {
            double dis = vgl_distance(pts[i], pts[j]);
            if (dis > dis_max) {
                dis_max = dis;
                merged_seg = vgl_line_segment_2d<double>(pts[i], pts[j]);
            }
        }
    }
    return true;
}

//   3 2 1
//   4 * 0
//   5 6 7
// discrete direction from p1 to p2
int VglPlus::octaveDirection(double dx, double dy)
{
    
    int dir = 0;
    
    // printf("dx dy is %f %f\n", dx, dy);
    // horizontal
    if(VnlPlus::isEqualZero(dx))
    {
        if (dy >= 0) {
            dir = 6;
        }
        else
        {
            dir = 2;
        }
    }
    else if(VnlPlus::isEqualZero(dy))
    {
        if (dx >= 0) {
            dir = 0;
        }
        else
        {
            dir = 4;
        }
    }
    else
    {
        double tanV = dy/dx;
        //    printf("tan v is %f\n", tanV);
        if(tanV > -0.4142f && tanV <= 0.4142f)
        {
            if (dx > 0) {
                dir = 0;
            }
            else
            {
                dir = 4;
            }
        }
        else if(tanV > 0.4142f && tanV < 2.4142f)
        {
            if (dx < 0) {
                dir = 3;
            }
            else
            {
                dir = 7;
            }
        }
        else if(fabs(tanV) >= 2.4142f)
        {
            if (dy < 0) {
                dir = 2;
            }
            else
            {
                dir = 6;
            }
        }
        else if(tanV > -2.4142f && tanV <= -0.4142f)
        {
            if (dx > 0) {
                dir = 1;
            }
            else
            {
                dir = 5;
            }
        }
    }
    //   printf("tan dir is %d\n\n", dir);
    
    return dir;
}

bool VglPlus::isLeftSide(const vgl_line_segment_2d<double> & seg, const vgl_point_2d<double> & p)
{
    vgl_point_2d<double> b = seg.point1();
    vgl_point_2d<double> a = seg.point2();
    return ((b.x() - a.x())*(p.y() - a.y()) - (b.y() - a.y())*(p.x() - a.x())) > 0;
}


class LinePoints
{
public:
    vcl_vector<vgl_point_2d<double> > pts_;
    bool valid_;
    vgl_line_2d<double> line_;
    vgl_line_segment_2d<double> seg_;  // merged segment
    
    LinePoints()
    {
        valid_ = true;
    }
    
    bool operator >(const LinePoints & other) const
    {
        return pts_.size() > other.pts_.size();
    }
};



void VglPlus::clamp_box(vgl_box_2d<double> & box, double min_x, double min_y, double max_x, double max_y)
{
    double box_min_x = box.min_x();
    double box_min_y = box.min_y();
    double box_max_x = box.max_x();
    double box_max_y = box.max_y();
    box_min_x = (box_min_x > min_x)? box_min_x: min_x;
    box_min_y = (box_min_y > min_y)? box_min_y: min_y;
    box_max_x = (box_max_x < max_x)? box_max_x: max_x;
    box_max_y = (box_max_y < max_y)? box_max_y: max_y;
    
    box = vgl_box_2d<double>(box_min_x, box_max_x, box_min_y, box_max_y);
}




















