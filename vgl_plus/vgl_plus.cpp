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
#include <vgl/vgl_intersection+.h>
#include <vgl/vgl_distance.h>
#include <vnl/vnl_math.h>
#include <vil/vil_image_view.h>
#include <vcl_algorithm.h>
#include <vgl/algo/vgl_homg_operators_2d.h>
#include <vgl/vgl_fit_ellipse_2d.h>
#include <vgl/vgl_fit_line_2d.h>
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

int VglPlus::lineEllipseIntersection(const vgl_line_2d<double> & line, const vgl_ellipse_2d<double> & ellipse,
                                     vgl_point_2d<double> & pt1, vgl_point_2d<double> & pt2, bool isMajorAxis)
{
    vgl_polygon<double> poly = ellipse.polygon();
    assert(poly.num_sheets() == 1);
    int num = 0;
    //  printf("sheet number is %u\n", poly.num_sheets());
    const vcl_vector< vgl_point_2d< double > > sheet = poly[0];
    assert( sheet.size() > 1 );
    for ( unsigned int v = 0; v < sheet.size() - 1; v++ )
    {
        vgl_line_segment_2d< double > edge( sheet[v], sheet[v+1] );
        vgl_point_2d<double> p;
        bool isIntersection = vgl_intersection(line, edge, &p);
        if (isIntersection) {
            if (num == 0) {
                pt1 = p;
                num++;
            }
            else if(num == 1)
            {
                pt2 = p;
                num++;
            }
        }
        if (num == 2) {
            break;
        }
    }
    
    // last line segment
    if(num != 2)
    {
        vgl_line_segment_2d< double > edge( sheet.back(), sheet.front());
        vgl_point_2d<double> p;
        bool isIntersection = vgl_intersection(line, edge, &p);
        if (isIntersection) {
            if (num == 0) {
                pt1 = p;
                num++;
            }
            else if(num == 1)
            {
                pt2 = p;
                num++;
            }
        }
    }
    
    double disDif = 20;
    if (num == 2 && isMajorAxis) {
        double dis = vgl_distance(pt1, pt2);
        // distance of two points should be as the similar length of minor or major axis
        double dis1 = vgl_distance(ellipse.major_diameter().point1(), ellipse.major_diameter().point2());
        double dis2 = vgl_distance(ellipse.minor_diameter().point1(), ellipse.minor_diameter().point2());
        if (fabs(dis - dis1) >= disDif && fabs(dis - dis2) >= disDif) {
            num = 0;
        }
    }
    return num;
}

int VglPlus::sampleElliseInImage(const vgl_ellipse_2d<double> & ellipse, int sampleNum,
                                 int imageW, int imageH, vcl_vector<vgl_point_2d<double> > & pts)
{
    vgl_polygon<double> poly = ellipse.polygon(sampleNum);
    assert(poly.num_sheets() == 1);
    const vcl_vector< vgl_point_2d< double > > sheet = poly[0];
    for (int i = 0; i<sheet.size(); i++) {
        if (VglPlus::vgl_inside_image(sheet[i], imageW, imageH)) {
            pts.push_back(sheet[i]);
        }
    }
    return (int)pts.size();
}

bool VglPlus::lineElliseTangency(const vgl_line_2d<double> & line, const vgl_ellipse_2d<double> & ellipse, vgl_point_2d<double> & pt,
                                 double disThreshold, int sampleNum)
{
    printf("VglPlus::lineElliseTangency is un stable\n");
    assert(0);
    vgl_polygon<double> poly = ellipse.polygon(sampleNum);
    assert(poly.num_sheets() == 1);
    
    const vcl_vector< vgl_point_2d< double > > sheet = poly[0];
    assert( sheet.size() > 1 );
    
    double a = line.a();
    double b = line.b();
    double c = line.c();
    int positive_num = 0;
    int negative_num = 0;
    int minDis = INT_MAX;
    for (int i = 0; i<sheet.size(); i++) {
        double dis = vgl_distance(line, sheet[i]);
        if (dis < minDis) {
            minDis = dis;
            pt = sheet[i];
        }
        double nominator = a * sheet[i].x() + b * sheet[i].y() + c;
        if (nominator >= 0) {
            positive_num++;
        }
        else
        {
            negative_num++;
        }
    }
    // distance is small enough and most of the point in the one side of the line
    if (minDis < disThreshold && fabs(positive_num- negative_num)/sampleNum >= 0.99) {
        return true;
    }
    return false;
}

bool VglPlus::lineEllipseTangencyByProjection(int imageW, int imageH,
                                              const vgl_line_2d<double> & line,
                                              const vgl_ellipse_2d<double> & ellipseInImage,
                                              vgl_point_2d<double> & pt, int sampleNum)
{
    vil_image_view<vxl_byte> maskImage(imageW, imageH, 3);
    maskImage.fill(0);
    
    VilPlus::draw_line(maskImage, line, VilPlus::white(), 2);
    
    vgl_polygon<double> poly = ellipseInImage.polygon(sampleNum);
    assert(poly.num_sheets() == 1);
    
    const vcl_vector< vgl_point_2d< double > > sheet = poly[0];
    assert( sheet.size() > 1 );
    
    vcl_vector<vgl_point_2d<double>> pts;
    for (int i = 0; i<sheet.size(); i++) {
        int x = vnl_math::rnd_halfinttoeven(sheet[i].x());
        int y = vnl_math::rnd_halfinttoeven(sheet[i].y());
        if (x >= 0 && x < imageW &&
            y >= 0 && y < imageH) {
            if (maskImage(x, y) == 255) {
                pts.push_back(sheet[i]);
            }
        }
    }
    
    if (pts.size() == 0) {
        return false;
    }
    
    // caluclate average position
    double avgX = 0.0;
    double avgY = 0.0;
    for (int i =0; i<pts.size(); i++) {
        avgX += pts[i].x();
        avgY += pts[i].y();
    }
    avgX /= pts.size();
    avgY /= pts.size();
    
    double stdX = 0.0;
    double stdY = 0.0;
    for (int i = 0; i<pts.size(); i++) {
        stdX += (avgX - pts[i].x()) * (avgX - pts[i].x());
        stdY += (avgY - pts[i].y()) * (avgY - pts[i].y());
    }
    
    stdX = sqrt(stdX/pts.size());
    stdY = sqrt(stdY/pts.size());
    printf("std x, y is %f %f\n", stdX, stdY);
    
    pt = vgl_point_2d<double>(avgX, avgY);
    return true;
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

class EllipsePoints
{
public:
    vcl_vector<vgl_point_2d<double> > pts_;
    bool valid_;   // not be merged by other ellipse
    vgl_conic<double> conic_;
    
    EllipsePoints()
    {
        valid_ = false;
    }
    bool operator <(const EllipsePoints & other) const
    {
        return pts_.size() < other.pts_.size();
    }
    bool operator >(const EllipsePoints & other) const
    {
        return pts_.size() > other.pts_.size();
    }
};



bool VglPlus::mergeEllipse(const vcl_vector<vgl_ellipse_2d<double> > & ellipses, const vcl_vector< vcl_vector<vgl_point_2d<double> > > & ellipse_pts,
                           const double distance_threshold,
                           vcl_vector<vgl_ellipse_2d<double> > & merged_ellipse, vcl_vector<vcl_vector<vgl_point_2d<double> > > & merged_points)
{
    assert(ellipses.size() == ellipse_pts.size());
    
    const double inlier_ratio_threshold = 0.5; //
    
    // order ellipse by pixels numbers
    vcl_vector<EllipsePoints> conics;
    for (int i = 0; i<ellipses.size(); i++) {
        EllipsePoints elli;
        elli.pts_ = ellipse_pts[i];
        elli.valid_ = true;
        elli.conic_ = vgl_conic<double>(ellipses[i]);
        conics.push_back(elli);
    }
    vcl_sort(conics.begin(), conics.end(), vcl_greater<EllipsePoints>());
    
    // greedily merge
    for (int i = 0; i<conics.size(); i++) {
        for (int j = i+1; j<conics.size(); j++) {
            // merge j to i
            if (conics[i].valid_ && conics[j].valid_) {
                vgl_conic<double> cur_conic = conics[i].conic_;
                vgl_box_2d<double> cur_box  = vgl_homg_operators_2d<double>::compute_bounding_box(cur_conic);
                cur_box.expand_about_centroid(10); // expand bounding box.
                vcl_vector<vgl_point_2d<double> > cur_pts = conics[j].pts_;
                vcl_vector<vgl_point_2d<double> > inlier_pts;
                for (int k = 0; k<cur_pts.size(); k++) {
                    vgl_point_2d<double> p = cur_pts[k];
                    if (cur_box.contains(p)) {
                        double dis = vgl_homg_operators_2d<double>::distance_squared(cur_conic, vgl_homg_point_2d<double>(p));
                        if (dis < distance_threshold * distance_threshold) {
                            inlier_pts.push_back(p);
                        }
                    }
                }
                // merge j to i
                if (1.0 * inlier_pts.size()/cur_pts.size() > inlier_ratio_threshold) {
                    conics[i].pts_.insert(conics[i].pts_.end(), inlier_pts.begin(), inlier_pts.end());
                    conics[j].valid_ = false;
                    // update conic equation
                    vgl_ellipse_2d<double> elli = vgl_fit_ellipse_2d_DLT(conics[i].pts_);
                    conics[i].conic_ = vgl_conic<double>(elli);
                }
            }
        }
    }
    
    vcl_sort(conics.begin(), conics.end(), vcl_greater<EllipsePoints>());
    
    // return valid conic
    for (int i = 0; i<conics.size(); i++) {
        if (conics[i].valid_) {
            merged_ellipse.push_back(vgl_ellipse_2d<double>(conics[i].conic_));
            merged_points.push_back(conics[i].pts_);
        }
    }
    return true;
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

bool VglPlus::mergeLinesegments(const vcl_vector<vgl_line_segment_2d<double> > & segments,
                                const vcl_vector<vcl_vector<vgl_point_2d<double> > > & segment_pts,
                                const double distance_threshold,
                                vcl_vector<vgl_line_segment_2d<double> > & merged_segments,
                                vcl_vector<vcl_vector<vgl_point_2d<double> > > & merged_pts)
{
    assert(segments.size() == segment_pts.size());
    
    const double inlier_ratio_threshold = 0.7; //
    vcl_vector<LinePoints> line_pts;
    for (int i = 0; i<segments.size(); i++) {
        vgl_line_2d<double> line(segments[i].point1(), segments[i].point2());
        LinePoints lp;
        lp.line_ = line;
        lp.pts_ = segment_pts[i];
        lp.seg_ = segments[i];
        line_pts.push_back(lp);
    }
    
    vcl_sort(line_pts.begin(), line_pts.end(), vcl_greater<LinePoints>::greater());
    
    // greedy mergy
    for (int i = 0; i<line_pts.size(); i++) {
        for (int j = i+1; j<line_pts.size(); j++) {
            if (line_pts[i].valid_ && line_pts[j].valid_) {
                vgl_line_2d<double> cur_line = line_pts[i].line_;
                vcl_vector<vgl_point_2d<double> > cur_pts = line_pts[j].pts_; // points to be merged
                vcl_vector<vgl_point_2d<double> > inlier_pts;
                for (int k = 0; k<cur_pts.size(); k++) {
                    double dis = vgl_distance(cur_pts[k], cur_line);
                    if (dis < distance_threshold) {
                        inlier_pts.push_back(cur_pts[k]);
                    }
                }
                if (1.0 * inlier_pts.size()/cur_pts.size() > inlier_ratio_threshold) {
                    vcl_vector<vgl_point_2d<double> > all_pts;
                    all_pts.insert(all_pts.end(), inlier_pts.begin(), inlier_pts.end());
                    all_pts.insert(all_pts.end(), line_pts[i].pts_.begin(), line_pts[i].pts_.end());
                    
                    vgl_line_2d<double> estimated_line = vgl_fit_line_2d(line_pts[i].pts_);
                    // project the two line segment to the line and merge
                    vgl_point_2d<double> p1 = vgl_closest_point(estimated_line, line_pts[i].seg_.point1());
                    vgl_point_2d<double> p2 = vgl_closest_point(estimated_line, line_pts[i].seg_.point2());
                    vgl_point_2d<double> p3 = vgl_closest_point(estimated_line, line_pts[j].seg_.point1());
                    vgl_point_2d<double> p4 = vgl_closest_point(estimated_line, line_pts[j].seg_.point2());
                    vgl_line_segment_2d<double> merged_seg;
                    bool isMerged = VglPlus::mergeTwolineSegmentOnALine(vgl_line_segment_2d<double>(p1, p2),
                                                                        vgl_line_segment_2d<double>(p3, p4),
                                                                        merged_seg);
                    if (isMerged) {
                        line_pts[i].pts_.insert(line_pts[i].pts_.end(), inlier_pts.begin(), inlier_pts.end());
                        line_pts[i].line_ = estimated_line;
                        line_pts[i].seg_  = merged_seg;
                        line_pts[j].valid_ = false;
                    }
                }
            }
        }
    }
    
    vcl_sort(line_pts.begin(), line_pts.end(), vcl_greater<LinePoints>::greater());
    
    // output
    for (int i = 0; i<line_pts.size(); i++) {
        if (line_pts[i].valid_) {
            merged_segments.push_back(line_pts[i].seg_);
            merged_pts.push_back(line_pts[i].pts_);
        }
    }
    return true;
}

vcl_vector<vgl_point_2d<double> > VglPlus::intersection(const vgl_ellipse_2d<double> & ellipse, const vgl_line_segment_2d<double> & seg)
{
    vgl_conic<double> conic = vgl_conic<double>(ellipse);
    vcl_vector<vgl_point_2d<double> > intersections;
    
    vgl_homg_line_2d<double> homg_line(vgl_homg_point_2d<double>(seg.point1()), vgl_homg_point_2d<double>(seg.point2()));
    
    vcl_list<vgl_homg_point_2d<double> > pts = vgl_homg_operators_2d<double>::intersection(conic, homg_line);
    for (vcl_list<vgl_homg_point_2d<double>>::iterator ite = pts.begin(); ite != pts.end(); ite++) {
        if (ite->w() != 0.0) {
            double x = ite->x()/ite->w();
            double y = ite->y()/ite->w();
            double dis = vgl_distance(vgl_point_2d<double>(x, y), seg);
            if (dis < 1.0) {
                intersections.push_back(vgl_point_2d<double>(x, y));
            }
        }
    }
    
    return intersections;
}

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




















