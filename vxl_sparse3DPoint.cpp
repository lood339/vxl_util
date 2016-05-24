//
//  vxl_sparse_3D_point.cpp
//  CalibFromScene
//
//  Created by Jimmy Chen LOCAL on 6/9/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_sparse3DPoint.h"
#include <vcl_fstream.h>
#include <vpgl/vpgl_fundamental_matrix.h>
#include <vgl/vgl_line_2d.h>
#include <vgl/vgl_distance.h>
#include <vgl/vgl_closest_point.h>
#include "vxl_least_square.h"
#include <vnl/algo/vnl_matrix_inverse.h>


bool VxlSparse3DPoint::triangulateFromTwoCamera(const vpgl_perspective_camera<double> & leftCamera,
                                                const vpgl_perspective_camera<double> & rightCamera,
                                                const vcl_vector<vgl_point_2d<double> > & ptsLeft,
                                                const vcl_vector<vgl_point_2d<double> > & ptsRight,
                                                vcl_vector<vgl_point_3d<double> > & pts3D)
{
    assert(ptsLeft.size() == ptsRight.size());
    
    for (int i = 0; i<ptsLeft.size(); i++) {
        vgl_homg_line_3d_2_points<double> left_line  = leftCamera.backproject(vgl_homg_point_2d<double>(ptsLeft[i]));
        vgl_homg_line_3d_2_points<double> right_line = rightCamera.backproject(vgl_homg_point_2d<double>(ptsRight[i]));
        vcl_pair<vgl_homg_point_3d<double>, vgl_homg_point_3d<double> > pt_pair = vgl_closest_points(left_line, right_line);
        vgl_point_3d<double> pt1(pt_pair.first);
        vgl_point_3d<double> pt2(pt_pair.second);
        
        double x = (pt1.x() + pt2.x()) / 2.0;
        double y = (pt1.y() + pt2.y()) / 2.0;
        double z = (pt1.z() + pt2.z()) / 2.0;
        //    double dis = vgl_distance(pt1, pt2);
        
        //    printf("x, y, z, is %lf\t %lft %lf\t, distance is %lf\n", x, y, z, dis);
        pts3D.push_back(vgl_point_3d<double>(x, y, z));
    }
    return true;
}

bool VxlSparse3DPoint::triangulation(const vcl_vector< vnl_matrix_fixed<double, 3, 4> > & Pmatrix,
                                     const vcl_vector< vcl_vector<vgl_point_2d<double> > > & points,
                                     vcl_vector<vgl_homg_point_3d<double> > & points_3d)
{
    assert(Pmatrix.size() == 3);
    assert(points.size() == 3);
    assert(points[0].size() == points[1].size() && points[0].size() == points[2].size());
    
    const int N = points[0].size();
    for (int i = 0; i<N; i++) {
        
        vcl_vector<vcl_map<int, double> > A;
        vcl_vector<double> b;
        
        for (int j = 0; j<3; j++)
        {
            vnl_matrix_fixed<double, 3, 4> P = Pmatrix[j];
            double x = points[j][i].x();
            double y = points[j][i].y();
            // x
            {
                vcl_map<int, double> imap;
                imap[0] = P(0, 0);
                imap[1] = P(0, 1);
                imap[2] = P(0, 2);
                
                A.push_back(imap);
                b.push_back(x - P(0, 3));
            }
            //y
            {
                vcl_map<int, double> imap;
                imap[0] = P(1, 0);
                imap[1] = P(1, 1);
                imap[2] = P(1, 2);
                
                A.push_back(imap);
                b.push_back(y - P(1, 3));
            }
            // homo coordinate
            {
                vcl_map<int, double> imap;
                imap[0] = P(2, 0);
                imap[1] = P(2, 1);
                imap[2] = P(2, 2);
                
                A.push_back(imap);
                b.push_back(1.0 - P(2, 3));
            }
        }
        
        double pt[3] = {0};
        VxlLeastSquare::solver(A, b, true, 3, pt);
        points_3d.push_back(vgl_homg_point_3d<double>(pt[0], pt[1], pt[2], 1.0));
        
        
        // test
        {
            vpgl_proj_camera<double> projCamera(Pmatrix[0]);
            vgl_point_2d<double> p1 = projCamera.project(vgl_homg_point_3d<double>(pt[0], pt[1], pt[2], 1.0));
            vgl_point_2d<double> p2 = points[0][i];
            
            double dis = vgl_distance(p1, p2);
            printf("distance is %f\n", dis);            
            
        }
        
    }
    
    return true;
}




