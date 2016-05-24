//
//  vxl_mvl_plus.cpp
//  QuadCopter
//
//  Created by jimmy on 4/3/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vxl_mvl_plus.h"

#include <mvl/FMatrix.h>
#include <mvl/FMatrixComputeRANSAC.h>
#include <vcl_fstream.h>
#include <mvl/mvl_five_point_camera_pencil.h>
#include <mvl/mvl_three_view_six_point_structure.h>
#include <mvl/mvl_six_point_design_matrix_row.h>
#include <vcl_iostream.h>


bool VxlMvlPlus::fundamental_RANSAC(vcl_vector< vgl_point_2d< double > > const & first,
                                    vcl_vector< vgl_point_2d< double > > const & second,
                                    vcl_vector< bool > & inlier,
                                    double error_threshold,
                                    vpgl_fundamental_matrix<double> & F)
{
    assert(first.size() == second.size());
    assert(first.size() >= 4);
    
    vcl_vector<vgl_homg_point_2d<double> > pts1;
    vcl_vector<vgl_homg_point_2d<double> > pts2;
    for (int i = 0; i<first.size(); i++) {
        pts1.push_back((vgl_homg_point_2d<double>)first[i]);
        pts2.push_back((vgl_homg_point_2d<double>)second[i]);
    }
    
    // distable cout in bapl_bbf_tree construction function
    std::streambuf* cerr_sbuf = std::cerr.rdbuf(); // save original sbuf
    std::ofstream   fout("/dev/null");
    std::cerr.rdbuf(fout.rdbuf()); // redirect 'cout' to a 'fout'
    
    FMatrixComputeRANSAC computor(true, error_threshold);
    FMatrix f;
    bool isF = computor.compute(pts1, pts2, f);
    
    std::cerr.rdbuf(cerr_sbuf); // restore the original stream buffer
    
    if (!isF) {
        printf("can't find fundamental matrix\n");
        return false;
    }
    
    vnl_matrix_fixed<double,3,3> Fmat;
    for (int i = 0; i<3; i++) {
        for (int j = 0; j<3; j++) {
            Fmat[i][j] = f.get(i, j);
        }
    }
    
    inlier = computor.get_inliers();
    assert(inlier.size() == first.size());
    
    F = vpgl_fundamental_matrix<double>(Fmat);
    return true;
}

bool VxlMvlPlus::fundamental_RANSAC(vcl_vector< vgl_point_2d< double > > const & positions1,
                                    vcl_vector< vgl_point_2d< double > > const & positions2,
                                    vcl_vector< vcl_pair<int, int> > const & initialMatchedIndices,  // matched index first --> second
                                    double error_threshold,
                                    // output
                                    vcl_vector< vcl_pair<int, int> > & finalMatchedIndices,
                                    vpgl_fundamental_matrix<double> & F)
{
    bool isFind = false;
    
    // extract point pair from initial matches
    vcl_vector<vgl_point_2d<double> > pts1_matched;
    vcl_vector<vgl_point_2d<double> > pts2_matched;
    for (int i = 0; i<initialMatchedIndices.size(); i++) {
        pts1_matched.push_back(positions1[initialMatchedIndices[i].first]);
        pts2_matched.push_back(positions2[initialMatchedIndices[i].second]);
    }
    
    vcl_vector<bool> inliers;
    isFind = VxlMvlPlus::fundamental_RANSAC(pts1_matched, pts2_matched, inliers, error_threshold, F);
    assert(inliers.size() == initialMatchedIndices.size());
    
    if (isFind) {
        for (int i = 0; i<inliers.size(); i++) {
            if (inliers[i]) {
                finalMatchedIndices.push_back(initialMatchedIndices[i]);
            }
        }
    }
    
    printf("fundamental matrix RANSAC find %lu from %lu initial matchings\n", finalMatchedIndices.size(), initialMatchedIndices.size());
    return  isFind;
}

bool VxlMvlPlus::three_view_six_points_calib(const vcl_vector< vcl_vector<vgl_point_2d<double> > > & pointsVec,
                                             vcl_vector< vnl_matrix_fixed<double, 3, 4> > & Pmatrix,
                                             vgl_homg_point_3d<double> & Q)
{
    assert(pointsVec.size() == 3);
    for (int i = 0; i<3; i++) {
        assert(pointsVec[i].size() == 6);
    }
    
    mvl_three_view_six_point_structure mvl36;
    
    // set 6 (x, y)
    for (int i = 0; i<3; i++) {
        for (int j = 0; j<6; j++) {
            mvl36.u(i, j) = pointsVec[i][j].x();
            mvl36.v(i, j) = pointsVec[i][j].y();
        }
    }
    
    bool isFind = mvl36.compute();
    if (!isFind) {
        return false;
    }
    
    int validNum = 0;
    mvl_three_view_six_point_structure::solution_t slu; // only one solution can be used
    for (int i = 0; i<3; i++) {
        validNum += (mvl36.solution[i].valid == true ? 1:0);
        if (mvl36.solution[i].valid) {
            slu = mvl36.solution[i];
        }
    }
    
    // no ambiguity
    if (validNum != 1) {
        printf("%d ambituity solutions\n", validNum);
        for (int i = 0; i<3; i++) {
            if (mvl36.solution[i].valid)
            {
                for (int j = 0; j<3; j++) {
                    vcl_cout<<"P is: \n"<<mvl36.solution[i].P[j]<<vcl_endl;
                }
            }
            
            vcl_cout<<"Q is "<<vgl_point_3d<double>(mvl36.solution[i].Q[0], mvl36.solution[i].Q[1], mvl36.solution[i].Q[2])<<vcl_endl;
        }
        return false;
    }
    for (int i = 0; i<3; i++) {
        vcl_cout<<"P is: \n"<<slu.P[i]<<vcl_endl;
        Pmatrix.push_back(slu.P[i]);
    }
    vcl_cout<<"Q is: \n"<<slu.Q<<vcl_endl;
    Q = vgl_homg_point_3d<double>(slu.Q[0], slu.Q[1], slu.Q[2], slu.Q[3]);    
    return true;
}



















