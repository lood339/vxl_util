//
//  vpgl_p3p.h
//  OpenCV_VXL
//
//  Created by jimmy on 9/28/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OpenCV_VXL__vpgl_p3p__
#define __OpenCV_VXL__vpgl_p3p__


/*
 * P3p.h
 *
 *  Created on: Nov 2, 2010
 *      Author: Laurent Kneip
 * Description: Compute the absolute pose of a camera using three 3D-to-2D correspondences
 *   Reference: A Novel Parametrization of the P3P-Problem for a Direct Computation of
 *              Absolute Camera Position and Orientation
 *
 *       Input: featureVectors: 3x3 matrix with UNITARY feature vectors (each column is a vector)
 *              worldPoints: 3x3 matrix with corresponding 3D world points (each column is a point)
 *              solutions: 3x16 matrix that will contain the solutions
 *                         form: [ 3x1 position(solution1) 3x3 orientation(solution1) 3x1 position(solution2) 3x3 orientation(solution2) ... ]
 *                         the obtained orientation matrices are defined as transforming points from the cam to the world frame
 *      Output: int: 0 if correct execution
 *                  -1 if world points aligned
 */

#include <vnl/vnl_vector.h>
#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_matrix_fixed.h>
#include <vnl/vnl_matrix.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_point_2d.h>


class vpgl_p3p
{
public:
	vpgl_p3p();
	~vpgl_p3p();
    
    // K: camera intrinsic matrix
    //Output: poses: 3x16 matrix that will contain the solutions
    //                     form: [ 3x1 position(solution1) 3x3 orientation(solution1) 3x1 position(solution2) 3x3 orientation(solution2) ... ]
    //                     the obtained orientation matrices are defined as transforming points from the world to the camera frame
    static bool compute_poses(const vcl_vector<vgl_point_3d<double> > & world_points,
                              const vcl_vector<vgl_point_2d<double> > & image_points,
                              const vnl_matrix_fixed<double, 3, 3> & K,
                              vcl_vector<vnl_matrix_fixed<double, 3, 4> > & solutions);
    
    // feature_vectors: one column for one feature fector
    // solutions_3_16: 3 * 16, 4 3*4 solution
	static int compute_poses(const vnl_matrix_fixed<double, 3, 3> & feature_vectors,
                             const vnl_matrix_fixed<double, 3, 3> & world_points,
                             vcl_vector<vnl_matrix_fixed<double, 3, 4> > & solutions );
    
private:
	static int solve_quartic( const vnl_vector_fixed<double, 5> & factors, vnl_vector_fixed<double, 4> & real_roots );
};





#endif /* defined(__OpenCV_VXL__vpgl_p3p__) */
