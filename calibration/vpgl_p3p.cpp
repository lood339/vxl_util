//
//  vpgl_p3p.cpp
//  OpenCV_VXL
//
//  Created by jimmy on 9/28/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vpgl_p3p.h"
#include <vnl/vnl_cross.h>
#include <vnl/vnl_inverse.h>

vpgl_p3p::vpgl_p3p()
{
    
}

vpgl_p3p::~vpgl_p3p()
{
    
}

bool vpgl_p3p::compute_poses(const vcl_vector<vgl_point_3d<double> > & world_points,
                              const vcl_vector<vgl_point_2d<double> > & image_points,
                              const vnl_matrix_fixed<double, 3, 3> & K,
                              vcl_vector<vnl_matrix_fixed<double, 3, 4> > & solutions)
{
    if (world_points.size() != 3 || image_points.size() != 3) {
        return false;
    }
    
    vnl_matrix<double> invK = vnl_inverse(K);
    
    vnl_matrix_fixed<double, 3, 3> world_points_mat;
    vnl_matrix_fixed<double, 3, 3> feature_vectors_mat;
    for (int i = 0; i<world_points.size(); i++) {
        double x = world_points[i].x();
        double y = world_points[i].y();
        double z = world_points[i].z();
        
        vnl_vector_fixed<double, 3> p(x, y, z);
        world_points_mat.set_column(i, p);
        
        // image coordinates to camera coordinate
        vgl_point_2d<double> q = image_points[i];
        vnl_vector_fixed<double, 3> q_v(q.x(), q.y(), 1.0);
        vnl_vector_fixed<double, 3> f_v = invK * q_v;
        f_v = f_v.normalize();
        
        feature_vectors_mat.set_column(i, f_v);
    }
    
    int ret = vpgl_p3p::compute_poses(feature_vectors_mat, world_points_mat, solutions);
    return ret == 0;
}

int vpgl_p3p::compute_poses(const vnl_matrix_fixed<double, 3, 3> & feature_vectors,
                            const vnl_matrix_fixed<double, 3, 3> & world_points,
                            vcl_vector<vnl_matrix_fixed<double, 3, 4> > & solutions )
{
    // Extraction of world points
    
//	TooN::Vector<3> P1 = worldPoints.T()[0];
//	TooN::Vector<3> P2 = worldPoints.T()[1];
//	TooN::Vector<3> P3 = worldPoints.T()[2];
    vnl_vector_fixed<double, 3> P1 = world_points.get_column(0);
    vnl_vector_fixed<double, 3> P2 = world_points.get_column(1);
    vnl_vector_fixed<double, 3> P3 = world_points.get_column(2);
    
    
	// Verification that world points are not colinear
    
	//TooN::Vector<3> temp1 = P2 - P1;
	//TooN::Vector<3> temp2 = P3 - P1;
    vnl_vector_fixed<double, 3> temp1 = P2 - P1;
    vnl_vector_fixed<double, 3> temp2 = P3 - P1;
    
//	if(TooN::norm(temp1 ^ temp2) == 0)
//		return -1;
    if (vnl_cross_3d(temp1, temp2).two_norm() == 0.0) {
        return -1;
    }
    
    
	// Extraction of feature vectors
    
//	TooN::Vector<3> f1 = featureVectors.T()[0];
//	TooN::Vector<3> f2 = featureVectors.T()[1];
//	TooN::Vector<3> f3 = featureVectors.T()[2];
    vnl_vector_fixed<double, 3> f1 = feature_vectors.get_column(0);
    vnl_vector_fixed<double, 3> f2 = feature_vectors.get_column(1);
    vnl_vector_fixed<double, 3> f3 = feature_vectors.get_column(2);
    
    
	// Creation of intermediate camera frame
    
//	TooN::Vector<3> e1 = f1;
//	TooN::Vector<3> e3 = f1 ^ f2;
//	e3 = e3 / TooN::norm(e3);
//	TooN::Vector<3> e2 = e3 ^ e1;
    vnl_vector_fixed<double, 3> e1 = f1;
    vnl_vector_fixed<double, 3> e3 = vnl_cross_3d(f1, f2);
    e3 = e3.normalize();
    vnl_vector_fixed<double, 3> e2 = vnl_cross_3d(e3, e1);
    
     
//	TooN::Matrix<3,3> T;
//	T[0] = e1;
//	T[1] = e2;
//	T[2] = e3;
//	f3 = T*f3;
    vnl_matrix_fixed<double, 3, 3> T;
    T.set_row(0, e1);
    T.set_row(1, e2);
    T.set_row(2, e3);
    f3 = T*f3;
    
	// Reinforce that f3[2] > 0 for having theta in [0;pi]
    
    /*
	if( f3[2] > 0 )
	{
		f1 = featureVectors.T()[1];
		f2 = featureVectors.T()[0];
		f3 = featureVectors.T()[2];
        
		e1 = f1;
		e3 = f1 ^ f2;
		e3 = e3 / TooN::norm(e3);
		e2 = e3 ^ e1;
        
		T[0] = e1;
		T[1] = e2;
		T[2] = e3;
        
		f3 = T*f3;
        
		P1 = worldPoints.T()[1];
		P2 = worldPoints.T()[0];
		P3 = worldPoints.T()[2];
	}
     */
    if( f3[2] > 0 )
	{
		f1 = feature_vectors.get_column(1);
        f2 = feature_vectors.get_column(0);
        f3 = feature_vectors.get_column(2);
       
        e1 = f1;
        e3 = vnl_cross_3d(f1, f2);
        e3 = e3.normalize();
        e2 = vnl_cross_3d(e3, e1);
       
        T.set_row(0, e1);
        T.set_row(1, e2);
        T.set_row(2, e3);
        f3 = T*f3;
        
        P1 = world_points.get_column(1);
        P2 = world_points.get_column(0);
        P3 = world_points.get_column(2);
	}
     
	// Creation of intermediate world frame
    
//	TooN::Vector<3> n1 = P2-P1;
//	n1 = n1 / TooN::norm(n1);
//	TooN::Vector<3> n3 = n1 ^ (P3-P1);
//	n3 = n3 / TooN::norm(n3);
//	TooN::Vector<3> n2 = n3 ^ n1;
    
//	TooN::Matrix<3,3> N;
//	N[0] = n1;
//	N[1] = n2;
//	N[2] = n3;
    vnl_vector_fixed<double, 3> n1 = P2 - P1;
    n1 = n1.normalize();
    vnl_vector_fixed<double, 3> n3 = vnl_cross_3d(n1, P3 - P1);
    n3 = n3.normalize();
    vnl_vector_fixed<double, 3> n2 = vnl_cross_3d(n3, n1);
    
    vnl_matrix_fixed<double, 3, 3> N;
    N.set_row(0, n1);
    N.set_row(1, n2);
    N.set_row(2, n3);
    
	// Extraction of known parameters
    /*
	P3 = N*(P3-P1);
    
	double d_12 = TooN::norm(P2-P1);
	double f_1 = f3[0]/f3[2];
	double f_2 = f3[1]/f3[2];
	double p_1 = P3[0];
	double p_2 = P3[1];
    
	double cos_beta = f1 * f2;
	double b = 1/(1-pow(cos_beta,2)) - 1;
    
	if (cos_beta < 0)
		b = -sqrt(b);
	else
		b = sqrt(b);
     */
    P3 = N*(P3-P1);
    
	double d_12 = (P2-P1).two_norm();
	double f_1 = f3[0]/f3[2];
	double f_2 = f3[1]/f3[2];
	double p_1 = P3[0];
	double p_2 = P3[1];
    
	double cos_beta = dot_product(f1, f2);
	double b = 1/(1-pow(cos_beta,2)) - 1;
    
	if (cos_beta < 0)
    {
		b = -sqrt(b);
    }
	else
    {
		b = sqrt(b);
    }
    
    
	// Definition of temporary variables for avoiding multiple computation
    
	double f_1_pw2 = pow(f_1,2);
	double f_2_pw2 = pow(f_2,2);
	double p_1_pw2 = pow(p_1,2);
	double p_1_pw3 = p_1_pw2 * p_1;
	double p_1_pw4 = p_1_pw3 * p_1;
	double p_2_pw2 = pow(p_2,2);
	double p_2_pw3 = p_2_pw2 * p_2;
	double p_2_pw4 = p_2_pw3 * p_2;
	double d_12_pw2 = pow(d_12,2);
	double b_pw2 = pow(b,2);
    
	// Computation of factors of 4th degree polynomial
    
//	TooN::Vector<5> factors;

    vnl_vector_fixed<double, 5> factors;
    
    factors[0] = -f_2_pw2*p_2_pw4
                -p_2_pw4*f_1_pw2
                -p_2_pw4;
    
	factors[1] = 2*p_2_pw3*d_12*b
                +2*f_2_pw2*p_2_pw3*d_12*b
                -2*f_2*p_2_pw3*f_1*d_12;
    
	factors[2] = -f_2_pw2*p_2_pw2*p_1_pw2
                -f_2_pw2*p_2_pw2*d_12_pw2*b_pw2
                -f_2_pw2*p_2_pw2*d_12_pw2
                +f_2_pw2*p_2_pw4
    +p_2_pw4*f_1_pw2
    +2*p_1*p_2_pw2*d_12
    +2*f_1*f_2*p_1*p_2_pw2*d_12*b
    -p_2_pw2*p_1_pw2*f_1_pw2
    +2*p_1*p_2_pw2*f_2_pw2*d_12
    -p_2_pw2*d_12_pw2*b_pw2
    -2*p_1_pw2*p_2_pw2;
    
	factors[3] = 2*p_1_pw2*p_2*d_12*b
    +2*f_2*p_2_pw3*f_1*d_12
    -2*f_2_pw2*p_2_pw3*d_12*b
    -2*p_1*p_2*d_12_pw2*b;
    
	factors[4] = -2*f_2*p_2_pw2*f_1*p_1*d_12*b
    +f_2_pw2*p_2_pw2*d_12_pw2
    +2*p_1_pw3*d_12
    -p_1_pw2*d_12_pw2
    +f_2_pw2*p_2_pw2*p_1_pw2
    -p_1_pw4
    -2*f_2_pw2*p_2_pw2*p_1*d_12
    +p_2_pw2*f_1_pw2*p_1_pw2
    +f_2_pw2*p_2_pw2*d_12_pw2*b_pw2;
    
    
	// Computation of roots
    
//	TooN::Vector<4> realRoots;
    
//	this->solveQuartic( factors, realRoots );
    vnl_vector_fixed<double, 4> real_roots;
    vpgl_p3p::solve_quartic(factors, real_roots);
    
    // Backsubstitution of each solution
    
    for(unsigned int i=0; i<4; i++)
	{
		double cot_alpha = (-f_1*p_1/f_2-real_roots[i]*p_2+d_12*b)/(-f_1*real_roots[i]*p_2/f_2+p_1-d_12);
        
		double cos_theta = real_roots[i];
		double sin_theta = sqrt(1-pow(real_roots[i],2));
		double sin_alpha = sqrt(1/(pow(cot_alpha,2)+1));
		double cos_alpha = sqrt(1-pow(sin_alpha,2));
        
		if (cot_alpha < 0)
        {
			cos_alpha = -cos_alpha;
        }
        
		//TooN::Vector<3> C = TooN::makeVector(
         //                                    d_12*cos_alpha*(sin_alpha*b+cos_alpha),
           //                                  cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha),
             //                                sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha));
        vnl_vector_fixed<double, 3> C;
        C[0] = d_12*cos_alpha*(sin_alpha*b+cos_alpha);
        C[1] = cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha);
        C[2] = sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha);
        
		//C = P1 + N.T()*C;
        C = P1 + N.transpose() * C;
        
		//TooN::Matrix<3,3> R;
		//R[0] = TooN::makeVector(	-cos_alpha,		-sin_alpha*cos_theta,	-sin_alpha*sin_theta );
        //R[1] = TooN::makeVector(	sin_alpha,		-cos_alpha*cos_theta,	-cos_alpha*sin_theta );
        //R[2] = TooN::makeVector(	0,				-sin_theta,				cos_theta );
        vnl_matrix_fixed<double, 3, 3> R;
        R.set_row(0, vnl_vector_fixed<double, 3>(-cos_alpha,		-sin_alpha*cos_theta,	-sin_alpha*sin_theta));
        R.set_row(1, vnl_vector_fixed<double, 3>(sin_alpha,		-cos_alpha*cos_theta,	-cos_alpha*sin_theta));
        R.set_row(2, vnl_vector_fixed<double, 3>(0,				-sin_theta,				cos_theta));
        
	//	R = N.T()*R.T()*T;
        R = N.transpose() * R.transpose() * T;
        
	//	solutions.T()[i*4] = C;
	//	solutions.T()[i*4+1] = R.T()[0];
	//	solutions.T()[i*4+2] = R.T()[1];
	//	solutions.T()[i*4+3] = R.T()[2];
        
        // vnl_matrix_fixed& set_column(unsigned j, vnl_vector_fixed<T,num_rows> const& v);
        vnl_matrix_fixed<double, 3, 4> CR;
        CR.set_column(0, C);
     //   CR.set_column(1, R.get_column(0));
     //   CR.set_column(2, R.get_column(1));
     //   CR.set_column(3, R.get_column(2));
        // transpose R so that R is from world to camera coordiante, the original algorithm is from camera to world coordinate
        CR.set_column(1, R.get_row(0));
        CR.set_column(2, R.get_row(1));
        CR.set_column(3, R.get_row(2));
        
        solutions.push_back(CR);   
    }
    
    /*
	for(int i=0; i<4; i++)
	{
		double cot_alpha = (-f_1*p_1/f_2-realRoots[i]*p_2+d_12*b)/(-f_1*realRoots[i]*p_2/f_2+p_1-d_12);
        
		double cos_theta = realRoots[i];
		double sin_theta = sqrt(1-pow(realRoots[i],2));
		double sin_alpha = sqrt(1/(pow(cot_alpha,2)+1));
		double cos_alpha = sqrt(1-pow(sin_alpha,2));
        
		if (cot_alpha < 0)
			cos_alpha = -cos_alpha;
        
		TooN::Vector<3> C = TooN::makeVector(
                                             d_12*cos_alpha*(sin_alpha*b+cos_alpha),
                                             cos_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha),
                                             sin_theta*d_12*sin_alpha*(sin_alpha*b+cos_alpha));
        
		C = P1 + N.T()*C;
        
		TooN::Matrix<3,3> R;
		R[0] = TooN::makeVector(	-cos_alpha,		-sin_alpha*cos_theta,	-sin_alpha*sin_theta );
		R[1] = TooN::makeVector(	sin_alpha,		-cos_alpha*cos_theta,	-cos_alpha*sin_theta );
		R[2] = TooN::makeVector(	0,				-sin_theta,				cos_theta );
        
		R = N.T()*R.T()*T;
        
		solutions.T()[i*4] = C;
		solutions.T()[i*4+1] = R.T()[0];
		solutions.T()[i*4+2] = R.T()[1];
		solutions.T()[i*4+3] = R.T()[2];
	}
     */
    
	return 0;
}

int vpgl_p3p::solve_quartic( const vnl_vector_fixed<double, 5> & factors, vnl_vector_fixed<double, 4> & real_roots)
{
    double A = factors[0];
	double B = factors[1];
	double C = factors[2];
	double D = factors[3];
	double E = factors[4];
    
	double A_pw2 = A*A;
	double B_pw2 = B*B;
	double A_pw3 = A_pw2*A;
	double B_pw3 = B_pw2*B;
	double A_pw4 = A_pw3*A;
	double B_pw4 = B_pw3*B;
    
	double alpha = -3*B_pw2/(8*A_pw2)+C/A;
	double beta = B_pw3/(8*A_pw3)-B*C/(2*A_pw2)+D/A;
	double gamma = -3*B_pw4/(256*A_pw4)+B_pw2*C/(16*A_pw3)-B*D/(4*A_pw2)+E/A;
    
	double alpha_pw2 = alpha*alpha;
	double alpha_pw3 = alpha_pw2*alpha;
    
	std::complex<double> P (-alpha_pw2/12-gamma,0);
	std::complex<double> Q (-alpha_pw3/108+alpha*gamma/3-pow(beta,2)/8,0);
	std::complex<double> R = -Q/2.0+sqrt(pow(Q,2.0)/4.0+pow(P,3.0)/27.0);
    
	std::complex<double> U = pow(R,(1.0/3.0));
	std::complex<double> y;
    
	if (U.real() == 0)
    {
		y = -5.0*alpha/6.0-pow(Q,(1.0/3.0));
    }
	else
    {
		y = -5.0*alpha/6.0-P/(3.0*U)+U;
    }
    
	std::complex<double> w = sqrt(alpha+2.0*y);
    
	std::complex<double> temp;
    
	temp = -B/(4.0*A) + 0.5*(w+sqrt(-(3.0*alpha+2.0*y+2.0*beta/w)));
	real_roots[0] = temp.real();
	temp = -B/(4.0*A) + 0.5*(w-sqrt(-(3.0*alpha+2.0*y+2.0*beta/w)));
	real_roots[1] = temp.real();
	temp = -B/(4.0*A) + 0.5*(-w+sqrt(-(3.0*alpha+2.0*y-2.0*beta/w)));
	real_roots[2] = temp.real();
	temp = -B/(4.0*A) + 0.5*(-w-sqrt(-(3.0*alpha+2.0*y-2.0*beta/w)));
	real_roots[3] = temp.real();
    
	return 0;
}