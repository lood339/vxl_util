//
//  vnl_plus.h
//  OnlineStereo
//
//  Created by jimmy on 12/27/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vnl_plus__
#define __OnlineStereo__vnl_plus__

#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vcl_vector.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_polynomial.h>
#include <vnl/vnl_matrix_fixed.h>
#include <vgl/vgl_ellipse_2d.h>

class VnlPlus
{
public:
    static vnl_vector<double> removeElementByIndex(const vnl_vector<double> & v, const unsigned s, const unsigned e);
    static vnl_vector<double> concat(const vnl_vector<double> &v1, const vnl_vector<double> &v2);
    
    static unsigned int value_to_bin_number(double v_min, double interval, double value, const unsigned nBin);
    static double bin_number_to_value(double v_min, double interval, int bin);
    
    // get peak value
    static double histogramPeakValue(const vcl_vector<double> & values,
                                     const double resolution);
    
    
    // [s e)
    static vnl_matrix<double> removeRows(const vnl_matrix<double> & m, const unsigned s, const unsigned e);
    static vnl_matrix<double> removeCols(const vnl_matrix<double> & m, const unsigned s, const unsigned e);
    
    // ratio of magnitude to cumulative average magnitude (RMCAM)
    // -1 for no such peak value
    // all is positive values
    static int maxinumRMCAM(const vcl_vector<double> & data, double & max_rmcam);
    
 

    // x = index, y = data
    static bool vnl_fit_polynomial(const vcl_vector<double> & data, vnl_polynomial<double> & poly, int degree = 3);
    
    // eigen values decend by index
    // return value: singularities,
    static int vnl_eigen_vector_decompose(const vnl_matrix<double> & squareMatrix,
                                          vcl_vector<vnl_vector<double> > & eigenVectors,
                                          vcl_vector<double> & eigenValues);
    
    // Ax = b
    static void Axb(const vnl_matrix<double> & A, const vnl_matrix<double> & b, vnl_matrix<double> &x);
    
    // approximate M by a rotation matrix
    static bool approximateRotationMatrix(const vnl_matrix_fixed<double, 3, 3> & M, vnl_matrix_fixed<double, 3, 3> & R);
    
    static vnl_matrix<double> vector_2_mat(const vcl_vector<vcl_vector<double> > & data);
    static vnl_matrix<double> vector_2_mat(const vcl_vector<vnl_vector<double> > & data);
    static vnl_vector<double> vector_2_vec(const vcl_vector<double> & data);
    static vnl_vector<double> vector_2_vec(const vcl_vector<int> & data);
    
    
    static bool isEqual(double v1, double v2)
    {
        return fabs(v1 - v2) < vnl_math::sqrteps;
    }
    static bool isEqualZero(double v1)
    {
        return fabs(v1) < vnl_math::sqrteps;
    }
    
    static void mean_std(const double *pData, unsigned int nData, double & mean, double & sigma);   
    
    static double average(const double *pData, unsigned int nData);
    
    // (idx1, y1) --- (idx3, y?) --- (idx2, y2)
    static double linearInterpolate(int idx1, int idx2, double y1, double y2, int idx3);
    
    // chisquare_val: 5.991 --> 95%
	//  9.210 --> 99%
    static bool cov2ellipse(const vnl_matrix_fixed<double, 2, 2> & cov,
                            const vgl_point_2d<double> & center, vgl_ellipse_2d<double> & ellipse,
                            double chiSquareVal = 9.21);
    
    
    // visualization
    static void print_part(const vnl_matrix<double> & m, int r, int c, char *name);
    
    // file = *.mat
    static void write_mat(const char *file, const vcl_vector<double> & data, const char *dataName = "data");
    
    static bool writeMat(const char *fileName, const vnl_matrix<double> & mat);
    static bool readMat(const char *fileName, vnl_matrix<double> & mat);
    
    
    static bool writeFnData(const char *file, const vcl_vector<int> &fns, const vcl_vector<vnl_vector<double> > & data);    
    static bool readFnDara(const char *file, const int data_dim, vcl_vector<int> &fns, vcl_vector<vnl_vector<double> > & data);

};



#endif /* defined(__OnlineStereo__vnl_plus__) */
