//
//  vnl_plus.cpp
//  OnlineStereo
//
//  Created by jimmy on 12/27/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vnl_plus.h"
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vnl/vnl_least_squares_function.h>
#include <vcl_numeric.h>
#include <vcl_iostream.h>
#include <vnl/algo/vnl_svd.h>
#include <vnl/algo/vnl_matrix_inverse.h>
#include <vnl/vnl_math.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vnl/algo/vnl_determinant.h>


vnl_vector<double> VnlPlus::removeElementByIndex(const vnl_vector<double> & v, const unsigned s, const unsigned e)
{
    assert(e > s);
    vnl_vector<double> ret(v.size() - (e - s), 0);
    for (int i = 0; i<ret.size(); i++) {
        if (i < s) {
            ret[i] = v[i];
        }
        else
        {
            ret[i] = v[i+e-s];
        }
    }
    return ret;
}

vnl_vector<double> VnlPlus::concat(const vnl_vector<double> &v1, const vnl_vector<double> &v2)
{
    vnl_vector<double> ret(v1.size() + v2.size(), 0);
    ret.update(v1, 0);
    ret.update(v2, v1.size());
    return ret;
}

unsigned VnlPlus::value_to_bin_number(double v_min, double interval, double value, const unsigned nBin)
{
    int num = (value - v_min)/interval;
    if (num < 0) {
        return 0;
    }
    if (num >= nBin) {
        return nBin - 1;
    }
    return (unsigned)num;
}

double VnlPlus::bin_number_to_value(double v_min, double interval, int bin)
{
    return v_min + bin * interval;
}


double VnlPlus::histogramPeakValue(const vcl_vector<double> & values,
                                   const double resolution)
{
    const double v_min = *std::min_element(values.begin(), values.end());
    const double v_max = *std::max_element(values.begin(), values.end());
    const unsigned int nBin = (v_max - v_min)/resolution;
    assert(nBin > 2);
    
    vnl_vector<double> hist(nBin, 0.0);
    for (int i = 0; i<values.size(); i++) {
        unsigned b = VnlPlus::value_to_bin_number(v_min, resolution, values[i], nBin);
        assert(b >=0 && b < nBin);
        hist[b] += 1.0;
    }
    
    // normalize
    hist /= values.size();
    
    unsigned int max_prob_index = hist.arg_max();
    double max_prob_value = VnlPlus::bin_number_to_value(v_min, resolution, max_prob_index);
    return max_prob_value;
}

vnl_matrix<double> VnlPlus::removeRows(const vnl_matrix<double> & m, const unsigned s, const unsigned e)
{
    assert(e > s);
    assert(e <= m.rows());
    
    if (s == 0) {
        return m.get_n_rows(e, m.rows() - (e - s));
    }
    else if(e == m.rows())
    {
        return m.get_n_rows(0, m.rows() - (e - s));
    }
    else
    {
        vnl_matrix<double> upMat  = m.get_n_rows(0, s);
        vnl_matrix<double> downMat = m.get_n_rows(e, m.rows() - e);
        vnl_matrix<double> ret(m.rows() - e + s, m.cols(), 0);
        ret.update(upMat, 0, 0);
        ret.update(downMat, s, 0);
        return ret;
    }
}

vnl_matrix<double> VnlPlus::removeCols(const vnl_matrix<double> & m, const unsigned s, const unsigned e)
{
    assert(e > s);
    assert(e <= m.cols());
    const int N = m.cols() - e + s;
    if (s == 0) {
        return m.get_n_columns(e, N);
    }
    else if(e == m.cols())
    {
        return m.get_n_columns(0, N);
    }
    else
    {
        vnl_matrix<double> leftMat = m.get_n_columns(0, s);
        vnl_matrix<double> rightMat = m.get_n_columns(e, m.cols() - e);
        vnl_matrix<double> ret(m.rows(), N, 0);
        ret.update(leftMat, 0, 0);
        ret.update(rightMat, 0, s);
        return ret;
    }
}

int VnlPlus::maxinumRMCAM(const vcl_vector<double> & data, double & max_rmcam)
{
    assert(data.size() > 0);
    
    int idx = 0;
    double maxRatio = 1;
    double sum = data[0];
    int num = 1;
    for (int i = 1; i<data.size(); i++) {
        double avg = sum/num;
        if (!VnlPlus::isEqualZero(avg)) {
            double r = data[i]/avg;
            if (r > maxRatio) {
                maxRatio = r;
                idx = i;
            }
        }
        sum += data[i];
        num++;
    }
    max_rmcam = maxRatio;
    return idx;
}




class vnl_fit_polynomialResidual: public vnl_least_squares_function
{
protected:
    const vcl_vector<double> data_;
    const int degree_;
public:
    vnl_fit_polynomialResidual(const vcl_vector<double> & data, int degree):
    vnl_least_squares_function(degree + 1, (unsigned int)data.size(), no_gradient),
    data_(data),
    degree_(degree)
    {
        assert(data.size() >= degree * 10);
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        assert(x.size() == degree_ + 1);
        
        vcl_vector<double> coeff(degree_ + 1, 0);
        for (int i = 0; i<x.size(); i++) {
            coeff[i] = x[i];
        }
        
        vnl_polynomial<double> poly(coeff);
        for (int i = 0; i<data_.size(); i++) {
            double val = poly.evaluate(i);
            fx[i] = data_[i] - val;
        }
    }
    
    void getPolynomial(vnl_vector<double> const &x, vnl_polynomial<double> & poly)
    {
        vcl_vector<double> coeff(degree_ + 1, 0);
        for (int i = 0; i<x.size(); i++) {
            coeff[i] = x[i];
        }
        poly = vnl_polynomial<double>(coeff);
    }
};


bool VnlPlus::vnl_fit_polynomial(const vcl_vector<double> & data, vnl_polynomial<double> &poly, int degree)
{
    assert(data.size() >= degree * 10);
    assert( degree >= 1);
    
    vnl_fit_polynomialResidual residual(data, degree);
    
    double val_mean = vcl_accumulate(data.begin(), data.end(), 0.0);
    val_mean /= data.size();
    
    vnl_vector<double> x(degree + 1, 0.0);
    x[0] = val_mean;                       //constant order first
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed.\n";
        vcl_cerr<<"x = "<<x<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    residual.getPolynomial(x, poly);
    
    return true;
}

int VnlPlus::vnl_eigen_vector_decompose(const vnl_matrix<double> & squareMatrix,
                                        vcl_vector<vnl_vector<double> > & eigenVectors,
                                        vcl_vector<double> & eigenValues)
{
    assert(eigenVectors.size() == 0);
    assert(eigenValues.size() == 0);
    
    // get the eigen vector for minimum eigen value
    vnl_svd<double> svd(squareMatrix);
    vnl_matrix<double> eigen_vectors  = svd.V();
    
    for (int i = 0; i<eigen_vectors.cols(); i++) {
        eigenVectors.push_back(eigen_vectors.get_column(i));
        eigenValues.push_back(svd.W(i, i));
    }
    assert(eigenVectors.size() == eigenValues.size());
    
    return svd.singularities();
}

void VnlPlus::Axb(const vnl_matrix<double> & A, const vnl_matrix<double> & b, vnl_matrix<double> &x)
{
    assert(A.rows() == b.rows());
    assert(b.cols() == 1);
    
    x = vnl_matrix_inverse<double>((A.transpose() * A)) * A.transpose() * b;
}

// Finding the nearest orthonormal matrix
// http://people.csail.mit.edu/bkph/articles/Nearest_Orthonormal_Matrix.pdf
bool VnlPlus::approximateRotationMatrix(const vnl_matrix_fixed<double, 3, 3> & M, vnl_matrix_fixed<double, 3, 3> & R)
{
    vnl_matrix<double> m = M.as_matrix();
    vnl_matrix<double> mTm = m.transpose() * m;
    
    vnl_svd<double> isvd(mTm);
    if (isvd.singularities() != 0) {
        return false;
    }
    
    // eigen values
    double lambda_1 = isvd.W(0, 0);
    double lambda_2 = isvd.W(1, 1);
    double lambda_3 = isvd.W(2, 2);
    if (lambda_3 <= 0.0) {
        return false;
    }
    vnl_matrix<double> v = isvd.V();  // eigen vectors
    

    vnl_matrix<double> e1 = v.extract(3, 1, 0, 0);
    vnl_matrix<double> e2 = v.extract(3, 1, 0, 1);
    vnl_matrix<double> e3 = v.extract(3, 1, 0, 2);
    vnl_matrix<double> mTm_inv_half = 1.0/sqrt(lambda_1) * e1 * e1.transpose() + 1.0/sqrt(lambda_2) * e2 * e2.transpose() + 1.0/sqrt(lambda_3) * e3 * e3.transpose();
    
    vnl_matrix<double> r = m * mTm_inv_half;
    
    double det = vnl_determinant(r);
    if (det < 0.0) {
        // reflection instead of rotation
        return false;
    }
    R = vnl_matrix_fixed<double, 3, 3>(r);
    return true;
}

vnl_matrix<double> VnlPlus::vector_2_mat(const vcl_vector<vcl_vector<double> > & data)
{
    vcl_vector<vnl_vector<double> > vec_data;
    for (int i = 0; i<data.size(); i++) {
        vec_data.push_back(VnlPlus::vector_2_vec(data[i]));
    }
    return VnlPlus::vector_2_mat(vec_data);
}

vnl_matrix<double> VnlPlus::vector_2_mat(const vcl_vector<vnl_vector<double> > & data)
{
    assert(data.size() > 0);
    vnl_matrix<double> mat((int)data.size(), (int)data[0].size(), 0);
    for (int i = 0; i<data.size(); i++) {
        mat.set_row(i, data[i]);
    }
    return mat;
}

vnl_vector<double> VnlPlus::vector_2_vec(const vcl_vector<double> & data)
{
    return vnl_vector<double>(&data[0], (int)data.size());
}

vnl_vector<double> VnlPlus::vector_2_vec(const vcl_vector<int> & data)
{
    vcl_vector<double> d_data(data.size());
    for (int i = 0; i<data.size(); i++) {
        d_data[i] = data[i];
    }
    return vnl_vector<double>(&d_data[0], (int)d_data.size());
}


void VnlPlus::mean_std(const double *pData, unsigned int nData, double & mean, double & sigma)
{
    assert(nData != 0);
    mean = 0.0;
    sigma = 0.0;
    for (int i = 0; i<nData; i++) {
        mean += pData[i];
    }
    mean /= nData;
    for (int i = 0; i<nData; i++) {
        sigma += (pData[i] - mean) * (pData[i] - mean);
    }
    sigma = sqrt(sigma/nData);
}

double VnlPlus::average(const double *pData, unsigned int nData)
{
    assert(pData);
    assert(nData != 0);
    double mean = 0.0;
    for (int i = 0; i<nData; i++) {
        mean += pData[i];
    }
    mean /= nData;
    return mean;
}

// (idx1, y1) --- (idx3, y?) --- (idx2, y2)
double VnlPlus::linearInterpolate(int idx1, int idx2, double y1, double y2, int idx3)
{
    assert(idx1 <= idx3 && idx3 <= idx2);
    assert(idx1 < idx2);
    double dy = y2 - y1;
    return y1 + dy/(idx2 - idx1) * (idx3 - idx1);
}

/*
// http://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
bool VnlPlus::cov2ellipse(const vnl_matrix_fixed<double, 2, 2> & cov, const vgl_point_2d<double> & center,
                          vgl_ellipse_2d<double> & ellipse, double chiSquareVal)
{
    chiSquareVal = sqrt(chiSquareVal);

    vcl_vector<vnl_vector<double> > eigenVectors;
    vcl_vector<double> eigenValues;
    int num = VnlPlus::vnl_eigen_vector_decompose(cov.as_matrix(), eigenVectors, eigenValues);
    if (num != 0) {
        return false;
    }
    
    // vgl_ellipse_2d( vgl_point_2d< T > centre = vgl_point_2d< T >( 0, 0 ), T majorRadius = 2, T minorRadius = 1, T orientation = 0 );
    double major = 2.0 * chiSquareVal * sqrt(eigenValues[0]);
    double minor = 2.0 * chiSquareVal * sqrt(eigenValues[1]);
    double orientation = atan2(eigenVectors[0][1], eigenVectors[0][0]);
    
    ellipse = vgl_ellipse_2d<double>(center, major, minor, orientation);
    //printf("major, nomor is %f %f\n", major, minor);
   // vcl_cout<<"maximum eigen vector is "<<eigenVectors[0]<<vcl_endl;
    return true;     
}
*/

void VnlPlus::print_part(const vnl_matrix<double> & m, int r, int c, char *name)
{
    printf("print %s\n", name);
    for (int i = 0; i<r; i++) {
        for (int j = 0; j<c; j++) {
            printf("%f ", m(i, j));
        }
        printf("\n");
    }
    printf("\n");
}

void VnlPlus::write_mat(const char *file, const vcl_vector<double> & data, const char *dataName)
{
    assert(data.size() > 0);
    vnl_matlab_filewrite writer(file);
    
    vnl_vector<double> matData(&data[0], (int)data.size());
    writer.write(matData, dataName);
    printf("save to %s\n", file);
}

bool VnlPlus::writeMat(const char *fileName, const vnl_matrix<double> & mat)
{
    FILE *pf = fopen(fileName, "w");
    if (!pf) {
        vcl_cout<<"can not open: "<<fileName<<vcl_endl;
        return false;
    }
    const int nRow = mat.rows();
    const int nCol = mat.cols();
    fprintf(pf, "%d %d\n", nRow, nCol);
    for (int i = 0; i<nRow; i++) {
        for (int j = 0; j<nCol; j++) {
            double v = mat[i][j];
            fprintf(pf, "%lf ", v);
        }
        if (i != nRow - 1) {
            fprintf(pf, "\n");
        }
    }
    fclose(pf);
    printf("write to %s\n", fileName);
    return true;
}

bool VnlPlus::readMat(const char *fileName, vnl_matrix<double> & mat)
{
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        vcl_cout<<"can not open: "<<fileName<<vcl_endl;
        return false;
    }
    int nRow = 0;
    int nCol = 0;
    int ret = fscanf(pf, "%d %d", &nRow, &nCol);
    if (ret != 2) {
        printf("Error: file format error.\n");
        return false;
    }
    assert(nRow > 0);
    assert(nCol > 0);
    mat = vnl_matrix<double>(nRow, nCol, 0);
    for (int r = 0; r<nRow; r++) {
        for (int c = 0; c<nCol; c++) {
            double v = 0;
            ret = fscanf(pf, "%lf ", &v);
            if (ret != 1) {
                printf("Error: read data error.\n");
                return false;
            }
            mat[r][c] = v;
        }
    }
    fclose(pf);
    return true;
}


bool VnlPlus::writeFnData(const char *file, const vcl_vector<int> &fns, const vcl_vector<vnl_vector<double> > & data)
{
    assert(fns.size() == data.size());
    
    FILE *pf = fopen(file, "w");
    if (!pf) {
        printf("Error: can not write to %s\n", file);
        return false;
    }
    int sz = (int)data.front().size();
    fprintf(pf, "%d\t %d\n", (int)fns.size(), sz + 1);
    for (int i = 0; i<fns.size(); i++) {
        fprintf(pf, "%d\t ", fns[i]);
        assert(data[i].size() == sz);
        for (int j = 0; j<sz; j++) {
            fprintf(pf, "%f\t ", data[i][j]);
        }
        fprintf(pf, "\n");
    }
    fclose(pf);
    
    printf("save to %s\n", file);
    return true;
}

bool VnlPlus::readFnDara(const char *file, const int data_dim, vcl_vector<int> &fns, vcl_vector<vnl_vector<double> > & data)
{
    FILE *pf = fopen(file, "r");
    if (!pf) {
        printf("Error: can not read rom %s\n", file);
        return false;
    }
    int num = 0;
    int dim = 0;
    int ret = fscanf(pf, "%d %d", &num, &dim);
    assert(ret == 2);
    assert(dim == data_dim + 1);
    vnl_vector<double> d(data_dim, 0);
    for (int i = 0; i<num; i++) {
        int fn = 0;
        ret = fscanf(pf, "%d ", &fn);
        assert(ret == 1);
        fns.push_back(fn);
        for (int j = 0; j<data_dim; j++) {
            ret = fscanf(pf, "%lf ", &d[j]);
        }
        data.push_back(d);
    }
    assert(fns.size() == data.size());
    
    return true;
}


