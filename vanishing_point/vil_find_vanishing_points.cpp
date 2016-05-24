//
//  vil_find_vanishing_points.cpp
//  QuadCopter
//
//  Created by jimmy on 7/7/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_find_vanishing_points.h"

#include <math.h>
#include <stdio.h>
#include <vgl/vgl_point_2d.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vgl/vgl_homg_point_2d.h>


//Lines maju line v stlpcoch a su single!!!!!

static void lines_end_points(float * line, int * endpoints, float space_c, int numLines);
static void rasterize_lines(float * line, int * endpoints, int * space, int SpaceSize, int numLines);
static inline void lineH(int x0, int y0, int x1, int y1, int * space, int * y_steps, int weight);
static inline void lineV(int x0, int y0, int x1, int y1, int * space, int * y_steps, int weight);

template <typename T> int sgn(T val)
{
    return (T(0) <= val) - (val < T(0));
}

int round(float x)
{
    return ((x>=0) ? int(x + 0.5) : int(x - 0.5));
}
/*
 void mexFunction(int nlhs, mxArray * plhs[], int nrhs, const mxArray * prhs[])
 {
 #define SpaceSize prhs[0]
 #define Lines prhs[1]
 
 //Create Space fills with zeros and size SpaceSize
 int cSpaceSize[2] = {mxGetScalar(SpaceSize),mxGetScalar(SpaceSize)};
 mxArray * Space = mxCreateNumericArray(2, cSpaceSize, mxINT32_CLASS, mxREAL);
 int * pSpace = (int*)mxGetData(Space);
 
 //Get Lines data
 float * LinesData = (float*)mxGetData(Lines);
 float space_c = (cSpaceSize[0] - 1.f)/2;
 
 int numLines = mxGetN(Lines);
 
 //int cE[2] = {8, numLines};
 //mxArray * mxE = mxCreateNumericArray(2, cE, mxINT32_CLASS, mxREAL);
 //int * EndPoints = (int*)mxGetData(mxE);
 int * EndPoints =  (int*) malloc(sizeof(int)*8*numLines);
 
 //Get all EndPoints
 lines_end_points(LinesData, EndPoints, space_c, numLines);
 
 //Rasterize
 rasterize_lines(LinesData, EndPoints, pSpace, cSpaceSize[0], numLines);
 
 free(EndPoints);
 plhs[0] = Space;
 //plhs[0] = mxE;
 }
 */

// return new ed memory
void raseter_space(int spaceSize, float *lineData, int nLine, int * pSpace)
{
    int cSpaceSize[2] = {spaceSize, spaceSize};
    
    // memory for diamond space
    memset(pSpace, 0, sizeof(pSpace[0]) * spaceSize * spaceSize);
    float space_c = (cSpaceSize[0] - 1.f)/2;
    int * EndPoints =  (int*) malloc(sizeof(int)*8*nLine); // why 8 end points?
    
    //Get all EndPoints
    lines_end_points(lineData, EndPoints, space_c, nLine);
    
    //Rasterize
    rasterize_lines(lineData, EndPoints, pSpace, cSpaceSize[0], nLine);
    
    free(EndPoints);
}

/*
 function D = point_to_lines_dist(Point, Lines)
 x = Point(1);
 y = Point(2);
 
 T = [0,-1,1;1,-1,0;0,-1,0;...
 0,-1,1;1,1,0 ;0,-1,0;...
 0,1,1 ;1,-1,0;0,-1,0;...
 0,1,1 ;1,1,0 ;0,-1,0];
 
 L = Lines*T';
 
 P(:,1) = (L(:,1:3)*[x,y,1]')./sqrt(sum(L(:,1:2).^2,2));
 P(:,2) = (L(:,4:6)*[x,y,1]')./sqrt(sum(L(:,4:5).^2,2));
 P(:,3) = (L(:,7:9)*[x,y,1]')./sqrt(sum(L(:,7:8).^2,2));
 P(:,4) = (L(:,10:12)*[x,y,1]')./sqrt(sum(L(:,10:11).^2,2));
 
 D = min(abs(P),[],2);
 end
 */
vnl_matrix<double> sum_of_columns(const vnl_matrix<double> & m)
{
    vnl_matrix<double> col(m.rows(), 1);
    col.fill(0);
    for (int r = 0; r<m.rows(); r++) {
        double s = 0.0;
        for (int c = 0; c<m.cols(); c++) {
            s += m(r, c);
        }
        col(r, 0) = s;
    }
    return col;
}

void element_sqrt(vnl_matrix<double> & m)
{
    for (int r = 0; r<m.rows(); r++) {
        for (int c = 0; c<m.cols(); c++) {
            m(r, c) = sqrt(m(r, c));
        }
    }
}

void element_abs(vnl_matrix<double> & m)
{
    for (int r = 0; r<m.rows(); r++) {
        for (int c = 0; c<m.cols(); c++) {
            m(r, c) = fabs(m(r, c));
        }
    }
}

vnl_matrix<double> min_of_columns(const vnl_matrix<double> & m)
{
    vnl_matrix<double> col(m.rows(), 1);
    col.fill(0.0);
    for (int r = 0; r<m.rows(); r++) {
        double v_min = std::numeric_limits<double>::max();
        for (int c = 0; c<m.cols(); c++) {
            if (m(r, c) < v_min) {
                v_min = m(r, c);
            }
        }
        col(r, 0) = v_min;
    }
    
    return col;
}

// 4 function do not need to understand
static vnl_matrix<double> temp_function_1(const vnl_matrix<double> & L, const vnl_matrix<double> & pMat)
{
    vnl_matrix<double> numerator = L.extract(L.rows(), 3, 0, 0) * pMat;
    vnl_matrix<double> L12 = L.extract(L.rows(), 2, 0, 0);
    vnl_matrix<double> L12_2 = element_product(L12, L12); // .^2
    vnl_matrix<double> L12_2_sum = sum_of_columns(L12_2); // sum(, 2)
    element_sqrt(L12_2_sum); // sqrt()
    vnl_matrix<double> P1 = element_quotient(numerator, L12_2_sum);
    return P1;
}

static vnl_matrix<double> temp_function_2(const vnl_matrix<double> & L, const vnl_matrix<double> & pMat)
{
    vnl_matrix<double> numerator = L.extract(L.rows(), 3, 0, 3) * pMat;
    vnl_matrix<double> L12 = L.extract(L.rows(), 2, 0, 3);
    vnl_matrix<double> L12_2 = element_product(L12, L12); // .^2
    vnl_matrix<double> L12_2_sum = sum_of_columns(L12_2); // sum(, 2)
    element_sqrt(L12_2_sum); // sqrt()
    vnl_matrix<double> P2 = element_quotient(numerator, L12_2_sum);
    return P2;
}

static vnl_matrix<double> temp_function_3(const vnl_matrix<double> & L, const vnl_matrix<double> & pMat)
{
    vnl_matrix<double> numerator = L.extract(L.rows(), 3, 0, 6) * pMat;
    vnl_matrix<double> L12 = L.extract(L.rows(), 2, 0, 6);
    vnl_matrix<double> L12_2 = element_product(L12, L12); // .^2
    vnl_matrix<double> L12_2_sum = sum_of_columns(L12_2); // sum(, 2)
    element_sqrt(L12_2_sum); // sqrt()
    vnl_matrix<double> P3 = element_quotient(numerator, L12_2_sum);
    return P3;
}

static vnl_matrix<double> temp_function_4(const vnl_matrix<double> & L, const vnl_matrix<double> & pMat)
{
    vnl_matrix<double> numerator = L.extract(L.rows(), 3, 0, 9) * pMat;
    vnl_matrix<double> L12 = L.extract(L.rows(), 2, 0, 9);
    vnl_matrix<double> L12_2 = element_product(L12, L12); // .^2
    vnl_matrix<double> L12_2_sum = sum_of_columns(L12_2); // sum(, 2)
    element_sqrt(L12_2_sum); // sqrt()
    vnl_matrix<double> P4 = element_quotient(numerator, L12_2_sum);
    return P4;
}


vnl_matrix<double> point_to_lines_dist(const vgl_point_2d<double> & p, const vnl_matrix<double> & lines)
{
    assert(lines.cols() == 3);
    
    double x = p.x();
    double y = p.y();
    
    // ? magic number
    double Tdata[] = {0,-1,1, 1,-1,0, 0,-1,0,
                    0,-1,1,1,1,0 ,0,-1,0,
                    0,1,1 ,1,-1,0,0,-1,0,
                    0,1,1 ,1,1,0 ,0,-1,0};
    vnl_matrix<double> T(12, 3, 12 * 3, Tdata);
    vnl_matrix<double> L = lines * T.transpose();
    
    // P(:,1) = (L(:,1:3)*[x,y,1]')./sqrt(sum(L(:,1:2).^2,2));
    vnl_matrix<double> pMat(3, 1);
    pMat(0, 0) = x;
    pMat(1, 0) = y;
    pMat(2, 0) = 1.0;
    
    vnl_matrix<double> P1 = temp_function_1(L, pMat);
    vnl_matrix<double> P2 = temp_function_2(L, pMat);
    vnl_matrix<double> P3 = temp_function_3(L, pMat);
    vnl_matrix<double> P4 = temp_function_4(L, pMat);
    
    element_abs(P1);
    element_abs(P2);
    element_abs(P3);
    element_abs(P4);
    vnl_matrix<double> P(P1.rows(), 4);
    P.update(P1, 0, 0);
    P.update(P2, 0, 1);
    P.update(P3, 0, 2);
    P.update(P4, 0, 3);
    
    vnl_matrix<double> D = min_of_columns(P);
    
    return D;
}

/*
 function NormVP = normalize_PC_points(VanP, SpaceSize)
 NormVP = (2.*VanP -(SpaceSize + 1))./(SpaceSize - 1);
 end
 */

vgl_point_2d<double> normalize_PC_points(const vgl_point_2d<double> & p, int spaceSize)
{
    double x = (2.0 * p.x() - (spaceSize + 1))/(spaceSize - 1);
    double y = (2.0 * p.y() - (spaceSize + 1))/(spaceSize - 1);
    return vgl_point_2d<double>(x, y);
}

/*
 function [VanPC] = find_maximum(Space, R)
 [r, c] = find(max(Space(:)) == Space);
 
 S = padarray(double(Space),[R,R]);
 
 O = S(r(1):r(1)+R*2, c(1):c(1)+R*2);
 
 [mc,mr] = meshgrid([-R:R],[-R:R]);
 SR = O.*mr;
 SC = O.*mc;
 
 C = c(1) + sum(SC(:))/sum(O(:));
 R = r(1) + sum(SR(:))/sum(O(:));
 
 VanPC = [C,R];
 end
 */
// every colum is the samte
static vnl_matrix<double> mc_mesh_grid(int R)
{
    vnl_matrix<double> mc(R*2 + 1, R*2 + 1);
    for (int c = -R;  c <= R; c++) {
        for (int r = 0; r<mc.rows(); r++) {
            mc(r, c+ R) = c;
        }
    }
    return mc;
}

static vnl_matrix<double> mr_mesh_grid(int R)
{
    vnl_matrix<double> mr(R*2 + 1, R*2 + 1);
    for (int r = -R;  r<= R; r++) {
        for (int c = 0; c <= mr.cols(); c++) {
            mr(r+R, c) = r;
        }
    }
    return mr;
}

static double element_sum(const vnl_matrix<double> &m)
{
    double sum = 0;
    for (int r = 0; r < m.rows(); r++) {
        for (int c = 0; c < m.cols(); c++) {
            sum += m(r, c);
        }
    }
    return sum;
}


vgl_point_2d<double> find_maximum(int * space, int spaceSize, int subPixelRadius)
{
    int maxV = 0;
    int maxIdx = 0;
    vnl_matrix<double> spaceMat(spaceSize, spaceSize);
    for (int i =0; i<spaceSize * spaceSize; i++) {
        if (space[i] > maxV) {
            maxV = space[i];
            maxIdx = i;
        }
        int r = i/spaceSize;
        int c = i%spaceSize;
        spaceMat(r, c) = space[i];
    }
    
 //   vnl_matlab_filewrite writer("space1.mat");
 //   writer.write(spaceMat, "space");
 //   printf("save to %s\n", "space1.mat");
    
    double r = maxIdx/spaceSize; // r,c is subscribe in matlab
    double c = maxIdx%spaceSize;
    // close to boundary
    vnl_matrix<double> expandedMat(spaceSize + subPixelRadius * 2, spaceSize + subPixelRadius * 2);
    expandedMat.fill(0);
    expandedMat.update(spaceMat, subPixelRadius, subPixelRadius);
    
    //O = S(r(1):r(1)+R*2, c(1):c(1)+R*2);
    vnl_matrix<double> O = expandedMat.extract(subPixelRadius * 2 + 1, subPixelRadius * 2 + 1, r, c);
    vnl_matrix<double> mc = mc_mesh_grid(subPixelRadius);
    vnl_matrix<double> mr = mr_mesh_grid(subPixelRadius);
    
    //SR = O.*mr;
    //SC = O.*mc;
    vnl_matrix<double> SR = element_product(O, mr);
    vnl_matrix<double> SC = element_product(O, mc);
    
    double sum_O  = element_sum(O);
    double sum_SR = element_sum(SR);
    double sum_SC = element_sum(SC);
    
    // C = c(1) + sum(SC(:))/sum(O(:));
    // R = r(1) + sum(SR(:))/sum(O(:));
    
    double C = c + sum_SC/sum_O;
    double R = r + sum_SR/sum_O;
    
    
    vgl_point_2d<double> maxLoc( R + 1.0, C + 1.0); // first index is 1 as in matlab, reverse R, C, as the data is column dominant
    
    return maxLoc;
}


void rasterize_lines(float * line, int * endpoints, int * space, int cSpaceSize, int numLines)
{
    int * v_steps = (int*)malloc(sizeof(int)*cSpaceSize);
    for(int i = 0; i < cSpaceSize; i++)
        v_steps[i] = i*cSpaceSize;
    
    for(int i = 0; i < numLines; i++)
    {
        int * end = endpoints + i*8;
        int weight =  int(*(line + i*4 + 3));
        
        for(int j=0; j<6; j+=2)
        {
            if(abs(end[j+3] - end[j+1]) > abs(end[j+2] - end[j]))
                lineV(end[j], end[j+1], end[j+2], end[j+3], space, v_steps, weight);
            else
                lineH(end[j], end[j+1], end[j+2], end[j+3], space, v_steps, weight);
        }
        space[v_steps[end[7]] + end[6]] += weight;
    }
    free(v_steps);
}

void lines_end_points(float * line, int * endpoints, float space_c, int numLines)
{
    int center = round(space_c);
    for(int i = 0; i < numLines; i++)
    {
        float a = *(line + i*4);
        float b = *(line + i*4 + 1);
        float c = *(line + i*4 + 2);
        
        float alpha = float(sgn(a*b));
        float beta = float(sgn(b*c));
        float gamma = float(sgn(a*c));
        
        int * end = endpoints + i*8;
        
        float a_x = alpha*a / (c + gamma*a);
        float b_x = -alpha*c / (c + gamma*a);
        
        end[1] = round((a_x + 1) * space_c);
        end[0] = round((b_x + 1) * space_c);
        
        end[3] = round((b / (c + beta*b) + 1) * space_c);
        end[2] = center;
        
        end[5] = center;
        end[4] = round((b / (a + alpha*b) + 1) * space_c);
        
        end[7] = round((-a_x + 1) * space_c);
        end[6] = round((-b_x + 1) * space_c);
    }
}

inline void lineH(int x0, int y0, int x1, int y1, int * space, int * y_steps, int weight)
{
    float slope = (float)(y1 - y0)/(x1 - x0);
    
	//float y_iter = y0 + 0.5f;
    float y_start = y0 + 0.5f;
    float y_iter = y_start;
    
    int step = (x0 < x1) ? 1 : -1;
    slope *= step;
    
    for(int x = x0, c = 1; x != x1; x+=step, c++)
	{
        space[y_steps[int(y_iter)] + x] += weight;
        //y_iter += slope;
        y_iter = y_start + c*slope;
	}
    
}

inline void lineV(int x0, int y0, int x1, int y1, int * space, int * y_steps, int weight)
{
    float slope = (x1 - x0)/(float)(y1 - y0);
    
	//float x_iter = x0 + 0.5f;
    float x_start = x0 + 0.5f;
    float x_iter = x_start;
    int step = (y0 < y1) ? 1 : -1;
    slope *= step;
    
    for(int y = y0, c = 1; y != y1; y+=step, c++)
	{
        space[y_steps[y] + int(x_iter)] += weight;
        //x_iter += slope;
        x_iter = x_start + c*slope;
	}
}
/*
function VanishCC = PC_point_to_CC(Normalization, VanishPC, ImgSize)
u = VanishPC(:,1);
v = VanishPC(:,2);
NormVanishCC = [v, sign(v).*v + sign(u).*u - 1, u];

reg = abs(NormVanishCC(:,3)) > 0.005;

NormVanishCC(reg,:) = bsxfun(@rdivide, NormVanishCC(reg,:), NormVanishCC(reg,3));
NormVanishCC(~reg,:) = normr(NormVanishCC(~reg,:));

VanishCC = NormVanishCC;
VanishCC(~reg,3) = 0;

w = ImgSize(2);
h = ImgSize(1);
m = max(ImgSize);

VanishCC(reg,1) = (VanishCC(reg,1)./Normalization.*(m - 1) + w + 1)./2;
VanishCC(reg,2) = (VanishCC(reg,2)./Normalization.*(m - 1) + h + 1)./2;
end
 */

vgl_homg_point_2d<double> PC_point_to_CC(double normalization, const vgl_point_2d<double> & PC_norm_VC, int imageW, int imageH)
{
    const double u = PC_norm_VC.x();
    const double v = PC_norm_VC.y();
    
    vnl_vector_fixed<double, 3 > normVanishCC(v, sgn(v) * v + sgn(u) * u - 1, u);
    bool reg = fabs(normVanishCC[2]) >= 0.005;
    
    // NormVanishCC(reg,:) = bsxfun(@rdivide, NormVanishCC(reg,:), NormVanishCC(reg,3));
    // NormVanishCC(~reg,:) = normr(NormVanishCC(~reg,:));
    if (reg) {
        normVanishCC /= normVanishCC[2];
    }
    else
    {
        normVanishCC = normVanishCC.normalize();
    }
    
    vnl_vector_fixed<double, 3> vanishCC = normVanishCC;
    if (!reg) {
        vanishCC[2] = 0.0;
    }
    
    int m = std::max(imageW, imageH);
    double x = (vanishCC[0]/normalization * (m - 1) + imageW + 1)/2.0;
    double y = (vanishCC[1]/normalization * (m - 1) + imageH + 1)/2.0;
    
    vgl_homg_point_2d<double> vp(x, y, vanishCC[2]);
    return vp;
}


vcl_vector<vgl_point_2d<double> > VilFindVanishingPoints::findVanP(const vcl_vector<vnl_vector_fixed<double, 4> > & lines,
                                                                   const VanishingPointParameter & para)
{
    const int spaceSize = para.spaceSize_;
    const int nVP = para.nVP_;
    const double Threshold = 0.05;
    
    int nLine = (int)lines.size();
    float *lineData = new float[nLine * 4];
    for (int i =0; i<lines.size(); i++) {
        for (int j = 0; j<4; j++) {
            lineData[4 * i + j] = lines[i][j];
        }
    }
    vnl_matrix<double> lines_123 = vnl_matrix<double>(nLine, 3); // data of lines with 1,2,3 dimension
    for (int i = 0; i<lines.size(); i++) {
        for (int j = 0; j<3; j++) {
            lines_123(i, j) = lines[i][j];
        }
    }
    // memory for diamond space
    int *pSpace = new int[spaceSize * spaceSize];
    
    vcl_vector<vgl_point_2d<double> > normed_PC_VPs;
    for (int i =0; i<nVP && i < 3; i++) {
        // to release memory space at the end of loop
        // space is column dominant
        raseter_space(spaceSize, lineData, nLine, pSpace);
        
        // position with maximum value
        vgl_point_2d<double> max_p = find_maximum(pSpace, spaceSize, 2.0);
        
      //  vcl_cout<<"max_p is "<<max_p<<vcl_endl;
        vgl_point_2d<double> norm_max_p = normalize_PC_points(max_p, spaceSize);
      //  vcl_cout<<"normalized max p is "<<norm_max_p<<vcl_endl;
        
        normed_PC_VPs.push_back(norm_max_p);
        
        //get lines close to VP
        vnl_matrix<double> Distance = point_to_lines_dist(norm_max_p, lines_123);
        assert(Distance.cols() == 1);
        
        //%remove lines
        //LinesData(:,(Distance < Threshold)') = [];
        vcl_vector<int> distance_index;
        for (int j = 0; j<Distance.rows(); j++) {
            if (Distance(j, 0) >= Threshold) {
                distance_index.push_back(j);
            }
        }
        
        // update lineData and lines_123
        float *pNewLineData = new float[distance_index.size() * 4];
        for (int j = 0; j<distance_index.size(); j++) {
            int idx = distance_index[j];
            for (int k = 0; k<4; k++) {
                pNewLineData[4 * j + k] = lineData[4 * idx + k];
            }
        }
        
        vnl_matrix<double> new_lines_123((int)distance_index.size(), 3);
        for (int j = 0; j<distance_index.size(); j++) {
            int idx = distance_index[j];
            for (int k = 0; k<3; k++) {
                new_lines_123(j, k) = lineData[4 * idx + k];
            }
        }
        
        delete []lineData;
        lineData = pNewLineData;
        lines_123 = new_lines_123;
        nLine = (int)distance_index.size();
    }
    
    // piont from parallal coordinate to xy coordinate
    vcl_vector<vgl_point_2d<double> > VPs;
    for (int i = 0; i<normed_PC_VPs.size(); i++) {
        // parallel coordinate to Cartesian coordinate
        vgl_homg_point_2d<double> p = PC_point_to_CC(1.0, normed_PC_VPs[i], para.imageW_, para.imageH_);
        if (p.w() != 0.0) {
            VPs.push_back(vgl_point_2d<double>(p));
          //  vcl_cout<<"vanishing points "<<VPs.back()<<vcl_endl;
        }
    }
         
    
    delete []lineData;
    delete []pSpace;
    return VPs;
}