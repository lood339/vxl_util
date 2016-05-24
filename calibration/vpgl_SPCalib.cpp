//
//  vpgl_SPCalib.cpp
//  QuadCopter
//
//  Created by jimmy on 6/23/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include <assert.h>
#include <vnl/vnl_math.h>
#include "vpgl_SPCalib.h"
#include "vnl_plus.h"
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include "vgl_vnl_operator.h"

static double cosFromTan(const double tanPan)
{
    return   1.0 /sqrt(1.0 + tanPan * tanPan); // 14
}
static double sinFromTan(const double tanPan)
{
    return tanPan/sqrt(1.0 + tanPan * tanPan); // 15
}

int VpglSPCalib::estimatePanTilt(const vgl_point_3d<double> &wld_p, const vgl_point_2d<double> &img_p,
                                  double & pan1, double & tilt1, double & pan2, double & tilt2)
{
    const double X = wld_p.x();
    const double Y = wld_p.y();
    const double Z = wld_p.z();
    const double U = img_p.x();
    const double V = img_p.y();
    
    // first case, only one solution
    // pixel in in the middle columne of the image
    if (VnlPlus::isEqualZero(U)) {
        // this branch is not tested
        assert(fabs(Y) > 0.00001); // may have problem when pan is close to pi/2
        double tanPan = -X/Y;   // 13
        double cosPan = cosFromTan(tanPan); // 14
        double sinPan = sinFromTan(tanPan); // 15
        double tanTilt_numerator   = Z + V * Y * cosPan - V * X * sinPan; // 13
        double tanTilt_denominator = Y * cosPan - X * sinPan - V * Z;
        
        pan1  = atan2(-X, Y); // or (- Y, X) ?
        tilt1 = atan2(tanTilt_denominator, tanTilt_numerator);
        
        assert(pan1 >= -0.5 * vnl_math::pi);
        assert(pan1 <= 0.5  * vnl_math::pi);
        return 1;
    }
    else
    {
        // second case
        double a = (V*V + 1.0)*Y*Y - U*U*(X*X + Z*Z); // 19
        double b =  2.0 * X * Y * (U * U + V * V + 1.0);     // 20
        double c = (V* V + 1.0) * X * X - U * U * (Z * Z + Y * Y); // 21
        
        if (VnlPlus::isEqualZero(a)) {
            // may have unique solution
            if (!VnlPlus::isEqualZero(b)) {
                
                double tanPan = -c/b;
                double cosPan = 1.0   /sqrt(1 + tanPan * tanPan);
                double sinPan = tanPan/sqrt(1 + tanPan * tanPan);
                double det_Apan = U * (Z*Z + (Y*cosPan - X*sinPan) * (Y*cosPan - X*sinPan)); // 11
                assert(!VnlPlus::isEqualZero(det_Apan));
                
                double cosTilt = (Y * cosPan - V * Z - X * sinPan) * (X * cosPan + Y * sinPan)/(det_Apan);  // 16
                double sinTilt = (Z + V * Y * cosPan - V * X * sinPan) * (X * cosPan + Y * sinPan) /(det_Apan); // 17
                tilt1 = atan2(sinTilt, cosTilt);
                pan1 = atan2(-c, b);   // 22
                return 1;
            }
            else
            {
                // no solution
                return 0;
            }
        }
        else
        {
            // two solutions
            double delta = b * b - 4 * a * c;
            if (delta < 0) {
                return 0;
            }
            else if(VnlPlus::isEqualZero(delta)) // delta is very small
            {
                double tanPan = -b/(2.0 * a);
                double cosPan = 1.0   /sqrt(1 + tanPan * tanPan);
                double sinPan = tanPan/sqrt(1 + tanPan * tanPan);
                double det_Apan = U * (Z*Z + (Y*cosPan - X*sinPan) * (Y*cosPan - X*sinPan)); // 11
                assert(!VnlPlus::isEqualZero(det_Apan));
                
                double cosTilt = (Y * cosPan - V * Z - X * sinPan) * (X * cosPan + Y * sinPan)/(det_Apan);  // 16
                double sinTilt = (Z + V * Y * cosPan - V * X * sinPan) * (X * cosPan + Y * sinPan) /(det_Apan); // 17
                tilt1 = atan2(sinTilt, cosTilt);
                pan1 = atan2(-b, 2.0 * a);
                return 1;
            }
            else
            {
                // two solution
                {
                    double tanPan = (-b + sqrt(delta))/(2.0 * a); // 22
                    double cosPan = cosFromTan(tanPan);
                    double sinPan = sinFromTan(tanPan);
                    double det_Apan = U * (Z*Z + (Y*cosPan - X*sinPan) * (Y*cosPan - X*sinPan)); // 11
                    assert(!VnlPlus::isEqualZero(det_Apan));
                    
                    double cosTilt = (Y * cosPan - V * Z - X * sinPan) * (X * cosPan + Y * sinPan)/(det_Apan);  // 16 original equation
                    double sinTilt = (Z + V * Y * cosPan - V * X * sinPan) * (X * cosPan + Y * sinPan) /(det_Apan); // 17
                    
                    tilt1 = atan2(sinTilt, cosTilt);
                    pan1 = atan2(-b + sqrt(delta), 2.0 * a);
                }
                
                {
                    double tanPan = (-b - sqrt(delta))/(2.0 * a);
                    double cosPan = cosFromTan(tanPan);
                    double sinPan = sinFromTan(tanPan);
                    double det_Apan = U * (Z*Z + (Y*cosPan - X*sinPan) * (Y*cosPan - X*sinPan)); // 11
                    assert(!VnlPlus::isEqualZero(det_Apan));
                    
                    double cosTilt = (Y * cosPan - V * Z - X * sinPan) * (X * cosPan + Y * sinPan)/(det_Apan);  // 16 original equation
                    double sinTilt = (Z + V * Y * cosPan - V * X * sinPan) * (X * cosPan + Y * sinPan) /(det_Apan); // 17
                    
                    tilt2 = atan2(sinTilt, cosTilt);
                    pan2 = atan2(-b - sqrt(delta), 2.0 * a);
                }
                return 2;
            }
        }
    }
    return 0;
}

int VpglSPCalib::estimateYPanXTilt(const vgl_point_3d<double> &wldPt, const vgl_point_2d<double> & projectPt,
                                   vnl_vector_fixed<double, 4> & panTilts)
{
    // from document "single point pan-tilt camera calibration", section 2.2 trick replacement
    const double X = wldPt.x();
    const double Y = wldPt.y();
    const double Z = wldPt.z();
    const double U = projectPt.x();
    const double V = projectPt.y();
    
    // first case, only one solution
    // pixel in in the middle columne of the image
    double pan1 = 0.0;
    double tilt1 = 0.0;
    double pan2 = 0.0;
    double tilt2 = 0.0;
    if (VnlPlus::isEqualZero(U)) {
        assert(!VnlPlus::isEqualZero(Z));
        
        double tanPan = X/Z;   //
        double cosPan = cosFromTan(tanPan); //
        double sinPan = sinFromTan(tanPan); //
        double tanTilt_numerator   = V * X * sinPan + V *  Z * cosPan - Y;
        double tanTilt_denominator = V * Y + X * sinPan + Z * cosPan;
        
        pan1  = atan2(X, Z);
        tilt1 = atan2(tanTilt_denominator, tanTilt_numerator);
        
        assert(pan1 >= -0.5 * vnl_math::pi);
        assert(pan1 <= 0.5  * vnl_math::pi);
        panTilts[0] = pan1;
        panTilts[1] = tilt1;
        return 1;
    }
    else
    {
        // second case
        double a = (V*V + 1.0)*Z*Z - U*U*(X*X + Y*Y);
        double b =  -2.0 * X * Z * (U * U + V * V + 1.0);
        double c = (V*V + 1.0) * X * X - U * U * (Y * Y + Z * Z);
        
        if (VnlPlus::isEqualZero(a)) {
            // may have unique solution
            if (!VnlPlus::isEqualZero(b)) {
                
                double tanPan = -c/b;
                double cosPan = cosFromTan(tanPan);
                double sinPan = sinFromTan(tanPan);
                double det_Apan = U * (Y*Y + (Z*cosPan + X*sinPan) * (Z*cosPan + X*sinPan));
                assert(!VnlPlus::isEqualZero(det_Apan));
                
                double cosTilt = (V*Y + X*sinPan + Z*cosPan)   * (X*cosPan - Z*sinPan)/(det_Apan);
                double sinTilt = (V*X*sinPan + V*Z*cosPan - Y) * (X*cosPan - Z*sinPan)/(det_Apan);
                tilt1 = atan2(sinTilt, cosTilt);
                pan1  = atan2(-c, b);
                
                panTilts[0] = pan1;
                panTilts[1] = tilt1;
                return 1;
            }
            else
            {
                // no solution
                return 0;
            }
        }
        else
        {
            // two solutions
            double delta = b * b - 4 * a * c;
            if (delta < 0) {
                return 0;
            }
            else if(VnlPlus::isEqualZero(delta)) // delta is very small
            {
                double tanPan = -b/(2.0 * a);
                double cosPan = cosFromTan(tanPan);
                double sinPan = sinFromTan(tanPan);
                double det_Apan = U * (Y*Y + (Z*cosPan + X*sinPan) * (Z*cosPan + X*sinPan));
                assert(!VnlPlus::isEqualZero(det_Apan));
                
                double cosTilt = (V*Y + X*sinPan + Z*cosPan)   * (X*cosPan - Z*sinPan)/(det_Apan);
                double sinTilt = (V*X*sinPan + V*Z*cosPan - Y) * (X*cosPan - Z*sinPan)/(det_Apan);
                tilt1 = atan2(sinTilt, cosTilt);
                pan1 = atan2(-b, 2.0 * a);
                
                panTilts[0] = pan1;
                panTilts[1] = tilt1;
                return 1;
            }
            else
            {
                // two solution
                {
                    double tanPan = (-b + sqrt(delta))/(2.0 * a); // 22
                    double cosPan = cosFromTan(tanPan);
                    double sinPan = sinFromTan(tanPan);
                    double det_Apan = U * (Y*Y + (Z*cosPan + X*sinPan) * (Z*cosPan + X*sinPan));
                    assert(!VnlPlus::isEqualZero(det_Apan));
                    
                    double cosTilt = (V*Y + X*sinPan + Z*cosPan)   * (X*cosPan - Z*sinPan)/(det_Apan);
                    double sinTilt = (V*X*sinPan + V*Z*cosPan - Y) * (X*cosPan - Z*sinPan)/(det_Apan);
                    
                    tilt1 = atan2(sinTilt, cosTilt);
                    pan1 = atan2(-b + sqrt(delta), 2.0 * a);
                }
                
                {
                    double tanPan = (-b - sqrt(delta))/(2.0 * a);
                    double cosPan = cosFromTan(tanPan);
                    double sinPan = sinFromTan(tanPan);
                    double det_Apan = U * (Y*Y + (Z*cosPan + X*sinPan) * (Z*cosPan + X*sinPan));
                    assert(!VnlPlus::isEqualZero(det_Apan));
                    
                    double cosTilt = (V*Y + X*sinPan + Z*cosPan)   * (X*cosPan - Z*sinPan)/(det_Apan);
                    double sinTilt = (V*X*sinPan + V*Z*cosPan - Y) * (X*cosPan - Z*sinPan)/(det_Apan);
                    
                    tilt2 = atan2(sinTilt, cosTilt);
                    pan2 = atan2(-b - sqrt(delta), 2.0 * a);
                }
                
                panTilts[0] = pan1;
                panTilts[1] = tilt1;
                panTilts[2] = pan2;
                panTilts[3] = tilt2;
                return 2;
            }
        }
    }
    return 0;
}

int VpglSPCalib::estimateFocalLength(const vgl_point_3d<double> & wldPt, const vgl_point_2d<double> & projectPt,
                                     const vgl_point_2d<double> & principlePoint, double & fl)
{
    const double X = wldPt.x();
    const double Y = wldPt.y();
    const double Z = wldPt.z();
    const double U = projectPt.x();
    const double V = projectPt.y();
    const double s = principlePoint.x();
    const double t = principlePoint.y();
    
    if (VnlPlus::isEqualZero(X) && VnlPlus::isEqualZero(Y)) {
        return  0;
    }
    else if(VnlPlus::isEqualZero(X))
    {
        fl = (V*Z - t*Z)/Y;
        return 1;
    }
    else if(VnlPlus::isEqualZero(Y))
    {
        fl = (V*Z - t*Z)/Y;
        return 1;
    }
    else
    {
        fl = 0.0;
        fl += (V*Z - t*Z)/Y;
        fl += (U*Z - s*Z)/X;
        fl /= 2.0;
        return 1;
    }
    return 0;
}

class estimateFocalLength_residual: public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_point_3d<double> > wldPts_;
    const vcl_vector<vgl_point_2d<double> > projectPts_;
    const vgl_point_2d<double> pp_;
    
public:
    estimateFocalLength_residual(const vcl_vector<vgl_point_3d<double> > & wldPts, const vcl_vector<vgl_point_2d<double> > & projectPts,
                                 const vgl_point_2d<double> & pp):
    vnl_least_squares_function(1, (unsigned int)wldPts.size() * 2, no_gradient),
    wldPts_(wldPts),
    projectPts_(projectPts),
    pp_(pp)
    {
        assert(wldPts.size() == projectPts.size());
        assert(wldPts.size() >= 1);
    }
    
    void f(const vnl_vector<double> &x, vnl_vector<double> &fx)
    {
        double fl = x[0];
        vnl_matrix_fixed<double, 3, 3> K;
        K(0, 0) = fl; K(0, 1) = 0;  K(0, 2) = pp_.x();
        K(1, 0) = 0 ; K(1, 1) = fl; K(1, 2) = pp_.y();
        K(2, 0) = 0 ; K(2, 1) = 0;  K(2, 2) = 1.0;
        
        int idx = 0;
        for (int i = 0; i<wldPts_.size(); i++) {
            vgl_point_3d<double> p = K * wldPts_[i];
            double x = p.x()/p.z();
            double y = p.y()/p.z();
            fx[idx] = projectPts_[i].x() - x;
            idx++;
            fx[idx] = projectPts_[i].y() - y;
            idx++;
        }
    }
};

bool VpglSPCalib::estimateFocalLength(const vcl_vector<vgl_point_3d<double> > & wldPts, const vcl_vector<vgl_point_2d<double> > & projectPts,
                                      const vgl_point_2d<double> & pp, const double & init_fl, double & fl)
{
    assert(wldPts.size() == projectPts.size());
    assert(wldPts.size() >= 2);
    
    estimateFocalLength_residual residual(wldPts, projectPts, pp);
    
    vnl_vector<double> x(1, 0.0);
    x[0] = init_fl;
    
    vnl_levenberg_marquardt lmq(residual);
    bool isMinimizeOk = lmq.minimize(x);
    if (!isMinimizeOk) {
        printf(("Error: LMQ cao not converge.\n"));
        lmq.diagnose_outcome();
        return false;
    }
    //lmq.diagnose_outcome();
    fl = x[0];
    return true;
}
