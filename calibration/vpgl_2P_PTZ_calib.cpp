//
//  vpgl_2P_PTZ_calib.cpp
//  QuadCopter
//
//  Created by jimmy on 6/26/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vpgl_2P_PTZ_calib.h"
#include <assert.h>
#include "vpgl_SPCalib.h"
#include <vnl/vnl_inverse.h>
#include "vgl_vnl_operator.h"
#include <vnl/vnl_math.h>
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vgl/vgl_distance.h>
#include "vnl_plus.h"


static vnl_matrix_fixed<double, 3, 3> RotationfromPanTilt(double pan, double tilt)
{
    double sp = sin(pan);
    double cp = cos(pan);
    double st = sin(tilt);
    double ct = cos(tilt);
    vnl_matrix_fixed<double, 3, 3> curR;
    curR(0, 0) = cp;       curR(0, 1) = 0;       curR(0, 2) = -sp;
    curR(1, 0) = st * sp;  curR(1, 1) =  ct;     curR(1, 2) = st * cp;
    curR(2, 0) = ct * sp;  curR(2, 1) = -st;     curR(2, 2) = ct * cp;
    return curR;
}

bool Vpgl2PPTZCalib::iterative_calib(const vgl_point_3d<double> & cc, const vgl_rotation_3d<double> & Rs,
                                     const vgl_point_2d<double> & pp, const double init_fl,
                                     const vcl_vector<vgl_point_3d<double> >  & wld_pts,
                                     const vcl_vector<vgl_point_2d <double> > & img_pts,
                                     vnl_vector_fixed<double, 3> & ptz,
                                     vpgl_perspective_camera<double> & camera,
                                     const TwopointCalibParameter & para,
                                     int max_iter_num,
                                     bool verbose)
{
    assert(wld_pts.size() == 2);
    assert(img_pts.size() == 2);
    
    // parameters
    vnl_vector_fixed<double, 2> threshold;
    threshold[0] = - vnl_math::pi/ 2.0;
    threshold[1] =   vnl_math::pi/ 2.0;
    const double pan_convergence_threshold  = para.pan_convergence_;
    const double tilt_convergence_threshold = para.tilt_convergence_;
    const double fl_convergence_threshold   = para.fl_convergence_;
    
    double prev_fl    = init_fl;
    double prev_pan   = 0;
    double prev_tilt  = 0;
    double cur_fl     = 0;
    double cur_pan    = 0;
    double cur_tilt   = 0;
    
    bool isConverge = false;
    for (int i = 0; i<max_iter_num; i++) {
        // Step 1, estimate pan, tilt from previous focal length
        vpgl_calibration_matrix<double> curK(prev_fl, pp);
        vgl_point_3d<double> pt1(wld_pts[0].x() - cc.x(), wld_pts[0].y() - cc.y(), wld_pts[0].z() - cc.z()); // transpose
        if (i%2 == 1) {
            pt1 = vgl_point_3d<double>(wld_pts[1].x() - cc.x(), wld_pts[1].y() - cc.y(), wld_pts[1].z() - cc.z());
        }
        pt1 = Rs.as_matrix() * pt1; // rotate
        
        vnl_matrix_fixed<double, 3, 3> K_inv = vnl_inverse(curK.get_matrix());
        vgl_point_2d<double> proj_p = K_inv * img_pts[0];
        if (i%2 == 1) {
            proj_p = K_inv * img_pts[1];
        }
       
        vnl_vector_fixed<double, 4> curPantilts;
        int num = VpglSPCalib::estimateYPanXTilt(pt1, proj_p, curPantilts);
        //vcl_cout<<"pan tilt: "<<curPantilts*180.0/vnl_math::pi<<vcl_endl;
        // get valid pan, tilt angles. It is not necessary as the rotation matrix of two solution should be the same
        vnl_vector_fixed<double, 2> validPanTilt;
        if (num == 0) {
            printf("Warning: estimated pan, tilt angle pair is zero.\n");
            continue;
        }
        else if(num == 1)
        {
            if (curPantilts[0] >= threshold[0] && curPantilts[0] <= threshold[1] &&
                curPantilts[1] >= threshold[0] && curPantilts[1] <= threshold[1])
            {
                validPanTilt[0] = curPantilts[0];
                validPanTilt[1] = curPantilts[1];
            }
            else
            {
                if(verbose){
                    printf("Warning: can not find valid pan, tilt angle  1\n");
                }
                continue;
            }
        }
        else
        {
            bool isValid = Vpgl2PPTZCalib::getValidPanTilt(curPantilts[0], curPantilts[1], curPantilts[2], curPantilts[3], threshold, validPanTilt);
            if (!isValid) {
                if(verbose){
                    printf("Warning: can not find valid pan, tilt angle  2\n");
                }
                continue;
            }
        }
        
        // Step 2, estimate focal length by given pan, tilt
        cur_pan  = validPanTilt[0];
        cur_tilt = validPanTilt[1];
        vnl_matrix_fixed<double, 3, 3> curR = RotationfromPanTilt(cur_pan, cur_tilt);
        
        // points in camera coordiante after translation and rotation
        vcl_vector<vgl_point_3d<double> > pts1;
        for (int j = 0; j<wld_pts.size(); j++) {
            vgl_point_3d<double> pt(wld_pts[j].x() - cc.x(), wld_pts[j].y() - cc.y(), wld_pts[j].z() - cc.z());
            pt = Rs.as_matrix() * pt;  // transpose by stationary matrix first
            pt = curR * pt;
            pts1.push_back(pt);
        }
        bool isEsttimatedFL = VpglSPCalib::estimateFocalLength(pts1, img_pts, pp, prev_fl, cur_fl);
        if (!isEsttimatedFL) {
            continue;
        }
        
        // Step 3. LMQ optimize whole parameter PTZ
        vnl_vector_fixed<double, 3> init_ptz;
        vnl_vector_fixed<double, 3> opt_ptz;
        init_ptz[0] = cur_pan;
        init_ptz[1] = cur_tilt;
        init_ptz[2] = cur_fl;
        bool isOptPTZ = Vpgl2PPTZCalib::optimizePTZ(cc, Rs, pp, wld_pts, img_pts, init_ptz, opt_ptz);
        if (isOptPTZ) {
            cur_pan  = opt_ptz[0];
            cur_tilt = opt_ptz[1];
            cur_fl   = opt_ptz[2];
        }
        // measure if it is converge
        if (i != 0) {
            if (fabs(cur_fl - prev_fl)     <= fl_convergence_threshold &&
                fabs(cur_pan - prev_pan)   <= pan_convergence_threshold &&
                fabs(cur_tilt - prev_tilt) <= tilt_convergence_threshold) {
                if (verbose) {
                    printf("converge at %d iteration. \n", i);
                }
                isConverge = true;
                break;
            }
        }
      //  printf("cur fl, pan, tilt %f %f %f\n", cur_fl, cur_pan * 180.0 / vnl_math::pi, cur_tilt * 180.0 / vnl_math::pi);
        
        prev_fl  = cur_fl;
        prev_pan  = cur_pan;
        prev_tilt = cur_tilt;
    }
   // printf("\n\n\n");
    if (isConverge)
    {
        ptz[0] = cur_pan  * 180.0 / vnl_math::pi;
        ptz[1] = cur_tilt * 180.0 / vnl_math::pi;
        ptz[2] = cur_fl;
        
        // composite a camera
        vpgl_calibration_matrix<double> K(cur_fl, pp);
        vnl_matrix_fixed<double, 3, 3> R_pantilt = RotationfromPanTilt(cur_pan, cur_tilt);
        vnl_matrix_fixed<double, 3, 3> rData = R_pantilt * Rs.as_matrix();
        
        vgl_rotation_3d<double> R(rData);
        camera.set_calibration(K);
        camera.set_camera_center(cc);
        camera.set_rotation(R);
        
        return true;
    }
    else
    {
        // only for test purpose
        ptz[0] = cur_pan  * 180.0 / vnl_math::pi;
        ptz[1] = cur_tilt * 180.0 / vnl_math::pi;
        ptz[2] = cur_fl;
        
        vpgl_calibration_matrix<double> K(cur_fl, pp);
        vnl_matrix_fixed<double, 3, 3> R_pantilt = RotationfromPanTilt(cur_pan, cur_tilt);
        vnl_matrix_fixed<double, 3, 3> rData = R_pantilt * Rs.as_matrix();
        
        vgl_rotation_3d<double> R(rData);
        camera.set_calibration(K);
        camera.set_camera_center(cc);
        camera.set_rotation(R);
        
        return false;
    }
    return false;
}

bool Vpgl2PPTZCalib::approximate_focal_length(const vgl_rotation_3d<double> & Rs,
                                              const vgl_point_3d<double> & cc,
                                              const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                              const vcl_vector<vgl_point_2d <double> > & img_pts,
                                              double & fl)
{
    assert(wld_pts.size() == 2);
    assert(img_pts.size() == 2);
    
    // translate and rotate world coordinate to camera coordinate
    vcl_vector<vgl_point_3d<double> > pts;
    for (int i = 0; i<wld_pts.size(); i++) {
        vgl_point_3d<double> p = vgl_point_3d<double>(wld_pts[i].x() - cc.x(), wld_pts[i].y() - cc.y(), wld_pts[i].z() - cc.z());
        p = Rs.as_matrix() * p;
        pts.push_back(p);
    }
    vgl_point_3d<double> mid = centre(pts[0], pts[1]);
    double dis_image = vgl_distance(img_pts[0], img_pts[1]);
    double dis_world = vgl_distance(pts[0], pts[1]);
    if (VnlPlus::isEqualZero(dis_world)) {
        return false;
    }
    double dis_mid = sqrt(mid.x() * mid.x() + mid.y() * mid.y() + mid.z() * mid.z());
    fl = dis_image/dis_world * dis_mid;
    return true;    
}

bool Vpgl2PPTZCalib::getValidPanTilt(double & pan1, double & tilt1, double & pan2, double & tilt2,
                                     const vnl_vector_fixed<double, 2> & threshold,
                                     vnl_vector_fixed<double, 2> & panTilt)
{
    const double min_v = threshold[0];
    const double max_v = threshold[1];
    
    if (pan1 >= min_v && pan1 <= max_v && tilt1 >= min_v && tilt1 <= max_v) {
        panTilt[0] = pan1;
        panTilt[1] = tilt1;
        
        return true;
    }
    else if (pan2 >= min_v && pan2 <= max_v && tilt2 >= min_v && tilt2 <= max_v)
    {
        panTilt[0] = pan2;
        panTilt[1] = tilt2;
        return true;
    }
    else
    {
      //  printf("Error: can not find valid pan, tilt: %f %f %f %f\n", pan1, tilt1, pan2, tilt2);
        return false;
    }
}

class optimizePTZ_residual: public vnl_least_squares_function
{
protected:
    const vcl_vector<vgl_point_3d<double> > wldPts_;
    const vcl_vector<vgl_point_2d<double> > imgPts_;
    const vgl_point_2d<double> pp_;   // principle point
    const vgl_point_3d<double> cc_;   // camera center
    const vgl_rotation_3d<double> Rs_; // stationary rotation
    
public:
    optimizePTZ_residual(const vcl_vector<vgl_point_3d<double> > & wldPts,
                                 const vcl_vector<vgl_point_2d<double> > & imgPts,
                                 const vgl_point_2d<double> & pp, const vgl_point_3d<double> & cc, const vgl_rotation_3d<double> & Rs):
    vnl_least_squares_function(3, (unsigned int)wldPts.size() * 2, no_gradient),
    wldPts_(wldPts),
    imgPts_(imgPts),
    pp_(pp),
    cc_(cc),
    Rs_(Rs)
    {
        assert(wldPts.size() == imgPts.size());
        assert(wldPts.size() >= 1);
    }
    
    void f(const vnl_vector<double> &x, vnl_vector<double> &fx)
    {        
        double pan  = x[0];
        double tilt = x[1];
        double fl   = x[2];
        vnl_matrix_fixed<double, 3, 3> K;
        K(0, 0) = fl; K(0, 1) = 0;  K(0, 2) = pp_.x();
        K(1, 0) = 0 ; K(1, 1) = fl; K(1, 2) = pp_.y();
        K(2, 0) = 0 ; K(2, 1) = 0;  K(2, 2) = 1.0;
        
        vnl_matrix_fixed<double, 3, 3> R_pan_tilt = RotationfromPanTilt(pan, tilt);
        vnl_matrix_fixed<double, 3, 3> R = R_pan_tilt * Rs_.as_matrix();
        
        int idx = 0;
        for (int i = 0; i<wldPts_.size(); i++) {
            vgl_point_3d<double> p(wldPts_[i].x() - cc_.x(), wldPts_[i].y() - cc_.y() , wldPts_[i].z() - cc_.z()); // translation
            p = R * p;  // rotation
            p = K * p;  // projection
            double x = p.x()/p.z();
            double y = p.y()/p.z();
            fx[idx] = imgPts_[i].x() - x;
            idx++;
            fx[idx] = imgPts_[i].y() - y;
            idx++;
        }
        
    }
};


bool Vpgl2PPTZCalib::optimizePTZ(const vgl_point_3d<double> & cc, const vgl_rotation_3d<double> & Rs,
                                  const vgl_point_2d<double> & pp,
                                  const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                  const vcl_vector<vgl_point_2d <double> > & img_pts,
                                  const vnl_vector_fixed<double, 3> & init_ptz,
                                  vnl_vector_fixed<double, 3> & opt_ptz)
{
    assert(wld_pts.size() == img_pts.size());
    assert(wld_pts.size() >= 2);
    
    optimizePTZ_residual residual(wld_pts, img_pts, pp, cc, Rs);
    
    vnl_vector<double> x(3, 0.0);
    x[0] = init_ptz[0];
    x[1] = init_ptz[1];
    x[2] = init_ptz[2];
    
    vnl_levenberg_marquardt lmq(residual);
    bool isMinimizeOk = lmq.minimize(x);
    if (!isMinimizeOk) {
        printf(("Error: LMQ cao not converge.\n"));
        lmq.diagnose_outcome();
        return false;
    }
    
    opt_ptz[0] = x[0];
    opt_ptz[1] = x[1];
    opt_ptz[2] = x[2];
    return true;
}

/************         Vpgl2PPTZCalibTUM           *************/
bool Vpgl2PPTZCalibTUM::focal_length(const vgl_point_3d<double> & cc,
                                     const vgl_point_2d<double> & pp,
                                     const vcl_vector<vgl_point_3d<double> > & wld_pts,
                                     const vcl_vector<vgl_point_2d <double> > & img_pts,
                                     double & fl)
{
    assert(wld_pts.size() == 2);
    assert(img_pts.size() == 2);
    
    vnl_vector_fixed<double, 2> p1(img_pts[0].x(), img_pts[0].y());
    vnl_vector_fixed<double, 2> p2(img_pts[1].x(), img_pts[1].y());
    vnl_vector_fixed<double, 3> v1(wld_pts[0].x() - cc.x(), wld_pts[0].y() - cc.y(), wld_pts[0].z() - cc.z());
    vnl_vector_fixed<double, 3> v2(wld_pts[1].x() - cc.x(), wld_pts[1].y() - cc.y(), wld_pts[1].z() - cc.z());
    
    vnl_vector_fixed<double, 2> ppVec(pp.x(), pp.y());
    p1 = p1 - ppVec;
    p2 = p2 - ppVec;
    // normalize v1, v2;
    v1.normalize();
    v2.normalize();
    
    
    double a = dot_product(p1, p1);
    double b = dot_product(p2, p2);
    double c = dot_product(p1, p2);
    double d = dot_product(v1, v2);
    
    double delta = (d*d *(a+b) - 2.0*c)*(d*d *(a+b) - 2.0*c) - 4.0 *(d*d*a*b-c*c)*(d*d-1.0);
    if (delta < 0.0) {
        printf("Error: sqrt a negative number.\n");
        printf("delta is %f\n", delta);
        return false;
    }
    
    double numerator   = 2.0 * (d*d*a*b - c*c);
    double denominator = 2.0*c - d*d *(a+b) + sqrt(delta);
    if (denominator == 0.0) {
        printf("Error: denominator is zero.\n");
        return false;
    }
    double fl_2 = numerator/denominator;
    if (fl_2 <= 0.0) {        
        printf("Error: focal length is not a real number. f^2 is %f\n", fl_2);
        printf("numerator, denominator is %f %f\n", numerator, denominator);
        return false;
    }
    fl = sqrt(fl_2);
    return true;
}

bool Vpgl2PPTZCalibTUM::focal_length_from_calibrated_camera(const vpgl_perspective_camera<double> & camera,
                                                            const vcl_vector<vgl_point_2d<double> > & pts_1,
                                                            const vcl_vector<vgl_point_2d<double> > & pts_2,
                                                            double &fl)
{
    assert(pts_1.size() == 2);
    assert(pts_2.size() == 2);
    
    // cos theta of pts_1 points
    vgl_line_3d_2_points<double> line1 = camera.backproject(pts_1[0]); // all the first point is from camera center
    vgl_line_3d_2_points<double> line2 = camera.backproject(pts_1[1]);
    vgl_point_3d<double> cc = camera.get_camera_center();
    vgl_point_2d<double> pp = camera.get_calibration().principal_point();
    vcl_vector<vgl_point_3d<double> > wld_pts;
    wld_pts.push_back(line1.point2());
    wld_pts.push_back(line2.point2());
    
    // similar as TUM method
    return Vpgl2PPTZCalibTUM::focal_length(cc, pp, wld_pts, pts_2, fl);
}

                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                










