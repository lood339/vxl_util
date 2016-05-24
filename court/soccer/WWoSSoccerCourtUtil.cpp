//
//  WWoSSoccerCourtUtil.cpp
//  QuadCopter
//
//  Created by jimmy on 6/20/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "WWoSSoccerCourtUtil.h"
#include "vxl_ptz_camera.h"
#include "vil_plus.h"
#include "SoccerModelLeftSideView.h"
#include "vpgl_ptz_estimation.h"
#include <vgl/vgl_distance.h>

/******************       WWoSSoccerCourtUtil        ************************/
bool WWoSSoccerCourtUtil::mapToOverviewImage(const vpgl_perspective_camera<double> & camera, const vil_image_view<vxl_byte> & image,
                                             const vil_image_view<vxl_byte> &overviewTemplate, vil_image_view<vxl_byte> & composedImage)
{
    assert(image.nplanes() == 3);
    assert(overviewTemplate.nplanes() == 3);
    
    WWoSSoccerCourt court;
    vgl_h_matrix_2d<double> H;
    bool isH = court.getImage2TopviewImageHomography(camera, image.ni(), image.nj(), H);
    if (!isH) {
        return false;
    }
    
    composedImage.deep_copy(overviewTemplate);
    return VilPlus::homography_warp_fill(image, H, overviewTemplate, composedImage);
}

bool WWoSSoccerCourtUtil::cameraToPTZ(const vpgl_perspective_camera<double> & camera, double & pan, double & tilt, double & fl)
{
    //estimte fl, pan, tilt from camera
    
    //53.8854 -8.04871 15.1881 1.57187 0.146873 -0.129615
    /*
     vgl_point_3d<double> cc(53.8854, -8.04871, 15.1881); //camera center
     vnl_vector_fixed<double, 3> rod;    //
     rod[0] =  1.57187;
     rod[1] =  0.146873;
     rod[2] = -0.129615;
     */
    // 1.57061 -0.00440067 0.021745
    vgl_point_3d<double> cc(53.8528, -8.37071, 15.0785); //camera center
    vnl_vector_fixed<double, 3> rod;    //
    rod[0] =   1.57061;
    rod[1] =  -0.00440067;
    rod[2] =   0.021745;
    
    vnl_vector_fixed<double, 6> coeff;
    coeff.fill(0.0);
    
    const int width = 1280;
    const int height = 720;
    
    VxlPTZCamera::PtsPairs corres;
    vcl_vector<vgl_point_2d<double>> dumpPts;
    WWoSSoccerCourt wwosCourt;
    wwosCourt.projectCourtPoints(camera, width, height, corres.wld_pts, dumpPts, corres.img_pts, 10);
    
    if (corres.wld_pts.size() < 5) {
        printf("Error: point pairs less than 5\n");
        return false;
    }
    
    vnl_vector_fixed<double, 3> flPanTilt;
    vpgl_perspective_camera<double> estimatedCamera;
    
    bool isEstimated = VxlPTZCamera::estimateFlPanTiltByFixingModelPositionRotation(corres, camera, cc, rod, coeff, flPanTilt, estimatedCamera);
    if (!isEstimated) {
        return false;
    }
    
    fl  = flPanTilt[0];
    pan = flPanTilt[1];
    tilt   = flPanTilt[2];
    return true;
}

bool WWoSSoccerCourtUtil::cameraToPTZOldNotUsed(const vpgl_perspective_camera<double> & camera, double & pan, double & tilt, double & fl)
{
    //estimte fl, pan, tilt from camera
    
    //53.8854 -8.04871 15.1881 1.57187 0.146873 -0.129615
    vgl_point_3d<double> cc(53.8854, -8.04871, 15.1881); //camera center
    vnl_vector_fixed<double, 3> rod;    //
    rod[0] =  1.57187;
    rod[1] =  0.146873;
    rod[2] = -0.129615;
    
    const int width = 1280;
    const int height = 720;
    
    VxlPTZCamera::PtsPairs corres;
    vcl_vector<vgl_point_2d<double>> dumpPts;
    WWoSSoccerCourt wwosCourt;
    wwosCourt.projectCourtPoints(camera, width, height, corres.wld_pts, dumpPts, corres.img_pts, 10);
    
    if (corres.wld_pts.size() < 5) {
        printf("Error: point pairs less than 5\n");
        return false;
    }
    
    vnl_vector_fixed<double, 6> coeff;
    coeff.fill(0.0);
    
    vnl_vector_fixed<double, 3> flPanTilt;
    vpgl_perspective_camera<double> estimatedCamera;
    
    bool isEstimated = VxlPTZCamera::estimateFlPanTiltByFixingModelPositionRotation(corres, camera, cc, rod, coeff, flPanTilt, estimatedCamera);
    if (!isEstimated) {
        return false;
    }
    
    fl  = flPanTilt[0];
    pan = flPanTilt[1];
    tilt   = flPanTilt[2];
    return true;
}

bool WWoSSoccerCourtUtil::cameraToPTZ(const vpgl_perspective_camera<double> & camera, vpgl_ptz_camera & ptzCamera)
{
    double pan = 0.0;
    double tilt = 0.0;
    double fl = 0.0;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera, pan, tilt, fl);
    if (!isPTZ) {
        return false;
    }
    /*
     vgl_point_3d<double> cc(53.8854, -8.04871, 15.1881);   // camera center
     vnl_vector_fixed<double, 3> rod;                       // stationary rotation matrix
     rod[0] =  1.57187;
     rod[1] =  0.146873;
     rod[2] = -0.129615;
     */
    vgl_point_3d<double> cc(53.8528, -8.37071, 15.0785); //camera center
    vnl_vector_fixed<double, 3> rod;    //
    rod[0] =   1.57061;
    rod[1] =  -0.00440067;
    rod[2] =   0.021745;
    
    vgl_point_2d<double> pp(1280/2.0, 720/2.0);
    vnl_vector_fixed<double, 6> coeff;
    coeff.fill(0.0);
    
    ptzCamera = vpgl_ptz_camera(pp, cc, rod, coeff, pan, tilt, fl);
    return true;
}

bool WWoSSoccerCourtUtil::PTZToCamera(const double pan, const double tilt, const double fl, vpgl_perspective_camera<double> & camera)
{
    //estimte fl, pan, tilt from camera
    /*
     vgl_point_3d<double> cc(53.8854, -8.04871, 15.1881); //camera center
     vnl_vector_fixed<double, 3> rod;    //
     rod[0] =  1.57187;
     rod[1] =  0.146873;
     rod[2] = -0.129615;
     */
    vgl_point_3d<double> cc(53.8528, -8.37071, 15.0785); //camera center
    vnl_vector_fixed<double, 3> rod;    //
    rod[0] =   1.57061;
    rod[1] =  -0.00440067;
    rod[2] =   0.021745;
    
    vnl_vector_fixed<double, 6> coeff;
    coeff.fill(0);
    
    vgl_point_2d<double> pp(640, 360);  //principle point
    return VxlPTZCamera::composeCamera(fl, coeff, pp, pan, tilt, rod, cc, camera);
}

bool WWoSSoccerCourtUtil::PTZToCameraOldNotUsed(const double pan, const double tilt, const double fl, vpgl_perspective_camera<double> & camera)
{
    //estimte fl, pan, tilt from camera
    vgl_point_3d<double> cc(53.8854, -8.04871, 15.1881); //camera center
    vnl_vector_fixed<double, 3> rod;    //
    rod[0] =  1.57187;
    rod[1] =  0.146873;
    rod[2] = -0.129615;
    
    vnl_vector_fixed<double, 6> coeff;
    coeff.fill(0);
    
    vgl_point_2d<double> pp(640, 360);  //principle point
    return VxlPTZCamera::composeCamera(fl, coeff, pp, pan, tilt, rod, cc, camera);
}

bool WWoSSoccerCourtUtil::interpolateCameras(const vpgl_perspective_camera<double> & camera1, const int fn1,
                                             const vpgl_perspective_camera<double> & camera2, const int fn2,
                                             const vcl_vector<int> & fns, vcl_vector<vpgl_perspective_camera<double> > & cameras)
{
    assert(fn1 < fn2);
    
    //check each frame number
    for (int i = 0; i<fns.size(); i++) {
        int fn = fns[i];
        if (fn < fn1 || fn > fn2) {
            printf("fn = %d not in side of (%d %d)\n", fn, fn1, fn2);
            return false;
        }
    }
    
    double pan1, tilt1, fl1;
    double pan2, tilt2, fl2;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera1, pan1, tilt1, fl1);
    if (!isPTZ) {
        return false;
    }
    isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera2, pan2, tilt2, fl2);
    if (!isPTZ) {
        return false;
    }
    
    for (int i = 0; i<fns.size(); i++) {
        int fn = fns[i];
        double pan3, tilt3, fl3;
        pan3  = VxlPTZCamera::linearInterpolate(pan1, pan2, fn1, fn2, fn);
        tilt3 = VxlPTZCamera::linearInterpolate(tilt1, tilt2, fn1, fn2, fn);
        fl3   = VxlPTZCamera::linearInterpolate(fl1, fl2, fn1, fn2, fn);
        vpgl_perspective_camera<double> camera;
        bool isCamera = WWoSSoccerCourtUtil::PTZToCamera(pan3, tilt3, fl3, camera);
        if (!isCamera) {
            printf("Error: can not recovery camera from (fl, pan, tilt) %f %f %f\n", fl3, pan3, tilt3);
            return false;
        }
        cameras.push_back(camera);
    }
    assert(fns.size() == cameras.size());
    
    return true;
}


int WWoSSoccerCourtUtil::WWoS2014PTZToMalkinCamera(const int ptzFrame)
{
    const int p1 = 5396;
    const int p2 = 411396;
    const int h1 = 2251;
    const int h2 = 171584;
    
    return 1.0 * (ptzFrame - p1)/(p2 - p1)*(h2 - h1) + h1;
}












