//
//  WWoSSoccerCourtSideViewUtil.cpp
//  QuadCopter
//
//  Created by jimmy on 8/4/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "WWoSSoccerCourtSideViewUtil.h"
#include "wwosSoccerCourt.h"
#include "SoccerModelLeftSideView.h"
#include "vpgl_ptz_estimation.h"
#include <vgl/vgl_distance.h>

/****************         WWoSSoccerCourtSideViewUtil         *******************/
bool WWoSSoccerCourtSideViewUtil::cameraToPTZ(const vpgl_perspective_camera<double> & camera, vnl_vector_fixed<double, 3> & ptz)
{
    WWoSSoccerCourt soccerCourt;
    SoccerModelLeftSideView sideViewModel;
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    const double segment_image_length = 100.0;
    
    vgl_point_3d<double> cc(-15.213795, 14.944021, 5.002864);
    vnl_vector_fixed<double, 3> rod(1.220866, -1.226907, 1.201566);
    vgl_rotation_3d<double> SR(rod);
    
    LinePointsInCameraview oneView;
    oneView.camera_ = camera;
    
    // get point correspondences
    soccerCourt.projectCourtPoints(camera, w, h, oneView.wld_pts_, oneView.img_pts_, 20);
    
    // collect points locate on the line
    vcl_vector<PointsOnLine> lines;
    for (int nodeId = 0; nodeId<21; nodeId++) {
        vgl_line_segment_2d<double> wld_seg;
        vgl_line_segment_2d<double> img_seg;
        bool isProj = sideViewModel.projectNode(camera, nodeId,  wld_seg, img_seg);
        if (isProj) {
            //
            PointsOnLine pol;
            vgl_point_3d<double> p1(wld_seg.point1().x(), wld_seg.point1().y(), 0.0);
            vgl_point_3d<double> p2(wld_seg.point2().x(), wld_seg.point2().y(), 0.0);
            pol.line_ = vgl_line_3d_2_points<double>(p1, p2);
            
            vgl_point_2d<double> q1 = img_seg.point1();
            vgl_point_2d<double> q2 = img_seg.point2();
            double distance = vgl_distance(q1, q2);
            int num = distance/segment_image_length;  // the number is decided by its length in image space
            
            for (int j = 0; j<num; j++) {
                double t1 = 1.0 * j / num;
                double t2 = 1.0 * (j+1)/num;
                vgl_point_2d<double> p3 = img_seg.point_t(t1);
                vgl_point_2d<double> p4 = img_seg.point_t(t2);
                pol.pts_.push_back(centre(p3, p4));
            }
            if (!pol.pts_.empty()) {
                lines.push_back(pol);
            }
        }
    }
    oneView.lines_ = lines;
    
    vpgl_perspective_camera<double> estiamtedCamera;
    bool isPTZ = VpglPTZEstimation::camera2PTZByPointsOnLines(oneView, cc, SR, estiamtedCamera, ptz);
    return isPTZ;
}

bool WWoSSoccerCourtSideViewUtil::PTZToCamera(const vnl_vector_fixed<double, 3> & ptz, vpgl_perspective_camera<double> & camera)
{
    vgl_point_3d<double>        cc(-15.213795, 14.944021, 5.002864);
    vnl_vector_fixed<double, 3> rod(1.220866, -1.226907, 1.201566);
    vnl_vector_fixed<double, 6> coeff;
    coeff.fill(0.0);
    vgl_point_2d<double> pp(1280/2.0, 720/2.0);
    
    double pan  = ptz[0];
    double tilt = ptz[1];
    double fl   = ptz[2];
    vpgl_ptz_camera ptzCamera;
    bool isPTZ = ptzCamera.setPTZ(pp, cc, rod, coeff, pan, tilt, fl);
    if (isPTZ) {
        camera = ptzCamera;
    }
    return isPTZ;
}
