//
//  VpglPTZCameraUtil.cpp
//  OnlineStereo
//
//  Created by jimmy on 6/30/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "VpglPTZCameraUtil.h"
#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include "vgl_plus.h"

/***************************    VpglPTZCameraUtil       ************************************************************/
class VpglPTZCameraUtilRefineCameraByPtsAndVPsResidual:public vnl_least_squares_function
{
protected:
    const VpglPTZCameraUtil::Correspondence corres_;
    const vpgl_ptz_camera initCamera_;
    
public:
    
    VpglPTZCameraUtilRefineCameraByPtsAndVPsResidual(const VpglPTZCameraUtil::Correspondence & corres, int pts_num, const vpgl_ptz_camera & initCamera):
    vnl_least_squares_function(3, pts_num * 2, no_gradient),
    corres_(corres),
    initCamera_(initCamera)
    {
        
    }
    
    void f(vnl_vector<double> const &x, vnl_vector<double> &fx)
    {
        vpgl_ptz_camera curPTZ = initCamera_;
        curPTZ.setPTZ(x[0], x[1], x[2]);
        // loop over point feature
        int idx = 0;
        for (int i = 0; i<corres_.wld_pts.size(); i++) {
            vgl_point_3d<double> p(corres_.wld_pts[i].x(), corres_.wld_pts[i].y(), 0.0);
            vgl_point_2d<double> q = curPTZ.project(p);
            
            fx[idx] = corres_.img_pts[i].x() - q.x();
            idx++;
            fx[idx] = corres_.img_pts[i].y() - q.y();
            idx++;
        }
        
        // loop over all vanishing points
        for (int i = 0; i<corres_.lineSets.size(); i++) {
            // project lines into image, then get intersecton of these lines
            vcl_vector<vgl_line_2d<double> > projectedLines;
            for (int j = 0; j<corres_.lineSets[i].size(); j++) {
                vgl_line_2d<double> line = curPTZ.project(corres_.lineSets[i][j]);
                projectedLines.push_back(line);
            }
            assert(projectedLines.size() >= 2);
            
            // intersection of projected lines is the vanishing point in the image
            vgl_point_2d<double> q;
            bool isInterSect = VglPlus::intersectionFromLineSet(projectedLines, q);
            assert(isInterSect);
            
            fx[idx] = corres_.vanishingPoints[i].x() - q.x();
            idx++;
            fx[idx] = corres_.vanishingPoints[i].y() - q.y();
            idx++;
        }
    }
    
    void getCamera(vnl_vector<double> &x, vpgl_ptz_camera & camera)
    {
        camera = initCamera_;
        camera.setPTZ(x[0], x[1], x[2]);
    }
};


bool VpglPTZCameraUtil::refineCameraByPtsAndVPs(const vpgl_ptz_camera & initPTZ, const Correspondence & corres, vpgl_ptz_camera & finalCamera)
{
    assert(corres.img_pts.size() + corres.vanishingPoints.size() >= 2);
    
    
    int pts_num = (int)corres.img_pts.size() + (int)corres.vanishingPoints.size();
    VpglPTZCameraUtilRefineCameraByPtsAndVPsResidual residual(corres, pts_num, initPTZ);
    
    
    vnl_vector<double> x(3);
    x[0] = initPTZ.pan();
    x[1] = initPTZ.tilt();
    x[2] = initPTZ.focal_length();
    
    vcl_cout<<"initial ptz is "<<x<<vcl_endl;
    
    vnl_levenberg_marquardt lmq(residual);
    
    bool isMinized = lmq.minimize(x);
    if (!isMinized) {
        vcl_cerr<<"Error: minimization failed."<<vcl_endl;
        lmq.diagnose_outcome();
        return false;
    }
    lmq.diagnose_outcome();
    
    residual.getCamera(x, finalCamera);
    
    vcl_cout<<"final ptz is "<<x<<vcl_endl;
    
    return true;
}
