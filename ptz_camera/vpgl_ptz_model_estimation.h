//
//  vpgl_ptz_model_estimation.h
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VpglPtzOpt__vpgl_ptz_model_estimation__
#define __VpglPtzOpt__vpgl_ptz_model_estimation__

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>
#include <vpgl/vpgl_perspective_camera.h>

class VpglPTZModelEstimation
{
public:
    struct Correspondence
    {
        vcl_vector<vgl_point_2d<double> > wld_pts;
        vcl_vector<vgl_point_2d<double> > img_pts;
    };
    
    // estimate common camera center with varying focal length
    // focal length is fixed before and after estimation, but the extrinsic parameters (R) changes
    // each camera should have more than 4 correspondence
    static bool estimateCommonCameraCenter(const vcl_vector<Correspondence> &corres, const vcl_vector<vpgl_perspective_camera<double> > &initCameras,
                                           vgl_point_3d<double> & cameraCenter, vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras);
    // estimate projection center
    // camera center is fixed
    static bool estimateProjectionCenters(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                          const vgl_point_3d<double> & cameraCenter,
                                          vcl_vector< vgl_point_3d<double> > & projectionCenters, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    //estiamte projection centers
    //fix camera center, pan, tilt angles, and focal length
    static bool estimateProjectionCentersFixPanTilt(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                    const vgl_point_3d<double> & cameraCenter, const vcl_vector<double> & panAngles, const vcl_vector<double> & tiltAngles,
                                                    vcl_vector< vgl_point_3d<double> > & projectionCenters, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    //estiamte projection centers
    //fix camera center, pan, tilt angles, and focal length
    static bool estimateProjectionCentersFixFlPanTilt(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                      const vgl_point_3d<double> & cameraCenter,
                                                      const vcl_vector<vnl_vector_fixed<double, 3> > & fl_pan_tilt_s,
                                                      vcl_vector< vgl_point_3d<double> > & projectionCenters, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    // P = K Q S D
    // C = [0  0 0]
    // pin hole camera model
    static bool estimateCommomRotationCenterAndStationaryRotation(const vcl_vector<Correspondence> & corres,
                                                                  const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                  vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras);
    
    // fix the first pan angle as zero
    // P = K Q S D
    // C = [0  0 0]
    // pin hole camera model
    static bool estimateCommomRotationCenterAndStationaryRotationFixFirstPan(const vcl_vector<Correspondence> & corres,
                                                                             const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                             vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras);
    
    // P = K*C*Q*S*D
    // C = [c1 + c3 * f, c2 + c4 * f, c3 + c5 * f]
    static bool estimateProjecionCenterWithFocalLength(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                       vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    static bool checkProjectionCenterCoefficientWihFolcalLength(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                const vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
                                                               
    
    // P = K*C*Q*S*D
    // C = [c1 + c3 * f, c2 + c4 * f, c3 + c5 * f]
    // focal lenth and tilt angle is fixed during estimation
    static bool estimateProjectionCenterWithFixedFocalLengthTilt(const vcl_vector<Correspondence> & corres,
                                                                const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    // P = K*Q*S*D
    // pin-hole model
    // focal lenth and tilt angle is fixed during estimation
    static bool pinHoleModelEstimateProjectionCenterWithFixedFocalLengthTilt(const vcl_vector<Correspondence> & corres,
                                                                             const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                             vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    
    // P = K*C*Q*S*D
    // C = [0, c1, c2 + c3 * f]
    static bool estimateProjecionCenterWithFocalLengthCz(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                         vnl_vector_fixed<double, 3> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    // P = K*C*Q*S*D
    // C = [cx, cy, cz]
    static bool estimateProjectionCenterIndividually(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                     const vgl_point_3d<double> & cameraCenter,
                                                     vcl_vector< vgl_point_3d<double> > & projectionCenters,
                                                     vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    // P = K*C*Q*S*D
    // C = [c1 + c3 * f, c2 + c4 * f, c3 + c5 * f]
    static bool estimateProjecionCenterWithFocalLengthFixFL(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                            const vcl_vector<double> & focalLengths, const vgl_point_3d<double> & cameraCenter,
                                                            vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    // P = K*C*Q*S*D
    // C = [c1 + c3 * f, c2 + c4 * f, c3 + c5 * f]
    //  6 + 3 * x
    static bool estimateProjectionCenterWithCameracenterRotationFix(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                    const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                    vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    // P = K*C*Q*S*D
    // C = [c1, c2, c3 + c4 * f]
    // 4 + 3 * x
    static bool estimateProjectionCenterWithCameracenterRotationFixRestrictedModel(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                                   const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                                   vnl_vector_fixed<double, 4> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    
    
    
    // P = K*C*Q*S*D
    // C = [c1 + c3 * f, c2 + c4 * f, c3 + c5 * f]
    //  6 + 2 * x
    static bool estimateProjectionCenterWithCameracenterRotationFocalLengthFix(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                               const vcl_vector<double> & focalLengths,
                                                                               const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                               vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    // P = K*C*Q*S*D
    // C = [c1 + c3 * f, c2 + c4 * f, c3 + c5 * f]
    // fixing all other parameters
    static bool estimateProjectionCenterFixAllOthers(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                     const vcl_vector<vnl_vector_fixed<double, 3> >  & fl_pan_tilts,
                                                     const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                     vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
    // P = K R T, T is varing, K is fixed as in focalLengths
    static bool estimateRotationCenterFixFL(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                            const vcl_vector<double> & focalLengths,  vcl_vector<vpgl_perspective_camera<double> > & estimatedCameras);
    
    // P = K*C*Q*S*D
    // stds: standard deviration of projeciton error in (x, y) and continues focal lenth
    /*
    static bool estimateProjectionCenterByMaximumProbability(const vcl_vector<Correspondence> & corres, const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                             const vnl_vector_fixed<double, 3> & std_dev_s, const vgl_point_3d<double> & cameraCenter,
                                                             vnl_vector_fixed<double, 6> & coeff, vcl_vector<vpgl_proj_camera<double> > & estimatedCameras,
                                                             const vcl_string & saveMatlabFile = vcl_string("fl_pan_tilt.mat"));
     */
    
    // P = K*C*Q*S*D by given C,S, DFd
    // estimate Q (pan, tilt) angles
    // cameraCenter: estimated common camera rotation center from multiple frames
    // rod: rodrigues angle of model rotation
    // coeff: c123456, [0,0,0,0,0,0] for projection center just as rotation center
    // flPanTilt: focal length, pan, tilt.
    static bool estimateFlPanTiltByFixingModelPositionRotation(const Correspondence & corre, const vpgl_perspective_camera<double> & initCamera,
                                                                const vgl_point_3d<double> & cameraCenter, const vnl_vector_fixed<double, 3> & rod,
                                                                const vnl_vector_fixed<double, 6> & coeff, vnl_vector_fixed<double, 3> & flPanTilt,
                                                                vpgl_proj_camera<double> & estimatedCamera);
    
    // thresholds: maximum difference between fitted curve and estimated camera in
    // thresholds[0] focal length, thresholds[1] pan, thresholds[2] tilt
    // static bool pickupPoorEstimationInImageSequence(const vcl_vector< vpgl_perspective_camera<double> > & initCameras, const vgl_point_3d<double> & cameraCenter,
    //                                                const vnl_vector_fixed<double, 3> & rod,
    //                                                const vnl_vector_fixed<double, 6> & coeff, const vnl_vector_fixed<double, 3> & thresholds,
    //                                                vcl_vector<int> & poorEstimationIndex,
    //                                                vcl_vector<vpgl_perspective_camera<double>> & interpolatedCameras);
    
    
    // for CMU data with ground truth of pan, tilt angle
    // KCRD model, fixed focal lenth,
    static bool estimateRotationCenterStationaryRotationByPanTilt(const vcl_vector<Correspondence> & corres,
                                                                  const vcl_vector<vpgl_perspective_camera<double> > & initCameras,
                                                                  const vcl_vector<double> & pans,
                                                                  const vcl_vector<double> & tilts,
                                                                  vcl_vector<vpgl_proj_camera<double> > & estimatedCameras);
    
};



#endif /* defined(__VpglPtzOpt__vpgl_ptz_model_estimation__) */
