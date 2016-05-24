//
//  SoccerLineEdgeletModel.h
//  QuadCopter
//
//  Created by jimmy on 7/9/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__SoccerLineEdgeletModel__
#define __QuadCopter__SoccerLineEdgeletModel__

#include <vgl/vgl_line_segment_2d.h>
#include <vnl/vnl_vector.h>
#include <vcl_queue.h>
#include <vcl_list.h>
#include <vgl/vgl_line_2d.h>
#include <vil/vil_image_view.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vcl_vector.h>
#include "SoccerLineParameters.h"
#include "vil_gmm.h"

// soccer line mode
// edgelet --> line --> model
// the model is complex so that it is not useful.

class SMLine;
class SoccerLineEdgeletModel;


// soccer model edgelet
class SMEdgelet
{
    friend SMLine;
    friend SoccerLineEdgeletModel;
private:
    SMLine *pParentLine_;    // line in the model
    SoccerLineEdgeletModel * pParentModel_; // the soccer field model
    
    vgl_line_segment_2d<double> seg_wld_;
    vgl_line_segment_2d<double> seg_img_;
    double magnitude_;  // threshold
    double lambda_;     // threshold
    vnl_vector<double>  histogram_; // color histogram
    
    vcl_queue<SMEdgelet *> prev_edgelet_; // edgelet in previous frames
    double prob_;                       // edge confidence probability
    
};

// soccer model line
class SMLine
{
    friend SMEdgelet;
    friend SoccerLineEdgeletModel;
private:
    SoccerLineEdgeletModel * pParentModel_;
    int node_id_;
    SoccerLineType type_;        // border, or not
    vgl_line_segment_2d<double> seg_wld_;   // line segment in world coordiante, fixed
    vgl_line_segment_2d<double> init_seg_img_; // initial line segment by initial camera
    vgl_line_segment_2d<double> seg_img_;      // tracked line segment in image
    vgl_line_2d<double> line_img_;             // line in image
    
    vcl_list<SMLine *> childEdgelet_;
    double prob_;
    bool is_find_line_;   
    
public:
    SMLine();
    ~SMLine();
    
    bool track_line();
    bool track_border();
    
    bool intersectPoint(const SMLine & otherLine, vgl_point_2d<double> & wldPt, vgl_point_2d<double> &imgPt) const;
    
};

class SoccerLineEdgeletModel
{
    friend SMEdgelet;
    friend SMLine;
    
private:
    vil_image_view<vxl_byte> image_;
    vil_image_view<double> magnitude_;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    vil_image_view<vxl_byte> pixelTypes_;   // 0, grass, 1, commercial board
    
    static VilGMM grass_gmm_;
    static VilGMM cb_gmm_;
    
    vcl_vector<SMLine *> lines_;
    
    vpgl_perspective_camera<double> init_camera_;
    vpgl_perspective_camera<double> final_camera_;
    double prob_;   
   
    vcl_vector<vpgl_perspective_camera<double> > cadindate_cameras_;
    // previous models
    vcl_queue<SoccerLineEdgeletModel *> prev_models_;
    
public:
    SoccerLineEdgeletModel();
    ~SoccerLineEdgeletModel();
    
    void initByFirstCameraAndImage(const vpgl_perspective_camera<double> & camera, const vil_image_view<vxl_byte>& image);
    bool calibrate(const vil_image_view<vxl_byte> & image);
    void clearData(void);
    
    
    vpgl_perspective_camera<double> camera(){return final_camera_;}
    
private:
    bool getCameraFromKLT(vpgl_perspective_camera<double> & curCamera) const;
    // calibration from two point method
    bool getCameraFromTwoPointCalibration(const vcl_vector<vgl_point_2d<double> > & wldPts, const vcl_vector<vgl_point_2d<double> > & imgPts);
    
    
    
    
};


#endif /* defined(__QuadCopter__SoccerLineEdgeletModel__) */
