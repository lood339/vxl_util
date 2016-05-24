//
//  SoccerLineEdgeletModel.cpp
//  QuadCopter
//
//  Created by jimmy on 7/9/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "SoccerLineEdgeletModel.h"
#include "vil_plus.h"
#include "vil_klt.h"
#include "vxl_vrel_plus.h"
#include "wwosSoccerCourt.h"
#include "vpgl_plus.h"
#include <vnl/vnl_inverse.h>
#include "vgl_vnl_operator.h"
#include "SoccerModelLeftSideView.h"
#include "vil_line_tracking.h"
#include "vil_weak_line_tracking.h"
#include "vil_gmm_util.h"
#include "SoccerBorderLineTracking.h"
#include <vgl/vgl_closest_point.h>
#include <vgl/vgl_intersection.h>
#include "vpgl_2P_PTZCalibUtil.h"
#include <vgl/vgl_distance.h>
#include <vcl_queue.h>
#include "vxl_canny_edgelet.h"


// hard code
static char grass_gmm_file[] = "/Users/jimmy/Desktop/images/WWoS_soccer_video_2/gmm_data/camera2_grass_2.gmm";
static char cb_gmm_file[] = "/Users/jimmy/Desktop/images/WWoS_soccer_video_2/gmm_data/camera2_commercial_5.gmm";


/***************************************    SMLine    *************************************************/
SMLine::SMLine()
{
    is_find_line_ = false;    
}
SMLine::~SMLine()
{
    
}

bool SMLine::track_line()
{
    if(type_ == FarTouchBorder || type_ == RightGoalBorder)
    {
        return false;
    }
    SoccerModelLeftSideView lineModel;
    const int nodeId = this->node_id_;
    vil_image_view<vxl_byte> image = this->pParentModel_->image_;
    vil_image_view<double> magnitude = this->pParentModel_->magnitude_;
    vil_image_view<double> grad_i = this->pParentModel_->grad_i;
    vil_image_view<double> grad_j = this->pParentModel_->grad_j;
    const double onlinePixelRatioThreshold = 0.1;
    vpgl_perspective_camera<double> initCamera = this->pParentModel_->init_camera_;
    
    WeakLineTrackingParameter para;
    para.lambda_ = lineModel.nodeLambda(nodeId);
    para.mag_threshold_1_ = lineModel.nodeMagnitude(nodeId);
    para.line_width_      = lineModel.nodeScanWidth(initCamera, nodeId);
    para.test_node_ = node_id_;
    
    vgl_line_segment_2d<double> initLineSeg = this->init_seg_img_;
    vgl_line_segment_2d<double> refinedLineSeg;
    vgl_line_segment_2d<double> finalLineSeg;
    vgl_line_2d<double> finalLine;
    bool isFindFinal = false;
    
    // initial
    // 1, initial line segment close to line in the image?
    double onLinePixelRatio = VilLineTracking::onEdgePixelRatio(magnitude, grad_i, grad_j, initLineSeg, para.mag_threshold_1_, para.edge_neighbor_size_);
    if (onLinePixelRatio < onlinePixelRatioThreshold)
    {
        // 2, refine line by scan in one direction
        bool isRefined = VilWeakLineTracking::refineLineSegmentByMultipleParameter(image, magnitude, initLineSeg, para, refinedLineSeg);
        if (!isRefined)
        {
            refinedLineSeg = initLineSeg;
        }
    }
    else
    {
        refinedLineSeg = initLineSeg;
    }
    
    if (lineModel.hasDoubleEdge(initCamera, nodeId))
    {
        // 3, refine line using half edge scan method
        isFindFinal = VilWeakLineTracking::trackingLineFromSegmentByMultipleParameter(image, magnitude, grad_i, grad_j, refinedLineSeg, para, finalLine, finalLineSeg);
        if (!isFindFinal) {
            printf("node %d failed in find center line.\n", this->node_id_);
        }
    }
    else
    {
        finalLine = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
        finalLineSeg = refinedLineSeg;
        isFindFinal = true;
    }
    
    // check finel line
    if (isFindFinal) {
        // 4, check if it is a good line segment
        double onLinePixelRatio = VilLineTracking::onEdgePixelRatio(magnitude, grad_i, grad_j, finalLineSeg, para.mag_threshold_1_, 5);
        if (onLinePixelRatio > onlinePixelRatioThreshold)
        {
            vgl_vector_2d<double> dir1 = initLineSeg.direction();
            vgl_vector_2d<double> dir2 = finalLine.direction();
            double cosAngle = fabs(cos_angle(dir1, dir2));
            // 5, check line by angle with initial line segment
            if (cosAngle >= para.parallel_line_direction_threshold_) {
                this->line_img_ = finalLine;
                this->prob_ = onLinePixelRatio;
                this->seg_img_ = finalLineSeg;
                this->is_find_line_ = true;
                return true;
            }
            else
            {
                printf("node %d failed in find parallel checking.\n", this->node_id_);                
            }
        }
        else
        {
            printf("node %d failed in on_edge_pixel_ratio test.\n", this->node_id_);
        }
    }
    return false;
}

bool SMLine::track_border()
{
    if (type_ == Internal) {
        return false;
    }
    if (type_ == FarTouchBorder) {
   
        vgl_line_segment_2d<double> initLineSeg = this->init_seg_img_;
        vil_image_view<vxl_byte> image = this->pParentModel_->image_;
        vil_image_view<vxl_byte> pixelType = this->pParentModel_->pixelTypes_;
        FarCommerticalBoarderLineParameter para;
        vgl_line_2d<double> cbLine;
        bool isFind = SoccerBorderLineTracking::commercialBoardDetectionByColorClassifer(image, pixelType, init_seg_img_, para, cbLine);
        if (isFind) {
            is_find_line_ = true;
            line_img_ = cbLine;
            vgl_point_2d<double> p1 = vgl_closest_point(cbLine, init_seg_img_.point1());
            vgl_point_2d<double> p2 = vgl_closest_point(cbLine, init_seg_img_.point2());
            seg_img_ = vgl_line_segment_2d<double>(p1, p2);
            return true;
        }
    }
    if (type_ == RightGoalBorder) {
        vgl_line_segment_2d<double> initLineSeg = this->init_seg_img_;
        vil_image_view<vxl_byte> image = this->pParentModel_->image_;
        vil_image_view<vxl_byte> pixelType = this->pParentModel_->pixelTypes_;
        RightBorderLine para;
        vgl_line_2d<double> cbLine;
        bool isFind = SoccerBorderLineTracking::rightBorderDetectionByColorClassifer(image, pixelType, init_seg_img_, para, cbLine);
        if (isFind) {
            is_find_line_ = true;
            line_img_ = cbLine;
            vgl_point_2d<double> p1 = vgl_closest_point(cbLine, init_seg_img_.point1());
            vgl_point_2d<double> p2 = vgl_closest_point(cbLine, init_seg_img_.point2());
            seg_img_ = vgl_line_segment_2d<double>(p1, p2);
            return true;
        }        
    }
    return false;
}

bool SMLine::intersectPoint(const SMLine & otherLine, vgl_point_2d<double> & wldPt, vgl_point_2d<double> &imgPt) const
{
    if (this->is_find_line_ && otherLine.is_find_line_) {
        bool isIntersect1 = vgl_intersection(this->line_img_, otherLine.line_img_, imgPt);
        bool isIntersect2 = vgl_intersection(vgl_line_2d<double>(this->seg_wld_.point1(), this->seg_wld_.point2()),
                                             vgl_line_2d<double>(otherLine.seg_wld_.point1(), otherLine.seg_wld_.point2()),
                                             wldPt);
        return isIntersect1 && isIntersect2;
    }
    return false;
}


/***************************************    SoccerLineEdgeletModel    ************************************/
VilGMM SoccerLineEdgeletModel::grass_gmm_ = VilGMM(2);
VilGMM SoccerLineEdgeletModel::cb_gmm_    = VilGMM(5);

SoccerLineEdgeletModel::SoccerLineEdgeletModel()
{
    
}

SoccerLineEdgeletModel::~SoccerLineEdgeletModel()
{
    
}

void SoccerLineEdgeletModel::initByFirstCameraAndImage(const vpgl_perspective_camera<double> & camera, const vil_image_view<vxl_byte>& image)
{
    SoccerLineEdgeletModel * pModel = new SoccerLineEdgeletModel();
    pModel->final_camera_ = camera;
    pModel->image_ = image;
    VilPlus::vil_gradient(image, pModel->magnitude_, pModel->grad_i, pModel->grad_j);
    prev_models_.push(pModel);
    
    SoccerLineEdgeletModel::grass_gmm_.read(grass_gmm_file);
    SoccerLineEdgeletModel::cb_gmm_.read(cb_gmm_file);
}

bool SoccerLineEdgeletModel::calibrate(const vil_image_view<vxl_byte> & image)
{
    if (prev_models_.empty()) {
        printf("Error: model needs previous camera parameters.\n");
        return false;
    }
    image_.deep_copy(image);
    VilPlus::vil_gradient(image_, magnitude_, grad_i, grad_j);
    this->clearData();
    
    // classify pixels
    VilGMMUtil::binaryClassify(grass_gmm_, cb_gmm_, image_, 0, 1, pixelTypes_);
    
    //init_camera_ = prev_models_.back()->final_camera_;
    bool isInitFromKLT = this->getCameraFromKLT(init_camera_);
    if (!isInitFromKLT) {
        printf("Error: initial camera from KLT failed.\n");
        return false;
    }
    
    // get line intersection
    SoccerModelLeftSideView lineModel;
    const int nodeNum = lineModel.longLineSegmentNum();
    for (int node = 0; node<nodeNum; node++) {
        vgl_line_segment_2d<double> wld_seg;
        vgl_line_segment_2d<double> img_seg;
        if(lineModel.projectNode(init_camera_, node, wld_seg, img_seg))
        {
            SMLine *smLine = new SMLine();
            smLine->node_id_ = node;
            smLine->seg_wld_ = wld_seg;
            smLine->init_seg_img_ = img_seg;
            smLine->pParentModel_ = this;
            smLine->is_find_line_ = false;
            smLine->type_ = lineModel.nodetype(node);
            this->lines_.push_back(smLine);
        }
    }
    
    // line tracking, and border line tracking
    vil_image_view<vxl_byte> showImage;
    showImage.deep_copy(image_);
    for (int i = 0; i<this->lines_.size(); i++) {
        bool findLine = false;
        if (this->lines_[i]->node_id_ == 26) {
            int test = 1;
        }
        // internal line
        if (this->lines_[i]->type_ == Internal) {
            findLine = this->lines_[i]->track_line();
        }
        if (findLine) {
            VilPlus::draw_segment(showImage, this->lines_[i]->seg_img_, VilPlus::green());
        }
        
        findLine = this->lines_[i]->track_border();
        if (findLine)
        {
            VilPlus::draw_segment(showImage, this->lines_[i]->seg_img_, VilPlus::red(), 1.0);
        }
    }
    
    vcl_vector<vgl_point_2d<double> > wld_intersection;
    vcl_vector<vgl_point_2d<double> > img_intersection;
    // line intersection from lines
    for (int i = 0; i<this->lines_.size(); i++) {
        for (int j = 0; j<this->lines_.size(); j++) {
            int node1 = this->lines_[i]->node_id_;
            int node2 = this->lines_[j]->node_id_;
            if (lineModel.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = this->lines_[i]->intersectPoint(*(this->lines_[j]), wld_pt, img_pt);
                if (isIntersect) {
                    VilPlus::draw_cross(showImage, img_pt, 5, VilPlus::blue(), 2);
                    wld_intersection.push_back(wld_pt);
                    img_intersection.push_back(img_pt);
                }
            }
        }
    }
    
    VilPlus::vil_save(showImage, "lines_intersection.jpg");
    
    bool isCalib = this->getCameraFromTwoPointCalibration(wld_intersection, img_intersection);
    if (isCalib)
    {
        // save to as previous models
        SoccerLineEdgeletModel * pModel = new SoccerLineEdgeletModel();
        pModel->final_camera_ = this->final_camera_;
        pModel->image_ = this->image_;
        VilPlus::vil_gradient(pModel->image_, pModel->magnitude_, pModel->grad_i, pModel->grad_j);
        prev_models_.push(pModel);
        if (prev_models_.size() > 10) {
            prev_models_.pop();
        }
    }
    return isCalib;
}

void SoccerLineEdgeletModel::clearData(void)
{
    for (int i = 0; i<this->lines_.size(); i++)
    {
        if (lines_[i]) {
            delete lines_[i];
            lines_[i] = NULL;
        }
    }
    lines_.clear();
    this->cadindate_cameras_.clear();
}

bool SoccerLineEdgeletModel::getCameraFromKLT(vpgl_perspective_camera<double> & curCamera) const
{
    const int nFeature = 500;
    vpgl_perspective_camera<double> prevCamera = prev_models_.back()->final_camera_;
    
    WWoSSoccerCourt court;
    // get initial camera model from KLT
    vcl_vector<VilKLTFeature> feature1;
    vcl_vector<VilKLTFeature> feature2;
    vil_image_view<vxl_byte> prevImage = prev_models_.back()->image_;
    
    // random sampleing and tracking
    VilKLT::randomTracking(prevImage, image_, nFeature, feature1, feature2);
    vcl_vector<vgl_point_2d<double> > pts1;
    vcl_vector<vgl_point_2d<double> > pts2;
    for (int i = 0; i<feature2.size(); i++) {
        if (feature2[i].value_ == 0) {
            pts1.push_back(feature1[i].loc_);
            pts2.push_back(feature2[i].loc_);
        }
    }
    
    vcl_vector<bool> inlier;
    vgl_h_matrix_2d<double> curH;
    bool isHomography = VrelPlus::homography_RANSAC(pts1, pts2, inlier, curH, 1.0);
    assert(isHomography);
    
    vcl_vector<vgl_point_2d<double> > pts1_inlier;
    vcl_vector<vgl_point_2d<double> > pts2_inlier;
    for (int i =0; i<inlier.size(); i++) {
        if (inlier[i]) {
            pts1_inlier.push_back(pts1[i]);
            pts2_inlier.push_back(pts2[i]);
        }
    }
    
    bool isCalib = VpglPlus::calibrate_pure_rotate_camera(pts1_inlier, pts2_inlier, prevCamera, curCamera);
    return isCalib;
}

struct CameraEdgePixel
{
    int nEdgePixel_;
    vpgl_perspective_camera<double> camera_;
    
    bool operator < (const CameraEdgePixel & other) const
    {
        return nEdgePixel_ < other.nEdgePixel_;
    }
};

bool SoccerLineEdgeletModel::getCameraFromTwoPointCalibration(const vcl_vector<vgl_point_2d<double> > & wldPts, const vcl_vector<vgl_point_2d<double> > & imgPts)
{
    // testing code
    const double min_gradient_magnitude2 = 50.0;
    const int line_draw_width = 3;
    
    WWoSSoccerCourt court;
    
    // grass labels
    vil_image_view<vxl_byte> labelImage;
    VilGMMUtil::binaryClassify(grass_gmm_, cb_gmm_, image_, 0, 1, labelImage);
    vil_image_view<vxl_byte> edgeMask;
    CannyParameter cannyPara;
    cannyPara.min_gradient_magnitude2_ = min_gradient_magnitude2; //@hard code
    VxlCannyEdgelet::canny(image_, cannyPara, edgeMask);
    
    // edges in grassland area
    for (int j = 0; j<edgeMask.nj(); j++) {
        for (int i = 0; i<edgeMask.ni(); i++) {
            if (labelImage(i, j) == 1) {  // audience area
                edgeMask(i, j) = 0;
            }
        }
    }
    
    vcl_priority_queue<CameraEdgePixel> candidateCameras;
    
    assert(wldPts.size() == imgPts.size());
    for (int i = 0; i<wldPts.size(); i++) {
        for (int j = i + 1; j<wldPts.size(); j++) {
            // 2 point calibration
            vcl_vector<vgl_point_3d<double> > pts1;
            vcl_vector<vgl_point_2d<double> > pts2;
            pts1.push_back(vgl_point_3d<double>(wldPts[i].x(), wldPts[i].y(), 0.0));
            pts1.push_back(vgl_point_3d<double>(wldPts[j].x(), wldPts[j].y(), 0.0));
            pts2.push_back(imgPts[i]);
            pts2.push_back(imgPts[j]);
            
            double distance = vgl_distance(imgPts[i], imgPts[j]);
            
            vnl_vector_fixed<double, 3> ptz;
            vpgl_perspective_camera<double> camera;
            vpgl_ptz_camera ptzCamera;
            bool isCalib = Vpgl2PPTZCalibUtil::wwosSoccerLeftSideviewPTZ_calib(pts1, pts2, ptz, camera, ptzCamera);
            if (isCalib) {
                const int w = image_.ni();
                const int h = image_.nj();
                vil_image_view<vxl_byte> projectedLineMask(w, h, 3);
                projectedLineMask.fill(0);
                court.overlayLines(camera, projectedLineMask, VilPlus::white(), line_draw_width);
                
                // count the number of pixels in edge area and also in projected area
                int num = 0.0;
                for (int y = 0; y<h; y++) {
                    for (int x =0; x<w; x++) {
                        if (edgeMask(x, y) == 255 && projectedLineMask(x, y, 0) != 0) {
                            num++;
                        }
                    }
                }                
                CameraEdgePixel cp;
                cp.nEdgePixel_ = num;
                cp.camera_ = camera;
                candidateCameras.push(cp);
            }
        }
    }

    // three cameras with largest number of edge pixels
    int index = 0;
    while (!candidateCameras.empty()) {
        vpgl_perspective_camera<double> camera = candidateCameras.top().camera_;
        candidateCameras.pop();
        index++;
        this->cadindate_cameras_.push_back(camera);
        if (index >= 10) {
            break;
        }
        {
            vil_image_view<vxl_byte> showImage;
            showImage.deep_copy(image_);
            court.overlayLines(camera, showImage);
            char buf[1024] = {NULL};
            sprintf(buf, "result_2/candidate_camera_%d.jpg", index);
            VilPlus::vil_save(showImage, buf);
        }
    }
    if (this->cadindate_cameras_.empty()) {
        return false;
    }
    this->final_camera_ = this->cadindate_cameras_[0];
    return true;
}
