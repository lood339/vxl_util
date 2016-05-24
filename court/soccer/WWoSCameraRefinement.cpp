//
//  WWoSCameraRefinement.cpp
//  OnlineStereo
//
//  Created by jimmy on 2/19/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "WWoSCameraRefinement.h"
#include "wwosSoccerCourt.h"
//#include "vil_gmm.h"
#include "vil_plus.h"
#include "SoccerGraphCut.h"
#include <vcl_algorithm.h>
#include <vcl_map.h>
#include <vgl/vgl_distance.h>
#include <vgl/vgl_intersection.h>

#include "vxl_plus.h"
#include "vxl_vrel_plus.h"
#include "vpgl_plus.h"
#include <vil/vil_crop.h>
#include "SoccerShortLineModel.h"
#include "vil_line_tracking.h"
#include "vil_ellipse_tracking.h"
#include "vgl_plus.h"
#include "WWoSSoccerCourtUtil.h"
#include "VpglPTZCameraUtil.h"
#include "vpgl_ptz_estimation.h"
#include "cvx_LSD.h"
#include "vxl_ELSD.h"
#include "vxlOpenCV.h"
#include "vxl_half_edge_line_tracking.h"

/*
class WhiteLinePair
{
    public:
    int node_;            // world line index
    int image_line_idx_;  // image line from Hough transform
    double distance_;     // min distance from projected line end point to image line
public:
    WhiteLinePair(int n, int img_line_idx, double dis)
    {
        node_ = n;
        image_line_idx_ = img_line_idx;
        distance_ = dis;
    }
    
    bool operator <(const WhiteLinePair & other) const
    {
        return distance_ < other.distance_;
    }
    
};
 */

/*
bool WWoSCameraRefinement::refineCameraByWhiteLines(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & initCamera,
                                                    const WhiteLineRefinementPara &para, vpgl_perspective_camera<double> & finalCamera)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    // detect wite pixel
    WWoSSoccerCourt wwosCourt;
    SoccerGraphCutUtil graphcutUtil;
    vcl_vector<vgl_point_2d<double>> whitePixels;
    VilGMMUtil::whiteLinePixelDetection(para.green_gmm_, para.white_gmm_, image, para.white_pixel_para_, whitePixels);
    
    vcl_vector<vgl_point_2d<double>> refinedWhitePixels;
    graphcutUtil.removeWhitePixelAwayFromLines(initCamera, whitePixels, para.min_distance_to_project_line_, refinedWhitePixels);
    
    vil_image_view<vxl_byte> edgeMask(w, h, 1);
    edgeMask.fill(0);
    for (int i = 0; i<refinedWhitePixels.size(); i++)
    {
        int x = refinedWhitePixels[i].x();
        int y = refinedWhitePixels[i].y();
        edgeMask(x, y) = 255;
    }
    {
        VilPlus::vil_save(edgeMask, "edgemask.jpg");
    }
    
    // detect white lines in image    
    vcl_vector<vgl_line_2d<double> > whiteLines;
    VxlHoughLine::oneByOneLineDetection(edgeMask, para.hough_line_para_, whiteLines);
    if (whiteLines.size() < 3 ) {
        printf("Warning 1: not enough white lines were detected.\n");
        return false;
    }
    printf("detected %lu white lines in image.\n", whiteLines.size());
    
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_color_lines(showImage, whiteLines, 2);
        VilPlus::vil_save(showImage, "detected_lines.jpg");
    }
    
    
    // project lines from initial camera
    vcl_vector<SGCCalibLine> calibLines;
    graphcutUtil.sampleCalibLines(initCamera, w, h, para.sample_length_, calibLines);
    
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        for (int i = 0; i<calibLines.size(); i++) {
           // VilPlus::draw_line(showImage, calibLines[i].projectedLine(), VilPlus::green());
            VilPlus::draw_edge(showImage, calibLines[i].point1(), calibLines[i].point2(), VilPlus::green());
        }
        VilPlus::vil_save(showImage, "projected_lines.jpg");
    }
    vcl_vector<WhiteLinePair> linePairs;
    // find initial line correspondence
    for (int i = 0; i<whiteLines.size(); i++) {
        double minDistance = INT_MAX;
        int index = -1;
        vgl_vector_2d<double> dir1 = whiteLines[i].direction();
        for (int j = 0; j<calibLines.size(); j++) {
            vgl_vector_2d<double> dir2 = calibLines[j].projectedLine().direction();
            double cosAngle = fabs(cos_angle(dir1, dir2));
            if (cosAngle > para.line_direction_threshold_) {
                // measure distance from the end point (projected lines) to detected line
                double dis = vgl_distance(whiteLines[i], calibLines[j].point1()) + vgl_distance(whiteLines[i], calibLines[j].point2());
                if (dis < minDistance) {
                    minDistance = dis;
                    index = j;
                }
            }
        }
        if (index != -1) {
            WhiteLinePair linePair(calibLines[index].node(), i, minDistance);
            linePairs.push_back(linePair);            
        }
    }
    
    // sort by distance
    vcl_sort(linePairs.begin(), linePairs.end());
    
    // one world line can only map to one image line
    vcl_vector<vcl_pair<int, int> > nodeMatches;
    for (int i = 0; i<linePairs.size() && linePairs[i].distance_ < para.max_line_pair_distance_; i++) {
        int node = linePairs[i].node_;
        bool isFound = false;
        for (int j = 0; j<nodeMatches.size(); j++) {
            if (nodeMatches[j].first == node || nodeMatches[j].second == linePairs[i].image_line_idx_) {
                isFound = true;
                break;
            }
        }
        if (!isFound)
        {
            nodeMatches.push_back(vcl_pair<int, int>(node, linePairs[i].image_line_idx_));
        }
    }
    
    if (nodeMatches.size() < 3) {
        printf("Warning 2: line correspondece number is %lu, too small.\n", nodeMatches.size());
        return false;
    }
    
    // refine camera by intersection
    vcl_vector<vgl_point_2d<double> > wld_intersection;
    vcl_vector<vgl_point_2d<double> > img_intersection;
    for (int i = 0; i<nodeMatches.size(); i++) {
        for (int j = i+1; j<nodeMatches.size(); j++) {
            if (graphcutUtil.isValidNodePair(nodeMatches[i].first, nodeMatches[j].first)) {
                vgl_point_2d<double> p = graphcutUtil.intersectionInWorld(nodeMatches[i].first, nodeMatches[j].first);
                vgl_point_2d<double> q;
                bool isIntersect = vgl_intersection(whiteLines[nodeMatches[i].second], whiteLines[nodeMatches[j].second], q);
                if (isIntersect) {
                    wld_intersection.push_back(p);
                    img_intersection.push_back(q);
                }
            }
        }
    }
    assert(wld_intersection.size() == img_intersection.size());
    
    if (wld_intersection.size() < 2) {
        printf("Warning 3: line intersection number is %lu, too small.\n", wld_intersection.size());
        return false;
    }    
    
    // minimize match error
    vpgl_ptz_camera initPTZ;
    vpgl_ptz_camera finalPTZ;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(initCamera, initPTZ);
    if (!isPTZ) {
        printf("Warning 4: initial camear failed to PTZ\n");
        return false;
    }
    
    VpglPTZCameraUtil::Correspondence corres;
    corres.wld_pts = wld_intersection;
    corres.img_pts = img_intersection;
    
    bool isRefind = VpglPTZCameraUtil::refineCameraByPtsAndVPs(initPTZ, corres, finalPTZ);
    if (!isRefind) {
        printf("Warning 5: failed to refine the PTZ camera\n");
        return false;
    }
    
    WWoSSoccerCourtUtil::PTZToCamera(finalPTZ.pan(), finalPTZ.tilt(), finalPTZ.focal_length(), finalCamera);
    return true;
}
 */

/*
bool WWoSCameraRefinement::refinePenaltyLineIntersection(const vil_image_view<vxl_byte> & image,
                                                         const vpgl_perspective_camera<double> & initCamera,
                                                         const PatchMatchParameter & para,
                                                         vgl_point_2d<double> & p1,
                                                         vgl_point_2d<double> & p2, bool isLeft)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    WWoSSoccerCourt wwosCourt;
    SoccerGraphCutUtil sgcUtil;
    
    vil_image_view<vxl_byte> courtImage;
    vil_image_view<vxl_byte> warpedImage;
    wwosCourt.courtRGBImage(courtImage);
    wwosCourt.projectTopviewImage(courtImage, initCamera, w, h, warpedImage, 20);
    
    vcl_vector<vgl_point_2d<double> > pts_3d = sgcUtil.penaltyArcIntersectionWorld();
    assert(pts_3d.size() == 4);
    vgl_point_2d<double> p3_model;
    vgl_point_2d<double> p4_model;
    if (isLeft) {
        p3_model = pts_3d[0];
        p4_model = pts_3d[1];
    }
    else
    {
        p3_model = pts_3d[2];
        p4_model = pts_3d[3];
    }
    
    // project from model to image space
    vgl_point_2d<double> p3 = initCamera.project(vgl_point_3d<double>(p3_model.x(), p3_model.y(), 0.0));
    vgl_point_2d<double> p4 = initCamera.project(vgl_point_3d<double>(p4_model.x(), p4_model.y(), 0.0));
    
    // two possible pairs
    double d1 = vgl_distance(p1, p3);
    double d2 = vgl_distance(p2, p4);
    double d3 = vgl_distance(p1, p4);
    double d4 = vgl_distance(p2, p3);
    if (d3 + d4 < d1 + d2) {
        vcl_swap(p3, p4);
    }
    if (!vgl_inside_image(p1, w, h, -para.patchSize_/2-1) ||
        !vgl_inside_image(p2, w, h, -para.patchSize_/2-1) ||
        !vgl_inside_image(p3, w, h, -para.patchSize_/2-1) ||
        !vgl_inside_image(p4, w, h, -para.patchSize_/2-1)) {
        printf("Warning: patch out of image, refinement stopped.\n");
        return false;
    }
    
    // pair p1 and p3
    vgl_point_2d<double> p1Refined;
    {
        int cx = p3.x();
        int cy = p3.y();
        int patternSize = para.patchSize_;
        vil_image_view<vxl_byte> pattern = vil_crop(warpedImage, cx-patternSize/2, patternSize, cy-patternSize/2, patternSize);
        
        int cx2 = p1.x();
        int cy2 = p1.y();
        vil_image_view<vxl_byte> patch = vil_crop(image, cx2-patternSize/2, patternSize, cy2-patternSize/2, patternSize);
        
        // block vcl_cout
        std::streambuf* cout_sbuf = std::cout.rdbuf(); // save original sbuf
        std::ofstream   fout("/dev/null");
        std::cout.rdbuf(fout.rdbuf()); // redirect 'cout' to a 'fout'
        
        VilPointlessCalibWarp warp;
        vnl_vector_fixed<double, 2> initT;
        initT.fill(0);
        vnl_vector_fixed<double, 2> finalT;
        warp.warp2DPatternToImage(pattern, patch, para.iternationNum_, para.gradientRadius_, initT, finalT);
        p1Refined.set(p1.x() - finalT[0], p1.y() - finalT[1]);
        
        std::cout.rdbuf(cout_sbuf);
    }
    
    // pair p2 and p4
    vgl_point_2d<double> p2Refined;
    {
        int cx = p4.x();
        int cy = p4.y();
        int patternSize = para.patchSize_;
        vil_image_view<vxl_byte> pattern = vil_crop(warpedImage, cx-patternSize/2, patternSize, cy-patternSize/2, patternSize);
        
        int cx2 = p2.x();
        int cy2 = p2.y();
        vil_image_view<vxl_byte> patch = vil_crop(image, cx2-patternSize/2, patternSize, cy2-patternSize/2, patternSize);
        
        // block vcl_cout
        std::streambuf* cout_sbuf = std::cout.rdbuf(); // save original sbuf
        std::ofstream   fout("/dev/null");
        std::cout.rdbuf(fout.rdbuf()); // redirect 'cout' to a 'fout'
        
        VilPointlessCalibWarp warp;
        vnl_vector_fixed<double, 2> initT;
        initT.fill(0);
        vnl_vector_fixed<double, 2> finalT;
        warp.warp2DPatternToImage(pattern, patch, para.iternationNum_, para.gradientRadius_, initT, finalT);
        p2Refined.set(p2.x() - finalT[0], p2.y() - finalT[1]);
        
        std::cout.rdbuf(cout_sbuf);
    }
    double d5 = vgl_distance(p1Refined, p2Refined);
    double d6 = vgl_distance(p3, p4);
    printf("distance difference is %f\n", fabs(d5 - d6));
    
    if (fabs(d5 - d6) > para.distance_difference_threshold_) {
        return false;
    }
    p1 = p1Refined;
    p2 = p2Refined;
    return true;
}
 */

bool WWoSCameraRefinement::refineCameraByEllipseTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                                         const LineTrackingParameter & trackingPara, vpgl_perspective_camera<double> & finalCamera)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 15;
    const int lineWidth = trackingPara.line_thick_ness_;
    const int lineMinPixelNum = 20;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    SoccerGraphCutUtil sgcUtil;
    
    // detect center line
    SGCCalibLine calibLine;
    bool isSampled = sgcUtil.sampleCalibLine(camera, w, h, 0, 0.5, calibLine);  // sample every 0.5 meter
    if (!isSampled) {
        printf("Warning: can not sample center line.\n");
        return false;
    }
    
    
    vil_image_view<double> magnitude;
    VilPlus::vil_magnitude(image, magnitude);
    
    double magThreshold[calibLineNum] = { 0.05, 0.04, 0.04, 0.04, 0.04,
        0.02, 0.02, 0.02, 0.02, 0.02,
        0.02, 0.02, 0.02, 0.02, 0.02};
    double lambdaTHreshold[calibLineNum] = {2, 2, 2, 2, 2,
        1.5, 1.5, 1.5, 1.5, 1.5,
        1.5, 1.5, 1.5, 1.5, 1.5};
    
    
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    // draw line segment in the image
    maskImage.fill(0);
    VilPlus::draw_segment(maskImage, calibLine.point1(), calibLine.point2(), VilPlus::white(), lineWidth);
    
    AverageMagnitudeParameter para;
    para.direction_ = calibLine.scan_direction();
    para.mag_threshold_ = magThreshold[calibLine.node()];
    para.lambda_ = lambdaTHreshold[calibLine.node()];
    
    // detect edge point candidate
    vcl_vector<vgl_point_2d<double> > side_edge_pts1;
    vcl_vector<vgl_point_2d<double> > side_edge_pts2;
    vxl_half_edge_line_tracking::detectLinePixelByAverageGradient(magnitude, maskImage, para, side_edge_pts1, side_edge_pts2);
    
    // hough line detection for two edge line
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    houghPara.minPixeNum_ = lineMinPixelNum;
    vgl_line_2d<double> dumpLine;
    vcl_vector<vgl_point_2d<double> > inlier_pts1;
    vcl_vector<vgl_point_2d<double> > inlier_pts2;
    bool isOk1 = VxlHoughLine::detectOneLine(side_edge_pts1, w, h, houghPara, dumpLine, inlier_pts1);
    bool isOk2 = VxlHoughLine::detectOneLine(side_edge_pts2, w, h, houghPara, dumpLine, inlier_pts2);
    if (!isOk1 || !isOk2) {
        printf("failed to detect side lines\n");
        return false;
    }
    
    // estimate center line from edge line
    vgl_line_2d<double> centerLine;
    vcl_vector<vgl_point_2d<double> > center_pts;
    bool isFindCenterLine = false;
    if (inlier_pts1.size() > inlier_pts2.size()) {
        isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts1, inlier_pts2, center_pts, centerLine);
    }
    else
    {
        isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts2, inlier_pts1, center_pts, centerLine);
    }
    if (!isFindCenterLine)
    {
        printf("Warning: can not find center line .\n");
        return false;
    }
    
    // check the directon different
    vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
    vgl_vector_2d<double> dir2 = centerLine.direction();
    double cosAngle = fabs(cos_angle(dir1, dir2));
    
    if (cosAngle > lineDirectionThreshold) {
        calibLine.setDetectedLine(centerLine);
    }
    else
    {
        printf("Warning: side line does not close to parallel\n");
        return false;
    }
    
    // check ellipse inside image
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    bool isSoccerInside = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, trackingPara.circle_ratio_);
    if (isSoccerInside) {
        // detect ellipse
        vil_image_view<vxl_byte> nonEllipseMaskImage = vil_image_view<vxl_byte>(w, h, 3);
        nonEllipseMaskImage.fill(0);
        maskImage.fill(0);
        
        AverageMagnitudeParameter para;
        para.mag_threshold_ = 0.05;
        para.lambda_    = 2.0;
        para.direction_ = 1;
        
        // get bounding box
        WWoSSoccerCourt wwosCourt;
        wwosCourt.overlayEllipseBoundingBox(camera, maskImage, VilPlus::white(), 8, 8);
        
        vcl_vector<vgl_point_2d<double> > outsideUpPixels;
        vcl_vector<vgl_point_2d<double> > outsideDownPixels;
        vxl_half_edge_line_tracking::detectEllipsePixelByAverageGradient(magnitude, maskImage, nonEllipseMaskImage, para, outsideUpPixels, outsideDownPixels);
        
        double threshold  = 2.0;
        double fail_ratio = 0.001;
        vgl_ellipse_2d<double> ellipse;
        vcl_vector<vgl_point_2d<double> > inliers;
        int maxIter = 2000;
        bool isFit = VrelPlus::fit_ellipse_RANSAC(outsideUpPixels, outsideDownPixels, threshold,fail_ratio, ellipse, inliers, maxIter);
        if (isFit) {
            vgl_point_2d<double> pt1;
            vgl_point_2d<double> pt2;
            bool isOk = VglPlus::lineEllipseIntersection(calibLine.detectedLine(), ellipse, pt1, pt2);
            if (isOk) {
                // add center line ellipse intersection
                vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                for (int i = 0; i<pts_paris.size(); i++) {
                    wld_pts.push_back(pts_paris[i].first);
                    img_pts.push_back(pts_paris[i].second);
                }
            }
        }
    }
    
    if (wld_pts.size() < 2) {
        printf("only find %lu intersections\n", wld_pts.size());
        return false;
    }
    
    // minimize match error
    vpgl_ptz_camera initPTZ;
    vpgl_ptz_camera finalPTZ;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera, initPTZ);
    if (!isPTZ) {
        printf("refine camera failed: initial camear failed to PTZ\n");
        return false;
    }
    
    VpglPTZCameraUtil::Correspondence corres;
    corres.wld_pts = wld_pts;
    corres.img_pts = img_pts;
    
    bool isRefind = VpglPTZCameraUtil::refineCameraByPtsAndVPs(initPTZ, corres, finalPTZ);
    if (!isRefind) {
        printf("Error: failed to refine the PTZ camera\n");
        return false;
    }
    
    WWoSSoccerCourtUtil::PTZToCamera(finalPTZ.pan(), finalPTZ.tilt(), finalPTZ.focal_length(), finalCamera);
    return true;
}

bool WWoSCameraRefinement::refineCameraByLineTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                                      const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 15;
    const int lineWidth = 10;
    const int lineMinPixelNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    SoccerGraphCutUtil sgcUtil;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sgcUtil.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    vil_image_view<double> magnitude;
    VilPlus::vil_magnitude(image, magnitude);
    
    double magThreshold[calibLineNum] = { 0.05, 0.04, 0.04, 0.04, 0.04,
                                          0.02, 0.02, 0.02, 0.02, 0.02,
                                          0.02, 0.02, 0.02, 0.02, 0.02};
    double lambdaTHreshold[calibLineNum] = {2, 2, 2, 2, 2,
                                            1.5, 1.5, 1.5, 1.5, 1.5,
                                            1.5, 1.5, 1.5, 1.5, 1.5};
    
    
    vcl_vector<SGCCalibLine> detectedCalibLines;
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        
        // draw line segment in the image
        maskImage.fill(0);
        VilPlus::draw_segment(maskImage, calibLine.point1(), calibLine.point2(), VilPlus::white(), lineWidth);
        
        AverageMagnitudeParameter para;
        para.direction_ = calibLine.scan_direction();
        para.mag_threshold_ = magThreshold[calibLine.node()];
        para.lambda_ = lambdaTHreshold[calibLine.node()];        
        
        // detect edge point candidate
        vcl_vector<vgl_point_2d<double> > side_edge_pts1;
        vcl_vector<vgl_point_2d<double> > side_edge_pts2;
        vxl_half_edge_line_tracking::detectLinePixelByAverageGradient(magnitude, maskImage, para, side_edge_pts1, side_edge_pts2);
        
        // filter side edge pts
        
        
        // hough line detection for two edge line
        VxlHoughParameter houghPara;
        houghPara.maxLineNum_ = 1;
        houghPara.minPixeNum_ = lineMinPixelNum;
        vgl_line_2d<double> dumpLine;
        vcl_vector<vgl_point_2d<double> > inlier_pts1;
        vcl_vector<vgl_point_2d<double> > inlier_pts2;
        VxlHoughLine::detectOneLine(side_edge_pts1, w, h, houghPara, dumpLine, inlier_pts1);
        VxlHoughLine::detectOneLine(side_edge_pts2, w, h, houghPara, dumpLine, inlier_pts2);
        
        // estimate center line from edge line
        vgl_line_2d<double> centerLine;
        vcl_vector<vgl_point_2d<double> > center_pts;
        bool isFindCenterLine = false;
        if (inlier_pts1.size() > inlier_pts2.size()) {
            isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts1, inlier_pts2, center_pts, centerLine);
        }
        else
        {
            isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts2, inlier_pts1, center_pts, centerLine);
        }
        if (isFindCenterLine) {
            // check the directon different
            vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
            vgl_vector_2d<double> dir2 = centerLine.direction();
            double cosAngle = fabs(cos_angle(dir1, dir2));
            
            if (cosAngle > lineDirectionThreshold) {
                calibLine.setDetectedLine(centerLine);
                detectedCalibLines.push_back(calibLine);
                
                // draw single line
                {
                    vil_image_view<vxl_byte> showImage;
                    showImage.deep_copy(image);
                  //  VilPlus::draw_line(showImage, centerLine, VilPlus::blue());
                    VilPlus::draw_dot(showImage, center_pts, VilPlus::blue());
                    
                    char buf[1024] = {NULL};
                    sprintf(buf, "center_line_%d.jpg", calibLine.node());
                    VilPlus::vil_save(showImage, buf);
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        return false;
    }
    {
        // draw line
        vcl_vector<vgl_line_2d<double> > lines;
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            lines.push_back(detectedCalibLines[i].detectedLine());
        }
        
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_lines(showImage, lines, VilPlus::green());
        VilPlus::vil_save(showImage, "detected_line.jpg");
    }
    
    // find intersections
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sgcUtil.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(wld_pt);
                        img_pts.push_back(img_pt);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    // exclude outliers
    
    if (wld_pts.size() < 2) {
        printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        return false;
    }
    
    // minimize match error
    vpgl_ptz_camera initPTZ;
    vpgl_ptz_camera finalPTZ;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera, initPTZ);
    if (!isPTZ) {
        printf("refine camera failed: initial camear failed to PTZ\n");
        return false;
    }
    
    VpglPTZCameraUtil::Correspondence corres;
    corres.wld_pts = wld_pts;
    corres.img_pts = img_pts;
    
    bool isRefind = VpglPTZCameraUtil::refineCameraByPtsAndVPs(initPTZ, corres, finalPTZ);
    if (!isRefind) {
        printf("Error: failed to refine the PTZ camera\n");
        return false;
    }
    
    WWoSSoccerCourtUtil::PTZToCamera(finalPTZ.pan(), finalPTZ.tilt(), finalPTZ.focal_length(), finalCamera);
    
    return true;
}

bool WWoSCameraRefinement::refineCameraByLineAndEllipseTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                                                const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 15;
//    const int lineWidth = para.line_thick_ness_;
    const int lineMinPixelNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    // 1. sample line segment
    SoccerGraphCutUtil sgcUtil;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sgcUtil.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    // 2. tracking lines in the image space
    vil_image_view<double> magnitude;
    VilPlus::vil_magnitude(image, magnitude);
    
    double magThreshold[calibLineNum] = { 0.05, 0.04, 0.04, 0.04, 0.04,
        0.02, 0.02, 0.02, 0.02, 0.02,
        0.02, 0.02, 0.02, 0.02, 0.02};
    double lambdaTHreshold[calibLineNum] = {2, 2.0, 2.0, 2, 2,
        1.5, 1.5, 1.5, 1.5, 1.5,
        1.5, 1.5, 1.5, 1.5, 1.5};
    double lineWidthes[calibLineNum] = {30, 20, 20, 20, 20,
                                        10, 10, 20, 20, 10,
                                        20, 30, 30, 20, 20};
    
    
    vcl_vector<SGCCalibLine> detectedCalibLines;
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];        
        
        // draw line segment in the image
        maskImage.fill(0);
        VilPlus::draw_segment(maskImage, calibLine.point1(), calibLine.point2(), VilPlus::white(), lineWidthes[calibLine.node()]);
        if(!silent)
        {
            char buf[1024] = {NULL};
            vil_image_view<vxl_byte> showImage;
            showImage.deep_copy(image);
            
            VilPlus::draw_segment(showImage, calibLine.point1(), calibLine.point2(), VilPlus::white(), lineWidthes[calibLine.node()]);
            
            sprintf(buf, "mask_%d.jpg", calibLine.node());
            VilPlus::vil_save(showImage, buf);
        }
        
        AverageMagnitudeParameter para;
        para.direction_ = calibLine.scan_direction();
        para.mag_threshold_ = magThreshold[calibLine.node()];
        para.lambda_ = lambdaTHreshold[calibLine.node()];
        
        // detect edge point candidate
        vcl_vector<vgl_point_2d<double> > side_edge_pts1;
        vcl_vector<vgl_point_2d<double> > side_edge_pts2;
        vxl_half_edge_line_tracking::detectLinePixelByAverageGradient(magnitude, maskImage, para, side_edge_pts1, side_edge_pts2);
        
        // filter side edge pts
        
        
        // hough line detection for two edge line
        VxlHoughParameter houghPara;
        houghPara.maxLineNum_ = 1;
        houghPara.minPixeNum_ = lineMinPixelNum;
        vgl_line_2d<double> dumpLine;
        vcl_vector<vgl_point_2d<double> > inlier_pts1;
        vcl_vector<vgl_point_2d<double> > inlier_pts2;
        VxlHoughLine::detectOneLine(side_edge_pts1, w, h, houghPara, dumpLine, inlier_pts1);
        VxlHoughLine::detectOneLine(side_edge_pts2, w, h, houghPara, dumpLine, inlier_pts2);
        
        // estimate center line from edge line
        vgl_line_2d<double> centerLine;
        vcl_vector<vgl_point_2d<double> > center_pts;
        bool isFindCenterLine = false;
        if (inlier_pts1.size() > inlier_pts2.size()) {
            isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts1, inlier_pts2, center_pts, centerLine);
        }
        else
        {
            isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts2, inlier_pts1, center_pts, centerLine);
        }
        if (isFindCenterLine) {
            // check the directon different
            vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
            vgl_vector_2d<double> dir2 = centerLine.direction();
            double cosAngle = fabs(cos_angle(dir1, dir2));
            
            if (cosAngle > lineDirectionThreshold) {
                calibLine.setDetectedLine(centerLine);
                detectedCalibLines.push_back(calibLine);
                
                // draw single line
                if(!silent)
                {
                    vil_image_view<vxl_byte> showImage;
                    showImage.deep_copy(image);
                    //  VilPlus::draw_line(showImage, centerLine, VilPlus::blue());
                    VilPlus::draw_dot(showImage, center_pts, VilPlus::blue());
                    
                    char buf[1024] = {NULL};
                    sprintf(buf, "center_line_%d.jpg", calibLine.node());
                    VilPlus::vil_save(showImage, buf);
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        return false;
    }
    if(!silent)
    {
        // draw line
        vcl_vector<vgl_line_2d<double> > lines;
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            lines.push_back(detectedCalibLines[i].detectedLine());
        }
        
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_lines(showImage, lines, VilPlus::green());
        VilPlus::vil_save(showImage, "detected_line.jpg");
    }
    
    // find intersections
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sgcUtil.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(wld_pt);
                        img_pts.push_back(img_pt);                        
                  //      printf("intersection node number is %d %d\n\n", node1, node2);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    printf("find %lu line-line intersection.\n", wld_pts.size());
    
    // check ellipse inside image
    bool isSoccerInside = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    if (isSoccerInside) {
        // find center line
        bool isCenterLine = false;
        SGCCalibLine centerLine;
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            if (detectedCalibLines[i].node() == 0) {
                isCenterLine = true;
                centerLine = detectedCalibLines[i];
                break;
            }
        }
        if (isCenterLine) {
            // detect ellipse
            vil_image_view<vxl_byte> nonEllipseMaskImage = vil_image_view<vxl_byte>(w, h, 3);
            nonEllipseMaskImage.fill(0);
            maskImage.fill(0);
            
            AverageMagnitudeParameter para;
            para.mag_threshold_ = 0.05;
            para.lambda_    = 2.0;
            para.direction_ = 1;
            
            // get bounding box
            WWoSSoccerCourt wwosCourt;
            wwosCourt.overlayEllipseBoundingBox(camera, maskImage, VilPlus::white(), 5, 5);
            
            vcl_vector<vgl_point_2d<double> > outsideUpPixels;
            vcl_vector<vgl_point_2d<double> > outsideDownPixels;
            vxl_half_edge_line_tracking::detectEllipsePixelByAverageGradient(magnitude, maskImage, nonEllipseMaskImage, para, outsideUpPixels, outsideDownPixels);
            
            double threshold  = 2.0;
            double fail_ratio = 0.001;
            vgl_ellipse_2d<double> ellipse;
            vcl_vector<vgl_point_2d<double> > inliers;
            int maxIter = 2000;
            bool isFit = VrelPlus::fit_ellipse_RANSAC(outsideUpPixels, outsideDownPixels, threshold,fail_ratio, ellipse, inliers, maxIter);
            if (isFit) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                bool isOk = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), ellipse, pt1, pt2);
                if (isOk) {
                    // add center line ellipse intersection
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    for (int i = 0; i<pts_paris.size(); i++) {
                        wld_pts.push_back(pts_paris[i].first);
                        img_pts.push_back(pts_paris[i].second);                       
                    }
                    printf("add center line, circle intersection. \n");
                    
                    if(!silent)
                    {
                        vil_image_view<vxl_byte> showImage;
                        showImage.deep_copy(image);
                        VilPlus::draw_cross(showImage, img_pts, 5, VilPlus::red(), 3);
                        VilPlus::vil_save(showImage, "center_line_circle_intersection.jpg");
                    }
                }
            }
        }
    }    
    
    // exclude outliers
    if (wld_pts.size() < 2) {
        printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        return false;
    }
    
    // minimize match error
    vpgl_ptz_camera initPTZ;
    vpgl_ptz_camera finalPTZ;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera, initPTZ);
    if (!isPTZ) {
        printf("refine camera failed: initial camear failed to PTZ\n");
        return false;
    }
    
    VpglPTZCameraUtil::Correspondence corres;
    corres.wld_pts = wld_pts;
    corres.img_pts = img_pts;
    
    bool isRefind = VpglPTZCameraUtil::refineCameraByPtsAndVPs(initPTZ, corres, finalPTZ);
    if (!isRefind) {
        printf("Error: failed to refine the PTZ camera\n");
        return false;
    }
    
    WWoSSoccerCourtUtil::PTZToCamera(finalPTZ.pan(), finalPTZ.tilt(), finalPTZ.focal_length(), finalCamera);
    return true;
}

bool WWoSCameraRefinement::refineCameraByShortLineAndEllipseTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                               const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    WWoSSoccerCourt wwosCourt;
    // 1. predict line segment in the image
    SoccerShortLineModel sslModel;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sslModel.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    // 2. tracking lines in the image space
   // vil_image_view<double> magnitude;
   // VilPlus::vil_magnitude(image, magnitude);
    
    // edge detection data
    vil_image_view<double> maganitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, maganitude, grad_i, grad_j, false);
    
    vcl_vector<SGCCalibLine> detectedCalibLines;
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        const int nodeId = calibLine.node();
        
        if (nodeId == 18) {
            int test = 1;
        }
        
        LinesegmntTrackingParameter para;
        para.lambda_ = sslModel.nodeLambda(nodeId);
        para.mag_threshold_1_ = sslModel.nodeMagnitude(nodeId);
        para.line_width_      = sslModel.nodeWidth(nodeId);
        para.online_pixel_ratio_ = sslModel.nodeOnlineRatio(nodeId);
        
        vgl_line_segment_2d<double> initLineSeg;
        vgl_line_segment_2d<double> refinedLineSeg;
        vgl_line_segment_2d<double> finalLineSeg;
        vgl_line_2d<double> finaline;
        bool isFindFinal = false;
        
        bool isSegment = sslModel.projectNode(camera, nodeId, initLineSeg);
        if (!isSegment) {
            continue;
        }
        // initial 
        bool isInitGoodEnough = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, initLineSeg, para, para.online_pixel_ratio_);
        if (!isInitGoodEnough) {
            // refine the initial line segment first
            bool isRefined = VilLineTracking::refineLineSegment(image, maganitude, initLineSeg, para, refinedLineSeg);
         //   bool isRefined = VilLineTracking::refineLineSegment(image, maganitude, grad_i, grad_j, initLineSeg, para, refinedLineSeg);
            if (!isRefined) {
              //  printf("Warning: refine segment failed\n");
                continue;
            }
        }
        else
        {
            refinedLineSeg = initLineSeg;
        }
        
        // double edge refinement
        if (sslModel.hasDoubleEdge(nodeId)) {
            para.line_width_ /= 2.0;
            isFindFinal = vxl_half_edge_line_tracking::trackingLineFromSegment(image, maganitude, grad_i, grad_j, refinedLineSeg, para, finaline, finalLineSeg);
        }
        else
        {
            finaline = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
            finalLineSeg = refinedLineSeg;
            isFindFinal = true;
        }
        
        if (isFindFinal) {
            // check if it is a good line segment
            bool isAnEdge = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, finalLineSeg, para, para.online_pixel_ratio_);
            if (isAnEdge) {
                vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
                vgl_vector_2d<double> dir2 = finaline.direction();
                double cosAngle = fabs(cos_angle(dir1, dir2));
                
                if (cosAngle > lineDirectionThreshold) {
                    calibLine.setDetectedLine(finaline);
                    calibLine.detected_image_line_segment_ = finalLineSeg;
                    detectedCalibLines.push_back(calibLine);
                    
                    if(!silent)
                    {
                        // draw detected line
                        vil_image_view<vxl_byte> showImage;
                        showImage.deep_copy(image);
                        VilPlus::draw_segment(showImage, initLineSeg, VilPlus::red());
                        VilPlus::draw_line(showImage, finaline, VilPlus::green());
                        char buf[1024] = {NULL};
                        sprintf(buf, "center_line_%d.jpg", calibLine.node());
                        VilPlus::vil_save(showImage, buf);
                    }
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        return false;
    }
    printf("detected %lu lines\n", detectedCalibLines.size());
    
    if (!silent) {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            VilPlus::draw_segment(showImage, detectedCalibLines[i].detected_image_line_segment_, VilPlus::green());
        }
        VilPlus::vil_save(showImage, "detected_linesegment.jpg");
    }
    
    // calibration correspondence
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    // 3. tracking center circle in the image
    // check ellipse inside image
    bool hasCenterCircle = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    if (hasCenterCircle) {
        // tracking center circle in image
        ElliseTrackingParameter centerCircleTrackingPara;
        vgl_ellipse_2d<double> centerEllipse;
        wwosCourt.centerCircleBoundingBox(camera, w, h, centerCircleTrackingPara.bounding_box_, 5, 5);
        bool isCenterCircleTracked = VilEllipseTracking::trackingInBoundingBox(image, maganitude, centerCircleTrackingPara, centerEllipse);
        if (isCenterCircleTracked) {
            // line and center circle intersection
            bool isCenterLine = false;
            SGCCalibLine centerLine;
            for (int i = 0; i<detectedCalibLines.size(); i++) {
                if (detectedCalibLines[i].node() == 0) {
                    isCenterLine = true;
                    centerLine = detectedCalibLines[i];
                    break;
                }
            }
            if (isCenterLine) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                bool isIntersect = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), centerEllipse, pt1, pt2, true);
                if (isIntersect) {
                    SoccerGraphCutUtil sgcUtil;
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    for (int i = 0; i<pts_paris.size(); i++) {
                        wld_pts.push_back(pts_paris[i].first);
                        img_pts.push_back(pts_paris[i].second);
                    }
                    
                    printf("find circle line intersection\n");
                }
            }
        }
    }
    
    // 4. find correspondence between line intersections
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sslModel.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(wld_pt);
                        img_pts.push_back(img_pt);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    if (wld_pts.size() < 2) {
        printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        return false;
    }
    
    printf("find %lu intersections\n", wld_pts.size());
    
    if(!silent)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, img_pts, 3, VilPlus::green());
        char buf[1024] = {NULL};
        sprintf(buf, "geometry_intersection.jpg");
        VilPlus::vil_save(showImage, buf);
    }
    
    // 5. camera optimization
    // minimize match error
    vpgl_ptz_camera initPTZ;
    vpgl_ptz_camera finalPTZ;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera, initPTZ);
    if (!isPTZ) {
        printf("refine camera failed: initial camear failed to PTZ\n");
        return false;
    }
    
    VpglPTZCameraUtil::Correspondence corres;
    corres.wld_pts = wld_pts;
    corres.img_pts = img_pts;
    
 //   printf("constraint number is %lu\n", wld_pts.size());
    
    bool isRefind = VpglPTZCameraUtil::refineCameraByPtsAndVPs(initPTZ, corres, finalPTZ);
    if (!isRefind) {
        printf("Error: failed to refine the PTZ camera\n");
        return false;
    }
    
    WWoSSoccerCourtUtil::PTZToCamera(finalPTZ.pan(), finalPTZ.tilt(), finalPTZ.focal_length(), finalCamera);
    return true;
}

bool WWoSCameraRefinement::refineCameraByShortLineAndEllipseOptimizeByPointOnline(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                                            const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    WWoSSoccerCourt wwosCourt;
    // 1. predict line segment in the image
    SoccerShortLineModel sslModel;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sslModel.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    // 2. tracking lines in the image space
    // edge detection data
    vil_image_view<double> maganitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, maganitude, grad_i, grad_j, false);
    
    vcl_vector<SGCCalibLine> detectedCalibLines;  // verified line segment
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        const int nodeId = calibLine.node();
        
        if (nodeId == 0) {
            int test = 1;
        }
        
        LinesegmntTrackingParameter para;
        para.lambda_ = sslModel.nodeLambda(nodeId);
        para.mag_threshold_1_ = sslModel.nodeMagnitude(nodeId);
        para.line_width_      = sslModel.nodeWidth(nodeId);
        para.online_pixel_ratio_ = sslModel.nodeOnlineRatio(nodeId);
        
        vgl_line_segment_2d<double> initLineSeg;
        vgl_line_segment_2d<double> refinedLineSeg;
        vgl_line_segment_2d<double> finalLineSeg;
        vgl_line_2d<double> finaline;
        bool isFindFinal = false;
        
        bool isSegment = sslModel.projectNode(camera, nodeId, initLineSeg);
        if (!isSegment) {
            continue;
        }
        // initial
        bool isInitGoodEnough = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, initLineSeg, para, para.online_pixel_ratio_);
        if (!isInitGoodEnough) {
            // refine the initial line segment first
            bool isRefined = VilLineTracking::refineLineSegment(image, maganitude, grad_i, grad_j, initLineSeg, para, refinedLineSeg);
            if (!isRefined) {
                if (!silent) {
                    printf("Warning: refine segment failed\n");
                }
                continue;
            }
        }
        else
        {
            refinedLineSeg = initLineSeg;
        }
        
        // double edge refinement
        if (sslModel.hasDoubleEdge(nodeId)) {
            para.line_width_ /= 2.0;
            isFindFinal = vxl_half_edge_line_tracking::trackingLineFromSegment(image, maganitude, grad_i, grad_j, refinedLineSeg, para, finaline, finalLineSeg);
        }
        else
        {
            finaline = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
            finalLineSeg = refinedLineSeg;
            isFindFinal = true;
        }
        
        if (isFindFinal) {
            // check if it is a good line segment
            bool isAnEdge = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, finalLineSeg, para, para.online_pixel_ratio_);
            if (isAnEdge) {
                vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
                vgl_vector_2d<double> dir2 = finaline.direction();
                double cosAngle = fabs(cos_angle(dir1, dir2));
                
                if (cosAngle > lineDirectionThreshold) {
                    calibLine.setDetectedLine(finaline);
                    calibLine.detected_image_line_segment_ = finalLineSeg;
                    detectedCalibLines.push_back(calibLine);
                    
                    if(!silent)
                    {
                        // draw detected line
                        vil_image_view<vxl_byte> showImage;
                        showImage.deep_copy(image);
                        VilPlus::draw_segment(showImage, initLineSeg, VilPlus::red());
                        VilPlus::draw_line(showImage, finaline, VilPlus::green());
                        char buf[1024] = {NULL};
                        sprintf(buf, "center_line_%d.jpg", calibLine.node());
                        VilPlus::vil_save(showImage, buf);
                    }
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        if (!silent) {
            printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        }
        return false;
    }
    printf("detected %lu lines\n", detectedCalibLines.size());
    
    if (!silent) {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            VilPlus::draw_segment(showImage, detectedCalibLines[i].detected_image_line_segment_, VilPlus::green());
        }
        VilPlus::vil_save(showImage, "detected_linesegment.jpg");
    }
    
    // calibration correspondence
    vcl_vector<vgl_point_3d<double> > wld_pts;  // line intersection
    vcl_vector<vgl_point_2d<double> > img_pts;
    
    
    // 3. tracking center circle in the image
    // check ellipse inside image
    bool hasCenterCircle = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    bool isCenterCircleTracked = false;
    vgl_ellipse_2d<double> centerEllipse;
    if (hasCenterCircle) {
        // tracking center circle in image
        ElliseTrackingParameter centerCircleTrackingPara;
       
        wwosCourt.centerCircleBoundingBox(camera, w, h, centerCircleTrackingPara.bounding_box_, 5, 5);
        isCenterCircleTracked = VilEllipseTracking::trackingInBoundingBox(image, maganitude, centerCircleTrackingPara, centerEllipse);
        if (isCenterCircleTracked) {
            // line and center circle intersection
            bool isCenterLine = false;
            SGCCalibLine centerLine;
            for (int i = 0; i<detectedCalibLines.size(); i++) {
                if (detectedCalibLines[i].node() == 0) {
                    isCenterLine = true;
                    centerLine = detectedCalibLines[i];
                    break;
                }
            }
            isCenterCircleTracked = true;
            if (isCenterLine) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                bool isIntersect = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), centerEllipse, pt1, pt2, true);
                if (isIntersect) {
                    SoccerGraphCutUtil sgcUtil;
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    for (int i = 0; i<pts_paris.size(); i++) {
                        wld_pts.push_back(vgl_point_3d<double>(pts_paris[i].first.x(), pts_paris[i].first.y(), 0.0));
                        img_pts.push_back(pts_paris[i].second);
                    }
                    
                    if (!silent) {
                        printf("find circle line intersection\n");
                    }
                }
            }
        }
    }
    
    vcl_vector<int> detected_nodes;
    // 4. find correspondence between line intersections
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sslModel.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(vgl_point_3d<double>(wld_pt.x(), wld_pt.y(), 0.0));
                        img_pts.push_back(img_pt);
                        detected_nodes.push_back(node1);
                        detected_nodes.push_back(node2);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    vcl_sort(detected_nodes.begin(), detected_nodes.end());
    vcl_vector<int>::iterator ite = vcl_unique(detected_nodes.begin(), detected_nodes.end());
    detected_nodes.erase(ite, detected_nodes.end());
    if (!silent) {
        printf("find %lu interextion nodes\n", detected_nodes.size());
        printf("find %lu linesegments in image\n", detectedCalibLines.size());
    }
    
    
    
    if (wld_pts.size() < 2) {
        if (!silent) {
            printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        }        
        return false;
    }
    printf("find %lu intersections\n", wld_pts.size());
    
    if(!silent)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, img_pts, 3, VilPlus::green());
        if (isCenterCircleTracked) {
            VilPlus::draw_ellipse(showImage, centerEllipse, VilPlus::green());
            vcl_vector<vgl_point_2d<double> > ellipse_pts;
            VglPlus::sampleElliseInImage(centerEllipse, 10, w, h, ellipse_pts);
            VilPlus::draw_cross(showImage, ellipse_pts, 5, VilPlus::red());
        }
        char buf[1024] = {NULL};
        sprintf(buf, "geometry_intersection.jpg");
        VilPlus::vil_save(showImage, buf);
    }
    
    // 5. camera optimization
    LinePointsInCameraview linePoints;
    linePoints.wld_pts_ = wld_pts;
    linePoints.img_pts_ = img_pts;
    linePoints.camera_ = camera;
    const double segment_image_length = 50;
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        PointsOnLine pol;
        vcl_vector<vgl_point_2d<double> > pts;
        pol.line_ = vgl_line_3d_2_points<double>(detectedCalibLines[i].point1_3d(), detectedCalibLines[i].point2_3d());
        
        vgl_line_segment_2d<double> img_seg = detectedCalibLines[i].detected_image_line_segment_;
        
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
            linePoints.lines_.push_back(pol);
        }
    }
    if (isCenterCircleTracked) {
        PointsOnCircle circle;
        circle.circle_ = wwosCourt.getCenterCircle();
        VglPlus::sampleElliseInImage(centerEllipse, 10, w, h, circle.pts_);
        
        if (circle.pts_.size() != 0) {
            linePoints.circles_.push_back(circle);
        }
    }
    
    bool isEstimated = VpglPTZEstimation::estimageCamera(linePoints, finalCamera, false);
    return isEstimated;
}

bool WWoSCameraRefinement::modelTracking(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                         const LineTrackingParameter & para, SoccerModelTrackingResult & model, bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    WWoSSoccerCourt wwosCourt;
    // 1. predict line segment in the image
    SoccerShortLineModel sslModel;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sslModel.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    // 2. tracking lines in the image space
    // edge detection data
    vil_image_view<double> maganitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, maganitude, grad_i, grad_j, false);
    
    vcl_vector<SGCCalibLine> detectedCalibLines;  // verified line segment
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        const int nodeId = calibLine.node();
        
        if (nodeId == 0) {
            int test = 1;
        }
        
        LinesegmntTrackingParameter para;
        para.lambda_ = sslModel.nodeLambda(nodeId);
        para.mag_threshold_1_ = sslModel.nodeMagnitude(nodeId);
        para.line_width_      = sslModel.nodeWidth(nodeId);
        para.online_pixel_ratio_ = sslModel.nodeOnlineRatio(nodeId);
        
        vgl_line_segment_2d<double> initLineSeg;
        vgl_line_segment_2d<double> refinedLineSeg;
        vgl_line_segment_2d<double> finalLineSeg;
        vgl_line_2d<double> finaline;
        bool isFindFinal = false;
        
        bool isSegment = sslModel.projectNode(camera, nodeId, initLineSeg);
        if (!isSegment) {
            continue;
        }
        // initial
        bool isInitGoodEnough = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, initLineSeg, para, para.online_pixel_ratio_);
        if (!isInitGoodEnough) {
            // refine the initial line segment first
            bool isRefined = VilLineTracking::refineLineSegment(image, maganitude, grad_i, grad_j, initLineSeg, para, refinedLineSeg);
            if (!isRefined) {
                if (!silent) {
                    printf("Warning: refine segment failed\n");
                }
                continue;
            }
        }
        else
        {
            refinedLineSeg = initLineSeg;
        }
        
        // double edge refinement
        if (sslModel.hasDoubleEdge(nodeId)) {
            para.line_width_ /= 2.0;
            isFindFinal = vxl_half_edge_line_tracking::trackingLineFromSegment(image, maganitude, grad_i, grad_j, refinedLineSeg, para, finaline, finalLineSeg);
        }
        else
        {
            finaline = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
            finalLineSeg = refinedLineSeg;
            isFindFinal = true;
        }
        
        if (isFindFinal) {
            // check if it is a good line segment
            bool isAnEdge = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, finalLineSeg, para, para.online_pixel_ratio_);
            if (isAnEdge) {
                vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
                vgl_vector_2d<double> dir2 = finaline.direction();
                double cosAngle = fabs(cos_angle(dir1, dir2));
                
                if (cosAngle > lineDirectionThreshold) {
                    calibLine.setDetectedLine(finaline);
                    calibLine.detected_image_line_segment_ = finalLineSeg;
                    detectedCalibLines.push_back(calibLine);
                    
                    if(!silent)
                    {
                        // draw detected line
                        vil_image_view<vxl_byte> showImage;
                        showImage.deep_copy(image);
                        VilPlus::draw_segment(showImage, initLineSeg, VilPlus::red());
                        VilPlus::draw_line(showImage, finaline, VilPlus::green());
                        char buf[1024] = {NULL};
                        sprintf(buf, "center_line_%d.jpg", calibLine.node());
                        VilPlus::vil_save(showImage, buf);
                    }
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        if (!silent) {
            printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        }
        return false;
    }
    printf("detected %lu lines\n", detectedCalibLines.size());
    
    if (!silent) {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            VilPlus::draw_segment(showImage, detectedCalibLines[i].detected_image_line_segment_, VilPlus::green());
        }
        VilPlus::vil_save(showImage, "detected_linesegment.jpg");
    }
    
    // calibration correspondence
    vcl_vector<vgl_point_3d<double> > wld_pts;  // line intersection
    vcl_vector<vgl_point_2d<double> > img_pts;
    
    
    // 3. tracking center circle in the image
    // check ellipse inside image
    bool hasCenterCircle = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    bool isCenterCircleTracked = false;
    vgl_ellipse_2d<double> centerEllipse;
    if (hasCenterCircle) {
        // tracking center circle in image
        ElliseTrackingParameter centerCircleTrackingPara;
        
        wwosCourt.centerCircleBoundingBox(camera, w, h, centerCircleTrackingPara.bounding_box_, 5, 5);
        isCenterCircleTracked = VilEllipseTracking::trackingInBoundingBox(image, maganitude, centerCircleTrackingPara, centerEllipse);
        if (isCenterCircleTracked) {
            // line and center circle intersection
            bool isCenterLine = false;
            SGCCalibLine centerLine;
            for (int i = 0; i<detectedCalibLines.size(); i++) {
                if (detectedCalibLines[i].node() == 0) {
                    isCenterLine = true;
                    centerLine = detectedCalibLines[i];
                    break;
                }
            }
            isCenterCircleTracked = true;
            if (isCenterLine) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                bool isIntersect = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), centerEllipse, pt1, pt2, true);
                if (isIntersect) {
                    SoccerGraphCutUtil sgcUtil;
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    for (int i = 0; i<pts_paris.size(); i++) {
                        wld_pts.push_back(vgl_point_3d<double>(pts_paris[i].first.x(), pts_paris[i].first.y(), 0.0));
                        img_pts.push_back(pts_paris[i].second);
                    }
                    
                    if (!silent) {
                        printf("find circle line intersection\n");
                    }
                }
            }
        }
    }
    
    // left  ellipse
    bool has_left_circle = WWoSSoccerCourt::isPenaltyEllipseInsideImage(camera, w, h, true);
    bool is_left_circle_tracked = false;
    vgl_ellipse_2d<double> left_ellipse;
    vcl_vector<vgl_point_2d<double> > left_ellipse_points;
    if (has_left_circle) {
        vgl_box_2d<int> box;
        wwosCourt.penaltyEllipseBoundingBox(camera, w, h, box, true, 5, 5);
        vil_image_view<vxl_byte> sub_image = vil_crop(image, box.min_x(), box.width(), box.min_y(), box.height());
        is_left_circle_tracked = VxlELSD::detect_largest_ellipse(sub_image, left_ellipse, left_ellipse_points);
        // shift position
        for (int i = 0; i<left_ellipse_points.size(); i++) {
            left_ellipse_points[i] = vgl_point_2d<double>(left_ellipse_points[i].x() + box.min_x(), left_ellipse_points[i].y() + box.min_y());
        }
    }
    
    
    // right ellipse
    
    vcl_vector<int> detected_nodes;
    // 4. find correspondence between line intersections
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sslModel.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(vgl_point_3d<double>(wld_pt.x(), wld_pt.y(), 0.0));
                        img_pts.push_back(img_pt);
                        detected_nodes.push_back(node1);
                        detected_nodes.push_back(node2);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    vcl_sort(detected_nodes.begin(), detected_nodes.end());
    vcl_vector<int>::iterator ite = vcl_unique(detected_nodes.begin(), detected_nodes.end());
    detected_nodes.erase(ite, detected_nodes.end());
    if (!silent) {
        printf("find %lu interextion nodes\n", detected_nodes.size());
        printf("find %lu linesegments in image\n", detectedCalibLines.size());
    }
    
    
    
    if (wld_pts.size() < 2) {
        if (!silent) {
            printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        }
        return false;
    }
    printf("find %lu intersections\n", wld_pts.size());
    
    if(!silent)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, img_pts, 3, VilPlus::green());
        if (isCenterCircleTracked) {
            VilPlus::draw_ellipse(showImage, centerEllipse, VilPlus::green());
            vcl_vector<vgl_point_2d<double> > ellipse_pts;
            VglPlus::sampleElliseInImage(centerEllipse, 10, w, h, ellipse_pts);
            VilPlus::draw_cross(showImage, ellipse_pts, 5, VilPlus::red());
        }
        char buf[1024] = {NULL};
        sprintf(buf, "geometry_intersection.jpg");
        VilPlus::vil_save(showImage, buf);
    }    
    
    model.detected_calib_Lines_ = detectedCalibLines;
    model.wld_pts_ = wld_pts;
    model.img_pts_ = img_pts;
    model.detected_node_id_ = detected_nodes;
    model.is_centercircle_tracked_ = isCenterCircleTracked;
    if (isCenterCircleTracked) {
        model.center_ellipse_ = centerEllipse;
    }
    
    model.is_left_ellipe_detected_ = is_left_circle_tracked;
    if (is_left_circle_tracked) {
        model.left_ellipse_ = left_ellipse;
        model.left_ellipse_points_ = left_ellipse_points;
    }
    return true;
}


bool WWoSCameraRefinement::refineCameraByShortLineAndEllipseOptimizeByDetectPointOnline(const vil_image_view<vxl_byte> & image,
                                                                                        const vpgl_perspective_camera<double> & initCamera,
                                                                                        const LineTrackingParameter & para,
                                                                                        vpgl_perspective_camera<double> & finalCamera, bool silent)
{
    SoccerModelTrackingResult model;
    bool isTracked = WWoSCameraRefinement::modelTracking(image, initCamera, para, model, silent);
    if (!isTracked) {
        return false;
    }
    
    vcl_vector<SGCCalibLine> detectedCalibLines = model.detected_calib_Lines_;
    vcl_vector<vgl_point_3d<double> > wld_pts = model.wld_pts_;
    vcl_vector<vgl_point_2d<double> > img_pts = model.img_pts_;
    bool isCenterCircleTracked = model.is_centercircle_tracked_;
    vgl_ellipse_2d<double> centerEllipse = model.center_ellipse_;
    bool is_left_ellipse_tracked = model.is_left_ellipe_detected_;
    vgl_ellipse_2d<double> leftEllipse = model.left_ellipse_;
    vcl_vector<vgl_point_2d<double> > left_ellipse_points = model.left_ellipse_points_;
    
    
    WWoSSoccerCourt wwosCourt;
    const int w = image.ni();
    const int h = image.nj();
    
    // 5. camera optimization
    LinePointsInCameraview linePoints;
    linePoints.wld_pts_ = wld_pts;
    linePoints.img_pts_ = img_pts;
    linePoints.camera_ = initCamera;
    const double segment_image_length = 50;
    
    vil_image_view<vxl_byte> show_image;
    show_image.deep_copy(image);
    
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        PointsOnLine pol;
        vcl_vector<vgl_point_2d<double> > pts;
        pol.line_ = vgl_line_3d_2_points<double>(detectedCalibLines[i].point1_3d(), detectedCalibLines[i].point2_3d());
        
        vgl_line_segment_2d<double> img_seg = detectedCalibLines[i].detected_image_line_segment_;
        
        vgl_point_2d<double> q1 = img_seg.point1();
        vgl_point_2d<double> q2 = img_seg.point2();
        double distance = vgl_distance(q1, q2);
        int num = distance/segment_image_length;  // the number is decided by its length in image space
        
        for (int j = 0; j<num; j++) {
            double t1 = 1.0 * j / num;
            double t2 = 1.0 * (j+1)/num;
            vgl_point_2d<double> p3 = img_seg.point_t(t1);
            vgl_point_2d<double> p4 = img_seg.point_t(t2);
            
            vgl_box_2d<double> box(p3, p4);
            box.expand_about_centroid(15);
            
            VglPlus::clamp_box(box, 0, 0, w-1, h-1);
            if (box.width() > 5 && box.height() > 5) {
                vil_image_view<vxl_byte> sub_image = vil_crop(image, box.min_x(), box.width(), box.min_y(), box.height());
                vgl_line_segment_2d<double> sub_line_seg;
                bool is_detected = CvxLSD::detect_longest_line(sub_image, sub_line_seg);
                if (is_detected) {
                    vgl_point_2d<double> p5(sub_line_seg.point1().x() + box.min_x(), sub_line_seg.point1().y() + box.min_y());
                    vgl_point_2d<double> p6(sub_line_seg.point2().x() + box.min_x(), sub_line_seg.point2().y() + box.min_y());
                    
                    pol.pts_.push_back(centre(p5, p6));
                    VilPlus::draw_segment(show_image, p5, p6, VilPlus::green());
                }
            }
        }
        if (!pol.pts_.empty()) {
            linePoints.lines_.push_back(pol);
        }
    }
    if (isCenterCircleTracked) {
        PointsOnCircle circle;
        circle.circle_ = wwosCourt.getCenterCircle();
        VglPlus::sampleElliseInImage(centerEllipse, 10, w, h, circle.pts_);
        
        if (circle.pts_.size() != 0) {
            linePoints.circles_.push_back(circle);
            VilPlus::draw_cross(show_image, circle.pts_, 5, VilPlus::green());
        }
    }
    if (is_left_ellipse_tracked) {
        PointsOnCircle circle;
        circle.circle_ = wwosCourt.getLeftCircle();
        
        // randomly sample 3 points
        if (left_ellipse_points.size() > 10) {
            for (int i = 5; i<left_ellipse_points.size()-5; i += left_ellipse_points.size()/4) {
                circle.pts_.push_back(left_ellipse_points[i]);
            }
        }
        if (circle.pts_.size() != 0) {
            linePoints.circles_.push_back(circle);
            VilPlus::draw_cross(show_image, circle.pts_, 5, VilPlus::red());
        }
    }
    
  //  VilPlus::vil_save(show_image, "lsd_line_segment.jpg");
  //  VxlOpenCVImage::imshow(show_image, "LSD line, ellipse");
    
    bool isEstimated = VpglPTZEstimation::estimageCamera(linePoints, finalCamera, false);
    return isEstimated;
}


bool WWoSCameraRefinement::refineCameraByShortLineAndEllipseTrackingLuMethod(const vil_image_view<vxl_byte> & image,
                                                                             const vpgl_perspective_camera<double> & camera,
                                                                             const LineTrackingParameter & para,
                                                                             vpgl_perspective_camera<double> & finalCamera,
                                                                             bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    WWoSSoccerCourt wwosCourt;
    // 1. predict line segment in the image
    SoccerShortLineModel sslModel;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sslModel.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    // 2. tracking lines in the image space
    vil_image_view<double> magnitude;
    VilPlus::vil_magnitude(image, magnitude);
    
    // edge detection data
    vil_image_view<double> magenitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, magenitude, grad_i, grad_j);
    
    vcl_vector<SGCCalibLine> detectedCalibLines;
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        const int nodeId = calibLine.node();
        
        if (nodeId != 15) {
            //        continue;
        }
        
        LinesegmntTrackingParameter para;
        para.lambda_ = sslModel.nodeLambda(nodeId);
        para.mag_threshold_1_ = sslModel.nodeMagnitude(nodeId);
        para.line_width_      = sslModel.nodeWidth(nodeId);
        
        vgl_line_segment_2d<double> initLineSeg;
        vgl_line_segment_2d<double> refinedLineSeg;
        vgl_line_segment_2d<double> finalLineSeg;
        vgl_line_2d<double> finaline;
        bool isFindFinal = false;
        
        bool isSegment = sslModel.projectNode(camera, nodeId, initLineSeg);
        if (!isSegment) {
            continue;
        }
        // initial
        bool isInitGoodEnough = VilLineTracking::isLineSegmentOnEdge(image, magenitude, grad_i, grad_j, initLineSeg, para, 0.6);
        if (!isInitGoodEnough) {
            // refine the initial line segment first
            bool isRefined = VilLineTracking::refineLineSegmentByMagnitudePeak(image, magenitude, grad_i, grad_j, initLineSeg, para, refinedLineSeg, finaline);
            if (!isRefined) {
              //  printf("Warning: refine segment failed\n");
                continue;
            }
        }
        else
        {
            refinedLineSeg = initLineSeg;
        }
        
        finaline = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
        finalLineSeg = refinedLineSeg;
        isFindFinal = true;
        
        // check if it is a good line segment
        bool isAnEdge = VilLineTracking::isLineSegmentOnEdge(image, magenitude, grad_i, grad_j, finalLineSeg, para);
        if (isAnEdge) {
            vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
            vgl_vector_2d<double> dir2 = finaline.direction();
            double cosAngle = fabs(cos_angle(dir1, dir2));
            
            if (cosAngle > lineDirectionThreshold) {
                calibLine.setDetectedLine(finaline);
                detectedCalibLines.push_back(calibLine);
                
                if(!silent)
                {
                    // draw detected line
                    vil_image_view<vxl_byte> showImage;
                    showImage.deep_copy(image);
                    VilPlus::draw_line(showImage, finaline, VilPlus::green());
                    char buf[1024] = {NULL};
                    sprintf(buf, "center_line_%d.jpg", calibLine.node());
                    VilPlus::vil_save(showImage, buf);
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        return false;
    }
    printf("detected %lu lines\n", detectedCalibLines.size());
    
    // calibration correspondence
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    // 3. tracking center circle in the image
    // check ellipse inside image
    bool hasCenterCircle = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    if (hasCenterCircle) {
        // tracking center circle in image
        ElliseTrackingParameter centerCircleTrackingPara;
        vgl_ellipse_2d<double> centerEllipse;
        wwosCourt.centerCircleBoundingBox(camera, w, h, centerCircleTrackingPara.bounding_box_, 5, 5);
        bool isCenterCircleTracked = VilEllipseTracking::trackingInBoundingBox(image, magnitude, centerCircleTrackingPara, centerEllipse);
        if (isCenterCircleTracked) {
            // line and center circle intersection
            bool isCenterLine = false;
            SGCCalibLine centerLine;
            for (int i = 0; i<detectedCalibLines.size(); i++) {
                if (detectedCalibLines[i].node() == 0) {
                    isCenterLine = true;
                    centerLine = detectedCalibLines[i];
                    break;
                }
            }
            if (isCenterLine) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                bool isIntersect = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), centerEllipse, pt1, pt2, true);
                if (isIntersect) {
                    SoccerGraphCutUtil sgcUtil;
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    for (int i = 0; i<pts_paris.size(); i++) {
                        wld_pts.push_back(pts_paris[i].first);
                        img_pts.push_back(pts_paris[i].second);
                    }
                    
                    printf("find circle line intersection\n");
                }
            }
        }
    }
    
    // 4. find correspondence between line intersections
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sslModel.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(wld_pt);
                        img_pts.push_back(img_pt);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    if (wld_pts.size() < 2) {
        printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        return false;
    }
    
    printf("find %lu intersections\n", wld_pts.size());
    
    if(!silent)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, img_pts, 3, VilPlus::green());
        char buf[1024] = {NULL};
        sprintf(buf, "geometry_intersection.jpg");
        VilPlus::vil_save(showImage, buf);
    }
    
    // 5. camera optimization
    // minimize match error
    vpgl_ptz_camera initPTZ;
    vpgl_ptz_camera finalPTZ;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera, initPTZ);
    if (!isPTZ) {
        printf("refine camera failed: initial camear failed to PTZ\n");
        return false;
    }
    
    VpglPTZCameraUtil::Correspondence corres;
    corres.wld_pts = wld_pts;
    corres.img_pts = img_pts;
    
    bool isRefind = VpglPTZCameraUtil::refineCameraByPtsAndVPs(initPTZ, corres, finalPTZ);
    if (!isRefind) {
        printf("Error: failed to refine the PTZ camera\n");
        return false;
    }
    
    WWoSSoccerCourtUtil::PTZToCamera(finalPTZ.pan(), finalPTZ.tilt(), finalPTZ.focal_length(), finalCamera);
    return true;
}

bool WWoSCameraRefinement::refineCameraByShortLineAndEllipseTrackingThomasMethod(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                                                                 const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    WWoSSoccerCourt wwosCourt;
    // 1. predict line segment in the image
    SoccerShortLineModel sslModel;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sslModel.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    // 2. tracking lines in the image space
    vil_image_view<double> magnitude;
    VilPlus::vil_magnitude(image, magnitude);
    vil_image_view<double> blueChannel(image.ni(), image.nj(), 1);
    vil_image_view<bool> isGrassMask;
    for (int y = 0; y<image.nj(); y++) {
        for (int x = 0; x<image.ni(); x++) {
            blueChannel(x, y) = image(x, y, 2);
        }
    }
    
    VilLineTracking::LineFilterParameter filterOutputPara;
    
    // edge detection data
    vil_image_view<double> magenitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, magenitude, grad_i, grad_j);
    
    vcl_vector<SGCCalibLine> detectedCalibLines;
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        const int nodeId = calibLine.node();
        
        LinesegmntTrackingParameter para;
        para.lambda_ = sslModel.nodeLambda(nodeId);
        para.mag_threshold_1_ = sslModel.nodeMagnitude(nodeId);
        para.line_width_      = sslModel.nodeWidth(nodeId);
        
        vgl_line_segment_2d<double> initLineSeg;
        vgl_line_segment_2d<double> refinedLineSeg;
        vgl_line_segment_2d<double> finalLineSeg;
        vgl_line_2d<double> finaline;
        bool isFindFinal = false;
        
        bool isSegment = sslModel.projectNode(camera, nodeId, initLineSeg);
        if (!isSegment) {
            continue;
        }
        // initial
        bool isInitGoodEnough = VilLineTracking::isLineSegmentOnEdge(image, magenitude, grad_i, grad_j, initLineSeg, para, 0.6);
        if (!isInitGoodEnough) {
            // refine the initial line segment first         
            bool isRefined = VilLineTracking::refineLineSegmentByLineFilter(image, magenitude, grad_i, grad_j,
                                                                       blueChannel, isGrassMask, para, filterOutputPara,
                                                                       initLineSeg, refinedLineSeg, finaline);
            if (!isRefined) {
             //   printf("Warning: refine segment failed\n");
                continue;
            }
        }
        else
        {
            refinedLineSeg = initLineSeg;
        }
        
        finaline = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
        finalLineSeg = refinedLineSeg;
        isFindFinal = true;
        
        // check if it is a good line segment
        bool isAnEdge = VilLineTracking::isLineSegmentOnEdge(image, magenitude, grad_i, grad_j, finalLineSeg, para);
        if (isAnEdge) {
            vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
            vgl_vector_2d<double> dir2 = finaline.direction();
            double cosAngle = fabs(cos_angle(dir1, dir2));
            
            if (cosAngle > lineDirectionThreshold) {
                calibLine.setDetectedLine(finaline);
                detectedCalibLines.push_back(calibLine);
                
                if(!silent)
                {
                    // draw detected line
                    vil_image_view<vxl_byte> showImage;
                    showImage.deep_copy(image);
                    VilPlus::draw_line(showImage, finaline, VilPlus::green());
                    char buf[1024] = {NULL};
                    sprintf(buf, "center_line_%d.jpg", calibLine.node());
                    VilPlus::vil_save(showImage, buf);
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        return false;
    }
    printf("detected %lu lines\n", detectedCalibLines.size());
    
    // calibration correspondence
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    // 3. tracking center circle in the image
    // check ellipse inside image
    bool hasCenterCircle = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    if (hasCenterCircle) {
        // tracking center circle in image
        ElliseTrackingParameter centerCircleTrackingPara;
        vgl_ellipse_2d<double> centerEllipse;
        wwosCourt.centerCircleBoundingBox(camera, w, h, centerCircleTrackingPara.bounding_box_, 5, 5);
        bool isCenterCircleTracked = VilEllipseTracking::trackingInBoundingBox(image, magnitude, centerCircleTrackingPara, centerEllipse);
        if (isCenterCircleTracked) {
            // line and center circle intersection
            bool isCenterLine = false;
            SGCCalibLine centerLine;
            for (int i = 0; i<detectedCalibLines.size(); i++) {
                if (detectedCalibLines[i].node() == 0) {
                    isCenterLine = true;
                    centerLine = detectedCalibLines[i];
                    break;
                }
            }
            if (isCenterLine) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                bool isIntersect = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), centerEllipse, pt1, pt2, true);
                if (isIntersect) {
                    SoccerGraphCutUtil sgcUtil;
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    for (int i = 0; i<pts_paris.size(); i++) {
                        wld_pts.push_back(pts_paris[i].first);
                        img_pts.push_back(pts_paris[i].second);
                    }
                    
                    printf("find circle line intersection\n");
                }
            }
        }
    }
    
    // 4. find correspondence between line intersections
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sslModel.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(wld_pt);
                        img_pts.push_back(img_pt);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    if (wld_pts.size() < 2) {
        printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        return false;
    }
    
    printf("find %lu intersections\n", wld_pts.size());
    
    if(!silent)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, img_pts, 3, VilPlus::green());
        char buf[1024] = {NULL};
        sprintf(buf, "geometry_intersection.jpg");
        VilPlus::vil_save(showImage, buf);
    }
    
    // 5. camera optimization
    // minimize match error
    vpgl_ptz_camera initPTZ;
    vpgl_ptz_camera finalPTZ;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera, initPTZ);
    if (!isPTZ) {
        printf("refine camera failed: initial camear failed to PTZ\n");
        return false;
    }
    
    VpglPTZCameraUtil::Correspondence corres;
    corres.wld_pts = wld_pts;
    corres.img_pts = img_pts;
    
    bool isRefind = VpglPTZCameraUtil::refineCameraByPtsAndVPs(initPTZ, corres, finalPTZ);
    if (!isRefind) {
        printf("Error: failed to refine the PTZ camera\n");
        return false;
    }
    
    WWoSSoccerCourtUtil::PTZToCamera(finalPTZ.pan(), finalPTZ.tilt(), finalPTZ.focal_length(), finalCamera);
    return true;
}

bool WWoSCameraRefinement::refineCenterLine(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                            vgl_line_segment_2d<double> & centerLine, bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    WWoSSoccerCourt wwosCourt;
    // 1. predict line segment in the image
    SoccerShortLineModel sslModel;
    SGCCalibLine initCalibLine;
    bool isSampled = sslModel.sampleCalibLine(camera, w, h, 0, 0.5, initCalibLine);  // sample every 0.5 meter
    if (!isSampled) {
        return false;
    }
    
    // 2. tracking lines in the image space
    vil_image_view<double> maganitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, maganitude, grad_i, grad_j, false);
    
    SGCCalibLine detectedCalibLine;
    
    const int nodeId = initCalibLine.node();
    
    LinesegmntTrackingParameter seg_para;
    seg_para.lambda_ = sslModel.nodeLambda(nodeId);
    seg_para.mag_threshold_1_ = sslModel.nodeMagnitude(nodeId);
    seg_para.line_width_      = sslModel.nodeWidth(nodeId);
    seg_para.online_pixel_ratio_ = sslModel.nodeOnlineRatio(nodeId);
    
    vgl_line_segment_2d<double> initLineSeg;
    vgl_line_segment_2d<double> refinedLineSeg;
    vgl_line_segment_2d<double> finalLineSeg;
    vgl_line_2d<double> finaline;
    bool isFindFinal = false;
    
    bool isSegment = sslModel.projectNode(camera, nodeId, initLineSeg);
    if (!isSegment) {
        return false;
    }
    // initial
    bool isInitGoodEnough = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, initLineSeg, seg_para, seg_para.online_pixel_ratio_);
    if (!isInitGoodEnough) {
        // refine the initial line segment first
        bool isRefined = VilLineTracking::refineLineSegment(image, maganitude, initLineSeg, seg_para, refinedLineSeg);
        if (!isRefined) {
          //  printf("Warning: refine segment failed\n");
            return false;
        }
    }
    else
    {
        refinedLineSeg = initLineSeg;
    }
    
    // double edge refinement
    if (sslModel.hasDoubleEdge(nodeId)) {
        seg_para.line_width_ /= 2.0;
        isFindFinal = vxl_half_edge_line_tracking::trackingLineFromSegment(image, maganitude, grad_i, grad_j, refinedLineSeg, seg_para, finaline, finalLineSeg);
    }
    else
    {
        finaline = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
        finalLineSeg = refinedLineSeg;
        isFindFinal = true;
    }
    
    if (isFindFinal) {
        // check if it is a good line segment
        bool isAnEdge = VilLineTracking::isLineSegmentOnEdge(image, maganitude, grad_i, grad_j, finalLineSeg, seg_para, seg_para.online_pixel_ratio_);
        if (isAnEdge) {
            vgl_vector_2d<double> dir1 = initCalibLine.projectedLine().direction();
            vgl_vector_2d<double> dir2 = finaline.direction();
            double cosAngle = fabs(cos_angle(dir1, dir2));
            
            if (cosAngle > lineDirectionThreshold) {
                centerLine = finalLineSeg;
                return true;
            }
        }
    }
    return false;
}

bool WWoSCameraRefinement::refineCameraByShortLineAndEllipseTracking_debug(const vil_image_view<vxl_byte> & image, const vpgl_perspective_camera<double> & camera,
                                                     const LineTrackingParameter & para, vpgl_perspective_camera<double> & finalCamera, bool silent)
{
    assert(image.nplanes() == 3);
    
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    printf("1\n");
    WWoSSoccerCourt wwosCourt;
    // 1. predict line segment in the image
    SoccerShortLineModel sslModel;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sslModel.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    printf("2\n");
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    // 2. tracking lines in the image space
    vil_image_view<double> magnitude;
    VilPlus::vil_magnitude(image, magnitude);
    
    // edge detection data
    vil_image_view<double> magenitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, magenitude, grad_i, grad_j);
    
    vcl_vector<SGCCalibLine> detectedCalibLines;
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        const int nodeId = calibLine.node();
        
        if (nodeId != 15) {
            //        continue;
        }
        
        printf("3\n");
        LinesegmntTrackingParameter para;
        para.lambda_ = sslModel.nodeLambda(nodeId);
        para.mag_threshold_1_ = sslModel.nodeMagnitude(nodeId);
        para.line_width_      = sslModel.nodeWidth(nodeId);
        
        vgl_line_segment_2d<double> initLineSeg;
        vgl_line_segment_2d<double> refinedLineSeg;
        vgl_line_segment_2d<double> finalLineSeg;
        vgl_line_2d<double> finaline;
        bool isFindFinal = false;
        
        bool isSegment = sslModel.projectNode(camera, nodeId, initLineSeg);
        if (!isSegment) {
            continue;
        }
        printf("4\n");
        // initial
        bool isInitGoodEnough = VilLineTracking::isLineSegmentOnEdge(image, magenitude, grad_i, grad_j, initLineSeg, para, 0.6);
        printf("5\n");
        if (!isInitGoodEnough) {
            // refine the initial line segment first
            bool isRefined = VilLineTracking::refineLineSegment(image, magenitude, initLineSeg, para, refinedLineSeg);
            printf("6\n");
            if (!isRefined) {
            //    printf("Warning: refine segment failed\n");
                continue;
            }
        }
        else
        {
            refinedLineSeg = initLineSeg;
        }
        
        // double edge refinement
        if (sslModel.hasDoubleEdge(nodeId)) {
            para.line_width_ /= 2.0;
            printf("7\n");
            isFindFinal = vxl_half_edge_line_tracking::trackingLineFromSegment(image, magenitude, grad_i, grad_j, refinedLineSeg, para, finaline, finalLineSeg);
            printf("8\n");
        }
        else
        {
            finaline = vgl_line_2d<double>(refinedLineSeg.point1(), refinedLineSeg.point2());
            finalLineSeg = refinedLineSeg;
            isFindFinal = true;
        }
        
        if (isFindFinal) {
            // check if it is a good line segment
            printf("9\n");
            bool isAnEdge = VilLineTracking::isLineSegmentOnEdge(image, magenitude, grad_i, grad_j, finalLineSeg, para);
            printf("10\n");
            if (isAnEdge) {
                vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
                vgl_vector_2d<double> dir2 = finaline.direction();
                double cosAngle = fabs(cos_angle(dir1, dir2));
                
                if (cosAngle > lineDirectionThreshold) {
                    calibLine.setDetectedLine(finaline);
                    detectedCalibLines.push_back(calibLine);
                    
                    if(!silent)
                    {
                        // draw detected line
                        vil_image_view<vxl_byte> showImage;
                        showImage.deep_copy(image);
                        VilPlus::draw_line(showImage, finaline, VilPlus::green());
                        char buf[1024] = {NULL};
                        sprintf(buf, "center_line_%d.jpg", calibLine.node());
                        VilPlus::vil_save(showImage, buf);
                    }
                }
            }
        }
    }
    printf("11\n");
    
    if (detectedCalibLines.size() < 3) {
        printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        return false;
    }
    printf("detected %lu lines\n", detectedCalibLines.size());
    
    // calibration correspondence
    vcl_vector<vgl_point_2d<double> > wld_pts;
    vcl_vector<vgl_point_2d<double> > img_pts;
    // 3. tracking center circle in the image
    // check ellipse inside image
    bool hasCenterCircle = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    printf("12\n");
    if (hasCenterCircle) {
        // tracking center circle in image
        ElliseTrackingParameter centerCircleTrackingPara;
        vgl_ellipse_2d<double> centerEllipse;
        wwosCourt.centerCircleBoundingBox(camera, w, h, centerCircleTrackingPara.bounding_box_, 5, 5);
        printf("13\n");
        bool isCenterCircleTracked = VilEllipseTracking::trackingInBoundingBox(image, magnitude, centerCircleTrackingPara, centerEllipse);
        printf("14\n");
        if (isCenterCircleTracked) {
            // line and center circle intersection
            bool isCenterLine = false;
            SGCCalibLine centerLine;
            for (int i = 0; i<detectedCalibLines.size(); i++) {
                if (detectedCalibLines[i].node() == 0) {
                    isCenterLine = true;
                    centerLine = detectedCalibLines[i];
                    break;
                }
            }
            if (isCenterLine) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                printf("15\n");
                bool isIntersect = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), centerEllipse, pt1, pt2, true);
                printf("16\n");
                if (isIntersect) {
                    SoccerGraphCutUtil sgcUtil;
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    printf("17\n");
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    printf("18\n");
                    for (int i = 0; i<pts_paris.size(); i++) {
                        wld_pts.push_back(pts_paris[i].first);
                        img_pts.push_back(pts_paris[i].second);
                    }
                    
                    printf("find circle line intersection\n");
                }
            }
        }
    }
    
    printf("19\n");
    // 4. find correspondence between line intersections
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sslModel.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        wld_pts.push_back(wld_pt);
                        img_pts.push_back(img_pt);
                    }
                }
            }
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    if (wld_pts.size() < 2) {
        printf("refine camera failed. only find %lu intersection correspondence.\n", wld_pts.size());
        return false;
    }
    
    printf("20\n");
    printf("find %lu intersections\n", wld_pts.size());
    
    if(!silent)
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_cross(showImage, img_pts, 3, VilPlus::green());
        char buf[1024] = {NULL};
        sprintf(buf, "geometry_intersection.jpg");
        VilPlus::vil_save(showImage, buf);
    }
    
    // 5. camera optimization
    // minimize match error
    vpgl_ptz_camera initPTZ;
    vpgl_ptz_camera finalPTZ;
    bool isPTZ = WWoSSoccerCourtUtil::cameraToPTZ(camera, initPTZ);
    if (!isPTZ) {
        printf("refine camera failed: initial camear failed to PTZ\n");
        return false;
    }
    
    VpglPTZCameraUtil::Correspondence corres;
    corres.wld_pts = wld_pts;
    corres.img_pts = img_pts;
    
    bool isRefind = VpglPTZCameraUtil::refineCameraByPtsAndVPs(initPTZ, corres, finalPTZ);
    if (!isRefind) {
        printf("Error: failed to refine the PTZ camera\n");
        return false;
    }
    
    WWoSSoccerCourtUtil::PTZToCamera(finalPTZ.pan(), finalPTZ.tilt(), finalPTZ.focal_length(), finalCamera);
    printf("21\n");
    return true;
    
}

bool WWoSCameraRefinement::intersectionDetectionByLineAndEllipseTracking(const vil_image_view<vxl_byte> & image,
                                                                         const vpgl_perspective_camera<double> & camera,
                                                                         const LineTrackingParameter & para,
                                                                         vcl_vector<NodeIntersection> & intersections)
{
    const int w = image.ni();
    const int h = image.nj();
    
    const int calibLineNum = 15;
    //    const int lineWidth = para.line_thick_ness_;
    const int lineMinPixelNum = 20;
    const int boundaryThreshold = 150;
    const double lineDirectionThreshold = cos(30.0/180*3.14);
    
    SoccerGraphCutUtil sgcUtil;
    vcl_vector<SGCCalibLine> calibLines;
    for (int i = 0; i<calibLineNum; i++) {
        SGCCalibLine calibLine;
        bool isSampled = sgcUtil.sampleCalibLine(camera, w, h, i, 0.5, calibLine);  // sample every 0.5 meter
        if (isSampled) {
            calibLines.push_back(calibLine);
        }
    }
    
    if (calibLines.size() < 3) {
        printf("refine camera failed. only %lu calibration lines.\n", calibLines.size());
        return false;
    }
    
    vil_image_view<double> magnitude;
    VilPlus::vil_magnitude(image, magnitude);
    
    double magThreshold[calibLineNum] = { 0.05, 0.04, 0.04, 0.04, 0.04,
        0.02, 0.02, 0.02, 0.02, 0.02,
        0.02, 0.02, 0.02, 0.02, 0.02};
    double lambdaTHreshold[calibLineNum] = {2, 2.0, 2.0, 2, 2,
        1.5, 1.5, 1.5, 1.5, 1.5,
        1.5, 1.5, 1.5, 1.5, 1.5};
    double lineWidthes[calibLineNum] = {30, 20, 20, 20, 20,
        20, 20, 20, 20, 10,
        20, 30, 30, 20, 20};
    
    
    vcl_vector<SGCCalibLine> detectedCalibLines;
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    for (int i = 0; i<calibLines.size(); i++) {
        SGCCalibLine calibLine = calibLines[i];
        
        // draw line segment in the image
        maskImage.fill(0);
        VilPlus::draw_segment(maskImage, calibLine.point1(), calibLine.point2(), VilPlus::white(), lineWidthes[calibLine.node()]);
        {
            char buf[1024] = {NULL};
            vil_image_view<vxl_byte> showImage;
            showImage.deep_copy(image);
            
            VilPlus::draw_segment(showImage, calibLine.point1(), calibLine.point2(), VilPlus::white(), lineWidthes[calibLine.node()]);
            
            sprintf(buf, "mask_%d.jpg", calibLine.node());
            VilPlus::vil_save(showImage, buf);
        }
        
        AverageMagnitudeParameter para;
        para.direction_ = calibLine.scan_direction();
        para.mag_threshold_ = magThreshold[calibLine.node()];
        para.lambda_ = lambdaTHreshold[calibLine.node()];
        
        // detect edge point candidate
        vcl_vector<vgl_point_2d<double> > side_edge_pts1;
        vcl_vector<vgl_point_2d<double> > side_edge_pts2;
        vxl_half_edge_line_tracking::detectLinePixelByAverageGradient(magnitude, maskImage, para, side_edge_pts1, side_edge_pts2);
        
        // filter side edge pts
        
        
        // hough line detection for two edge line
        VxlHoughParameter houghPara;
        houghPara.maxLineNum_ = 1;
        houghPara.minPixeNum_ = lineMinPixelNum;
        vgl_line_2d<double> dumpLine;
        vcl_vector<vgl_point_2d<double> > inlier_pts1;
        vcl_vector<vgl_point_2d<double> > inlier_pts2;
        VxlHoughLine::detectOneLine(side_edge_pts1, w, h, houghPara, dumpLine, inlier_pts1);
        VxlHoughLine::detectOneLine(side_edge_pts2, w, h, houghPara, dumpLine, inlier_pts2);
        
        // estimate center line from edge line
        vgl_line_2d<double> centerLine;
        vcl_vector<vgl_point_2d<double> > center_pts;
        bool isFindCenterLine = false;
        if (inlier_pts1.size() > inlier_pts2.size()) {
            isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts1, inlier_pts2, center_pts, centerLine);
        }
        else
        {
            isFindCenterLine = vxl_half_edge_line_tracking::estimateCenterLine(inlier_pts2, inlier_pts1, center_pts, centerLine);
        }
        if (isFindCenterLine) {
            // check the directon different
            vgl_vector_2d<double> dir1 = calibLine.projectedLine().direction();
            vgl_vector_2d<double> dir2 = centerLine.direction();
            double cosAngle = fabs(cos_angle(dir1, dir2));
            
            if (cosAngle > lineDirectionThreshold) {
                calibLine.setDetectedLine(centerLine);
                detectedCalibLines.push_back(calibLine);
                
                // draw single line
                {
                    vil_image_view<vxl_byte> showImage;
                    showImage.deep_copy(image);
                    //  VilPlus::draw_line(showImage, centerLine, VilPlus::blue());
                    VilPlus::draw_dot(showImage, center_pts, VilPlus::blue());
                    
                    char buf[1024] = {NULL};
                    sprintf(buf, "center_line_%d.jpg", calibLine.node());
                    VilPlus::vil_save(showImage, buf);
                }
            }
        }
    }
    
    if (detectedCalibLines.size() < 3) {
        printf("refine camera failed. only %lu lines were detected in image.\n", detectedCalibLines.size());
        return false;
    }
    {
        // draw line
        vcl_vector<vgl_line_2d<double> > lines;
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            lines.push_back(detectedCalibLines[i].detectedLine());
        }
        
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(image);
        VilPlus::draw_lines(showImage, lines, VilPlus::green());
        VilPlus::vil_save(showImage, "detected_line.jpg");
    }
    
    // find intersections
    for (int i = 0; i<detectedCalibLines.size(); i++) {
        for (int j = i+1; j<detectedCalibLines.size(); j++) {
            int node1 = detectedCalibLines[i].node();
            int node2 = detectedCalibLines[j].node();
            
            // intersection should inside/close to image
            if (sgcUtil.isValidNodePair(node1, node2)) {
                vgl_point_2d<double> wld_pt;
                vgl_point_2d<double> img_pt;
                bool isIntersect = detectedCalibLines[i].intersection(detectedCalibLines[j], wld_pt, img_pt);
                if (isIntersect) {
                    if (vgl_inside_image(img_pt, w, h,boundaryThreshold)) {
                        NodeIntersection intersction;
                        intersction.wld_pt_ = wld_pt;
                        intersction.img_pt_ = img_pt;
                        intersction.node_1_ = node1;
                        intersction.node_2_ = node2;
                        intersections.push_back(intersction);
                        printf("intersection node number is %d %d\n\n", node1, node2);
                    }
                }
            }
        }
    }
    
    
    // check ellipse inside image
    bool isSoccerInside = WWoSSoccerCourt::isEllipseInsideImage(camera, w, h, para.circle_ratio_);
    if (isSoccerInside) {
        // find center line
        bool isCenterLine = false;
        SGCCalibLine centerLine;
        for (int i = 0; i<detectedCalibLines.size(); i++) {
            if (detectedCalibLines[i].node() == 0) {
                isCenterLine = true;
                centerLine = detectedCalibLines[i];
                break;
            }
        }
        if (isCenterLine) {
            // detect ellipse
            vil_image_view<vxl_byte> nonEllipseMaskImage = vil_image_view<vxl_byte>(w, h, 3);
            nonEllipseMaskImage.fill(0);
            maskImage.fill(0);
            
            AverageMagnitudeParameter para;
            para.mag_threshold_ = 0.05;
            para.lambda_    = 2.0;
            para.direction_ = 1;
            
            // get bounding box
            WWoSSoccerCourt wwosCourt;
            wwosCourt.overlayEllipseBoundingBox(camera, maskImage, VilPlus::white(), 5, 5);
            
            vcl_vector<vgl_point_2d<double> > outsideUpPixels;
            vcl_vector<vgl_point_2d<double> > outsideDownPixels;
            vxl_half_edge_line_tracking::detectEllipsePixelByAverageGradient(magnitude, maskImage, nonEllipseMaskImage, para, outsideUpPixels, outsideDownPixels);
            
            double threshold  = 2.0;
            double fail_ratio = 0.001;
            vgl_ellipse_2d<double> ellipse;
            vcl_vector<vgl_point_2d<double> > inliers;
            int maxIter = 2000;
            bool isFit = VrelPlus::fit_ellipse_RANSAC(outsideUpPixels, outsideDownPixels, threshold,fail_ratio, ellipse, inliers, maxIter);
            if (isFit) {
                vgl_point_2d<double> pt1;
                vgl_point_2d<double> pt2;
                bool isOk = VglPlus::lineEllipseIntersection(centerLine.detectedLine(), ellipse, pt1, pt2);
                if (isOk) {
                    // add center line ellipse intersection
                    vcl_vector<vcl_pair<vgl_point_2d<double>, vgl_point_2d<double> > > pts_paris;
                    sgcUtil.lineCircleIntersectionPairs(camera, pt1, pt2, pts_paris);
                    for (int i = 0; i<pts_paris.size(); i++) {
                        NodeIntersection intersction;
                        intersction.wld_pt_ = pts_paris[i].first;
                        intersction.img_pt_ = pts_paris[i].second;
                        intersction.node_1_ = 0;
                        intersction.node_2_ = 100;
                        intersections.push_back(intersction);                 
                    }
                    printf("add center line, circle intersection. \n");
                }
            }
        }
    }
    
    return intersections.size() >= 4;
}






