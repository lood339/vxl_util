//
//  basketballCourt.h
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VpglPtzOpt__basketballCourt__
#define __VpglPtzOpt__basketballCourt__


#include <vxl_config.h>
#include <vcl_vector.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vgl/vgl_line_segment_3d.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vgl/vgl_transform_2d.h>

class BasketballCourt
{
public:
    BasketballCourt();
    virtual ~BasketballCourt();
    
    virtual void courtImage(vil_image_view<vxl_byte> &image) = 0;
    virtual void courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image) = 0;
    virtual void courtImageWithLogo(vil_image_view<vxl_byte> &image, int logo_gray) = 0;
    virtual void getWeightImageWithLogo(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt) = 0;
    virtual void getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt) = 0;
    virtual void overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image) = 0;
    virtual vgl_transform_2d< double > imageToWorld() = 0;
    virtual vcl_string name() = 0;
};


class CMUBasketballCourt:public BasketballCourt
{
public:
    CMUBasketballCourt();
    ~CMUBasketballCourt();
    
    
    virtual void courtImage(vil_image_view<vxl_byte> &image);

    // gray court image
    virtual void courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image);    
  
    
    virtual void getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt);
    
    virtual void overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image);
    
    void overlayPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image);
    
    virtual vgl_transform_2d< double > imageToWorld();
    
    virtual vcl_string name() {return vcl_string("CMUBasketballCourt");}
    
    virtual void courtImageWithLogo(vil_image_view<vxl_byte> &image, int logo_gray);
    virtual void getWeightImageWithLogo(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt);

    
    void getProjectedPoints(const vpgl_perspective_camera<double> &camera, int width, int height, vcl_vector<vgl_point_2d<double> > &outPts);
    
    
    
    // unit in meter
    static vcl_vector<vgl_point_2d<double> > getCourtPoints();
    
private:
    vcl_vector< vgl_line_segment_2d< double > > getLineSegments();
    vcl_vector< vgl_line_segment_2d< double > > getDivisionLine();
    vcl_vector< vgl_point_2d<double> > getPointsOnEdges();
    
};


class DisneyWorldBasketballCourt:public BasketballCourt
{
    vil_image_view<vxl_byte> m_logoImage;            //image space
    vcl_vector<vgl_point_2d<double> > m_logoPoints;  // world space, unit meter
    
public:
    DisneyWorldBasketballCourt();
    ~DisneyWorldBasketballCourt();
    
    void setDefaultLogoImage();
    // transform: from logo image to court image
    void setLogoImage(const vil_image_view<vxl_byte> &logoImage, const vgl_h_matrix_2d<double> &transform);
    
    void setLogoPoints(const vgl_transform_2d<double> & transform);
    
    void setLogoPoints(const vcl_vector<vgl_point_2d<double> > & points, const vgl_transform_2d<double> & transform);

    virtual void courtImage(vil_image_view<vxl_byte> &image);
    
    virtual void courtImageWithLogo(vil_image_view<vxl_byte> &image, int logo_gray = 60);
    
    virtual void courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image);
    
    virtual void getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt);
    
    virtual void getWeightImageWithLogo(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt);
    
    virtual void overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image);
    
    // visualization purpose
    void overlayAllLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image, const vcl_vector<vxl_byte> & color);
    
    void getAllLines(const vpgl_perspective_camera<double> & camera, int width, int height, vcl_vector< vgl_line_segment_2d< double > > &lines);
    
    void overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image, const vcl_vector< vxl_byte >& colour);
    
    void projectLines(const vpgl_proj_camera<double> & camera, vil_image_view<vxl_byte> & image);
    
    // alpha: width X height, 255 for inside court, 0 for outside court
    void getProjectedCourtArea(const vpgl_perspective_camera<double> & camera, int width, int height, vil_image_view<vxl_byte> & alpha);
    
    // visualize court area in the image space in the topview area
    // output alpha size is 1268 * 740, channel num 1
    bool getCourtArareInTopview(const vpgl_perspective_camera<double> & camera, int width, int height, vil_image_view<vxl_byte> & alpha);
    
    // project an rectangular area to camera image
    // startP = vgl_point_2d<int>(30, 30)
    // endP = vgl_point_2d<int>(70 * 2 + 94 *12 - 30, 70 * 2 + 50 * 12 - 70)
    void getProjectedCourtArea(const vpgl_perspective_camera<double> & camera, int width, int height, vil_image_view<vxl_byte> & alpha,
                               const vgl_point_2d<int> &startP, const vgl_point_2d<int> & endP);
    
    // project topview image by a camera
    bool projectTopviewImage(const vil_image_view<vxl_byte> &topview, const vpgl_perspective_camera<double> & camera, int width, int height,
                             vil_image_view<vxl_byte> & outImage, int threshold);
    
    void overlayPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image);
    
    void overlayKeyPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image);
    
    void getImageKeyPoints(const vpgl_perspective_camera<double> & camera, int width, int height, vcl_vector<vgl_point_2d<double> > & pts);
    void getWorldImageKeyPoints(const vpgl_perspective_camera<double> & camera, int width, int height,
                                vcl_vector<vgl_point_2d<double> > & wld_pts, vcl_vector<vgl_point_2d<double> > & img_pts);
    
    // topview image (pixels) to world (meters)
    virtual vgl_transform_2d< double > imageToWorld();
    
    virtual vcl_string name() {return vcl_string("DisneyWorldBasketballCourt");}
    
    void getPorjectedPointsEdgeOrientation(const vpgl_perspective_camera<double> &camera, int width, int height, vcl_vector<vgl_point_2d<double> > &outPts,
                                           vcl_vector<vnl_vector_fixed<double, 2> > &outEdgeOrientation);

    
    static vcl_vector<vgl_point_2d<double> > getCourtPoints();
    static vcl_vector<vgl_point_2d<double> > getCalibPoints();     // points used in generate 3D/2D correspondence
    static vcl_vector<vgl_point_2d<double> > getCourtKeyPoints();  // points used in re-calibration
    static vcl_vector<vgl_point_2d<double> > getPatchMatchingPoints();   // points used on patch matching
    
    static void projectCourtPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                   vcl_vector<vgl_point_2d<double> > & points_world,     //meter
                                   vcl_vector<vgl_point_2d<double> > & points_court_image, //pixel
                                   vcl_vector<vgl_point_2d<double>> & points_camera_image,
                                   int threshold); // threshold away from the image boundary
    
    static void projectCalibPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                   vcl_vector<vgl_point_2d<double> > & points_world,     //meter
                                   vcl_vector<vgl_point_2d<double> > & points_court_image, //pixel
                                   vcl_vector<vgl_point_2d<double> >  & points_camera_image,
                                   int threshold); // threshold away from the image boundary
    
    // project camare to an image (may out of image space)
    static void projectCourtPoints(const vpgl_perspective_camera<double> & camera,
                                   vcl_vector<vgl_point_2d<double> > & pts_world,
                                   vcl_vector<vgl_point_2d<double> > & pts_image);
    
    // project camera to an image (must inside of image space)
    static void projectCourtPoints(const vpgl_perspective_camera<double> & camera, int width, int height,
                                   vcl_vector<vgl_point_2d<double> > & pts_world,
                                   vcl_vector<vgl_point_2d<double> > & pts_image);
    
    // back project features positions in projected topview image to get their location in topview (world coordinate) image
    // positions: feature positions in warped keyframe image
    // positions_world: meter, z = 0
    void backProjectpoints(const vpgl_perspective_camera<double> &camera, const vcl_vector<vgl_point_2d<double> > & positions,
                           vcl_vector<vgl_point_2d<double> > & positions_world, bool insideCourt,
                           int width = 1280, int height = 720);
    // topview image to image (used as keyframes) space
    bool topviewToCameraHomography(const vpgl_perspective_camera<double> &camera, vgl_h_matrix_2d<double> &H, int width, int height);
    
    bool worldToCameraHomography(const vpgl_perspective_camera<double> &camera, vgl_h_matrix_2d<double> &H, int width, int height);
    
    // estimate alignment quality by SSD from topview to image
    // SSD: sum of squared difference
    // SAD: sum of absolute difference
    void alignmentQuality(const vil_image_view<vxl_byte> &topview, const vil_image_view<vxl_byte> &image,
                          const vcl_vector<vpgl_perspective_camera<double> > & cameras,
                          vcl_vector<double> & SSDs, vcl_vector<double> & SADs);
    
    // estimate alignment quality by SSD from topview to image for a single camera
    void alignmentQuality(const vil_image_view<vxl_byte> &topview, const vil_image_view<vxl_byte> &image,
                          const vpgl_perspective_camera<double> & camera,
                          double & SSD, double & SAD, bool isAverage);
    
    // camera is estimated, not accurate
    
    bool isBlurImage(const vil_image_view<vxl_byte> & topview, const vil_image_view<vxl_byte> & image,
                     const vcl_vector<vpgl_perspective_camera<double> > & camera);
    
    //  corners1, corners2: projected court corners in the query image (usually, one for keyframe, one for query image)
    //  keyframe and query image have same size (destWidth, destHeight)
    static void commonCourtCorners(const vpgl_perspective_camera<double> &firstCamera, const vpgl_perspective_camera<double> & secondCamera,
                                   int destWidht, int destHeight, bool insideImage,
                                   vcl_vector<vgl_point_2d<double> > & firstCorners, vcl_vector<vgl_point_2d<double> > & secondCorners);
    
    // keyFrameCamera: camera in first image
    // H: homography from first image to second image
    // camera: camera in second image
    static bool getCameraFromSequentialHomo(const vpgl_perspective_camera<double> & keyFrameCamera, const vgl_h_matrix_2d<double> & H,
                                            vpgl_perspective_camera<double> & camera);
    
    // query image camera by matching from key frame to query frame
    static bool getCameraFromSequentialMatching(const vcl_vector<vgl_point_2d<double> > & keyframePts,
                                                const vcl_vector<vgl_point_2d<double> > & queryFramePts,
                                                const vpgl_perspective_camera<double> & keyFrameCamera,
                                                vpgl_perspective_camera<double> & camera);
    
    
    static vcl_vector< vgl_line_segment_2d< double > > getLineSegments();
private:
    vcl_vector< vgl_line_segment_2d< double > > getAllLineSegments();
    vcl_vector< vgl_line_segment_2d< double > > getDivisionLine();
    vcl_vector< vgl_line_segment_2d< double > > getLogoAreaLineSegments(int width);
    
    void getPointsDirectionOnEdges(vcl_vector<vgl_point_2d<double> > &pts, vcl_vector<vnl_vector_fixed<double, 4> > &p_to_q);
};

// court based on topviewimage
class WaltDisneyWorldTopviewCourt:public BasketballCourt
{
private:
    vil_image_view<vxl_byte> topviewImage_;
    vcl_vector<vgl_point_2d<double> > wldPts_;
    vcl_vector<vgl_point_2d<double> > imgPts_;  // selected patch positions

    void setCorrespondences();
    
public:
    WaltDisneyWorldTopviewCourt( const vil_image_view<vxl_byte> &topviewImage);
    ~WaltDisneyWorldTopviewCourt();
    
    virtual void courtImage(vil_image_view<vxl_byte> &image);
    virtual void courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image);
    virtual void courtImageWithLogo(vil_image_view<vxl_byte> &image, int logo_gray);
    virtual void getWeightImageWithLogo(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt);
    virtual void getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt);
    virtual void overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image);
    virtual vgl_transform_2d< double > imageToWorld();
    virtual vcl_string name();
    
    // project image position in topview by H,
    void projectPoints(const vgl_h_matrix_2d<double> &H, int width, int height, int patchWidth,
                                   vcl_vector<vgl_point_2d<double> > & outImgPts, vcl_vector<vgl_point_2d<double> > & outWorldPts);
    
    vcl_vector<vgl_point_2d<double> > getPatchPositionsInWorld(){return  wldPts_;}
    vcl_vector<vgl_point_2d<double> > getPatchPositionsInImage(){return  imgPts_;}
    
};


#endif /* defined(__VpglPtzOpt__basketballCourt__) */
