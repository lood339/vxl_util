//
//  wwos_soccer_court.h
//  OnlineStereo
//
//  Created by jimmy on 1/31/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__wwos_soccer_court__
#define __OnlineStereo__wwos_soccer_court__

#include "SoccerCourt.h"

#include "WWoSCourtLine.h"
#include <vgl/vgl_box_2d.h>
#include <vgl/vgl_conic.h>



// Disney Wild World of Sports court
// width 118, height 70
class WWoSSoccerCourt: public SoccerCourt
{
    private:
    double field_width_;  // unit is yard
    double field_heigh_;
    
    public:
    
    WWoSSoccerCourt(double w = 118, double h = 70);
    ~WWoSSoccerCourt();
    
    virtual void courtImage(vil_image_view<vxl_byte> &image);
    void courtRGBImage(vil_image_view<vxl_byte> &image);
    // line_gray:   180
    // ground_gray: 120
    virtual void courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image);
    // lineWidth  : 5-10 * 2
    // gauss_sigma: around 100
    virtual void getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height,
                                int lineWidth, double gauss_sigma, vil_image_view<double> &wt);    

    
    void projectCourtPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                            vcl_vector<vgl_point_2d<double> > & world_pts,
                            vcl_vector<vgl_point_2d<double> > & overview_image_pts,
                            vcl_vector<vgl_point_2d<double> > & camera_image_pts,
                            int threshold);
    
    void projectCourtPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                            vcl_vector<vgl_point_3d<double> > & world_pts,
                            vcl_vector<vgl_point_2d<double> > & camera_image_pts,
                            int threshold) const;
    
    // calibration points as well as point located in the center of line segment
    void projectCourtSegmentPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                   vcl_vector<vgl_point_3d<double> > & world_pts,
                                   vcl_vector<vgl_point_2d<double> > & camera_image_pts,
                                   double sampleLength, // meter
                                   int threshold) const;
    //
    void getProjectedCourtArea(const vpgl_perspective_camera<double> & camera,
                               int width, int height, vil_image_view<vxl_byte> & alpha,
                               const vgl_point_2d<int> & startP, const vgl_point_2d<int> & endP);
    
    // cover border area of the soccer court
    void getProjectedCourtAreaCoverBorder(const vpgl_perspective_camera<double> & camera,
                                          int width, int height,
                                          vil_image_view<vxl_byte> & alpha);
    
    // get an alpha, mapping the court are in image to topview image
    // size of alpha is the same as topview image (default size)
    bool getCourtArareInTopview(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & alpha);
                               
    
    
    //   with some default parameter
    //   can only be used in wwos data
    void getProjectedCourtArea(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & alpha);
    
    // assume the
    bool getImage2TopviewImageHomography(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                         vgl_h_matrix_2d<double> &H);
    bool getCameraFromSequentialHomography(const vpgl_perspective_camera<double> & keyFrameCamera, const vgl_h_matrix_2d<double> & H,
                                           vpgl_perspective_camera<double> & camera);
    
    // H: from world coordinate to image coordinate
    bool getCameraFromHomography(const vgl_h_matrix_2d<double> & H, int imageW, int imageH, vpgl_perspective_camera<double> & camera);
    
    bool getRelativeHomography(const vpgl_perspective_camera<double> & camera1, const vpgl_perspective_camera<double> & camera2, int imageW, int imageH,
                               vgl_h_matrix_2d<double> & H);
    // get homography from 3D world to image
    bool getHomographyFromCamera(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, vgl_h_matrix_2d<double> & H);
        
    
    // overview image (pixels) to world (meters)
    virtual vgl_transform_2d< double > imageToWorld();
    // warp image to topview image, homography
    vgl_h_matrix_2d<double> imageToTopview(const vpgl_perspective_camera<double> & camera);
    
    // warp images from left and right stationary camera to topview image
    // this function is very specific
    bool warpStationaryCameraToTopviewImage(const vil_image_view<vxl_byte> & leftImage,
                                            const vil_image_view<vxl_byte> & rightImage,
                                            const vpgl_perspective_camera<double> & leftCamera,
                                            const vpgl_perspective_camera<double> & rightCamera,
                                            const vil_image_view<vxl_byte> & topviewImage,
                                            vil_image_view<vxl_byte> & warpedImage);
    
    virtual vcl_string name(){return vcl_string("WWoSSoccerCourt");}
    
    //  overview image of the soccer field with board
    int overviewImageWidth();
    int overviewImageHeight();    
    
    // sample lines from the court
    // sampleUnit: meter
    // assume lines are horizontal or vertical
    // sampleUnit: about 1.0 meter for soccer field
    void sampleLines(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                     double sampleUnit, vcl_vector<vcl_vector<LinePatchSampler> > & samplers);
    
    // sample white line from the court
    // sampleUnit: meter    
    // dense sample: sampleUnit 0.5 meter
    void sampleWhiteLineSegment(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                   double sampleUnit, vcl_vector<WWOSSoccerLinePair> & lineSegments);
    
    // cener circle as a conic
    vgl_conic<double> getCenterCircle(void);
    vgl_conic<double> getLeftCircle(void);
    //       0
    //    2     3
    //       1
    vcl_vector<vgl_point_2d<double> > centerCircleWldPts();
    void centerLineWld(vgl_line_3d_2_points<double> & line);
    vcl_vector<vgl_point_2d<double> > leftCircleIntersection();
    vcl_vector<vgl_point_2d<double> > centerCircleIntersection();
    vcl_vector<vgl_point_2d<double> > rightCircleIntersection();
    
    // world coordinate (meter) to overview image
    static vgl_point_2d<double> World2Image(const vgl_point_2d<double> &p);
    static void overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image);
    static void overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image, const vcl_vector<vxl_byte> & colour, int lineWidth = 1);
    static void overlayLines(const vgl_h_matrix_2d<double> & H, vil_image_view<vxl_byte> & image, const vcl_vector<vxl_byte> & colour, int lineWidth = 1);
    static void overlayPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image);
    
    void overlayLinesOnBackground(const vpgl_perspective_camera<double> & camera,
                                const vil_image_view<vxl_byte> & image,
                                vil_image_view<vxl_byte> & outImage,
                                const vcl_vector<vxl_byte> & colour, int line_thickness);
    
    // ratio: part of ellipse is inside image
    static bool isEllipseInsideImage(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, double ratio = 0.99);    
    
    static bool isPenaltyEllipseInsideImage(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, bool isLeft);
    
    // boardWidth, boardHeight expand the bournding box
    static void overlayEllipseBoundingBox(const vpgl_perspective_camera<double> & camera,
                                          vil_image_view<vxl_byte> & mask,
                                          const vcl_vector<vxl_byte> & colour, int boardWidth = 2, int boardHeight = 5);
    
    static bool centerCircleBoundingBox(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                        vgl_box_2d<int> & box, int extendBoundaryWidth = 5, int extendBoundardHeight = 5);
    
    static bool penaltyEllipseBoundingBox(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                          vgl_box_2d<int> & box, bool isLeft, int extendBoundaryWidth = 5, int extendBoundardHeight = 5);
    
    
    static void overlayPenaltyEllipseBoundingBox(const vpgl_perspective_camera<double> & camera,
                                                 vil_image_view<vxl_byte> & mask,
                                                 const vcl_vector<vxl_byte> & colour, bool isLeft,
                                                 int boardWidth = 5, int boardHeight = 10);
    
    static vcl_vector<vgl_point_2d<double> > getFieldCalibrationPoints(double w = 118, double h = 70);
    
    
    
  
    
    // project topview image to camera image
    bool projectTopviewImage(const vil_image_view<vxl_byte> &topview, const vpgl_perspective_camera<double> & camera,
                             int width, int height, vil_image_view<vxl_byte> & outImage, int threshold);
    
    // overlay topview image to the camera image
    void alignmentQuality(const vil_image_view<vxl_byte> &topview, const vil_image_view<vxl_byte> &image,
                          const vpgl_perspective_camera<double> & camera,
                          double & SSD, double & SAD, bool isAverage);
    
    // white lines that hard directly detected in the image
    vcl_vector<vgl_line_segment_2d<double> > inferencedWhiteLines();
    
    // for projected lines
    void getAllLines(const vpgl_perspective_camera<double> & camera, int width, int height, vcl_vector< vgl_line_segment_2d< double > > &lines);
    void getAllLines(vcl_vector<vgl_line_segment_3d<double> > & segments) const;
    void getImageKeyPoints(const vpgl_perspective_camera<double> & camera, int width, int height, vcl_vector<vgl_point_2d<double> > & pts);
    void getWorldImageKeyPoints(const vpgl_perspective_camera<double> & camera, int width, int height,
                                vcl_vector<vgl_point_2d<double> > & wld_pts_out, vcl_vector<vgl_point_2d<double> > & img_pts);   
    
     private:
    
    static vcl_vector<vgl_point_2d<double> > getFieldHockeyPoints(const double width = 118, const double height = 70);
    
   
    // supplementary lines from field hockey, bounary of audience area
    static vcl_vector< vgl_line_segment_2d< double > > getSupplementaryLineSegments(const double width = 118, const double height = 70);
    
private:
    // straight lines for sampling and line tracking
    vcl_vector<vgl_line_segment_2d<double> > straightLines();
    
    // white lines for line detection
    vcl_vector<vgl_line_segment_2d<double> > whiteLines();
    
    // center ellipse line segments
    vcl_vector<vgl_line_segment_2d<double> > ellipseLines();
    vcl_vector<vgl_line_segment_2d<double> > penaltyEllipseLines(bool isLeft);
    
    // line intersections used for calibration
    vcl_vector<vgl_point_2d<double> > calibration_points();
};


#endif /* defined(__OnlineStereo__wwos_soccer_court__) */
