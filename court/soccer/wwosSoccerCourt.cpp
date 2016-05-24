//
//  wwos_soccer_court.cpp
//  OnlineStereo
//
//  Created by jimmy on 1/31/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "wwosSoccerCourt.h"
#include <vgl/vgl_line_2d.h>
#include <vnl/vnl_math.h>
#include <vicl/vicl_line_segment.h>
#include <vicl/vicl_colours.h>
#include <vil/algo/vil_gauss_filter.h>
#include "vxl_plus.h"
#include <vgl/vgl_point_3d.h>
#include "vil_plus.h"
#include "vpgl_plus.h"
#include "lengthUnit.h"
#include <vnl/vnl_inverse.h>
#include <vgl/vgl_distance.h>

/*****************************    WWoSCourt            ****************************/
WWoSSoccerCourt::WWoSSoccerCourt(double w, double h)
{
    field_width_ = w;
    field_heigh_ = h;
    
}
WWoSSoccerCourt::~WWoSSoccerCourt()
{
    
}

void WWoSSoccerCourt::courtImage(vil_image_view<vxl_byte> &image)
{
    this->courtImage(2, 180, 120, image);
}

void WWoSSoccerCourt::courtRGBImage(vil_image_view<vxl_byte> &image)
{
    vil_image_view<vxl_byte> grayImage;
    this->courtImage(grayImage);
    
    assert(grayImage.nplanes() == 1);
    
    image = vil_image_view<vxl_byte>(grayImage.ni(), grayImage.nj(), 3);
    for (int j = 0; j<grayImage.nj(); j++) {
        for (int i = 0; i<grayImage.ni(); i++) {
            image(i, j, 0) = grayImage(i, j);
            image(i, j, 1) = grayImage(i, j);
            image(i, j, 2) = grayImage(i, j);
        }
    }
}

void WWoSSoccerCourt::courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image)
{
    int w = field_width_ * 36 / 3 + 70 * 2;
    int h = field_heigh_  * 36 / 3 + 70 * 2;
    
    image = vil_image_view<vxl_byte>(w, h, 1);
    image.fill(ground_gray);
    
    const double meter_to_3inch = 0.3048 * 3.0;
    double imageH = field_heigh_  * 36 / 3;
    
    {
        // supplemental markings
        vcl_vector<vxl_byte> black; // supplemental marking color is darker than grass
        black.push_back(100);
        vcl_vector< vgl_line_segment_2d<double> > supplementalMarkings = this->getSupplementaryLineSegments(field_width_, field_heigh_);
        for (int i = 0; i<supplementalMarkings.size(); i++) {
            vgl_point_2d<double> p1 = supplementalMarkings[i].point1();
            vgl_point_2d<double> p2 = supplementalMarkings[i].point2();
            
            p1.set(p1.x()/meter_to_3inch * 12, p1.y()/meter_to_3inch * 12);
            p2.set(p2.x()/meter_to_3inch * 12, p2.y()/meter_to_3inch * 12);
            
            //flip y
            p1.set(p1.x(), imageH - p1.y());
            p2.set(p2.x(), imageH - p2.y());
            
            //add 70 + 70
            p1.set(p1.x() + 70, p1.y() + 70);
            p2.set(p2.x() + 70, p2.y() + 70);
            
            //draw to the image
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), black, lineWidth/2);
        }
    }
    // main marking
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(field_width_, field_heigh_);
    
    //meter change to inch (pixel)  / 0.3048
    vcl_vector< vxl_byte > white;
    white.push_back(line_gray);
    
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> p1 = markings[i].point1();
        vgl_point_2d<double> p2 = markings[i].point2();
        
        p1.set(p1.x()/meter_to_3inch * 12, p1.y()/meter_to_3inch * 12);
        p2.set(p2.x()/meter_to_3inch * 12, p2.y()/meter_to_3inch * 12);
        
        //flip y
        p1.set(p1.x(), imageH - p1.y());
        p2.set(p2.x(), imageH - p2.y());
        
        //add 70 + 70
        p1.set(p1.x() + 70, p1.y() + 70);
        p2.set(p2.x() + 70, p2.y() + 70);
        
        //draw to the image
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), white, lineWidth);
    }
    
}

void WWoSSoccerCourt::getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth,
                                     double gauss_sigma, vil_image_view<double> &wt)
{
    vil_image_view<vxl_byte> wt_black_white = vil_image_view<vxl_byte>(width, height, 1);
    wt_black_white.fill(0);
    
    // markings
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(field_width_, field_heigh_);
    vcl_vector< vxl_byte > color_white;
    color_white.push_back(255);
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        vicl_overlay_line_segment(wt_black_white, vgl_line_segment_2d< double >( start, stop ), color_white, lineWidth);
    }
    
    // supplemental markings
    vcl_vector<vgl_line_segment_2d<double> > supplementalMarkings = this->getSupplementaryLineSegments(field_width_, field_heigh_);
    for ( unsigned int i = 0; i < supplementalMarkings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( supplementalMarkings[i].point1().x(), supplementalMarkings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( supplementalMarkings[i].point2().x(), supplementalMarkings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop  = vgl_point_2d< double >(camera.project(p2));
        
        vicl_overlay_line_segment(wt_black_white, vgl_line_segment_2d< double >( start, stop ), color_white, lineWidth);
    }
    
    
    wt = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(gauss_sigma);
    vil_gauss_filter_5tap(wt_black_white, wt, params);
    
    for (int y = 0; y<wt.nj(); y++) {
        for (int x = 0; x<wt.ni(); x++) {
            wt(x, y, 0) /= 255.0;
        }
    }
}

void WWoSSoccerCourt::projectCourtPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                         vcl_vector<vgl_point_2d<double> > & points_world,
                                         vcl_vector<vgl_point_2d<double> > & points_court_image,
                                         vcl_vector<vgl_point_2d<double> > & points_camera_image,
                                         int threshold)
{
    assert(points_world.size() == 0);
    assert(points_court_image.size() == 0);
    assert(points_camera_image.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > courtPts = WWoSSoccerCourt::getFieldCalibrationPoints(field_width_, field_heigh_);
    
    for (int i = 0; i<courtPts.size(); i++) {
        vgl_homg_point_3d<double> p(courtPts[i].x(), courtPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        
        if (vgl_inside_image(q, width, height, threshold))
        {
            points_world.push_back(vgl_point_2d<double>(p.x(), p.y()));
            points_camera_image.push_back(q);
        }
    }
    
    WWoSSoccerCourt court;
    
    // from world to image coordinate
    vgl_transform_2d<double> imageToWorld = court.imageToWorld();
    vgl_transform_2d<double> worldToImage = imageToWorld.inverse();
    for (int i = 0; i<points_world.size(); i++) {
        vgl_point_2d<double> p = points_world[i];
        vgl_point_2d<double> q = worldToImage(p);
        points_court_image.push_back(q);
    }
    
    assert(points_world.size() == points_court_image.size());
    assert(points_world.size() == points_camera_image.size());
}

void WWoSSoccerCourt::projectCourtPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                         vcl_vector<vgl_point_3d<double> > & world_pts,
                                         vcl_vector<vgl_point_2d<double> > & camera_image_pts,
                                         int threshold) const
{
    assert(world_pts.size() == 0);
    assert(camera_image_pts.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > courtPts = WWoSSoccerCourt::getFieldCalibrationPoints(field_width_, field_heigh_);
    
    for (int i = 0; i<courtPts.size(); i++)
    {
        vgl_homg_point_3d<double> p(courtPts[i].x(), courtPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        
        if (vgl_inside_image(q, width, height, threshold))
        {
            world_pts.push_back(vgl_point_3d<double>(p));
            camera_image_pts.push_back(q);
        }
    }
}

void WWoSSoccerCourt::projectCourtSegmentPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                                vcl_vector<vgl_point_3d<double> > & world_pts,
                                                vcl_vector<vgl_point_2d<double> > & camera_image_pts,
                                                double sampleLength,
                                                int threshold) const
{
    vcl_vector< vgl_point_3d<double> > courtPts = SoccerCourt::getCalibrationPoints(sampleLength, field_width_, field_heigh_);
    
    for (int i = 0; i<courtPts.size(); i++)
    {
        vgl_homg_point_3d<double> p(courtPts[i]);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        
        if (vgl_inside_image(q, width, height, threshold))
        {
            world_pts.push_back(courtPts[i]);
            camera_image_pts.push_back(q);
        }
    }
    
    assert(world_pts.size() == camera_image_pts.size());
}

void WWoSSoccerCourt::getProjectedCourtArea(const vpgl_perspective_camera<double> & camera, int width, int height,
                                            vil_image_view<vxl_byte> & alpha,
                                            const vgl_point_2d<int> & startP, const vgl_point_2d<int> & endP)
{
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_world_dump;
    vcl_vector<vgl_point_2d<double> > points_src;
    vcl_vector<vgl_point_2d<double> > points_dst;
    
    
    this->projectCourtPoints(camera, width, height, points_world_dump, points_src, points_dst, 10);
    
    assert(points_src.size() >= 4);
    assert(points_src.size() == points_dst.size());
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_src.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_src[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_dst[i]));
    }
    
    vgl_h_matrix_2d<double> H(src, dst);
    int w = 118 * 12 + 70 * 2;
    int h =  70 * 12 + 70 * 2;
    
    //    printf("overivew image size %d %d\n", w, h);
    
    vil_image_view<vxl_byte> image = vil_image_view<vxl_byte>(w, h, 1);
    image.fill(0);
    for (int y = startP.y(); y < endP.y(); y++) {
        for (int x = startP.x(); x < endP.x(); x++) {
            image(x, y, 0) = 255;
        }
    }
    
    alpha = vil_image_view<vxl_byte>(width, height, 1);
    alpha.fill(0);
    
    vil_image_view<vxl_byte> outImage;
    outImage.deep_copy(alpha);
    
    //warp court image to alpha
    VilPlus::homography_warp_fill(image, H, alpha, outImage);
    
    alpha = outImage;
}


void WWoSSoccerCourt::getProjectedCourtAreaCoverBorder(const vpgl_perspective_camera<double> & camera,
                                                       int width, int height,
                                                       vil_image_view<vxl_byte> & alpha)
{
    // H_topview_to_world
    vgl_h_matrix_2d<double> H_topview_to_world;
    {
        vcl_vector<vgl_homg_point_2d<double> > src;  // topview coordinate
        vcl_vector<vgl_homg_point_2d<double> > dst;
        int w = field_width_ * 36 / 3;  // in pixel
        int h = field_heigh_  * 36 / 3;

        src.push_back(vgl_homg_point_2d<double>(70, 70 + h, 1.0));
        dst.push_back(vgl_homg_point_2d<double>(0, 0, 1.0));
        src.push_back(vgl_homg_point_2d<double>(70+w, 70 + h, 1.0));
        dst.push_back(vgl_homg_point_2d<double>(field_width_, 0, 1.0));
        src.push_back(vgl_homg_point_2d<double>(70+w, 70, 1.0));
        dst.push_back(vgl_homg_point_2d<double>(field_width_, field_heigh_, 1.0));
        src.push_back(vgl_homg_point_2d<double>(70, 70, 1.0));
        dst.push_back(vgl_homg_point_2d<double>(0, field_heigh_, 1.0));
        
        H_topview_to_world = vgl_h_matrix_2d<double>(src, dst);
    }
    
    vnl_matrix_fixed<double, 3, 3> H_world_to_image = VpglPlus::homographyFromProjectiveCamera(camera);
    vgl_h_matrix_2d<double> H_topview_to_image = vgl_h_matrix_2d<double>(H_world_to_image) * H_topview_to_world;
    
    int w = 118 * 12 + 70 * 2;
    int h =  70 * 12 + 70 * 2;
    
    //    printf("overivew image size %d %d\n", w, h);
    
    vil_image_view<vxl_byte> image = vil_image_view<vxl_byte>(w, h, 1);
    vgl_point_2d<int> startP(30, 30);
    vgl_point_2d<int> endP(w - 40, h - 30);
    image.fill(0);
    for (int y = startP.y(); y < endP.y(); y++) {
        for (int x = startP.x(); x < endP.x(); x++) {
            image(x, y, 0) = 255;
        }
    }

    alpha = vil_image_view<vxl_byte>(width, height, 1);
    alpha.fill(0);
    
    vil_image_view<vxl_byte> outImage;
    outImage.deep_copy(alpha);
    
    //warp court image to alpha
    VilPlus::homography_warp_fill(image, H_topview_to_image, alpha, outImage);
    
    alpha = outImage;
}

bool WWoSSoccerCourt::getCourtArareInTopview(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & alpha_topview)
{
    int w = this->overviewImageWidth();
    int h = this->overviewImageHeight();
    
    vgl_h_matrix_2d<double> H_image2topview = this->imageToTopview(camera);
    vil_image_view<vxl_byte> topviewImage = vil_image_view<vxl_byte>(w, h, 1);
    vgl_point_2d<int> startP(70, 70);
    vgl_point_2d<int> endP(w - 70, h - 70);
    topviewImage.fill(0);
    for (int y = startP.y(); y < endP.y(); y++) {
        for (int x = startP.x(); x < endP.x(); x++) {
            topviewImage(x, y, 0) = 255;
        }
    }
    
    const int width  = camera.get_calibration().principal_point().x() * 2;
    const int height = camera.get_calibration().principal_point().y() * 2;
    vil_image_view<vxl_byte> alpha_camera_view = vil_image_view<vxl_byte>(width, height, 1);
    alpha_camera_view.fill(0);
    
    // warp topview to camera view
    vil_image_view<vxl_byte> outImage;
    outImage.deep_copy(alpha_camera_view);
    VilPlus::homography_warp_fill(topviewImage, H_image2topview.get_inverse(), alpha_camera_view, outImage);
    
    // reset topview Image;
    topviewImage.fill(0);
    alpha_topview.deep_copy(topviewImage);
    VilPlus::homography_warp_fill(outImage, H_image2topview, topviewImage, alpha_topview);
    return true;
    
}

void WWoSSoccerCourt::getProjectedCourtArea(const vpgl_perspective_camera<double> & camera,
                                            vil_image_view<vxl_byte> & alpha)
{
    WWoSSoccerCourt court;
    vgl_point_2d<int> startP(30, 30);
    vgl_point_2d<int> endP(court.overviewImageWidth() - 30, court.overviewImageHeight() - 30);
    court.getProjectedCourtArea(camera, 1280, 720, alpha, startP, endP);
}

bool WWoSSoccerCourt::getImage2TopviewImageHomography(const vpgl_perspective_camera<double> & camera,
                                                      int imageW, int imageH, vgl_h_matrix_2d<double> & H )
{
    WWoSSoccerCourt court;
    vcl_vector<vgl_point_2d<double> > dump_pts;
    vcl_vector<vgl_point_2d<double> > overview_image_pts;
    vcl_vector<vgl_point_2d<double> > camera_image_pts;
    int threshold = 15;
    court.projectCourtPoints(camera, imageW, imageH, dump_pts, overview_image_pts, camera_image_pts, threshold);
    
    if (overview_image_pts.size() < 4) {
        return false;
    }
    assert(overview_image_pts.size() == camera_image_pts.size());
    
    // from camera image to overview image
    vcl_vector<vgl_homg_point_2d<double> > pts1;
    vcl_vector<vgl_homg_point_2d<double> > pts2;
    for (int i = 0; i<camera_image_pts.size(); i++) {
        pts1.push_back(vgl_homg_point_2d<double>(camera_image_pts[i]));
        pts2.push_back(vgl_homg_point_2d<double>(overview_image_pts[i]));
    }
    H = vgl_h_matrix_2d<double>(pts1, pts2);
    return true;
}

bool WWoSSoccerCourt::getCameraFromSequentialHomography(const vpgl_perspective_camera<double> & keyFrameCamera, const vgl_h_matrix_2d<double> & H,
                                                        vpgl_perspective_camera<double> & camera)
{
    int width  = keyFrameCamera.get_calibration().principal_point().x() * 2;
    int height = keyFrameCamera.get_calibration().principal_point().y() * 2;
    vcl_vector<vgl_point_2d<double> >  pts_world;
    vcl_vector<vgl_point_2d<double> >  pts_image;
    vcl_vector<vgl_point_2d<double> >  pts_dump;
    
    this->projectCourtPoints(keyFrameCamera, width, height, pts_world, pts_dump, pts_image, 10);
    if (pts_image.size() < 5) {
        return false;
    }
    
    // warp points in first image to second image
    vcl_vector<vgl_point_2d<double> > pts_second_image;
    for (int i = 0; i<pts_image.size(); i++) {
        vgl_point_2d<double> p = pts_image[i];
        vgl_point_2d<double> q = (vgl_point_2d<double>)H(vgl_homg_point_2d<double>(p.x(), p.y(), 1.0));
        pts_second_image.push_back(q);
    }
    
    vpgl_perspective_camera<double> initCamera;
    bool isInit = VpglPlus::init_calib(pts_world, pts_second_image, keyFrameCamera.get_calibration().principal_point(), initCamera);
    if (!isInit) {
        return false;
    }
    bool isFinal = VpglPlus::optimize_perspective_camera(pts_world, pts_second_image, initCamera, camera);
    return isFinal;
}

bool WWoSSoccerCourt::getCameraFromHomography(const vgl_h_matrix_2d<double> & H, int imageW, int imageH,
                                              vpgl_perspective_camera<double> & camera)
{
    vcl_vector<vgl_point_2d<double> > allCourtPts = WWoSSoccerCourt::getFieldCalibrationPoints(field_width_, field_heigh_);
    vcl_vector<vgl_point_2d<double> > courtPts;
    vcl_vector<vgl_point_2d<double> > imgPts;
    
    for (int i = 0 ; i<allCourtPts.size(); i++) {
        vgl_point_2d<double> p = allCourtPts[i];
        vgl_point_2d<double> q = H(vgl_homg_point_2d<double>(p.x(), p.y(), 1.0));
        
        if (vgl_inside_image(q, imageW, imageH, 10)) {
            courtPts.push_back(p);
            imgPts.push_back(q);
        }
    }
    assert(courtPts.size() == imgPts.size());
    
    printf("point match number is %lu\n", courtPts.size());
    
    if (courtPts.size() < 5) {        
        return false;
    }
    
    vpgl_perspective_camera<double> initCamera;
    bool isInit = VpglPlus::init_calib(courtPts, imgPts, vgl_point_2d<double>(imageW/2.0, imageH/2.0), initCamera);
    if (!isInit) {
        return false;
    }
    bool isFinal = VpglPlus::optimize_perspective_camera(courtPts, imgPts, initCamera, camera);
    return isFinal;
}

bool WWoSSoccerCourt::getHomographyFromCamera(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, vgl_h_matrix_2d<double> & H)
{
    vcl_vector<vgl_point_2d<double> >      allCourtPts = WWoSSoccerCourt::getFieldCalibrationPoints(field_width_, field_heigh_);
    vcl_vector<vgl_homg_point_2d<double> > courtPts;
    vcl_vector<vgl_homg_point_2d<double> > imgPts;
    
    for (int i = 0 ; i<allCourtPts.size(); i++) {
        vgl_point_2d<double> p = allCourtPts[i];
        vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(p.x(), p.y(), 0.0));
        
        if (vgl_inside_image(q, imageW, imageH, 10)) {
            courtPts.push_back(vgl_homg_point_2d<double>(p));
            imgPts.push_back(vgl_homg_point_2d<double>(q));
        }
    }
    assert(courtPts.size() == imgPts.size());
    if (courtPts.size() < 4) {
        return false;
    }
    H = vgl_h_matrix_2d<double>(courtPts, imgPts);
    return true;
}

bool WWoSSoccerCourt::getRelativeHomography(const vpgl_perspective_camera<double> & camera1, const vpgl_perspective_camera<double> & camera2, int imageW, int imageH,
                                            vgl_h_matrix_2d<double> & H)
{
    vcl_vector<vgl_point_2d<double> > allCourtPts = WWoSSoccerCourt::getFieldCalibrationPoints(field_width_, field_heigh_);
    vcl_vector<vgl_homg_point_2d<double> > pts1;
    vcl_vector<vgl_homg_point_2d<double> > pts2;
    
    for (int i = 0 ; i<allCourtPts.size(); i++) {
        vgl_point_2d<double> p = allCourtPts[i];
        vgl_homg_point_2d<double> q1 = camera1.project(vgl_point_3d<double>(p.x(), p.y(), 0.0));
        vgl_homg_point_2d<double> q2 = camera2.project(vgl_point_3d<double>(p.x(), p.y(), 0.0));
        
        if (vgl_inside_image(q1, imageW, imageH, 10) && vgl_inside_image(q2, imageW, imageH, 10)) {
            pts1.push_back(q1);
            pts2.push_back(q2);            
        }
    }
    assert(pts1.size() == pts2.size());
    if (pts1.size() < 4) {
        return false;
    }
    
    H = vgl_h_matrix_2d<double>(pts1, pts2);   
    
    return true;
}


vgl_transform_2d< double > WWoSSoccerCourt::imageToWorld()
{
    double imageH = field_heigh_ * 36 / 3.0;
    double m1[9] = {
        1, 0, -70,
        0, 1, -70,
        0, 0,  1};
    double m2[9] = {
        1, 0, 0,
        0, -1, imageH,
        0, 0, 1};
    //every 1 pixel is 3 inch, inch to meter
    double m3[9] = {
        3 * 0.0254, 0, 0,
        0, 3 * 0.0254, 0,
        0, 0, 1
    };
    
    vgl_transform_2d< double > model = vgl_transform_2d<double>(vnl_matrix_fixed<double, 3, 3>(m3) * vnl_matrix_fixed<double, 3, 3>(m2) * vnl_matrix_fixed<double, 3, 3>(m1));
    return model;
}

vgl_h_matrix_2d<double> WWoSSoccerCourt::imageToTopview(const vpgl_perspective_camera<double> & camera)
{
    vnl_matrix_fixed<double, 3, 3> H_w_to_image = VpglPlus::homographyFromProjectiveCamera(camera);
    vnl_matrix_fixed<double, 3, 3> H_w_to_topview = this->imageToWorld().inverse();
    vgl_h_matrix_2d<double> H = H_w_to_topview * vnl_inverse(H_w_to_image);
    return H;
}

bool WWoSSoccerCourt::warpStationaryCameraToTopviewImage(const vil_image_view<vxl_byte> & leftImage,
                                        const vil_image_view<vxl_byte> & rightImage,
                                        const vpgl_perspective_camera<double> & leftCamera,
                                        const vpgl_perspective_camera<double> & rightCamera,
                                        const vil_image_view<vxl_byte> & topviewImage,
                                        vil_image_view<vxl_byte> & warpedImage)
{
    // warp left
    vil_image_view<vxl_byte> leftWarpedImage;
    vgl_h_matrix_2d<double> H1 = this->imageToTopview(leftCamera);
    leftWarpedImage.deep_copy(topviewImage);
    VilPlus::homography_warp_fill(leftImage, H1, topviewImage, leftWarpedImage);
    
    // warp right
    vil_image_view<vxl_byte> rightWarpedImage;
    vgl_h_matrix_2d<double> H2 = this->imageToTopview(rightCamera);
    rightWarpedImage.deep_copy(topviewImage);
    VilPlus::homography_warp_fill(rightImage, H2, topviewImage, rightWarpedImage);
    
    int w = topviewImage.ni();
    int h = topviewImage.nj();
    warpedImage.deep_copy(topviewImage);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            if (i < w/2) {
                warpedImage(i, j, 0) = leftWarpedImage(i, j, 0);
                warpedImage(i, j, 1) = leftWarpedImage(i, j, 1);
                warpedImage(i, j, 2) = leftWarpedImage(i, j, 2);
            }
            else
            {
                warpedImage(i, j, 0) = rightWarpedImage(i, j, 0);
                warpedImage(i, j, 1) = rightWarpedImage(i, j, 1);
                warpedImage(i, j, 2) = rightWarpedImage(i, j, 2);
            }
        }
    }
    return true;    
}

int WWoSSoccerCourt::overviewImageWidth()
{
    int w = field_width_ * 36 / 3 + 70 * 2;
    return w;
}

int WWoSSoccerCourt::overviewImageHeight()
{
    int h = field_heigh_  * 36 / 3 + 70 * 2;
    return h;
}

void WWoSSoccerCourt::sampleLines(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                  double sampleUnit, vcl_vector<vcl_vector<LinePatchSampler> > & samplers)
{    
    vcl_vector<vgl_line_segment_2d<double> > sLines = this->straightLines();
    
    for (int i = 0; i<sLines.size(); i++) {
        vcl_vector<LinePatchSampler> curSampler;
        vgl_point_3d<double> p1(sLines[i].point1().x(), sLines[i].point1().y(), 0);
        vgl_point_3d<double> p2(sLines[i].point2().x(), sLines[i].point2().y(), 0);
     //   vgl_point_3d<double> p_mid((p1.x() + p2.x())/2.0, (p1.y() + p2.y())/2.0, 0);
        
      //  vgl_point_2d<double> q1 = camera.project(p1);
     //   vgl_point_2d<double> q2 = camera.project(p2);
     //   vgl_point_2d<double> q_mid = camera.project(p_mid);
        
     //   if (vgl_inside_image(q1, imageW, imageH) || vgl_inside_image(q2, imageW, imageH) || vgl_inside_image(q_mid, imageW, imageH))
        // sample every line
        {
            // sample from p1 to p2
            vgl_vector_2d<double> dir = sLines[i].direction();
            double dx = p1.x() - p2.x();
            double dy = p1.y() - p2.y();
            double length = sqrt(dx * dx + dy * dy);
            for (double j = 0; j <= length; j += sampleUnit) {
                double x = p1.x() + dir.x() * j;
                double y = p1.y() + dir.y() * j;
                
                vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, 0.0));
                if (vgl_inside_image(q, imageW, imageH)) {
                    LinePatchSampler lps;
                    lps.line_id_ = i;
                    lps.center_pt_ = vgl_point_3d<double>(x, y, 0.0);
                    lps.im_loc_ = vgl_point_2d<double>(q.x(), q.y());
                    curSampler.push_back(lps);
                }
            }
        }
        if (curSampler.size() > 0) {
            samplers.push_back(curSampler);
        }
    }
}

void WWoSSoccerCourt::sampleWhiteLineSegment(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                         double sampleUnit, vcl_vector<WWOSSoccerLinePair> & lineSegments)
{
    vcl_vector<vgl_line_segment_2d<double> > wLines = this->whiteLines();
    for (int i = 0; i<wLines.size(); i++) {
        vgl_point_3d<double> p1(wLines[i].point1().x(), wLines[i].point1().y(), 0);
        vgl_point_3d<double> p2(wLines[i].point2().x(), wLines[i].point2().y(), 0);
        
        
        vgl_point_2d<double> q1(-1, -1); // (-1, -1) is the mask for unfined end point
        vgl_point_2d<double> q2(-1, -1);
        vgl_vector_2d<double> dir = wLines[i].direction();
        const double length = vgl_distance(p1, p2);
        
        // sample first point from p1 to p2 in the image
        bool isFound = false;
        for (double j = 0; j <= length; j += sampleUnit) {
            double x = p1.x() + dir.x() * j;
            double y = p1.y() + dir.y() * j;
            
            vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, 0.0));
            if (vgl_inside_image(q, imageW, imageH)) {
                q1 = q;
                isFound = true;
                break;
            }
        }
        if (!isFound) {
            continue;
        }
        isFound = false;
        
        // sample second point from p2 to p1 in the image
        for (double j = length; j >=  0; j -= sampleUnit) {
            double x = p1.x() + dir.x() * j;
            double y = p1.y() + dir.y() * j;
            
            vgl_point_2d<double> q = camera.project(vgl_point_3d<double>(x, y, 0.0));
            if (vgl_inside_image(q, imageW, imageH)) {
                q2 = q;
                isFound = true;
                break;
            }
        }
        if (!isFound) {
            continue;
        }
        
        double dis = vgl_distance(q1, q2);
        if (dis > 5 * sampleUnit) {
            WWOSSoccerLinePair lineSeg(i, wLines[i].point1(), wLines[i].point2());
           // lineSeg.line_id_ = i;
           // lineSeg.world_ = wLines[i];
            lineSeg.image_ = vgl_line_segment_2d<double>(q1, q2);
            
            lineSegments.push_back(lineSeg);
        }
    }
}

vgl_conic<double> WWoSSoccerCourt::getCenterCircle(void)
{
    const double x = field_width_ * LU_YARD2METER * 0.5;
    const double y = field_heigh_ * LU_YARD2METER * 0.5;
    const double radius = 10.0 * LU_YARD2METER;
    vgl_conic<double > conic(vgl_homg_point_2d<double>(x, y, 1.0), radius, radius, 0.0);
    return conic;
}

vgl_conic<double> WWoSSoccerCourt::getLeftCircle(void)
{
    const double x = 12.0 * LU_YARD2METER;
    const double y = field_heigh_ * LU_YARD2METER * 0.5;
    const double radius = 10.0 * LU_YARD2METER;
    vgl_conic<double > conic(vgl_homg_point_2d<double>(x, y, 1.0), radius, radius, 0.0);
    return conic;
}

vcl_vector<vgl_point_2d<double> > WWoSSoccerCourt::centerCircleWldPts()
{
    const double x = field_width_ * LU_YARD2METER * 0.5;
    const double y = field_heigh_ * LU_YARD2METER * 0.5;
    const double radius = 10.0 * LU_YARD2METER;
    
    vcl_vector<vgl_point_2d<double> > pts;
    pts.push_back(vgl_point_2d<double>(x, y + radius));
    pts.push_back(vgl_point_2d<double>(x, y - radius));
    pts.push_back(vgl_point_2d<double>(x - radius, y));
    pts.push_back(vgl_point_2d<double>(x + radius, y));
    return pts;
}

void WWoSSoccerCourt::centerLineWld(vgl_line_3d_2_points<double> & line)
{
    const double x = field_width_ * LU_YARD2METER * 0.5;
    const double y = field_heigh_ * LU_YARD2METER;
    
    vgl_point_3d<double> pt1(x, 0, 0);
    vgl_point_3d<double> pt2(x, y, 0);
    line = vgl_line_3d_2_points<double>(pt1, pt2);
}

vcl_vector<vgl_point_2d<double> > WWoSSoccerCourt::leftCircleIntersection()
{
    vcl_vector<vgl_point_2d<double> > calib_pts = this->calibration_points();
    vcl_vector<vgl_point_2d<double> > pts;
    pts.push_back(calib_pts[16]);
    pts.push_back(calib_pts[17]);
    return pts;
}

vcl_vector<vgl_point_2d<double> > WWoSSoccerCourt::centerCircleIntersection()
{
    vcl_vector<vgl_point_2d<double> > calib_pts = this->calibration_points();
    vcl_vector<vgl_point_2d<double> > pts;
    pts.push_back(calib_pts[21]);
    pts.push_back(calib_pts[22]);
    return pts;
}

vcl_vector<vgl_point_2d<double> > WWoSSoccerCourt::rightCircleIntersection()
{
    vcl_vector<vgl_point_2d<double> > calib_pts = this->calibration_points();
    vcl_vector<vgl_point_2d<double> > pts;
    pts.push_back(calib_pts[25]);
    pts.push_back(calib_pts[26]);
    return pts;
}

vcl_vector<vgl_line_segment_2d<double> > WWoSSoccerCourt::straightLines()
{
    double width  = field_width_;
    double height = field_heigh_;
    
  //  return segments;
    assert(width >= 100 && width <= 130);
    assert(height >= 50 && height <= 100);
    
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // one center line
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(0, height/2)));
    
    // two goal lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-width/2, height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, height/2)));
    
    // origin point in court center
    // two touch lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(0, -height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(width/2, -height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, height/2), vgl_point_2d< double >(0, height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, height/2), vgl_point_2d< double >(width/2, height/2)));
    
    // 18 yard line
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) ) );
    
    // 18 yard line vertical
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ) ) );    
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) ) );
    
    /*
    // symmetric for left and right court field
    for ( int dx = -1; dx <= +1; dx += 2 )
    {
        // penelty area (large)
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * dx, -22 ), vgl_point_2d< double >( (width/2 - 18) * dx, -22 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * dx, +22 ), vgl_point_2d< double >( (width/2 - 18) * dx, +22 ) ) );
        
        // penelty area (small)
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * dx, -10 ), vgl_point_2d< double >( (width/2 - 6) * dx, +10 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * dx, -10 ), vgl_point_2d< double >( (width/2 - 6) * dx, -10 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * dx, +10 ), vgl_point_2d< double >( (width/2 - 6) * dx, +10 ) ) );
        
        // penelty point
        {
            double delta = 0.5;
            vgl_point_2d<double> p((width/2 - 12) * dx, 0);
            vgl_point_2d<double> q1(p.x() - delta, p.y());
            vgl_point_2d<double> q2(p.x() + delta, p.y());
            vgl_point_2d<double> q3(p.x(), p.y() - delta);
            vgl_point_2d<double> q4(p.x(), p.y() + delta);
            
            markings.push_back( vgl_line_segment_2d< double >(q1, q2));
            markings.push_back( vgl_line_segment_2d< double >(q3, q4));
        }
        
        // arc near penelty area
        for ( unsigned int i = 0; i < 32; ++i )
        {
            double alpha = atan2(-8.0, 6.0);
            double range = 2.0 * fabs(alpha);
            double startTheta = range / 32 * i + alpha;
            double stopTheta  = range / 32 * ( i + 1 ) + alpha;
            
            vgl_point_2d<double> p1((width/2 - 12 - 10 * cos(startTheta)) * dx, 0 + 10 * sin(startTheta));
            vgl_point_2d<double> p2((width/2 - 12 - 10 * cos(stopTheta)) * dx, 0 + 10 * sin(stopTheta));
            
            markings.push_back( vgl_line_segment_2d< double >( p1, p2));
        }
        
        for (int dy = -1; dy <= +1; dy += 2) {
            
            // goal point
            {
                double delta = 0.5;
                vgl_point_2d<double> p(width/2 * dx, 4 * dy);
                vgl_point_2d<double> q1(p.x() - delta, p.y());
                vgl_point_2d<double> q2(p.x() + delta, p.y());
                vgl_point_2d<double> q3(p.x(), p.y() - delta);
                vgl_point_2d<double> q4(p.x(), p.y() + delta);
                
                markings.push_back( vgl_line_segment_2d< double >(q1, q2));
                markings.push_back( vgl_line_segment_2d< double >(q3, q4));
            }
            
            // 10 yard bar (vertical)
            {
                double delta = 0.5;
                vgl_point_2d<double> p(width/2 * dx, (height/2-10) * dy);
                vgl_point_2d<double> q1(p.x() - delta, p.y());
                vgl_point_2d<double> q2(p.x() + delta, p.y());
                vgl_point_2d<double> q3(p.x(), p.y() - delta);
                vgl_point_2d<double> q4(p.x(), p.y() + delta);
                
                markings.push_back( vgl_line_segment_2d< double >(q1, q2));
                markings.push_back( vgl_line_segment_2d< double >(q3, q4));
            }
            
            // 10 yard bar (horizontal)
            {
                double delta = 0.5;
                vgl_point_2d<double> p((width/2 - 10) * dx, (height/2) * dy);
                vgl_point_2d<double> q1(p.x() - delta, p.y());
                vgl_point_2d<double> q2(p.x() + delta, p.y());
                vgl_point_2d<double> q3(p.x(), p.y() - delta);
                vgl_point_2d<double> q4(p.x(), p.y() + delta);
                
                markings.push_back( vgl_line_segment_2d< double >(q1, q2));
                markings.push_back( vgl_line_segment_2d< double >(q3, q4));
            }
        }
    }
     */
    
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 ) ;
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    
    return markings;
}

vcl_vector<vgl_line_segment_2d<double> > WWoSSoccerCourt::whiteLines()
{
    double width  = field_width_;
    double height = field_heigh_;
    
    //  return segments;
    assert(width >= 100 && width <= 130);
    assert(height >= 50 && height <= 100);
    
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // one center line
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(0, height/2)));
    
    // two goal lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-width/2, height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, height/2)));
    
    // two touch lines, too weak to be detected
   // markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(0, -height/2)));
   // markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(width/2, -height/2)));
   // markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, height/2), vgl_point_2d< double >(0, height/2)));
   // markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, height/2), vgl_point_2d< double >(width/2, height/2)));
    
    // 18 yard line
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) ) );
    
    // 18 yard line vertical
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ) ) );
 //   markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) ) );
 //   markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) ) );
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 ) ;
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    return markings;
}

vcl_vector<vgl_line_segment_2d<double> > WWoSSoccerCourt::ellipseLines()
{
    double width  = field_width_;
    double height = field_heigh_;
   
    assert(width >= 100 && width <= 130);
    assert(height >= 50 && height <= 100);
    
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    //center circle
    for ( unsigned int i = 0; i < 64; ++i )
    {
        double startTheta = 2 * vnl_math::pi / 64 * i;
        double stopTheta  = 2 * vnl_math::pi / 64 * ( i + 1 );
        vgl_point_2d<double> p1(cos(startTheta) * 10, sin(startTheta) * 10);
        vgl_point_2d<double> p2(cos(stopTheta) * 10, sin(stopTheta) * 10);
        
        markings.push_back( vgl_line_segment_2d< double >(p1, p2));
    }
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 ) ;
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    
    return markings;
}

vcl_vector<vgl_line_segment_2d<double> > WWoSSoccerCourt::penaltyEllipseLines(bool isLeft)
{
    double width  = field_width_;
    double height = field_heigh_;
    assert(width >= 100 && width <= 130);
    assert(height >= 50 && height <= 100);
    
    vcl_vector< vgl_line_segment_2d< double > > markings;
    // arc near penelty area
    int dx = -1;
    if (!isLeft) {
        dx = 1;
    }
    for ( unsigned int i = 0; i < 32; ++i )
    {
        double alpha = atan2(-8.0, 6.0);
        double range = 2.0 * fabs(alpha);
        double startTheta = range / 32 * i + alpha;
        double stopTheta  = range / 32 * ( i + 1 ) + alpha;
        
        vgl_point_2d<double> p1((width/2 - 12 - 10 * cos(startTheta)) * dx, 0 + 10 * sin(startTheta));
        vgl_point_2d<double> p2((width/2 - 12 - 10 * cos(stopTheta)) * dx, 0 + 10 * sin(stopTheta));
        
        markings.push_back( vgl_line_segment_2d< double >( p1, p2));
    }
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 ) ;
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    return markings;
}


vcl_vector<vgl_line_segment_2d<double> > WWoSSoccerCourt::inferencedWhiteLines()
{
    double width  = field_width_;
    double height = field_heigh_;
    
    //  return segments;
    assert(width >= 100 && width <= 130);
    assert(height >= 50 && height <= 100);
    
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // 18 yard line vertical, far end
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) ) );
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 ) ;
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    return markings;
}

void WWoSSoccerCourt::getAllLines(const vpgl_perspective_camera<double> & camera, int width, int height, vcl_vector< vgl_line_segment_2d< double > > &lines)
{
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(field_width_, field_heigh_);
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        lines.push_back(vgl_line_segment_2d< double >( start, stop ));
    }
}

void WWoSSoccerCourt::getAllLines(vcl_vector<vgl_line_segment_3d<double> > & segments) const
{
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(field_width_, field_heigh_);    
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> p1 = markings[i].point1();
        vgl_point_2d<double> p2 = markings[i].point2();
        vgl_point_3d<double> p3(p1.x(), p1.y(), 0.0);
        vgl_point_3d<double> p4(p2.x(), p2.y(), 0.0);
        segments.push_back(vgl_line_segment_3d<double>(p3, p4));
    }
    assert(markings.size() == segments.size());
}

void WWoSSoccerCourt::getImageKeyPoints(const vpgl_perspective_camera<double> & camera, int width, int height, vcl_vector<vgl_point_2d<double> > & pts)
{
    vcl_vector<vgl_point_2d<double> > calibPts = WWoSSoccerCourt::getFieldCalibrationPoints(field_width_, field_heigh_);
    
    for (int i = 0; i<calibPts.size(); i++) {
        vgl_homg_point_3d<double> p(calibPts[i].x(), calibPts[i].y(), 0, 1.0);
        vgl_point_2d<double> q = vgl_point_2d<double>(camera.project(p));
        if (q.x() >= 0 && q.x() < width && q.y() >= 0 && q.y() < height) {
            pts.push_back(q);
        }
    }
}

void WWoSSoccerCourt::getWorldImageKeyPoints(const vpgl_perspective_camera<double> & camera, int width, int height,
                                             vcl_vector<vgl_point_2d<double> > & wld_pts_out, vcl_vector<vgl_point_2d<double> > & img_pts)
{
    vcl_vector<vgl_point_2d<double> > wld_pts = WWoSSoccerCourt::getFieldCalibrationPoints(field_width_, field_heigh_);
    
    for (int i = 0; i<wld_pts.size(); i++) {
        vgl_homg_point_3d<double> p(wld_pts[i].x(), wld_pts[i].y(), 0, 1.0);
        vgl_point_2d<double> q = vgl_point_2d<double>(camera.project(p));
        if (q.x() >= 0 && q.x() < width && q.y() >= 0 && q.y() < height) {
            img_pts.push_back(q);
            wld_pts_out.push_back(wld_pts[i]);
        }
    }
}



vgl_point_2d<double> WWoSSoccerCourt::World2Image(const vgl_point_2d<double> &p)
{
    // meter to inch, every 3 inch as pixel
    double x = p.x() * 39.37 /3;
    double y = p.y() * 39.37 /3;
    
    // flip y
    y = 840 - y;
    
    x += 70;
    y += 70;
    
    return vgl_point_2d<double>(x, y);
}

void WWoSSoccerCourt::overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image)
{
    assert(image.nplanes() == 3);
    
    double width = 118;
    double height = 70;
    
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(width, height);
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), vicl_colour::blue, 2);
    }
}

void WWoSSoccerCourt::overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image, const vcl_vector<vxl_byte> & colour, int lineWidth)
{
    assert(image.nplanes() == 3);
    
    double width = 118;
    double height = 70;
    
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(width, height);
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), colour, lineWidth);
    }
}

void WWoSSoccerCourt::overlayLines(const vgl_h_matrix_2d<double> & H, vil_image_view<vxl_byte> & image, const vcl_vector<vxl_byte> & colour, int lineWidth)
{
    assert(image.nplanes() == 3);
    
    double width = 118;
    double height = 70;
    
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(width, height);
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_2d<double> p1 = vgl_homg_point_2d<double>(markings[i].point1());
        vgl_homg_point_2d<double> p2 = vgl_homg_point_2d<double>(markings[i].point2());
        
        vgl_point_2d< double > start = H(p1);
        vgl_point_2d< double > stop  = H(p2);
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), colour, lineWidth);
    }
}

void WWoSSoccerCourt::overlayPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image)
{
    assert(image.nplanes() == 3);
    
    double width = 118;
    double height = 70;
    
    vcl_vector<vgl_point_2d<double> > points = SoccerCourt::getCalibratePoints(width, height);
    vcl_vector<vgl_point_2d<double> > hockeyPoints = WWoSSoccerCourt::getFieldHockeyPoints(width, height);
    for (int i = 0; i<hockeyPoints.size(); i++) {
        points.push_back(hockeyPoints[i]);
    }
    
    for (int i = 0; i<points.size(); i++) {
        vgl_homg_point_3d<double> p(points[i].x(), points[i].y(), 0, 1.0);
        vgl_point_2d<double> q = vgl_point_2d<double>(camera.project(p));
        if (q.x() >= 0 && q.x() < image.ni() && q.y() >= 0 && q.y() < image.nj()) {
            int w = 6;
            
            vgl_point_2d<double> q1, q2, q3, q4;
            q1.set(q.x(), q.y() - w);
            q2.set(q.x(), q.y() + w);
            q3.set(q.x() - w, q.y());
            q4.set(q.x() + w, q.y());
            
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), vicl_colour::red, 3);
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q3, q4), vicl_colour::red, 3);
            
            // for test only
            if (0) {
                vcl_cout<<"3D point is "<<p<<vcl_endl;
                vcl_cout<<"2D point is "<<q<<vcl_endl<<vcl_endl;
            }
        }
    }
}

void WWoSSoccerCourt::overlayLinesOnBackground(const vpgl_perspective_camera<double> & camera,
                                                const vil_image_view<vxl_byte> & image,
                                                vil_image_view<vxl_byte> & outImage,
                                                const vcl_vector<vxl_byte> & colour, int line_thickness)
{
    vcl_vector<vgl_line_segment_3d<double> > segments;
    this->getAllLines(segments);
    VilPlus::draw_line_on_background(camera, segments, image, outImage, colour,line_thickness);
}

bool WWoSSoccerCourt::isEllipseInsideImage(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, double ratio)
{
    WWoSSoccerCourt wwosCourt;
    vcl_vector<vgl_line_segment_2d<double> > ellipsePoints = wwosCourt.ellipseLines();
    
    int num = 0;
    for (int i = 0; i<ellipsePoints.size(); i++) {
        // only consider one point
        vgl_point_2d<double> p1 = ellipsePoints[i].point1();
        if (!camera.is_behind_camera(vgl_homg_point_3d<double>(p1.x(), p1.y(), 0.0, 1))) {
            vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(p1.x(), p1.y(), 0.0));
            if (vgl_inside_image(q1, imageW, imageH)) {
                num++;
            }
        }
    }
    if (1.0 * num/ellipsePoints.size() > ratio) {
        return true;
    }
    return false;
}

bool WWoSSoccerCourt::isPenaltyEllipseInsideImage(const vpgl_perspective_camera<double> & camera, int imageW, int imageH, bool isLeft)
{
    WWoSSoccerCourt wwosCourt;
    vcl_vector<vgl_line_segment_2d<double> > ellipsePoints = wwosCourt.penaltyEllipseLines(isLeft);
    
    for (int i = 0; i<ellipsePoints.size(); i++) {
        // only consider one point
        vgl_point_2d<double> p1 = ellipsePoints[i].point1();
        if (!camera.is_behind_camera(vgl_homg_point_3d<double>(p1.x(), p1.y(), 0.0, 1))) {
            vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(p1.x(), p1.y(), 0.0));
            if (!vgl_inside_image(q1, imageW, imageH)) {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    
    return true;
}

void WWoSSoccerCourt::overlayEllipseBoundingBox(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & mask,
                                                const vcl_vector<vxl_byte> & colour, int boardWidth, int boardHeight)
{
    assert(mask.nplanes() == 3);
    assert(colour.size() == 3);
    
    WWoSSoccerCourt wwosCourt;
    vcl_vector<vgl_line_segment_2d<double> > ellipsePoints = wwosCourt.ellipseLines();
    
    const int w = mask.ni();
    const int h = mask.nj();
    int minX = w;
    int minY = h;
    int maxX = 0;
    int maxY = 0;
    for (int i = 0; i<ellipsePoints.size(); i++) {
        // only consider one point
        vgl_point_2d<double> p1 = ellipsePoints[i].point1();
        if (!camera.is_behind_camera(vgl_homg_point_3d<double>(p1.x(), p1.y(), 0.0, 1))) {
            vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(p1.x(), p1.y(), 0.0));
            minX = q1.x() < minX? q1.x():minX;
            minY = q1.y() < minY? q1.y():minY;
            maxX = q1.x() > maxX? q1.x():maxX;
            maxY = q1.y() > maxY? q1.y():maxY;
        }
    }
    minX -= boardWidth;
    maxX += boardWidth;
    minY -= boardHeight;
    maxY += boardHeight;
    minX = minX >0 ? minX:0;
    minY = minY >0 ? minY:0;
    maxX = maxX <w ? maxX:w-1;
    maxY = maxY <h ? maxY:h-1;
    if (minX < maxX && minY < maxY) {
        for (int j = minY; j<maxY; j++) {
            for (int i = minX; i<maxX; i++) {
                for (int k = 0; k<3; k++) {
                    mask(i, j, k) = colour[k];
                }
            }
        }
    }
}

bool WWoSSoccerCourt::centerCircleBoundingBox(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                              vgl_box_2d<int> & box, int extendBoundaryWidth, int extendBoundardHeight)
{
    WWoSSoccerCourt wwosCourt;
    vcl_vector<vgl_line_segment_2d<double> > ellipsePoints = wwosCourt.ellipseLines();
    
    int minX = imageW;
    int minY = imageH;
    int maxX = 0;
    int maxY = 0;
    for (int i = 0; i<ellipsePoints.size(); i++) {
        // only consider one point
        vgl_point_2d<double> p1 = ellipsePoints[i].point1();
        if (!camera.is_behind_camera(vgl_homg_point_3d<double>(p1.x(), p1.y(), 0.0, 1))) {
            vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(p1.x(), p1.y(), 0.0));
            if (vgl_inside_image(q1, imageW, imageH)) {
                minX = q1.x() < minX? q1.x():minX;
                minY = q1.y() < minY? q1.y():minY;
                maxX = q1.x() > maxX? q1.x():maxX;
                maxY = q1.y() > maxY? q1.y():maxY;
            }
        }
    }
    minX -= extendBoundaryWidth;
    maxX += extendBoundaryWidth;
    minY -= extendBoundardHeight;
    maxY += extendBoundardHeight;
    minX = minX >=0 ? minX:0;
    minY = minY >=0 ? minY:0;
    maxX = maxX <imageW ? maxX:imageW-1;
    maxY = maxY <imageH ? maxY:imageH-1;
    
    assert(minX >= 0 && minX < imageW);
    assert(minY >= 0 && minY < imageH);
    
    box = vgl_box_2d<int>(minX, maxX, minY, maxY);
    return true;
}

bool WWoSSoccerCourt::penaltyEllipseBoundingBox(const vpgl_perspective_camera<double> & camera, int imageW, int imageH,
                                                vgl_box_2d<int> & box, bool isLeft, int extendBoundaryWidth, int extendBoundardHeight)
{
    WWoSSoccerCourt wwosCourt;
    vcl_vector<vgl_line_segment_2d<double> > ellipsePoints = wwosCourt.penaltyEllipseLines(isLeft);
    
    int minX = imageW;
    int minY = imageH;
    int maxX = 0;
    int maxY = 0;
    for (int i = 0; i<ellipsePoints.size(); i++) {
        // only consider one point
        vgl_point_2d<double> p1 = ellipsePoints[i].point1();
        if (!camera.is_behind_camera(vgl_homg_point_3d<double>(p1.x(), p1.y(), 0.0, 1))) {
            vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(p1.x(), p1.y(), 0.0));
            if (vgl_inside_image(q1, imageW, imageH)) {
                minX = q1.x() < minX? q1.x():minX;
                minY = q1.y() < minY? q1.y():minY;
                maxX = q1.x() > maxX? q1.x():maxX;
                maxY = q1.y() > maxY? q1.y():maxY;
            }
        }
    }
    minX -= extendBoundaryWidth;
    maxX += extendBoundaryWidth;
    minY -= extendBoundardHeight;
    maxY += extendBoundardHeight;
    minX = minX >=0 ? minX:0;
    minY = minY >=0 ? minY:0;
    maxX = maxX <imageW ? maxX:imageW-1;
    maxY = maxY <imageH ? maxY:imageH-1;
    
    assert(minX >= 0 && minX < imageW);
    assert(minY >= 0 && minY < imageH);
    
    box = vgl_box_2d<int>(minX, maxX, minY, maxY);
    return true;
}


void WWoSSoccerCourt::overlayPenaltyEllipseBoundingBox(const vpgl_perspective_camera<double> & camera,
                                                       vil_image_view<vxl_byte> & mask,
                                                       const vcl_vector<vxl_byte> & colour, bool isLeft,
                                                       int boardWidth, int boardHeight)
{    
    assert(mask.nplanes() == 3);
    assert(colour.size() == 3);
    
    WWoSSoccerCourt wwosCourt;
    vcl_vector<vgl_line_segment_2d<double> > ellipsePoints = wwosCourt.penaltyEllipseLines(isLeft);
    
    const int w = mask.ni();
    const int h = mask.nj();
    int minX = w;
    int minY = h;
    int maxX = 0;
    int maxY = 0;
    for (int i = 0; i<ellipsePoints.size(); i++) {
        // only consider one point
        vgl_point_2d<double> p1 = ellipsePoints[i].point1();
        if (!camera.is_behind_camera(vgl_homg_point_3d<double>(p1.x(), p1.y(), 0.0, 1))) {
            vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(p1.x(), p1.y(), 0.0));
            minX = q1.x() < minX? q1.x():minX;
            minY = q1.y() < minY? q1.y():minY;
            maxX = q1.x() > maxX? q1.x():maxX;
            maxY = q1.y() > maxY? q1.y():maxY;
        }
    }
    minX -= boardWidth;
    maxX += boardWidth;
    minY -= boardHeight;
    maxY += boardHeight;
    minX = minX >0 ? minX:0;
    minY = minY >0 ? minY:0;
    maxX = maxX <w ? maxX:w-1;
    maxY = maxY <h ? maxY:h-1;
    if (minX < maxX && minY < maxY) {
        for (int j = minY; j<maxY; j++) {
            for (int i = minX; i<maxX; i++) {
                for (int k = 0; k<3; k++) {
                    mask(i, j, k) = colour[k];
                }
            }
        }
    }
}

vcl_vector<vgl_point_2d<double> > WWoSSoccerCourt::getFieldCalibrationPoints(double width, double height)
{
    vcl_vector<vgl_point_2d<double> > pts(43);
    
    // left goal line
    pts[0].set(0, 0);
    pts[1].set(0, 10);
    pts[2].set(0, height/2 - 4);
    pts[3].set(0, height/2 - 10);
    pts[4].set(0, height/2 - 22);
    pts[5].set(0, height/2 + 4);
    pts[6].set(0, height/2 + 10);
    pts[7].set(0, height/2 + 22);
    pts[8].set(0, height - 10);
    pts[9].set(0, height);
    
    // 6 yard
    pts[10].set(6, height/2 - 10);
    pts[11].set(6, height/2 + 10);
    
    // 10 yard
    pts[12].set(10, 0);
    pts[13].set(10, height);
    
    // 18 yard
    pts[14].set(18, height/2 - 22);
    pts[15].set(18, height/2 + 22);
    pts[16].set(18, height/2 - 8);
    pts[17].set(18, height/2 + 8);
    
    // center line
    pts[18].set(width/2, 0);
    pts[19].set(width/2, height);
    pts[20].set(width/2, height/2);
    pts[21].set(width/2, height/2 - 10);
    pts[22].set(width/2, height/2 + 10);
    
    // right side
    // 18 yard
    pts[23].set(width - 18, height/2 - 22);
    pts[24].set(width - 18, height/2 + 22);
    pts[25].set(width - 18, height/2 - 8);
    pts[26].set(width - 18, height/2 + 8);
    
    // 10 yard
    pts[27].set(width - 10, 0);
    pts[28].set(width - 10, height);
    
    // 6 yard
    pts[29].set(width - 6, height/2 - 10);
    pts[30].set(width - 6, height/2 + 10);
    
    pts[31].set(width, 0);
    pts[32].set(width, 10);
    pts[33].set(width, height/2 - 4);
    pts[34].set(width, height/2 - 10);
    pts[35].set(width, height/2 - 22);
    pts[36].set(width, height/2 + 4);
    pts[37].set(width, height/2 + 10);
    pts[38].set(width, height/2 + 22);
    pts[39].set(width, height - 10);
    pts[40].set(width, height);
    
    // penelty point
    pts[41].set(12, height/2);
    pts[42].set(width-12, height/2);
    
    // additional points for calibration
    // two points in the center circle
    pts.push_back(vgl_point_2d<double>(width/2 - 10, height/2));
    pts.push_back(vgl_point_2d<double>(width/2 + 10, height/2));
    
    // align with the two black line
    pts.push_back(vgl_point_2d<double>(width/2 - 10, height));
    pts.push_back(vgl_point_2d<double>(width/2 + 10, height));
    
    // intersection with 18 yard line
    pts.push_back(vgl_point_2d<double>(18, 0));
    pts.push_back(vgl_point_2d<double>(width - 18, 0));
    
    pts.push_back(vgl_point_2d<double>(18, height));
    pts.push_back(vgl_point_2d<double>(width - 18, height));
    
    // field hockey points
    pts.push_back(vgl_point_2d<double>(width/2 - 10, 11.4));
    pts.push_back(vgl_point_2d<double>(width/2 + 10, 11.4));
    
    pts.push_back(vgl_point_2d<double>(width/2 - 10, height));
    pts.push_back(vgl_point_2d<double>(width/2 + 10, height));
    
    
    // yard to meter
    for (int i = 0; i<pts.size(); i++) {
        pts[i].set(pts[i].x() * 0.9144, pts[i].y() * 0.9144);
    }
    return pts;
}

vcl_vector<vgl_point_2d<double> > WWoSSoccerCourt::getFieldHockeyPoints(const double width, const double height)
{
    vcl_vector<vgl_point_2d<double> > pts;
    
    // field hockey points
    pts.push_back(vgl_point_2d<double>(width/2 - 10, 11.4));
    pts.push_back(vgl_point_2d<double>(width/2 + 10, 11.4));
    
    pts.push_back(vgl_point_2d<double>(width/2 - 10, height));
    pts.push_back(vgl_point_2d<double>(width/2 + 10, height));
    
    
    // yard to meter
    for (int i = 0; i<pts.size(); i++) {
        pts[i].set(pts[i].x() * 0.9144, pts[i].y() * 0.9144);
    }
    
    return pts;
}

vcl_vector< vgl_line_segment_2d< double > > WWoSSoccerCourt::getSupplementaryLineSegments(const double width, const double height)
{
    vcl_vector<vgl_line_segment_2d<double> > markings;
    
    // horizontal field hockey lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(0, 11.4), vgl_point_2d<double>(width/2 - 10, 11.4)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(width/2 + 10, 11.4), vgl_point_2d<double>(width, 11.4)));
    
    // vertical field hockey lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(width/2 - 10, 11.4), vgl_point_2d<double>(width/2 - 10, height)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(width/2 + 10, 11.4), vgl_point_2d<double>(width/2 + 10, height)));
    
    // connercial board line, above the far line
  //  markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(0, height + 2.5), vgl_point_2d<double>(width/2, height + 2.5)));
  //  markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d<double>(width/2, height + 2.5), vgl_point_2d<double>(width, height + 2.5)));
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x()) * 0.9144 );
        double y1 = ( (markings[i].point1().y()) * 0.9144 ) ;
        double x2 = ( (markings[i].point2().x()) * 0.9144 );
        double y2 = ( (markings[i].point2().y()) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    return markings;
}



bool WWoSSoccerCourt::projectTopviewImage(const vil_image_view<vxl_byte> &topview, const vpgl_perspective_camera<double> & camera,
                                                     int width, int height, vil_image_view<vxl_byte> & outImage, int threshold)
{
    assert(topview.nplanes() == 3);
    
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_src;
    vcl_vector<vgl_point_2d<double> > points_dst;
    vcl_vector<vgl_point_2d<double> > points_world_dump;
    
    this->projectCourtPoints(camera, width, height, points_world_dump, points_src, points_dst, threshold);
    
    if (points_src.size() < 4) {
        return false;
    }
    assert(points_src.size() >= 4);
    assert(points_src.size() == points_dst.size());
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_src.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_src[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_dst[i]));
    }
    
    
    vgl_h_matrix_2d<double> H(src, dst);
    
    outImage = vil_image_view<vxl_byte>(width, height, 3);
    outImage.fill(0);
    
    vil_image_view<vxl_byte> temp = vil_image_view<vxl_byte>(width, height, 3);
    temp.fill(0);
    
    VilPlus::homography_warp_fill(topview, H, temp, outImage);
    return true;
}


void WWoSSoccerCourt::alignmentQuality(const vil_image_view<vxl_byte> &topview, const vil_image_view<vxl_byte> &image,
                                                  const vpgl_perspective_camera<double> & camera,
                                                  double & SSD, double & SAD, bool isAverage)
{
    
    assert(topview.nplanes() == 3);
    assert(image.nplanes() == 3);
    
    int destWidth  = image.ni();
    int destHeight = image.nj();
    
    vil_image_view<vxl_byte> warpedTopview;
    bool isWarpOk = this->projectTopviewImage(topview, camera, destWidth, destHeight, warpedTopview, 20);
    if (!isWarpOk) {
        SSD = INT_MAX;
        SAD = INT_MAX;
        return;
    }
    vil_image_view<double> weightMap;
    this->getWeightImage(camera, destWidth, destHeight, 6, 100, weightMap);
    
    SSD = 0;
    SAD = 0;
    int num = 0;
    for (int y = 0; y<destHeight; y++) {
        for (int x = 0; x<destWidth; x++) {
            if (weightMap(x, y) != 0.0) {
                double wt = weightMap(x, y);
                double dif_r = wt * abs(warpedTopview(x, y, 0) - image(x, y, 0));
                double dif_g = wt * abs(warpedTopview(x, y, 1) - image(x, y, 1));
                double dif_b = wt * abs(warpedTopview(x, y, 2) - image(x, y, 2));
                SSD += dif_r * dif_r + dif_g * dif_g + dif_b * dif_b;
                SAD += dif_r + dif_g + dif_b;
                num++;
            }
        }
    }
    if (num == 0) {
        SSD = INT_MAX;
        SAD = INT_MAX;
    }
    if (isAverage && num != 0) {
        SSD /= num;
        SAD /= num;
    }
    
    if(0)
    {
        // only used in test
        vil_image_view<vxl_byte> weightedWarpredTopview(destWidth, destHeight, 3);
        vil_image_view<vxl_byte> weightedImage(destWidth, destHeight, 3);
        weightedWarpredTopview.fill(0);
        weightedImage.fill(0);
        
        for (int y = 0; y<destHeight; y++) {
            for (int x = 0; x<destWidth; x++) {
                if (weightMap(x, y) != 0.0) {
                    double wt = weightMap(x, y);
                    for (int j = 0; j<3; j++) {
                        weightedWarpredTopview(x, y, j) = wt * warpedTopview(x, y, j);
                        weightedImage(x, y, j) = wt * image(x, y, j);
                    }
                }
            }
        }
        VilPlus::vil_save(weightMap, "weight_map.jpg");
        VilPlus::vil_save(weightedWarpredTopview, "weighted_topview.jpg");
        VilPlus::vil_save(weightedImage, "weighted_image_.jpg");
    }
}

vcl_vector<vgl_point_2d<double> > WWoSSoccerCourt::calibration_points()
{
    double width  = field_width_;
    double height = field_heigh_;
    
    vcl_vector<vgl_point_2d<double> > pts(43);
    // left goal line
    pts[0].set(0, 0);
    pts[1].set(0, 10);
    pts[2].set(0, height/2 - 4);
    pts[3].set(0, height/2 - 10);
    pts[4].set(0, height/2 - 22);
    pts[5].set(0, height/2 + 4);
    pts[6].set(0, height/2 + 10);
    pts[7].set(0, height/2 + 22);
    pts[8].set(0, height - 10);
    pts[9].set(0, height);
    
    // 6 yard
    pts[10].set(6, height/2 - 10);
    pts[11].set(6, height/2 + 10);
    
    // 10 yard
    pts[12].set(10, 0);
    pts[13].set(10, height);
    
    // 18 yard
    pts[14].set(18, height/2 - 22);
    pts[15].set(18, height/2 + 22);
    pts[16].set(18, height/2 - 8);
    pts[17].set(18, height/2 + 8);
    
    // center line
    pts[18].set(width/2, 0);
    pts[19].set(width/2, height);
    pts[20].set(width/2, height/2);
    pts[21].set(width/2, height/2 - 10);
    pts[22].set(width/2, height/2 + 10);
    
    // right side
    // 18 yard
    pts[23].set(width - 18, height/2 - 22);
    pts[24].set(width - 18, height/2 + 22);
    pts[25].set(width - 18, height/2 - 8);
    pts[26].set(width - 18, height/2 + 8);
    
    // 10 yard
    pts[27].set(width - 10, 0);
    pts[28].set(width - 10, height);
    
    // 6 yard
    pts[29].set(width - 6, height/2 - 10);
    pts[30].set(width - 6, height/2 + 10);
    
    pts[31].set(width, 0);
    pts[32].set(width, 10);
    pts[33].set(width, height/2 - 4);
    pts[34].set(width, height/2 - 10);
    pts[35].set(width, height/2 - 22);
    pts[36].set(width, height/2 + 4);
    pts[37].set(width, height/2 + 10);
    pts[38].set(width, height/2 + 22);
    pts[39].set(width, height - 10);
    pts[40].set(width, height);
    
    // penelty point
    pts[41].set(12, height/2);
    pts[42].set(width-12, height/2);
    
    // yard to meter
    for (int i = 0; i<pts.size(); i++) {
        double x = pts[i].x() * LU_YARD2METER;
        double y = pts[i].y() * LU_YARD2METER;
        pts[i].set(x, y);
    }
    return pts;
}















