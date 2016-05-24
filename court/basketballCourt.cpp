//
//  basketballCourt.cpp
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "basketballCourt.h"
#include <vnl/vnl_math.h>
#include <vicl/vicl_line_segment.h>
#include <vicl/vicl_colours.h>
#include <vil/algo/vil_gauss_filter.h>
#include <vil/algo/vil_structuring_element.h>
#include <vil/algo/vil_binary_dilate.h>
#include <vil/vil_save.h>
#include <vnl/vnl_inverse.h>
#include "vxl_plus.h"
#include <vul/vul_string+.h>
#include <vil/vil_convert.h>
#include "vpgl_plus.h"
#include "vil_plus.h"

BasketballCourt::BasketballCourt()
{
    
}

BasketballCourt::~BasketballCourt()
{
    
}


/*
 -----------------------------------   CMUBasketballCourt   ------------------------------------------
 */


CMUBasketballCourt::CMUBasketballCourt()
{
    
}

CMUBasketballCourt::~CMUBasketballCourt()
{
    
}

void CMUBasketballCourt::courtImage(int lineWidth, int line_gray, int ground_gray,  vil_image_view<vxl_byte> &image)
{
    int w = 94 * 12 + 70 * 2;
    int h = 50 * 12 + 70 * 2;
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
    image = vil_image_view<vxl_byte>(w, h, 1);
    image.fill(ground_gray);
    
    //change to inch (pixel)  / 0.3048
    vcl_vector< vxl_byte > white;
    white.push_back(line_gray);
    const double meter_to_inch = 0.3048;
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> p1 = markings[i].point1();
        vgl_point_2d<double> p2 = markings[i].point2();
        
        p1.set(p1.x()/meter_to_inch * 12, p1.y()/meter_to_inch * 12);
        p2.set(p2.x()/meter_to_inch * 12, p2.y()/meter_to_inch * 12);
        
        //flip y
        p1.set(p1.x(), 600 - p1.y());
        p2.set(p2.x(), 600 - p2.y());
        
        //add 70 + 70
        p1.set(p1.x() + 70, p1.y() + 70);
        p2.set(p2.x() + 70, p2.y() + 70);
        
        //draw to the image
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), white, lineWidth);
    }
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d<double> p1 = divisionLines[i].point1();
        vgl_point_2d<double> p2 = divisionLines[i].point2();
        
        p1.set(p1.x()/meter_to_inch * 12, p1.y()/meter_to_inch * 12);
        p2.set(p2.x()/meter_to_inch * 12, p2.y()/meter_to_inch * 12);
        
        //flip y
        p1.set(p1.x(), 600 - p1.y());
        p2.set(p2.x(), 600 - p2.y());
        
        //add 70 + 70
        p1.set(p1.x() + 70, p1.y() + 70);
        p2.set(p2.x() + 70, p2.y() + 70);
        
        //draw to the image
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), white, lineWidth);
    }
}

void CMUBasketballCourt::courtImage(vil_image_view<vxl_byte> &image)
{
    this->courtImage(2, 25, 100, image);
}

void CMUBasketballCourt::courtImageWithLogo(vil_image_view<vxl_byte> &image, int logo_gray)
{
    vcl_cout<<"CMU court has no logo!\n";
    this->courtImage(image);
}

void CMUBasketballCourt::getWeightImageWithLogo(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt)
{
    vcl_cout<<"CMU court has no logo!\n";
    getWeightImage(camera, width, height, lineWidth, gauss_sigma, wt);
}



void CMUBasketballCourt::getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt)
{    
    vil_image_view<vxl_byte> wt_black_white = vil_image_view<vxl_byte>(width, height, 1);
    wt_black_white.fill(0);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
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
    
    wt = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(gauss_sigma);
    vil_gauss_filter_5tap(wt_black_white, wt, params);
    
    for (int y = 0; y<wt.nj(); y++) {
        for (int x = 0; x<wt.ni(); x++) {
            wt(x, y, 0) /= 255.0;
        }
    }
}

void CMUBasketballCourt::overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image)
{
    assert(image.nplanes() == 3);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
   
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), vicl_colour::green, 2);
    }
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d< double > start = vgl_point_2d< double >( camera.project (vgl_homg_point_3d< double >( divisionLines[i].point1().x(), divisionLines[i].point1().y(), 0 ) ) );
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project( vgl_homg_point_3d< double >( divisionLines[i].point2().x(), divisionLines[i].point2().y(), 0 ) ) );
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), vicl_colour::green, 3);
    }
}

void CMUBasketballCourt::overlayPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image)
{
    assert(image.nplanes() == 3);
    
    vcl_vector<vgl_point_2d<double> > points = CMUBasketballCourt::getCourtPoints();
    
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
            
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), vicl_colour::blue, 3);
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q3, q4), vicl_colour::blue, 3);
        }
    }
}

vgl_transform_2d< double > CMUBasketballCourt::imageToWorld()
{
    double m1[9] = {
        1, 0, -70,
        0, 1, -70,
        0, 0,  1};
    double m2[9] = {
        1, 0, 0,
        0, -1, 600,
        0, 0, 1};
    //inch to meter
    double m3[9] = {
        0.0254, 0, 0,
        0, 0.0254, 0,
        0, 0, 1
    };
    
    vgl_transform_2d< double > model = vgl_transform_2d<double>(vnl_matrix_fixed<double, 3, 3>(m3) * vnl_matrix_fixed<double, 3, 3>(m2) * vnl_matrix_fixed<double, 3, 3>(m1));
    return model;
}


void CMUBasketballCourt::getProjectedPoints(const vpgl_perspective_camera<double> &camera, int width, int height, vcl_vector<vgl_point_2d<double> > &outPts)
{
    assert(outPts.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > courtPts = getPointsOnEdges();
    for (unsigned int i = 0; i<courtPts.size(); i++) {
        vgl_homg_point_3d<double> p(courtPts[i].x(), courtPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        if (q.x() >= 0 && q.x() < width && q.y() >= 0 && q.y() < height) {
            outPts.push_back(q);
        }
    }
}




vcl_vector<vgl_point_2d<double> > CMUBasketballCourt::getCourtPoints()
{
    vcl_vector< vgl_point_2d<double> > markings;
    
    //corners
    markings.push_back(vgl_point_2d<double>(0, 0));
    markings.push_back(vgl_point_2d<double>(94, 0));
    markings.push_back(vgl_point_2d<double>(94, 50));
    markings.push_back(vgl_point_2d<double>(0, 50));
    
    //left small poly
    markings.push_back(vgl_point_2d<double>(0, 19));
    markings.push_back(vgl_point_2d<double>(19, 19));
    markings.push_back(vgl_point_2d<double>(19, 31));
    markings.push_back(vgl_point_2d<double>(0, 31));
    
    //right small poly
    markings.push_back(vgl_point_2d<double>(75, 19));
    markings.push_back(vgl_point_2d<double>(94, 19));
    markings.push_back(vgl_point_2d<double>(94, 31));
    markings.push_back(vgl_point_2d<double>(75, 31));
    
    //point on the arc
    markings.push_back(vgl_point_2d<double>(25, 25));
    markings.push_back(vgl_point_2d<double>(69, 25));
    
    //dashed arc
    markings.push_back(vgl_point_2d<double>(13, 25));
    markings.push_back(vgl_point_2d<double>(81, 25));
    
    //center point on the boundary and two --|--
    markings.push_back(vgl_point_2d<double>(47, 50));
    markings.push_back(vgl_point_2d<double>(47, 0));
    markings.push_back(vgl_point_2d<double>(28, 50));
    markings.push_back(vgl_point_2d<double>(66, 50));
    
    //3 point line arc center (boy)
    markings.push_back(vgl_point_2d<double>(26, 25));
    markings.push_back(vgl_point_2d<double>(68, 25));
    
    //2 point line arc center (professional)
    markings.push_back(vgl_point_2d<double>(29, 25));
    markings.push_back(vgl_point_2d<double>(65, 25));
    
    //2 fake point from intersection lines (left) to balance point distribution
    markings.push_back(vgl_point_2d<double>(11,  0));
    markings.push_back(vgl_point_2d<double>(11, 50));
    
    //2 fake point from intersection lines right
    markings.push_back(vgl_point_2d<double>(94 - 11,  0));
    markings.push_back(vgl_point_2d<double>(94 - 11, 50));
    
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> pt = markings[i];
        markings[i].set(pt.x() * 0.3048, pt.y() * 0.3048);
    }
    
    return markings;
}


vcl_vector< vgl_line_segment_2d< double > > CMUBasketballCourt::getLineSegments()
{
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // everything below is specified in inches, and the origin at the center of the court
    // afterwards I'll convert to the regular conventions (meters with origin in bottom left)
    
    // inner boundary:use short line instead of long lines
 //   markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( -47 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( +19 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +19 * 12, +25 * 12 ), vgl_point_2d< double >( 0 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, +25 * 12 ), vgl_point_2d< double >( -19 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -19 * 12, +25 * 12 ), vgl_point_2d< double >( 0 * 12, +25 * 12 ) ) );
    
 //   markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, -25 * 12 ), vgl_point_2d< double >( -47 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, -25 * 12 ), vgl_point_2d< double >( +19 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +19 * 12, -25 * 12 ), vgl_point_2d< double >( 0 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, -25 * 12 ), vgl_point_2d< double >( -19 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -19 * 12, -25 * 12 ), vgl_point_2d< double >( 0 * 12, -25 * 12 ) ) );

    
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( +47 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, +25 * 12 ), vgl_point_2d< double >( -47 * 12, -25 * 12 ) ) );
    
    //axes
    //markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 0, 0 ), vgl_point_2d< double >( 2 * 12, 0 ) ) );
  //  markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 0, 0 ), vgl_point_2d< double >( 0, 2 * 12 ) ) );
    
    // division line
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 0, -25 * 12 ), vgl_point_2d< double >( 0, + 25 * 12 ) ) );
    
    // symmetric for left and right court field
    for ( int dx = -1; dx <= +1; dx += 2 )
    {
        // division line
    //    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( dx, -25 * 12 ), vgl_point_2d< double >( dx, + 25 * 12 ) ) );
        
        // bench into court
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 19 * 12 * dx, 25 * 12 ), vgl_point_2d< double >( 19 * 12 * dx, +22 * 12 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 19 * 12 * dx, 22 * 12 ), vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, +22 * 12 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, 22 * 12 ), vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, +25 * 12 ) ) );
        
        // bench out to court, 2 inch width
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 19 * 12 * dx, 25 * 12 ), vgl_point_2d< double >( 19 * 12 * dx, +28 * 12 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 19 * 12 * dx, 28 * 12 ), vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, +28 * 12 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, 28 * 12 ), vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, +25 * 12 ) ) );
        
        
        // key
        for ( int dy = -1; dy <= +1; dy +=2 )
        {
            // inside lengthwise
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12 * dx, ( +6 * 12 - 2 ) * dy ), vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, ( +6 * 12 - 2 ) * dy ) ) );
            
            
            // outside lengthwise
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12 * dx, +6 * 12 * dy), vgl_point_2d< double >( +40 * 12 * dx, +6 * 12 * dy ) ) );
            
            // in feet length bar
       //     markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 6 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 6 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 4 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 4 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 2 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 2 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 0 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 0 ) * dy ) ) );
       //     markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +39 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +36 * 12 * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +36 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +36 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +36 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx , +6 * 12 * dy ), vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( +28 * 12 * dx, +6 * 12 * dy ) ) );
        }
        
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, +6 * 12 - 2 ), vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, -6 * 12 + 2 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 28 * 12 * dx, +6 * 12 - 2 ), vgl_point_2d< double >( 28 * 12 * dx, -6 * 12 + 2 ) ) );
        
        
        // small arc
        for ( unsigned int i = 0; i < 32; ++i )
        {
            double startTheta = vnl_math::pi / 32 * i + vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 32 * ( i + 1 ) + vnl_math::pi / 2;
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( startTheta ) * 6 * 12 ),
                                                              vgl_point_2d< double >( ( vcl_cos( stopTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( stopTheta ) * 6 * 12 ) ) );
        }
        
        // 3 point line
        for ( unsigned int i = 0; i < 64; ++i )
        {
            double startTheta = vnl_math::pi / 64 * i + vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 64 * ( i + 1 ) + vnl_math::pi / 2;
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (20 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (20 * 12 + 9) ),
                                                              vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (20 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (20 * 12 + 9) ) ) );
        }
        
        // female 3 point line
        if (0)
        {
            for ( unsigned int i = 0; i < 64; ++i )
            {
                double startTheta = vnl_math::pi / 64 * i + vnl_math::pi / 2;
                double stopTheta = vnl_math::pi / 64 * ( i + 1 ) + vnl_math::pi / 2;
                
                markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (25 * 12 - 63) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (25 * 12 - 63) ),
                                                                  vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (25 * 12 - 63) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (25 * 12 - 63) ) ) );
            }
        }
        
        
        if(1)
        {
            //center circle
            for ( unsigned int i = 0; i < 64; ++i )
            {
                double startTheta = vnl_math::pi / 64 * i + vnl_math::pi / 2;
                double stopTheta = vnl_math::pi / 64 * ( i + 1 ) + vnl_math::pi / 2;
                
                markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (6 * 12) ) * dx, vcl_sin( startTheta ) * (6 * 12) ),
                                                                  vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (6 * 12) ) * dx, vcl_sin( stopTheta ) * (6 * 12) ) ) );
            }
        }
        
        // samll arc under
        {
            for ( unsigned int i = 0; i < 32; ++i )
            {
                double startTheta = vnl_math::pi / 32 * i + vnl_math::pi / 2;
                double stopTheta = vnl_math::pi / 32 * ( i + 1 ) + vnl_math::pi / 2;
                
                markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * 3 * 12 + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (3 * 12) ),
                                                                  vgl_point_2d< double >( ( vcl_cos( stopTheta ) * 3 * 12 + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (3 * 12) ) ) );
            }
        }
        
        
        
    }
    
    // coord systems
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( markings[i].point1().x() / 12.0 + 47.0 ) * 0.3048;
        double y1 = ( markings[i].point1().y() / 12.0 + 25.0 ) * 0.3048;
        double x2 = ( markings[i].point2().x() / 12.0 + 47.0 ) * 0.3048;
        double y2 = ( markings[i].point2().y() / 12.0 + 25.0 ) * 0.3048;
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    
    return markings;
}



vcl_vector< vgl_line_segment_2d< double > > CMUBasketballCourt::getDivisionLine()
{
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 0, -25 * 12 ), vgl_point_2d< double >( 0, + 25 * 12 ) ) );
    
    // coord systems
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( markings[i].point1().x() / 12.0 + 47.0 ) * 0.3048;
        double y1 = ( markings[i].point1().y() / 12.0 + 25.0 ) * 0.3048;
        double x2 = ( markings[i].point2().x() / 12.0 + 47.0 ) * 0.3048;
        double y2 = ( markings[i].point2().y() / 12.0 + 25.0 ) * 0.3048;
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    
    return markings;
}

vcl_vector< vgl_point_2d<double> > CMUBasketballCourt::getPointsOnEdges()
{
    vcl_vector<vgl_point_2d<double> > court_points;
    
  //  markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( -47 * 12, +25 * 12 ) ) );
    {
        for (int x = -47; x<= 47; x++) {
            int y = +25 * 12;
            court_points.push_back(vgl_point_2d<double>(x * 12, y));
            
            y = -25 * 12;
            court_points.push_back(vgl_point_2d<double>(x * 12, y));
        }
        
        for (int y = -25; y<=25; y++) {
            int x = -47 * 12;
            court_points.push_back(vgl_point_2d<double>(x, y * 12));
            
            x = +47 * 12;
            court_points.push_back(vgl_point_2d<double>(x, y * 12));
        }
    }
    
    for (unsigned int i = 0; i<court_points.size(); i++) {
        double px = (court_points[i].x()/12.0 + 47.0) * 0.3048;
        double py = (court_points[i].y()/12.0 + 25.0) * 0.3048;
        
        court_points[i].set(px, py);
    }
    
    return court_points;
}


/*
    ----------------------------------------------- DisneyWorldBasketballCourt ------------------------------------------------
 */
DisneyWorldBasketballCourt::DisneyWorldBasketballCourt()
{
    int w = 94 * 12 + 70 * 2;
    int h = 50 * 12 + 70 * 2;
    
    m_logoImage = vil_image_view<vxl_byte>(w, h, 4);
    m_logoImage.fill(0);
}

DisneyWorldBasketballCourt::~DisneyWorldBasketballCourt()
{
    
   
    
}

void DisneyWorldBasketballCourt::setDefaultLogoImage()
{
    vcl_string file = vcl_string("/Users/jimmy/Data/PTZ_calibration/disney_world_logo/wdw_logo.png");
    vil_image_view<vxl_byte> logoImage = vil_load(file.c_str());
    
    double H1_data[9] = {
        0.101058, 0, 688.686,
        0, 0.101058, 557.952,
        0, 0, 1
    };
   
    
    double H2_data[9] = {
        0.106866, 0, 378.401,
        0, 0.106866, 139.527,
        0, 0, 1
    };
    
    
    vgl_h_matrix_2d<double> leftLogoTrans(H2_data);
    vgl_h_matrix_2d<double> rightLogoTrans(H1_data);
    
    this->setLogoImage(logoImage, leftLogoTrans);
    this->setLogoImage(logoImage, rightLogoTrans);
    vcl_cout<<"load default logo file from "<<file<<vcl_endl;
    vcl_cout<<"left logo transform "<<leftLogoTrans<<vcl_endl;
    vcl_cout<<"right logo transform "<<rightLogoTrans<<vcl_endl;
    
    //center logo
    file = vcl_string("/Users/jimmy/Data/PTZ_calibration/disney_world_logo/espn_center_logo_blur.png");
    vil_image_view<vxl_byte> centerLogoImage = vil_load(file.c_str());

    double Hc_data[9] = {
        0.277622, 0, 496.352,
        0, 0.277622, 239.961,
        0, 0, 1
    };
    vgl_h_matrix_2d<double> centerLogoTrans(Hc_data);
    this->setLogoImage(centerLogoImage, centerLogoTrans);
    vcl_cout<<"center logo file from "<<file<<vcl_endl;
    vcl_cout<<"center logo transform "<<centerLogoTrans<<vcl_endl;    
}

void DisneyWorldBasketballCourt::setLogoImage(const vil_image_view<vxl_byte> &logoImage, const vgl_h_matrix_2d<double> &transform)
{
    assert(logoImage.nplanes() == 4);
    
    vil_image_view<vxl_byte> outImage;
    outImage.deep_copy(m_logoImage);
    VilPlus::homography_warp_fill(logoImage, transform, m_logoImage, outImage);
    
    m_logoImage = outImage;
}

void DisneyWorldBasketballCourt::setLogoPoints(const vgl_transform_2d<double> & transform)
{
    // manually selected pts
    vcl_vector<vgl_point_2d<double> > pts;
    // w
    pts.push_back(vgl_point_2d<double>(192, 23));
    pts.push_back(vgl_point_2d<double>(118, 52));
    pts.push_back(vgl_point_2d<double>(126, 240));
    pts.push_back(vgl_point_2d<double>(145, 138));
    
    //a
    pts.push_back(vgl_point_2d<double>(320, 201));
    pts.push_back(vgl_point_2d<double>(297, 261));
    
    //l 383 258
    pts.push_back(vgl_point_2d<double>(383, 258));
    
    //t 416 162, 460, 331
    pts.push_back(vgl_point_2d<double>(416, 162));
    pts.push_back(vgl_point_2d<double>(460, 331));
    
    
    //D 538 221, 494 67, 721 221, 606 273
    pts.push_back(vgl_point_2d<double>(538, 221));
    pts.push_back(vgl_point_2d<double>(494, 67));
    pts.push_back(vgl_point_2d<double>(721, 221));
    pts.push_back(vgl_point_2d<double>(606, 273));
    
    //I 846 132, 850 297
    pts.push_back(vgl_point_2d<double>(846, 132));
    pts.push_back(vgl_point_2d<double>(850, 297));
    
    //S 896 205
    pts.push_back(vgl_point_2d<double>(896, 205));
    
    //N 1076 253, 1061 155
    pts.push_back(vgl_point_2d<double>(1076, 253));
    pts.push_back(vgl_point_2d<double>(1061, 155));
    
    
    //E 1135 266, 1155 200
    pts.push_back(vgl_point_2d<double>(1135, 266));
    pts.push_back(vgl_point_2d<double>(1155, 200));
    
    //Y 1304 202, 1213 401
    pts.push_back(vgl_point_2d<double>(1304, 202));
    pts.push_back(vgl_point_2d<double>(1213, 401));
    
    //W 1394, 298  1474, 298  1546, 128 1320, 128
    pts.push_back(vgl_point_2d<double>(1394, 298));
    pts.push_back(vgl_point_2d<double>(1474, 298));
    pts.push_back(vgl_point_2d<double>(1546, 128));
    pts.push_back(vgl_point_2d<double>(1320, 128));
    
    //O 1566, 185
    pts.push_back(vgl_point_2d<double>(1566, 185));
    
    //R  1686 294  1665, 180
    pts.push_back(vgl_point_2d<double>(1686, 294));
    pts.push_back(vgl_point_2d<double>(1665, 180));
    
    //L  1744, 123
    pts.push_back(vgl_point_2d<double>(1744, 123));
    
    //D  1858, 123  1876, 282
    pts.push_back(vgl_point_2d<double>(1858, 123));
    pts.push_back(vgl_point_2d<double>(1876, 282));
    
    this->setLogoPoints(pts, transform);    
}

void DisneyWorldBasketballCourt::setLogoPoints(const vcl_vector<vgl_point_2d<double> > & points, const vgl_transform_2d<double> & transform)
{
    m_logoPoints.resize(points.size());
   
    vgl_transform_2d<double> trans_to_world = this->imageToWorld();
    
    for (int i = 0; i<points.size(); i++) {
        vgl_point_2d<double> p = points[i];     //position in logo image
        vgl_point_2d<double> p1 = transform(p); //position in court image
        vgl_point_2d<double> p2 = trans_to_world(p1); //position in world coordinate
        m_logoPoints[i] = p2;
    }
}

void DisneyWorldBasketballCourt::courtImage(vil_image_view<vxl_byte> &image)
{
    this->courtImage(2, 100, 200, image);
}

void DisneyWorldBasketballCourt::courtImageWithLogo(vil_image_view<vxl_byte> &image, int logo_gray)
{
    this->courtImage(2, 100, 200, image);
    
    assert(image.ni() == m_logoImage.ni() && image.nj() == m_logoImage.nj());
    
    // for two "Disney world logo"
    for (int j = 0; j<image.nj(); j++) {
        for (int i = 0; i<image.ni(); i++) {
            if (m_logoImage(i, j, 3) != 0) {
                image(i, j, 0) = logo_gray;
            }
        }
    }
    
    // for center logo
    double rgb_to_gray[3] = {0.2125, 0.7154, 0.0721};
    for (int j = 0; j<image.nj(); j++) {
        for (int i = 0; i<image.ni(); i++) {
            if (m_logoImage(i, j, 3) != 0) {
                double r = m_logoImage(i, j, 0);
                double g = m_logoImage(i, j, 1);
                double b = m_logoImage(i, j, 2);
                int gray = r * rgb_to_gray[0] + g * rgb_to_gray[1] + b * rgb_to_gray[2];
                if (gray > 255) {
                    gray = 255;
                }
                image(i, j, 0) = gray;
            }
        }
    }
    
}

void DisneyWorldBasketballCourt::courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image)
{
    int w = 94 * 12 + 70 * 2;
    int h = 50 * 12 + 70 * 2;
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
    image = vil_image_view<vxl_byte>(w, h, 1);
    image.fill(ground_gray);
    
    //change to inch (pixel)  / 0.3048
    vcl_vector< vxl_byte > white;
    white.push_back(line_gray);
    const double meter_to_inch = 0.3048;
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> p1 = markings[i].point1();
        vgl_point_2d<double> p2 = markings[i].point2();
        
        p1.set(p1.x()/meter_to_inch * 12, p1.y()/meter_to_inch * 12);
        p2.set(p2.x()/meter_to_inch * 12, p2.y()/meter_to_inch * 12);
        
        //flip y
        p1.set(p1.x(), 600 - p1.y());
        p2.set(p2.x(), 600 - p2.y());
        
        //add 70 + 70
        p1.set(p1.x() + 70, p1.y() + 70);
        p2.set(p2.x() + 70, p2.y() + 70);
        
        //draw to the image
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), white, lineWidth);
    }
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d<double> p1 = divisionLines[i].point1();
        vgl_point_2d<double> p2 = divisionLines[i].point2();
        
        p1.set(p1.x()/meter_to_inch * 12, p1.y()/meter_to_inch * 12);
        p2.set(p2.x()/meter_to_inch * 12, p2.y()/meter_to_inch * 12);
        
        //flip y
        p1.set(p1.x(), 600 - p1.y());
        p2.set(p2.x(), 600 - p2.y());
        
        //add 70 + 70
        p1.set(p1.x() + 70, p1.y() + 70);
        p2.set(p2.x() + 70, p2.y() + 70);
        
        //draw to the image
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), white, 1); // lineWidth * 1.5
    }
}

void DisneyWorldBasketballCourt::getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt)
{
    vil_image_view<vxl_byte> wt_black_white = vil_image_view<vxl_byte>(width, height, 1);
    wt_black_white.fill(0);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
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
    
    wt = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(gauss_sigma);
    vil_gauss_filter_5tap(wt_black_white, wt, params);
    
    for (int y = 0; y<wt.nj(); y++) {
        for (int x = 0; x<wt.ni(); x++) {
            wt(x, y, 0) /= 255.0;
        }
    }
}

void DisneyWorldBasketballCourt::getWeightImageWithLogo(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt)
{
    vil_image_view<vxl_byte> wt_black_white = vil_image_view<vxl_byte>(width, height, 1);
    wt_black_white.fill(0);
    
    // line segment of court
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
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
    
    // line segment over logo area
    vcl_vector<vgl_line_segment_2d<double> > logos = getLogoAreaLineSegments(2);
    for ( unsigned int i = 0; i < logos.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( logos[i].point1().x(), logos[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( logos[i].point2().x(), logos[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        vicl_overlay_line_segment(wt_black_white, vgl_line_segment_2d< double >( start, stop ), color_white, 3);
    } 
    
    wt = vil_image_view<double>(width, height, 1);
    vil_gauss_filter_5tap_params params(gauss_sigma);
    vil_gauss_filter_5tap(wt_black_white, wt, params);
 //   vil_gauss_filter_2d(wt_black_white, wt, 1.0, 5);
    
    for (int y = 0; y<wt.nj(); y++) {
        for (int x = 0; x<wt.ni(); x++) {
            wt(x, y, 0) /= 255.0;
        }
    }
    
}

void DisneyWorldBasketballCourt::overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image)
{
    assert(image.nplanes() == 3);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), vicl_colour::green, 2);
    }
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d< double > start = vgl_point_2d< double >( camera.project (vgl_homg_point_3d< double >( divisionLines[i].point1().x(), divisionLines[i].point1().y(), 0 ) ) );
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project( vgl_homg_point_3d< double >( divisionLines[i].point2().x(), divisionLines[i].point2().y(), 0 ) ) );
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), vicl_colour::green, 3);
    }    
}

void DisneyWorldBasketballCourt::overlayAllLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image, const vcl_vector<vxl_byte> & color)
{
    assert(image.nplanes() == 3);
    assert(color.size() == 3);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getAllLineSegments();
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), color, 2);
    }
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d< double > start = vgl_point_2d< double >( camera.project (vgl_homg_point_3d< double >( divisionLines[i].point1().x(), divisionLines[i].point1().y(), 0 ) ) );
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project( vgl_homg_point_3d< double >( divisionLines[i].point2().x(), divisionLines[i].point2().y(), 0 ) ) );
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), color, 3);
    }
}

void DisneyWorldBasketballCourt::getAllLines(const vpgl_perspective_camera<double> & camera, int width, int height, vcl_vector< vgl_line_segment_2d< double > > &lines)
{
    vcl_vector< vgl_line_segment_2d< double > > markings = getAllLineSegments();
    
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
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d< double > start = vgl_point_2d< double >( camera.project (vgl_homg_point_3d< double >( divisionLines[i].point1().x(), divisionLines[i].point1().y(), 0 ) ) );
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project( vgl_homg_point_3d< double >( divisionLines[i].point2().x(), divisionLines[i].point2().y(), 0 ) ) );
        
        lines.push_back(vgl_line_segment_2d< double >( start, stop ));
    }
}

void DisneyWorldBasketballCourt::overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image, const vcl_vector< vxl_byte >& colour)
{
    assert(colour.size() == 3);
    assert(image.nplanes() == 3);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));        
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), colour, 2);
    }
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d< double > start = vgl_point_2d< double >( camera.project (vgl_homg_point_3d< double >( divisionLines[i].point1().x(), divisionLines[i].point1().y(), 0 ) ) );
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project( vgl_homg_point_3d< double >( divisionLines[i].point2().x(), divisionLines[i].point2().y(), 0 ) ) );
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), colour, 3);
    }
}

void DisneyWorldBasketballCourt::projectLines(const vpgl_proj_camera<double> & camera, vil_image_view<vxl_byte> & image)
{
    assert(image.nplanes() == 3);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = getLineSegments();
    
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( markings[i].point1().x(), markings[i].point1().y(), 0, 1.0 );
        vgl_homg_point_3d< double > p2( markings[i].point2().x(), markings[i].point2().y(), 0, 1.0 );
        
        vgl_point_2d< double > start = vgl_point_2d< double >(camera.project(p1));
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project(p2));
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), vicl_colour::green, 2);
    }
    
    vcl_vector< vgl_line_segment_2d< double > > divisionLines = getDivisionLine();
    for (int i = 0; i<divisionLines.size(); i++) {
        vgl_point_2d< double > start = vgl_point_2d< double >( camera.project (vgl_homg_point_3d< double >( divisionLines[i].point1().x(), divisionLines[i].point1().y(), 0 ) ) );
        vgl_point_2d< double > stop = vgl_point_2d< double >( camera.project( vgl_homg_point_3d< double >( divisionLines[i].point2().x(), divisionLines[i].point2().y(), 0 ) ) );
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d< double >( start, stop ), vicl_colour::green, 3);
    }
}

void DisneyWorldBasketballCourt::getProjectedCourtArea(const vpgl_perspective_camera<double> & camera, int width, int height, vil_image_view<vxl_byte> & alpha)
{
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_src;
    vcl_vector<vgl_point_2d<double> > points_dst;
    vcl_vector<vgl_point_2d<double> > points_world_dump;
    
    this->projectCourtPoints(camera, width, height, points_world_dump, points_src, points_dst, 20);
    
    assert(points_src.size() >= 4);
    assert(points_src.size() == points_dst.size());
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_src.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_src[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_dst[i]));
    }
   
    
    vgl_h_matrix_2d<double> H(src, dst);
    
    //black whit court image
    int w = 94 * 12 + 70 * 2;
    int h = 50 * 12 + 70 * 2;
    
    vil_image_view<vxl_byte> image = vil_image_view<vxl_byte>(w, h, 1);
    image.fill(0);
    for (int y = 70; y <= 50 * 12 + 70; y++) {
        for (int x = 70; x <= 94* 12 + 70; x++) {
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

bool DisneyWorldBasketballCourt::getCourtArareInTopview(const vpgl_perspective_camera<double> & camera, int width, int height, vil_image_view<vxl_byte> & alpha)
{
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_src;
    vcl_vector<vgl_point_2d<double> > points_dst;
    vcl_vector<vgl_point_2d<double> > points_world_dump;
    
    this->projectCourtPoints(camera, width, height, points_world_dump, points_src, points_dst, 20);
    
    if (points_src.size() < 4) {
        return false;
    }
   
    assert(points_src.size() == points_dst.size());
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_src.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_src[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_dst[i]));
    }
    
    
    vgl_h_matrix_2d<double> H(src, dst);
    
    //black whit court image
    int w = 94 * 12 + 70 * 2;
    int h = 50 * 12 + 70 * 2;
    
    vil_image_view<vxl_byte> courtImageTopview = vil_image_view<vxl_byte>(w, h, 1);
    courtImageTopview.fill(0);
    for (int y = 70; y <= 50 * 12 + 70; y++) {
        for (int x = 70; x <= 94* 12 + 70; x++) {
            courtImageTopview(x, y, 0) = 255;
        }
    }
    
    vil_image_view<vxl_byte> blackPattern(width, height, 1);
    blackPattern.fill(0);
    
    vil_image_view<vxl_byte> outImage;  // white black pattern in image space
    outImage.deep_copy(blackPattern);
    
    //warp court image to outImage
    // alpha as a white black pattern
    VilPlus::homography_warp_fill(courtImageTopview, H, blackPattern, outImage);
    
    // warp in image space to topview space
    blackPattern = vil_image_view<vxl_byte>(w, h, 1);
    blackPattern.fill(0);
    
    alpha = vil_image_view<vxl_byte>(w, h, 1);
    
    VilPlus::homography_warp_fill(outImage, H.get_inverse(), blackPattern, alpha);

    return true;
}

void DisneyWorldBasketballCourt::getProjectedCourtArea(const vpgl_perspective_camera<double> & camera, int width, int height,
                                                       vil_image_view<vxl_byte> & alpha,
                                                       const vgl_point_2d<int> & startP, const vgl_point_2d<int> & endP)
{
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_src;
    vcl_vector<vgl_point_2d<double> > points_dst;
    vcl_vector<vgl_point_2d<double> > points_world_dump;
    
    this->projectCourtPoints(camera, width, height, points_world_dump, points_src, points_dst, 20);
    
    assert(points_src.size() >= 4);
    assert(points_src.size() == points_dst.size());
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_src.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_src[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_dst[i]));
    }
    
    
    vgl_h_matrix_2d<double> H(src, dst);
    
    int w = 94 * 12 + 70 * 2;
    int h = 50 * 12 + 70 * 2;
    
    vil_image_view<vxl_byte> image = vil_image_view<vxl_byte>(w, h, 1);
    image.fill(0);
    for (int y = startP.y(); y <= endP.y(); y++) {
        for (int x = startP.x(); x <= endP.x(); x++) {
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

bool DisneyWorldBasketballCourt::projectTopviewImage(const vil_image_view<vxl_byte> &topview, const vpgl_perspective_camera<double> & camera,
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

void DisneyWorldBasketballCourt::overlayPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image)
{
    assert(image.nplanes() == 3);
    
    vcl_vector<vgl_point_2d<double> > points = DisneyWorldBasketballCourt::getCourtPoints();
    
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
            
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), vicl_colour::blue, 3);
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q3, q4), vicl_colour::blue, 3);
        }
    }
    
    // logo points
    for (int i = 0; i<m_logoPoints.size(); i++) {
        vgl_homg_point_3d<double> p(m_logoPoints[i].x(), m_logoPoints[i].y(), 0, 1.0);
        vgl_point_2d<double> q = vgl_point_2d<double>(camera.project(p));
        if (q.x() >= 0 && q.x() < image.ni() && q.y() >= 0 && q.y() < image.nj()) {
            int w = 6;
            
            vgl_point_2d<double> q1, q2, q3, q4;
            q1.set(q.x(), q.y() - w);
            q2.set(q.x(), q.y() + w);
            q3.set(q.x() - w, q.y());
            q4.set(q.x() + w, q.y());
            
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), vicl_colour::blue, 3);
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q3, q4), vicl_colour::blue, 3);
        }
    }
}

void DisneyWorldBasketballCourt::overlayKeyPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image)
{
    assert(image.nplanes() == 3);
    
    vcl_vector<vgl_point_2d<double> > wld_pts = DisneyWorldBasketballCourt::getCourtKeyPoints();
    
    for (int i = 0; i<wld_pts.size(); i++) {
        vgl_homg_point_3d<double> p(wld_pts[i].x(), wld_pts[i].y(), 0, 1.0);
        vgl_point_2d<double> q = vgl_point_2d<double>(camera.project(p));
        if (q.x() >= 0 && q.x() < image.ni() && q.y() >= 0 && q.y() < image.nj()) {
            int w = 6;
            
            vgl_point_2d<double> q1, q2, q3, q4;
            q1.set(q.x(), q.y() - w);
            q2.set(q.x(), q.y() + w);
            q3.set(q.x() - w, q.y());
            q4.set(q.x() + w, q.y());
            
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), vicl_colour::blue, 3);
            vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q3, q4), vicl_colour::blue, 3);
        }
    }
}

void DisneyWorldBasketballCourt::getImageKeyPoints(const vpgl_perspective_camera<double> & camera, int width, int height, vcl_vector<vgl_point_2d<double> > & pts)
{
    vcl_vector<vgl_point_2d<double> > wld_pts = DisneyWorldBasketballCourt::getCourtKeyPoints();
    
    for (int i = 0; i<wld_pts.size(); i++) {
        vgl_homg_point_3d<double> p(wld_pts[i].x(), wld_pts[i].y(), 0, 1.0);
        vgl_point_2d<double> q = vgl_point_2d<double>(camera.project(p));
        if (q.x() >= 0 && q.x() < width && q.y() >= 0 && q.y() < height) {
            pts.push_back(q);
        }
    }
}

void DisneyWorldBasketballCourt::getWorldImageKeyPoints(const vpgl_perspective_camera<double> & camera, int width, int height,
                                                        vcl_vector<vgl_point_2d<double> > & wld_pts_out, vcl_vector<vgl_point_2d<double> > & img_pts)
{
    vcl_vector<vgl_point_2d<double> > wld_pts = DisneyWorldBasketballCourt::getCourtKeyPoints();
    
    for (int i = 0; i<wld_pts.size(); i++) {
        vgl_homg_point_3d<double> p(wld_pts[i].x(), wld_pts[i].y(), 0, 1.0);
        vgl_point_2d<double> q = vgl_point_2d<double>(camera.project(p));
        if (q.x() >= 0 && q.x() < width && q.y() >= 0 && q.y() < height) {
            img_pts.push_back(q);
            wld_pts_out.push_back(wld_pts[i]);
        }
    }
}



void DisneyWorldBasketballCourt::getPorjectedPointsEdgeOrientation(const vpgl_perspective_camera<double> &camera, int width, int height, vcl_vector<vgl_point_2d<double> > &outPts,
                                                           vcl_vector<vnl_vector_fixed<double, 2> > &outEdgeOrientation)
{
    assert(outPts.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > courtPts;
    vcl_vector<vnl_vector_fixed<double, 4> > p_to_q;
    
    getPointsDirectionOnEdges(courtPts, p_to_q);
    
    for (unsigned int i = 0; i<courtPts.size(); i++) {
        vgl_homg_point_3d<double> p(courtPts[i].x(), courtPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        if (q.x() >= 0 && q.x() < width && q.y() >= 0 && q.y() < height) {
            outPts.push_back(q);
            
            vgl_homg_point_3d<double> p1(p_to_q[i][0], p_to_q[i][1], 0, 1.0);
            vgl_homg_point_3d<double> p2(p_to_q[i][2], p_to_q[i][3], 0, 1.0);
            
            vgl_point_2d<double> proj_p1 = (vgl_point_2d<double>)camera.project(p1);
            vgl_point_2d<double> proj_p2 = (vgl_point_2d<double>)camera.project(p2);
            vnl_vector_fixed<double, 2> n(proj_p2.y() - proj_p1.y(), -(proj_p2.x() - proj_p1.x()));  //perpendicular to line direction
            n.normalize();
            
            outEdgeOrientation.push_back(n);
        }
    }
}

void DisneyWorldBasketballCourt::projectCourtPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                                    vcl_vector<vgl_point_2d<double> > & points_world,
                                                    vcl_vector<vgl_point_2d<double> > & points_court_image,
                                                    vcl_vector<vgl_point_2d<double> > & points_camera_image,
                                                    int threshold)
{
    assert(points_world.size() == 0);
    assert(points_court_image.size() == 0);
    assert(points_camera_image.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > courtPts = DisneyWorldBasketballCourt::getCourtPoints();
    
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
    
    DisneyWorldBasketballCourt court;
    
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

void DisneyWorldBasketballCourt::projectCalibPoints(const vpgl_perspective_camera<double> &camera, int width, int height,
                                                    vcl_vector<vgl_point_2d<double> > & points_world,     //meter
                                                    vcl_vector<vgl_point_2d<double> > & points_court_image, //pixel
                                                    vcl_vector<vgl_point_2d<double>> & points_camera_image,
                                                    int threshold) // threshold away from the image boundary
{
    assert(points_world.size() == 0);
    assert(points_court_image.size() == 0);
    assert(points_camera_image.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > courtPts = DisneyWorldBasketballCourt::getCalibPoints();
    
    
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
    
    DisneyWorldBasketballCourt court;
    
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

void DisneyWorldBasketballCourt::projectCourtPoints(const vpgl_perspective_camera<double> & camera,
                                                    vcl_vector<vgl_point_2d<double> > & pts_world,
                                                    vcl_vector<vgl_point_2d<double> > & pts_image)
{
    assert(pts_world.size() == 0);
    assert(pts_image.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > calibPts = DisneyWorldBasketballCourt::getCalibPoints();
    
    for (int i = 0; i<calibPts.size(); i++) {
        vgl_homg_point_3d<double> p(calibPts[i].x(), calibPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        pts_world.push_back(calibPts[i]);
        pts_image.push_back(q);
    }
    assert(pts_world.size() == pts_image.size());
}

void DisneyWorldBasketballCourt::projectCourtPoints(const vpgl_perspective_camera<double> & camera, int width, int height,
                                                    vcl_vector<vgl_point_2d<double> > & pts_world,
                                                    vcl_vector<vgl_point_2d<double> > & pts_image)
{
    assert(pts_world.size() == 0);
    assert(pts_image.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > calibPts = DisneyWorldBasketballCourt::getCalibPoints();
    
    for (int i = 0; i<calibPts.size(); i++) {
        vgl_homg_point_3d<double> p(calibPts[i].x(), calibPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        if (vgl_inside_image(q, width, height)) {
            pts_world.push_back(calibPts[i]);
            pts_image.push_back(q);
        }
    }
    assert(pts_world.size() == pts_image.size());
    
}

void DisneyWorldBasketballCourt::backProjectpoints(const vpgl_perspective_camera<double> &camera, const vcl_vector<vgl_point_2d<double> > & positions,
                                                   vcl_vector<vgl_point_2d<double> > & positions_world, bool insideCourt, int width, int height)
{
    assert(positions.size() >= 4);
    
    // court area
    const int xmin = 70;
    const int ymin = 70;
    const int xmax = 70 + 94 * 12;
    const int ymax = 70 + 50 * 12;
   
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_src;
    vcl_vector<vgl_point_2d<double> > points_dst;
    vcl_vector<vgl_point_2d<double> > points_world_dump;
    
    this->projectCourtPoints(camera, width, height, points_world_dump, points_src, points_dst, 20);
    
    assert(points_src.size() >= 4);
    assert(points_src.size() == points_dst.size());
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_src.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_src[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_dst[i]));
    }
    
    vgl_h_matrix_2d<double> H(src, dst);
    
    vgl_h_matrix_2d<double> imageToTopview = H.get_inverse();
    vgl_transform_2d<double> topviewToWorld = this->imageToWorld();
    
    for (int i = 0; i<positions.size(); i++) {
        vgl_homg_point_2d<double> p(positions[i].x(), positions[i].y(), 1.0);
        vgl_homg_point_2d<double> projP = imageToTopview(p);
        double px = projP.x()/projP.w();
        double py = projP.y()/projP.w();
        if (insideCourt && (px <= xmin || px >= xmax || py <= ymin || py >= ymax)) {
            continue;
        }
        
        vgl_point_2d<double> pTopview(px, py);
        vgl_point_2d<double> pWorld = topviewToWorld(pTopview);
        
        positions_world.push_back(pWorld);
    }
}

bool DisneyWorldBasketballCourt::topviewToCameraHomography(const vpgl_perspective_camera<double> &camera, vgl_h_matrix_2d<double> & H, int width, int height)
{
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_world_dump;
    vcl_vector<vgl_point_2d<double> > points_topview_image;
    vcl_vector<vgl_point_2d<double> > points_camera_image;
   
    
    this->projectCourtPoints(camera, width, height, points_world_dump, points_topview_image, points_camera_image, 20);
    
    if (points_topview_image.size() < 4) {
        vcl_cerr<<"Error: number of correspondence less than 4."<<vcl_endl;
        return false;
    }
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_topview_image.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_topview_image[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_camera_image[i]));
    }
    
    H  = vgl_h_matrix_2d<double>(src, dst);
    
    return true;
}

bool DisneyWorldBasketballCourt::worldToCameraHomography(const vpgl_perspective_camera<double> &camera, vgl_h_matrix_2d<double> &H, int width, int height)
{
    //get H from court image to camera image
    vcl_vector<vgl_point_2d<double> > points_world;
    vcl_vector<vgl_point_2d<double> > points_topview_dump;
    vcl_vector<vgl_point_2d<double> > points_camera_image;
    
    
    this->projectCourtPoints(camera, width, height, points_world, points_topview_dump, points_camera_image, 20);
    
    if (points_world.size() < 4) {
        vcl_cerr<<"Error: number of correspondence less than 4."<<vcl_endl;
        return false;
    }
    
    vcl_vector<vgl_homg_point_2d<double> > src;
    vcl_vector<vgl_homg_point_2d<double> > dst;
    for (int i = 0; i<points_world.size(); i++) {
        src.push_back(vgl_homg_point_2d<double>(points_world[i]));
        dst.push_back(vgl_homg_point_2d<double>(points_camera_image[i]));
    }
    
    H  = vgl_h_matrix_2d<double>(src, dst);    
    return true;
}

void DisneyWorldBasketballCourt::alignmentQuality(const vil_image_view<vxl_byte> &topview, const vil_image_view<vxl_byte> &image,
                                                  const vcl_vector<vpgl_perspective_camera<double> > & cameras,
                                                  vcl_vector<double> & SSDs, vcl_vector<double> & SADs)
{
    assert(cameras.size() >= 1);
    assert(topview.nplanes() == 3);
    assert(image.nplanes() == 3);
    assert(SSDs.size() == 0);
    assert(SADs.size() == 0);
    
    int destWidth  = image.ni();
    int destHeight = image.nj();
    
    for (int i = 0; i<cameras.size(); i++) {
        vil_image_view<vxl_byte> warpedTopview;
        bool isWarpOk = this->projectTopviewImage(topview, cameras[i], destWidth, destHeight, warpedTopview, 20);
        if (!isWarpOk) {
            SSDs.push_back(INT_MAX);
            SADs.push_back(INT_MAX);
            continue;
        }
        vil_image_view<double> weightMap;
        this->getWeightImage(cameras[i], destWidth, destHeight, 4, 100, weightMap);
        
        double ssd = 0;
        double sad = 0;
        for (int y = 0; y<destHeight; y++) {
            for (int x = 0; x<destWidth; x++) {
                if (weightMap(x, y) != 0.0) {
                    double dif_r = abs(warpedTopview(x, y, 0) - image(x, y, 0));
                    double dif_g = abs(warpedTopview(x, y, 1) - image(x, y, 1));
                    double dif_b = abs(warpedTopview(x, y, 2) - image(x, y, 2));
                    sad += dif_r + dif_g + dif_b;
                    ssd += dif_r * dif_r + dif_g * dif_g + dif_b * dif_b;
                }
            }
        }
        SSDs.push_back(ssd);
        SADs.push_back(sad);
        
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
            
            VilPlus::vil_save(weightedWarpredTopview, "weighted_topview.jpg");
            VilPlus::vil_save(weightedImage, vcl_string("weighted_image_" + vul_string(i) + ".jpg").c_str());
        }
    }
}

void DisneyWorldBasketballCourt::alignmentQuality(const vil_image_view<vxl_byte> &topview, const vil_image_view<vxl_byte> &image,
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
    this->getWeightImage(camera, destWidth, destHeight, 4, 100, weightMap);
    
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
        
        VilPlus::vil_save(weightedWarpredTopview, "weighted_topview.jpg");
        VilPlus::vil_save(weightedImage, "weighted_image_.jpg");
    }

}

bool DisneyWorldBasketballCourt::isBlurImage(const vil_image_view<vxl_byte> & topview, const vil_image_view<vxl_byte> & image,
                                             const vcl_vector<vpgl_perspective_camera<double> > & camera)
{
    vcl_cout<<"DisneyWorldBasketballCourt::isBlurImage un-implemented.\n";
    
    return true;
}

void DisneyWorldBasketballCourt::commonCourtCorners(const vpgl_perspective_camera<double> &firstCamera, const vpgl_perspective_camera<double> & secondCamera,
                                                    int destWidht, int destHeight, bool insideImage,
                                                    vcl_vector<vgl_point_2d<double> > & firstCorners, vcl_vector<vgl_point_2d<double> > & secondCorners)
{
    assert(firstCorners.size() == 0);
    assert(secondCorners.size() == 0);
    
    vcl_vector<vgl_point_2d<double> > calibPts = DisneyWorldBasketballCourt::getCalibPoints();
    
    for (int i = 0; i<calibPts.size(); i++) {
        vgl_point_3d<double> p(calibPts[i].x(), calibPts[i].y(), 0.0);
        vgl_point_2d<double> p1 = (vgl_point_2d<double>)firstCamera.project(p);
        vgl_point_2d<double> p2 = (vgl_point_2d<double>)secondCamera.project(p);
        
        if (insideImage)
        {
            if(vgl_inside_image(p1, destWidht, destHeight) && vgl_inside_image(p2, destWidht, destHeight))
            {
                firstCorners.push_back(p1);
                secondCorners.push_back(p2);
            }
        }
        else
        {
            firstCorners.push_back(p1);
            secondCorners.push_back(p2);
        }
    }
    assert(firstCorners.size() == secondCorners.size());
}

bool DisneyWorldBasketballCourt::getCameraFromSequentialHomo(const vpgl_perspective_camera<double> & keyFrameCamera, const vgl_h_matrix_2d<double> & H,
                                                             vpgl_perspective_camera<double> & camera)
{
    int width  = keyFrameCamera.get_calibration().principal_point().x() * 2;
    int height = keyFrameCamera.get_calibration().principal_point().y() * 2;
    vcl_vector<vgl_point_2d<double> >  pts_world;
    vcl_vector<vgl_point_2d<double> >  pts_image;
    vcl_vector<vgl_point_2d<double> >  pts_dump;
    
    DisneyWorldBasketballCourt::projectCalibPoints(keyFrameCamera, width, height, pts_world, pts_dump, pts_image, 10);
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

bool DisneyWorldBasketballCourt::getCameraFromSequentialMatching(const vcl_vector<vgl_point_2d<double> > & keyframePts,
                                                                 const vcl_vector<vgl_point_2d<double> > & queryFramePts,
                                                                 const vpgl_perspective_camera<double> & keyFrameCamera,
                                                                 vpgl_perspective_camera<double> & camera)
{
    //vgl_h_matrix_2d(vcl_vector<vgl_homg_point_2d<T> > const& points1,
    //                vcl_vector<vgl_homg_point_2d<T> > const& points2);
    assert(keyframePts.size() == queryFramePts.size());
    assert(keyframePts.size() >= 4);
    
    vcl_vector<vgl_homg_point_2d<double> > points1;
    vcl_vector<vgl_homg_point_2d<double> > points2;
    for (int i = 0; i<keyframePts.size(); i++) {
        points1.push_back(vgl_homg_point_2d<double>(keyframePts[i]));
        points2.push_back(vgl_homg_point_2d<double>(queryFramePts[i]));
    }
    vgl_h_matrix_2d<double> H(points1, points2);
    
    return DisneyWorldBasketballCourt::getCameraFromSequentialHomo(keyFrameCamera, H, camera);
}

void  DisneyWorldBasketballCourt::getPointsDirectionOnEdges(vcl_vector<vgl_point_2d<double> > &pts, vcl_vector<vnl_vector_fixed<double, 4> > &p_to_q)
{
    {
        //horizontal far line
        for (int x = -47; x<= 47; x++) {
            int y = +25 * 12;
            pts.push_back(vgl_point_2d<double>(x * 12, y));
            p_to_q.push_back(vnl_vector_fixed<double, 4>(x, y, x + 1, y));  //point into court
            
            y = -25 * 12;
            pts.push_back(vgl_point_2d<double>(x * 12, y));
            p_to_q.push_back(vnl_vector_fixed<double, 4>(x, y, x + 1, y));
        }
        
        for (int y = -25; y<=25; y++) {
            int x = -47 * 12;
            //   pts.push_back(vgl_point_2d<double>(x, y * 12));
            //   p_to_q.push_back(vnl_vector_fixed<double, 4>(x, y, x-1, y));
            
            x = +47 * 12;
            pts.push_back(vgl_point_2d<double>(x, y * 12));
            p_to_q.push_back(vnl_vector_fixed<double, 4>(x, y, x, y + 1));
        }
    }
    
    //professional 3 point line
    for ( int dx = -1; dx <= +1; dx += 2 )
    {
        // professional 3 point line
        {
            double delta_theta = acos((double)(22 * 12)/ (double)(23 * 12 + 9));
            double from_theta = vnl_math::pi / 2 + delta_theta;
            double to_theta   = vnl_math::pi * 3 / 2 - delta_theta;
            double range = to_theta - from_theta;
            for (unsigned int i = 0; i<32; i++) {
                
                double startTheta = range / 32 * i + from_theta;
                double stopTheta  = range / 32 * ( i + 1 ) + from_theta;
                
                vgl_point_2d< double > p1 = vgl_point_2d< double >( ( vcl_cos( startTheta ) * (23 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (23 * 12 + 9));
                vgl_point_2d< double > p2 = vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (23 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (23 * 12 + 9) );
                vgl_point_2d< double > p_mid((p1.x() + p2.x())/2.0, (p1.y() + p2.y())/2.0);
                pts.push_back(p_mid);
                p_to_q.push_back(vnl_vector_fixed<double, 4>(p1.x(), p1.y(), p2.x(), p2.y()));
            }
        }
        
    }
    
    for (unsigned int i = 0; i<pts.size(); i++) {
        double px = (pts[i].x()/12.0 + 47.0) * 0.3048;
        double py = (pts[i].y()/12.0 + 25.0) * 0.3048;
        
        pts[i].set(px, py);
    }
    
    for (unsigned int i = 0; i<p_to_q.size(); i++) {
        p_to_q[i][0] = (p_to_q[i][0]/12.0 + 47.0) * 0.3048;
        p_to_q[i][1] = (p_to_q[i][1]/12.0 + 47.0) * 0.3048;
        p_to_q[i][2] = (p_to_q[i][2]/12.0 + 47.0) * 0.3048;
        p_to_q[i][3] = (p_to_q[i][3]/12.0 + 47.0) * 0.3048;
    }
    
    assert(pts.size() == p_to_q.size());
}


vgl_transform_2d< double > DisneyWorldBasketballCourt::imageToWorld()
{
    double m1[9] = {
        1, 0, -70,
        0, 1, -70,
        0, 0,  1};
    double m2[9] = {
        1, 0, 0,
        0, -1, 600,
        0, 0, 1};
    //inch to meter
    double m3[9] = {
        0.0254, 0, 0,
        0, 0.0254, 0,
        0, 0, 1
    };
    
    vgl_transform_2d< double > model = vgl_transform_2d<double>(vnl_matrix_fixed<double, 3, 3>(m3) * vnl_matrix_fixed<double, 3, 3>(m2) * vnl_matrix_fixed<double, 3, 3>(m1));
    return model;
}

vcl_vector<vgl_point_2d<double> > DisneyWorldBasketballCourt::getCourtPoints()
{
    vcl_vector< vgl_point_2d<double> > markings;
    
    //corners
    markings.push_back(vgl_point_2d<double>(0, 0));
    markings.push_back(vgl_point_2d<double>(94, 0));
    markings.push_back(vgl_point_2d<double>(94, 50));
    markings.push_back(vgl_point_2d<double>(0, 50));
    
    //left small poly
    markings.push_back(vgl_point_2d<double>(0, 19));
    markings.push_back(vgl_point_2d<double>(19, 19));
    markings.push_back(vgl_point_2d<double>(19, 31));
    markings.push_back(vgl_point_2d<double>(0, 31));
    
    //right small poly
    markings.push_back(vgl_point_2d<double>(75, 19));
    markings.push_back(vgl_point_2d<double>(94, 19));
    markings.push_back(vgl_point_2d<double>(94, 31));
    markings.push_back(vgl_point_2d<double>(75, 31));
    
    //point on the arc
    markings.push_back(vgl_point_2d<double>(25, 25));
    markings.push_back(vgl_point_2d<double>(69, 25));
    
    //center point on the boundary and two --|--
    markings.push_back(vgl_point_2d<double>(47, 50));
    markings.push_back(vgl_point_2d<double>(47, 0));
    markings.push_back(vgl_point_2d<double>(28, 50));
    markings.push_back(vgl_point_2d<double>(66, 50));
    
    //3 point line arc center (boy)
    markings.push_back(vgl_point_2d<double>(26, 25));
    markings.push_back(vgl_point_2d<double>(68, 25));
    
    //2 point line arc center (professional)
    markings.push_back(vgl_point_2d<double>(29, 25));
    markings.push_back(vgl_point_2d<double>(65, 25));
    
    //middle point
    markings.push_back(vgl_point_2d<double>(47, 25));
    
    //2 points (virtual)
    markings.push_back(vgl_point_2d<double>(28, 0));
    markings.push_back(vgl_point_2d<double>(66, 0));
    
    //4 points along short vertical bar
    markings.push_back(vgl_point_2d<double>(19, 0));
    markings.push_back(vgl_point_2d<double>(19, 50));
    markings.push_back(vgl_point_2d<double>(75, 0));
    markings.push_back(vgl_point_2d<double>(75, 50));
    
    
    
    //into meters
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> pt = markings[i];
        markings[i].set(pt.x() * 0.3048, pt.y() * 0.3048);
    }
    
    return markings;
}

vcl_vector<vgl_point_2d<double> > DisneyWorldBasketballCourt::getCalibPoints()
{
    // size 35
    vcl_vector< vgl_point_2d<double> > markings;
    
    //corners
    markings.push_back(vgl_point_2d<double>(0, 0));
    markings.push_back(vgl_point_2d<double>(94, 0));
    markings.push_back(vgl_point_2d<double>(94, 50));
    markings.push_back(vgl_point_2d<double>(0, 50));
    
    //left small poly
    markings.push_back(vgl_point_2d<double>(0, 19));
    markings.push_back(vgl_point_2d<double>(19, 19));
    markings.push_back(vgl_point_2d<double>(19, 31));
    markings.push_back(vgl_point_2d<double>(0, 31));
    
    //right small poly
    markings.push_back(vgl_point_2d<double>(75, 19));
    markings.push_back(vgl_point_2d<double>(94, 19));
    markings.push_back(vgl_point_2d<double>(94, 31));
    markings.push_back(vgl_point_2d<double>(75, 31));
    
    //point on the arc
    markings.push_back(vgl_point_2d<double>(25, 25));
    markings.push_back(vgl_point_2d<double>(69, 25));
    
    //center point on the boundary and two --|--
    markings.push_back(vgl_point_2d<double>(47, 50));
    markings.push_back(vgl_point_2d<double>(47, 0));
    markings.push_back(vgl_point_2d<double>(28, 50));
    markings.push_back(vgl_point_2d<double>(66, 50));
    
    //3 point line arc center (boy)
    markings.push_back(vgl_point_2d<double>(26, 25));
    markings.push_back(vgl_point_2d<double>(68, 25));
    
    //2 point line arc center (professional)
    markings.push_back(vgl_point_2d<double>(29, 25));
    markings.push_back(vgl_point_2d<double>(65, 25));
    
    //middle point
    markings.push_back(vgl_point_2d<double>(47, 25));
    
    //2 points (virtual)
    markings.push_back(vgl_point_2d<double>(28, 0));
    markings.push_back(vgl_point_2d<double>(66, 0));
    
    //4 points along short vertical bar
    markings.push_back(vgl_point_2d<double>(19, 0));
    markings.push_back(vgl_point_2d<double>(19, 50));
    markings.push_back(vgl_point_2d<double>(75, 0));
    markings.push_back(vgl_point_2d<double>(75, 50));
    
    // from professional 3 point line
    markings.push_back(vgl_point_2d<double>(14, 3));
    markings.push_back(vgl_point_2d<double>(94- 14, 3));
    
    // 4 points in middle circle
    markings.push_back(vgl_point_2d<double>(47, 31));
    markings.push_back(vgl_point_2d<double>(47, 19));
    markings.push_back(vgl_point_2d<double>(41, 25));
    markings.push_back(vgl_point_2d<double>(53, 25));
    
    //into meters
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> pt = markings[i];
        markings[i].set(pt.x() * 0.3048, pt.y() * 0.3048);
    }
    
    return markings;
}

vcl_vector<vgl_point_2d<double> > DisneyWorldBasketballCourt::getCourtKeyPoints()
{
    vcl_vector<vgl_point_2d<double> > courtPts;
    
    courtPts.resize(26);
    
    //out line
    courtPts[0].set(0, 0);
    courtPts[1].set(94, 0);
    courtPts[2].set(94, 50);
    courtPts[3].set(0, 50);
    
    //left small poly
    courtPts[4].set(0, 19);
    courtPts[5].set(19, 19);
    courtPts[6].set(19, 31);
    courtPts[7].set(0, 31);
    
    //right small poly
    courtPts[8].set(75, 19);
    courtPts[9].set(94, 19);
    courtPts[10].set(94, 31);
    courtPts[11].set(75, 31);
    
    //point on the arc
    courtPts[12].set(25, 25);
    courtPts[13].set(69, 25);
    
    //center point on the boundary and two --|--
    courtPts[14].set(47, 50);
    courtPts[15].set(47, 0);
    
    courtPts[16].set(28, 50);
    courtPts[17].set(66, 50);
    
    //professional 3 point line
    courtPts[18].set(14, 3);
    courtPts[19].set(94 - 14, 3);
    
    courtPts[20].set(11, 19);
    courtPts[21].set(94 - 11, 19);
    
    courtPts[22].set(0, 51/12.0);
    courtPts[23].set(94, 51/12.0);
    
    courtPts[24].set(0, 3);
    courtPts[25].set(94, 3);
    
    
    vcl_vector<vgl_point_2d<double> > line_intersection_pts;
    //@todo init vertical lines and horizontal lines
    line_intersection_pts.resize(10);
    
    // two virtual --|--
    line_intersection_pts[0].set(28, 0);
    line_intersection_pts[1].set(66, 0);
  
    
    // from professional 3 point line
    line_intersection_pts[2].set(14, 3);
    line_intersection_pts[3].set(94- 14, 3);
    
    
    // logo with division line
    line_intersection_pts[4].set(47, 35.6203);
    line_intersection_pts[5].set(47, 14.32);
    
    // two ||
    line_intersection_pts[6].set(33, 50);
    line_intersection_pts[7].set(94-33, 50);
    
    // two corners in logo
    line_intersection_pts[8].set(38.3, 50 - 27.2);
    line_intersection_pts[9].set(55.7, 50 - 27.2);
    
    
    
    // FEET to meter
    vcl_vector<vgl_point_2d<double> > pts;
    double scale = 0.3048;
    for (int i = 0; i<courtPts.size(); i++) {
        vgl_point_2d<double> pt = courtPts[i];
        courtPts[i].set(pt.x() * scale, pt.y() * scale);
        pts.push_back(courtPts[i]);
    }
    for (int i = 0; i<line_intersection_pts.size(); i++) {
        vgl_point_2d<double> pt = line_intersection_pts[i];
        line_intersection_pts[i].set(pt.x() * scale, pt.y() * scale);
        pts.push_back(line_intersection_pts[i]);
    }
    return pts;   
}

vcl_vector<vgl_point_2d<double> > DisneyWorldBasketballCourt::getPatchMatchingPoints()
{
    vcl_vector<vgl_point_2d<double> > courtPts;
    
    courtPts.resize(26);
    
    //out line
    courtPts[0].set(0, 0);
    courtPts[1].set(94, 0);
    courtPts[2].set(94, 50);
    courtPts[3].set(0, 50);
    
    //left small poly
    courtPts[4].set(0, 19);
    courtPts[5].set(19, 19);
    courtPts[6].set(19, 31);
    courtPts[7].set(0, 31);
    
    //right small poly
    courtPts[8].set(75, 19);
    courtPts[9].set(94, 19);
    courtPts[10].set(94, 31);
    courtPts[11].set(75, 31);
    
    //point on the arc
    courtPts[12].set(25, 25);
    courtPts[13].set(69, 25);
    
    //center point on the boundary and two --|--
    courtPts[14].set(47, 50);
    courtPts[15].set(47, 0);
    
    courtPts[16].set(28, 50);
    courtPts[17].set(66, 50);
    
    //professional 3 point line
    courtPts[18].set(14, 3);
    courtPts[19].set(94 - 14, 3);
    
    courtPts[20].set(11, 19);
    courtPts[21].set(94 - 11, 19);
    
    courtPts[22].set(0, 51/12.0);
    courtPts[23].set(94, 51/12.0);
    
    // two point in small square
    courtPts[24].set(7, 31);
    courtPts[25].set(87, 31);
    
    
    vcl_vector<vgl_point_2d<double> > line_intersection_pts;
    //@todo init vertical lines and horizontal lines
    line_intersection_pts.resize(10);
    
    // two virtual --|--
    line_intersection_pts[0].set(28, 0);
    line_intersection_pts[1].set(66, 0);
    
    
    // from professional 3 point line
    line_intersection_pts[2].set(14, 3);
    line_intersection_pts[3].set(94- 14, 3);
    
    // logo with division line
    line_intersection_pts[4].set(47, 35.6203);
    line_intersection_pts[5].set(47, 14.32);
    
    // two ||
    line_intersection_pts[6].set(33, 50);
    line_intersection_pts[7].set(94-33, 50);
    
    // two corners in logo
    line_intersection_pts[8].set(38.3, 50 - 27.2);
    line_intersection_pts[9].set(55.7, 50 - 27.2);
    
    
    // FEET to meter
    vcl_vector<vgl_point_2d<double> > pts;
    double scale = 0.3048;
    for (int i = 0; i<courtPts.size(); i++) {
        vgl_point_2d<double> pt = courtPts[i];
        courtPts[i].set(pt.x() * scale, pt.y() * scale);
        pts.push_back(courtPts[i]);
    }
    for (int i = 0; i<line_intersection_pts.size(); i++) {
        vgl_point_2d<double> pt = line_intersection_pts[i];
        line_intersection_pts[i].set(pt.x() * scale, pt.y() * scale);
        pts.push_back(line_intersection_pts[i]);
    }
    
    // point in the texture area
    {
        vcl_vector<vgl_point_2d<double> > topview_pts;
        topview_pts.push_back(vgl_point_2d<double>(444, 172)); // left D
        topview_pts.push_back(vgl_point_2d<double>(140, 56));  // left P
        topview_pts.push_back(vgl_point_2d<double>(509, 328)); // left corner
        topview_pts.push_back(vgl_point_2d<double>(876, 583)); // right d
        topview_pts.push_back(vgl_point_2d<double>(985, 60));  // right p
        topview_pts.push_back(vgl_point_2d<double>(760, 325)); // right corner
        topview_pts.push_back(vgl_point_2d<double>(240, 634));  // left 3 point line intersection
        topview_pts.push_back(vgl_point_2d<double>(1032, 634)); // right 3 point line intersection
        
        // topview image to world coordinate
        DisneyWorldBasketballCourt court;
        vgl_transform_2d< double > toWorld = court.imageToWorld();
        for (int i = 0; i<topview_pts.size(); i++) {
            vgl_point_2d<double> p = toWorld(topview_pts[i]);
            pts.push_back(p);
        }        
    }
    
    return pts;
    
}



vcl_vector< vgl_line_segment_2d< double > > DisneyWorldBasketballCourt::getLineSegments()
{
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // everything below is specified in inches, and the origin at the center of the court
    // afterwards I'll convert to the regular conventions (meters with origin in bottom left)
    
    // inner boundary:use short line instead of long lines
    //   markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( -47 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( +19 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +19 * 12, +25 * 12 ), vgl_point_2d< double >( 0 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, +25 * 12 ), vgl_point_2d< double >( -19 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -19 * 12, +25 * 12 ), vgl_point_2d< double >( 0 * 12, +25 * 12 ) ) );
    
    //   markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, -25 * 12 ), vgl_point_2d< double >( -47 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, -25 * 12 ), vgl_point_2d< double >( +19 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +19 * 12, -25 * 12 ), vgl_point_2d< double >( 0 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, -25 * 12 ), vgl_point_2d< double >( -19 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -19 * 12, -25 * 12 ), vgl_point_2d< double >( 0 * 12, -25 * 12 ) ) );
    
    
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( +47 * 12, -25 * 12 ) ) );
//    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, +25 * 12 ), vgl_point_2d< double >( -47 * 12, -25 * 12 ) ) );  //ambiguity with fake border
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, +25 * 12 ), vgl_point_2d< double >( -47 * 12, 25 * 12 - 5 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, -25 * 12 ), vgl_point_2d< double >( -47 * 12, -25 *12 + 5 * 12 ) ) );
        
    // division line
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 0, -25 * 12 ), vgl_point_2d< double >( 0, + 25 * 12 ) ) );
    
    // symmetric for left and right court field
    for ( int dx = -1; dx <= +1; dx += 2 )
    {
        // division line
        //    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( dx, -25 * 12 ), vgl_point_2d< double >( dx, + 25 * 12 ) ) );
        
        // bench out to court, 2 inch width
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 19 * 12 * dx, 25 * 12 ), vgl_point_2d< double >( 19 * 12 * dx, +28 * 12 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 19 * 12 * dx, 28 * 12 ), vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, +28 * 12 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, 28 * 12 ), vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, +25 * 12 ) ) );
        
        
        // key
        for ( int dy = -1; dy <= +1; dy +=2 )
        {
            // inside lengthwise
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12 * dx, ( +6 * 12 - 2 ) * dy ), vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, ( +6 * 12 - 2 ) * dy ) ) );
            
            
            // outside lengthwise
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12 * dx, +6 * 12 * dy), vgl_point_2d< double >( +40 * 12 * dx, +6 * 12 * dy ) ) );
            
            // in feet length bar
            //     markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 6 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 6 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 4 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 4 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 2 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 2 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 0 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 0 ) * dy ) ) );
            //     markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +39 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +36 * 12 * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +36 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +36 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +36 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx , +6 * 12 * dy ), vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( +28 * 12 * dx, +6 * 12 * dy ) ) );
        }
        
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, +6 * 12 - 2 ), vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, -6 * 12 + 2 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 28 * 12 * dx, +6 * 12 - 2 ), vgl_point_2d< double >( 28 * 12 * dx, -6 * 12 + 2 ) ) );
        
        
        // small arc
        for ( unsigned int i = 0; i < 32; ++i )
        {
            double startTheta = vnl_math::pi / 32 * i + vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 32 * ( i + 1 ) + vnl_math::pi / 2;
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( startTheta ) * 6 * 12 ),
                                                              vgl_point_2d< double >( ( vcl_cos( stopTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( stopTheta ) * 6 * 12 ) ) );
        }
        
        // dashed small arc
        for ( unsigned int i = 0; i < 30; ++i )
        {
            double startTheta = vnl_math::pi / 30 * i - vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 30 * ( i + 1 ) - vnl_math::pi / 2;
            
            if (i%4 == 0 || i%4 == 1) {
                markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( startTheta ) * 6 * 12 ),
                                                                  vgl_point_2d< double >( ( vcl_cos( stopTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( stopTheta ) * 6 * 12 ) ) );
            } 
        }
        
        
        // 3 point line
        for ( unsigned int i = 0; i < 64; ++i )
        {
            double startTheta = vnl_math::pi / 64 * i + vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 64 * ( i + 1 ) + vnl_math::pi / 2;
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (20 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (20 * 12 + 9) ),
                                                              vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (20 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (20 * 12 + 9) ) ) );
        }
        
        // female 3 point line
        for ( unsigned int i = 0; i < 64; ++i )
        {
            double startTheta = vnl_math::pi / 64 * i + vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 64 * ( i + 1 ) + vnl_math::pi / 2;
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (25 * 12 - 63) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (25 * 12 - 63) ),
                                                              vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (25 * 12 - 63) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (25 * 12 - 63) ) ) );
        }
        
        // professional 3 point line, very weak in gradient image
        if(1)
        {
            double delta_theta = acos((double)(22 * 12)/ (double)(23 * 12 + 9));
            double from_theta = vnl_math::pi / 2 + delta_theta;
            double to_theta   = vnl_math::pi * 3 / 2 - delta_theta;
            double range = to_theta - from_theta;
            for (unsigned int i = 0; i<64; i++) {
                
                double startTheta = range / 64 * i + from_theta;
                double stopTheta  = range / 64 * ( i + 1 ) + from_theta;
                
                
                markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (23 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (23 * 12 + 9) ),
                                                                   vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (23 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (23 * 12 + 9) ) ) );
            }
        }
        
        
    }
    
    // coord systems
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( markings[i].point1().x() / 12.0 + 47.0 ) * 0.3048;
        double y1 = ( markings[i].point1().y() / 12.0 + 25.0 ) * 0.3048;
        double x2 = ( markings[i].point2().x() / 12.0 + 47.0 ) * 0.3048;
        double y2 = ( markings[i].point2().y() / 12.0 + 25.0 ) * 0.3048;
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    
    return markings;
}

vcl_vector< vgl_line_segment_2d< double > > DisneyWorldBasketballCourt::getAllLineSegments()
{
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // everything below is specified in inches, and the origin at the center of the court
    // afterwards I'll convert to the regular conventions (meters with origin in bottom left)
    
    // inner boundary:use short line instead of long lines
    //   markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( -47 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( +19 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +19 * 12, +25 * 12 ), vgl_point_2d< double >( 0 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, +25 * 12 ), vgl_point_2d< double >( -19 * 12, +25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -19 * 12, +25 * 12 ), vgl_point_2d< double >( 0 * 12, +25 * 12 ) ) );
    
    //   markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, -25 * 12 ), vgl_point_2d< double >( -47 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, -25 * 12 ), vgl_point_2d< double >( +19 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +19 * 12, -25 * 12 ), vgl_point_2d< double >( 0 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, -25 * 12 ), vgl_point_2d< double >( -19 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -19 * 12, -25 * 12 ), vgl_point_2d< double >( 0 * 12, -25 * 12 ) ) );
    
    
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12, +25 * 12 ), vgl_point_2d< double >( +47 * 12, -25 * 12 ) ) );
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( -47 * 12, +25 * 12 ), vgl_point_2d< double >( -47 * 12, -25 * 12 ) ) );  
    
    // division line
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 0, -25 * 12 ), vgl_point_2d< double >( 0, + 25 * 12 ) ) );
    
    // symmetric for left and right court field
    for ( int dx = -1; dx <= +1; dx += 2 )
    {
        // division line
        //    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( dx, -25 * 12 ), vgl_point_2d< double >( dx, + 25 * 12 ) ) );
        
        // bench out to court, 2 inch width
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 19 * 12 * dx, 25 * 12 ), vgl_point_2d< double >( 19 * 12 * dx, +28 * 12 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 19 * 12 * dx, 28 * 12 ), vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, +28 * 12 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, 28 * 12 ), vgl_point_2d< double >( ( 19 * 12 + 1 ) * dx, +25 * 12 ) ) );
        
        
        // key
        for ( int dy = -1; dy <= +1; dy +=2 )
        {
            // inside lengthwise
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12 * dx, ( +6 * 12 - 2 ) * dy ), vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, ( +6 * 12 - 2 ) * dy ) ) );
            
            
            // outside lengthwise
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +47 * 12 * dx, +6 * 12 * dy), vgl_point_2d< double >( +40 * 12 * dx, +6 * 12 * dy ) ) );
            
            // in feet length bar
            //     markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 6 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 6 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 4 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 4 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 2 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 2 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +40 * 12 * dx, ( +6 * 12 + 0 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 0 ) * dy ) ) );
            //     markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +39 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( +39 * 12 * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +39 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +36 * 12 * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +36 * 12 * dx, +6 * 12 * dy ), vgl_point_2d< double >( +36 * 12 * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( +36 * 12 * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +36 * 12 - 2 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 2 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +33 * 12 - 4 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx , +6 * 12 * dy ), vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 4 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, ( +6 * 12 + 8 ) * dy ) ) );
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, ( +6 * 12 + 8 ) * dy ), vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, +6 * 12 * dy ) ) );
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( +30 * 12 - 6 ) * dx, +6 * 12 * dy ), vgl_point_2d< double >( +28 * 12 * dx, +6 * 12 * dy ) ) );
        }
        
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, +6 * 12 - 2 ), vgl_point_2d< double >( ( ( +47 - 18 ) * 12 - 10 ) * dx, -6 * 12 + 2 ) ) );
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 28 * 12 * dx, +6 * 12 - 2 ), vgl_point_2d< double >( 28 * 12 * dx, -6 * 12 + 2 ) ) );
        
        
        // small arc
        for ( unsigned int i = 0; i < 32; ++i )
        {
            double startTheta = vnl_math::pi / 32 * i + vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 32 * ( i + 1 ) + vnl_math::pi / 2;
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( startTheta ) * 6 * 12 ),
                                                              vgl_point_2d< double >( ( vcl_cos( stopTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( stopTheta ) * 6 * 12 ) ) );
        }
        
        // dashed small arc
        for ( unsigned int i = 0; i < 30; ++i )
        {
            double startTheta = vnl_math::pi / 30 * i - vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 30 * ( i + 1 ) - vnl_math::pi / 2;
            
            if (i%4 == 0 || i%4 == 1) {
                markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( startTheta ) * 6 * 12 ),
                                                                  vgl_point_2d< double >( ( vcl_cos( stopTheta ) * 6 * 12 + 28 * 12 ) * dx, vcl_sin( stopTheta ) * 6 * 12 ) ) );
            }
        }
        
        
        // 3 point line
        for ( unsigned int i = 0; i < 64; ++i )
        {
            double startTheta = vnl_math::pi / 64 * i + vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 64 * ( i + 1 ) + vnl_math::pi / 2;
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (20 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (20 * 12 + 9) ),
                                                              vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (20 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (20 * 12 + 9) ) ) );
        }
        
        // female 3 point line
        for ( unsigned int i = 0; i < 64; ++i )
        {
            double startTheta = vnl_math::pi / 64 * i + vnl_math::pi / 2;
            double stopTheta = vnl_math::pi / 64 * ( i + 1 ) + vnl_math::pi / 2;
            
            markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (25 * 12 - 63) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (25 * 12 - 63) ),
                                                              vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (25 * 12 - 63) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (25 * 12 - 63) ) ) );
        }
        
        // professional 3 point line, very weak in gradient image
        if(1)
        {
            double delta_theta = acos((double)(22 * 12)/ (double)(23 * 12 + 9));
            double from_theta = vnl_math::pi / 2 + delta_theta;
            double to_theta   = vnl_math::pi * 3 / 2 - delta_theta;
            double range = to_theta - from_theta;
            for (unsigned int i = 0; i<64; i++) {
                
                double startTheta = range / 64 * i + from_theta;
                double stopTheta  = range / 64 * ( i + 1 ) + from_theta;
                
                
                markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( ( vcl_cos( startTheta ) * (23 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( startTheta ) * (23 * 12 + 9) ),
                                                                  vgl_point_2d< double >( ( vcl_cos( stopTheta ) * (23 * 12 + 9) + 47 * 12 - 63 ) * dx, vcl_sin( stopTheta ) * (23 * 12 + 9) ) ) );
            }
        }
        
        
    }
    
    // coord systems
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( markings[i].point1().x() / 12.0 + 47.0 ) * 0.3048;
        double y1 = ( markings[i].point1().y() / 12.0 + 25.0 ) * 0.3048;
        double x2 = ( markings[i].point2().x() / 12.0 + 47.0 ) * 0.3048;
        double y2 = ( markings[i].point2().y() / 12.0 + 25.0 ) * 0.3048;
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    
    return markings;
}


vcl_vector< vgl_line_segment_2d< double > > DisneyWorldBasketballCourt::getLogoAreaLineSegments(int width)
{
    vcl_vector< vgl_line_segment_2d< double > > lines;
    
    // middle right "Walt Disney World" logo area
    // (51.5, 9.0)  (67.5  6.0) feet
    for (int y = 6.0 * 12; y <= 9.0 * 12; y += width) {
        vgl_point_2d<double> p1(51.5*12, y);
        vgl_point_2d<double> p2(67.5*12, y);
        
        lines.push_back(vgl_line_segment_2d<double>(p1, p2));
    }
    
    // left "Walt Disney World " logo area
    // (25.5 44.5) (42.0 40.5) feet
    for (int y = 40.5 * 12; y <= 44.5 * 12; y += width) {
        vgl_point_2d<double> p1(25.5*12, y);
        vgl_point_2d<double> p2(42.0*12, y);
        
        lines.push_back(vgl_line_segment_2d<double>(p1, p2));
    }
    
    // center logo, area too large
    if(0)
    {
        // (35.5 35.5) to (59, 14), may should refine with area
        for (int y = 14 * 12; y <= 35.5 * 12; y += width ) {
            vgl_point_2d<double> p1(35.5*12, y);
            vgl_point_2d<double> p2(59*12, y);
            
            lines.push_back(vgl_line_segment_2d<double>(p1, p2));
        }
    }
    
    //cross in center logo area
    if(0)
    {
        // 35.5 28.8; 58.6 22.0
        for (int y = 22.0 * 12; y <= 28.8 * 12; y += width) {
            vgl_point_2d<double> p1(35.5*12, y);
            vgl_point_2d<double> p2(58.6*12, y);
            
            lines.push_back(vgl_line_segment_2d<double>(p1, p2));
        }
        
        // 44.8 35.8; 48.5 13.6
        for (int y = 12.6 * 12; y <= 35.8 * 12; y += width) {
            vgl_point_2d<double> p1(44.8*12, y);
            vgl_point_2d<double> p2(48.5*12, y);
            
            lines.push_back(vgl_line_segment_2d<double>(p1, p2));
        }
        
    }
    
    
    // inch to meter
    for ( unsigned int i = 0; i < lines.size(); ++i )
    {
        double x1 = ( lines[i].point1().x() / 12.0 ) * 0.3048;
        double y1 = ( lines[i].point1().y() / 12.0 ) * 0.3048;
        double x2 = ( lines[i].point2().x() / 12.0 ) * 0.3048;
        double y2 = ( lines[i].point2().y() / 12.0 ) * 0.3048;
        
        lines[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    
    return lines;
}


vcl_vector< vgl_line_segment_2d< double > > DisneyWorldBasketballCourt::getDivisionLine()
{
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( 0, -25 * 12 ), vgl_point_2d< double >( 0, + 25 * 12 ) ) );
    
    // coord systems
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( markings[i].point1().x() / 12.0 + 47.0 ) * 0.3048;
        double y1 = ( markings[i].point1().y() / 12.0 + 25.0 ) * 0.3048;
        double x2 = ( markings[i].point2().x() / 12.0 + 47.0 ) * 0.3048;
        double y2 = ( markings[i].point2().y() / 12.0 + 25.0 ) * 0.3048;
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    
    return markings;
}

/*
  ******************************  WaltDisneyWorldTopviewCourt   ******************************************
 */

WaltDisneyWorldTopviewCourt::WaltDisneyWorldTopviewCourt( const vil_image_view<vxl_byte> &topviewImage)
{
    assert(topviewImage.nplanes() == 3);
    assert(topviewImage.ni() == 1268 && topviewImage.nj() == 740);
    
    topviewImage_.deep_copy(topviewImage);
    
    this->setCorrespondences();
}

WaltDisneyWorldTopviewCourt::~WaltDisneyWorldTopviewCourt()
{
    
}

void WaltDisneyWorldTopviewCourt::setCorrespondences()
{
    imgPts_.resize(25);
    wldPts_.resize(25);
    
    imgPts_[0] = vgl_point_2d<double>(71.542, 66.4883);  //Left top
    wldPts_[0] = vgl_point_2d<double>(0, 50.0);
    
    imgPts_[1] = vgl_point_2d<double>(73.1807,301.622);  //left rectangular
    wldPts_[1] = vgl_point_2d<double>(0, 31);
    
    imgPts_[2] = vgl_point_2d<double>(73.6074,442.765);  //left rectangular
    wldPts_[2] = vgl_point_2d<double>(0, 19);
    
    imgPts_[3] = vgl_point_2d<double>(298.551,299.958);  //left rectangular
    wldPts_[3] = vgl_point_2d<double>(19, 31);
    
    imgPts_[4] = vgl_point_2d<double>(298.775,441.221);  //left rectangular
    wldPts_[4] = vgl_point_2d<double>(19, 19);
    
    imgPts_[5] = vgl_point_2d<double>(1195.73,68.6699);  //right top
    wldPts_[5] = vgl_point_2d<double>(94, 50);
    
    imgPts_[6] = vgl_point_2d<double>(1195.55,299.608);  //right rectangular
    wldPts_[6] = vgl_point_2d<double>(94, 31);
    
    imgPts_[7] = vgl_point_2d<double>(1195.85,445.156);  //right rectangular
    wldPts_[7] = vgl_point_2d<double>(94, 19);
    
    imgPts_[8] = vgl_point_2d<double>(634.103,66.9316);  //middle division line
    wldPts_[8] = vgl_point_2d<double>(47, 50);
    
    imgPts_[9] = vgl_point_2d<double>(970.142,300.333);  //right rectangular
    wldPts_[9] = vgl_point_2d<double>(75, 31);
    
    imgPts_[10] = vgl_point_2d<double>(968.908,440.716);  //right rectangular
    wldPts_[10] = vgl_point_2d<double>(75, 19);
    
    imgPts_[11] = vgl_point_2d<double>(633.442,242.557);  //division line with logo top
    wldPts_[11] = vgl_point_2d<double>(47, 35.6203);
    
    imgPts_[12] = vgl_point_2d<double>(634.089,498.16);  //division line with logo down
    wldPts_[12] = vgl_point_2d<double>(47, 14.32);
    
    imgPts_[13] = vgl_point_2d<double>(370.456,369.043);  //3-point line with small circle (left)
    wldPts_[13] = vgl_point_2d<double>(25, 25);
    
    imgPts_[14] = vgl_point_2d<double>(898.659,372.35);  //3-point line with small circle (right)
    wldPts_[14] = vgl_point_2d<double>(69, 25);
    
    imgPts_[15] = vgl_point_2d<double>(72.1875,621.5);  //3-point line with left border line (down)
    wldPts_[15] = vgl_point_2d<double>(0, 4.25);
    
    imgPts_[16] = vgl_point_2d<double>(72.7354,119.914);  //3-point line with left border line (top)
    wldPts_[16] = vgl_point_2d<double>(0, 45.75);
    
    imgPts_[17] = vgl_point_2d<double>(1195.62,122.169);  //3-point line with right border line (top)
    wldPts_[17] = vgl_point_2d<double>(94, 45.75);
    
    imgPts_[18] = vgl_point_2d<double>(1194.55,619.776);  //3-point line with right border line (down)
    wldPts_[18] = vgl_point_2d<double>(94, 4.25);
    
    imgPts_[19] = vgl_point_2d<double>(155.487,441.492);  //left rectangular
    wldPts_[19] = vgl_point_2d<double>(7, 19);
    
    imgPts_[20] = vgl_point_2d<double>(157.225,300.298);  //left rectangular
    wldPts_[20] = vgl_point_2d<double>(7, 31);
    
    imgPts_[21] = vgl_point_2d<double>(1111.89,442.979);  //right rectangular
    wldPts_[21] = vgl_point_2d<double>(87, 19);
    
    imgPts_[22] = vgl_point_2d<double>(1110.46,297.888);  //right rectangular
    wldPts_[22] = vgl_point_2d<double>(87, 31);
    
    imgPts_[23] = vgl_point_2d<double>(516.854,374.143);  //S in center logo (left)
    wldPts_[23] = vgl_point_2d<double>(37.2378,24.6548);
    
    imgPts_[24] = vgl_point_2d<double>(754.022,349.582);  //S in center logo (right)
    wldPts_[24] = vgl_point_2d<double>(57.0019,26.7015);    
}

void WaltDisneyWorldTopviewCourt::courtImage(vil_image_view<vxl_byte> &image)
{
    vil_convert_planes_to_grey(topviewImage_, image);
}
void WaltDisneyWorldTopviewCourt::courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image)
{
    vil_convert_planes_to_grey(topviewImage_, image);
}
void WaltDisneyWorldTopviewCourt::courtImageWithLogo(vil_image_view<vxl_byte> &image, int logo_gray)
{
    vil_convert_planes_to_grey(topviewImage_, image);
}
void WaltDisneyWorldTopviewCourt::getWeightImageWithLogo(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt)
{
    vcl_cout<<"this function is not done!\n";
    
}
void WaltDisneyWorldTopviewCourt::getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt)
{
    vcl_cout<<"this function is not done!\n";
}
void WaltDisneyWorldTopviewCourt::overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> &image)
{
    vcl_cout<<"this function is not done!\n";
}
vgl_transform_2d< double > WaltDisneyWorldTopviewCourt::imageToWorld()
{
    double m1[9] = {
        1, 0, -70,
        0, 1, -70,
        0, 0,  1};
    double m2[9] = {
        1, 0, 0,
        0, -1, 600,
        0, 0, 1};
    //inch to meter
    double m3[9] = {
        0.0254, 0, 0,
        0, 0.0254, 0,
        0, 0, 1
    };
    
    vgl_transform_2d< double > model = vgl_transform_2d<double>(vnl_matrix_fixed<double, 3, 3>(m3) * vnl_matrix_fixed<double, 3, 3>(m2) * vnl_matrix_fixed<double, 3, 3>(m1));
    return model;
}

vcl_string WaltDisneyWorldTopviewCourt::name()
{
    return vcl_string("WaltDisneyWorldTopviewCourt");
}

void WaltDisneyWorldTopviewCourt::projectPoints(const vgl_h_matrix_2d<double> &H, int width, int height, int patchWidth, vcl_vector<vgl_point_2d<double> > & outImagePoints,
                                                            vcl_vector<vgl_point_2d<double> > & outWorldPts)
{
    int half_w = patchWidth/2;
    for (int i = 0; i<imgPts_.size(); i++) {
        vgl_homg_point_2d<double> p(imgPts_[i].x(), imgPts_[i].y(), 1.0);
        vgl_point_2d<double> q = (vgl_point_2d<double>)H(p);
        
        if (vgl_inside_rect(q, half_w + 1, half_w + 1, width - half_w, height - half_w)) {
            outImagePoints.push_back(q);
            outWorldPts.push_back(wldPts_[i]);
        }
    }
    assert(outImagePoints.size() == outWorldPts.size());
}





















