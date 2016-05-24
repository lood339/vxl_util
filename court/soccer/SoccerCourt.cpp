//
//  SoccerCourt.cpp
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "SoccerCourt.h"
#include <vgl/vgl_line_2d.h>
#include <vnl/vnl_math.h>
#include <vicl/vicl_line_segment.h>
#include <vicl/vicl_colours.h>
#include <vil/algo/vil_gauss_filter.h>
#include "vxl_plus.h"
#include <vgl/vgl_distance.h>

/*******************  SoccerCourt   ******************/
SoccerCourt::SoccerCourt()
{
    
}
SoccerCourt::~SoccerCourt()
{
    
}

vcl_vector< vgl_line_segment_2d< double > > SoccerCourt::getAllLineSegments(const double width, const double height)
{
    assert(width >= 100 && width <= 130);
    assert(height >= 50 && height <= 100);
    
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // origin point in court center
    // two touch lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(0, -height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(width/2, -height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, height/2), vgl_point_2d< double >(0, height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, height/2), vgl_point_2d< double >(width/2, height/2)));
    
    // two goal lines
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-width/2, height/2)));
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, height/2)));
    
    // one center line
    markings.push_back(vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(0, height/2)));
    
    // symmetric for left and right court field
    for ( int dx = -1; dx <= +1; dx += 2 )
    {
        // penelty area (large)
        markings.push_back( vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * dx, -22 ), vgl_point_2d< double >( (width/2 - 18) * dx, +22 ) ) );
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
    
    //center point
    {
        double delta = 0.5;
        vgl_point_2d<double> p(0, 0);
        vgl_point_2d<double> q1(p.x() - delta, p.y());
        vgl_point_2d<double> q2(p.x() + delta, p.y());
        vgl_point_2d<double> q3(p.x(), p.y() - delta);
        vgl_point_2d<double> q4(p.x(), p.y() + delta);
        
        markings.push_back( vgl_line_segment_2d< double >(q1, q2));
        markings.push_back( vgl_line_segment_2d< double >(q3, q4));
    }

    
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

vgl_point_2d<double> SoccerCourt::World2Image(const vgl_point_2d<double> &p)
{
    // meter to inch, every 3 inch as pixel
    double x = p.x() * 39.37 /3;
    double y = p.y() * 39.37 /3;
    
    // flip y
    y = 888 - y;
    
    x += 70;
    y += 70;
    
    return vgl_point_2d<double>(x, y);    
}

vcl_vector<vgl_point_2d<double> > SoccerCourt::getCalibratePoints(const double width, const double height)
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
    
    // yard to meter
    for ( unsigned int i = 0; i < pts.size(); ++i )
    {
        pts[i].set(pts[i].x() * 0.9144, pts[i].y() * 0.9144);
    }

    return pts;
}

vcl_vector< vgl_point_3d<double> > SoccerCourt::getCalibrationPoints(double segmentLength, const double width, const double height)
{
    vcl_vector< vgl_point_3d<double> > points;    
    vcl_vector<vgl_point_2d<double> > pts2d = SoccerCourt::getCalibratePoints(width, height);
    // 2D -- > 3D
    for (int i = 0; i<pts2d.size(); i++) {
        points.push_back(vgl_point_3d<double>(pts2d[i].x(), pts2d[i].y(), 0.0));
    }
    
    vcl_vector< vgl_line_segment_2d< double > > seg2d = SoccerCourt::getAllLineSegments(width, height);
    // sample in line segment
    for (int i = 0; i<seg2d.size(); i++) {
        vgl_line_segment_2d<double> seg = seg2d[i];
        double distance = vgl_distance(seg.point1(), seg.point2());
        assert(distance != 0.0);
        double dt = segmentLength/distance;
        // segment short than the segmentLength
        if (dt > segmentLength) {
            points.push_back(vgl_point_3d<double>(seg.point1().x(), seg.point1().y(), 0.0));
        }
        else
        {
            for (double t = dt; t < 1.0; t += dt) {
                vgl_point_2d<double> p = seg.point_t(t);
                points.push_back(vgl_point_3d<double>(p.x(), p.y(), 0.0));
            }
        }
        
    }
    
    return points;
}

/******************************         BarcelonaCourt         ***************************/

BarcelonaCourt::BarcelonaCourt()
{
    
}

BarcelonaCourt::~BarcelonaCourt()
{
    
}

void BarcelonaCourt::courtImage(vil_image_view<vxl_byte> &image)
{
    this->courtImage(3, 200, 128, image);
}

void BarcelonaCourt::courtImage(int lineWidth, int line_gray, int ground_gray, vil_image_view<vxl_byte> &image)
{
    int w = 115 * 36 / 3 + 70 * 2;
    int h = 74  * 36 / 3 + 70 * 2;
    
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(115, 74);
    
    image = vil_image_view<vxl_byte>(w, h, 1);
    image.fill(ground_gray);
    
    //meter change to inch (pixel)  / 0.3048
    vcl_vector< vxl_byte > white;
    white.push_back(line_gray);
    const double meter_to_3inch = 0.3048 * 3.0;
    for (int i = 0; i<markings.size(); i++) {
        vgl_point_2d<double> p1 = markings[i].point1();
        vgl_point_2d<double> p2 = markings[i].point2();
        
        p1.set(p1.x()/meter_to_3inch * 12, p1.y()/meter_to_3inch * 12);
        p2.set(p2.x()/meter_to_3inch * 12, p2.y()/meter_to_3inch * 12);
        
        //flip y
        p1.set(p1.x(), 888 - p1.y());
        p2.set(p2.x(), 888 - p2.y());
        
        //add 70 + 70
        p1.set(p1.x() + 70, p1.y() + 70);
        p2.set(p2.x() + 70, p2.y() + 70);
        
        //draw to the image
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), white, lineWidth);
    }
}

void BarcelonaCourt::getWeightImage(const vpgl_perspective_camera<double> & camera, int width, int height, int lineWidth, double gauss_sigma, vil_image_view<double> &wt)
{
    vil_image_view<vxl_byte> wt_black_white = vil_image_view<vxl_byte>(width, height, 1);
    wt_black_white.fill(0);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(115, 74);
    
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

vgl_transform_2d< double > BarcelonaCourt::imageToWorld()
{
    double m1[9] = {
        1, 0, -70,
        0, 1, -70,
        0, 0,  1};
    double m2[9] = {
        1, 0, 0,
        0, -1, 888,
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



void BarcelonaCourt::overlayLines(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image)
{
    assert(image.nplanes() == 3);
    
    vcl_vector< vgl_line_segment_2d< double > > markings = SoccerCourt::getAllLineSegments(115, 74);
    
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

void BarcelonaCourt::overlayPoints(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image)
{
    assert(image.nplanes() == 3);
    
    vcl_vector<vgl_point_2d<double> > points = SoccerCourt::getCalibratePoints();
    
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
        }
    }
}


