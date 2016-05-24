//
//  SoccerVerificationModel.cpp
//  OnlineStereo
//
//  Created by jimmy on 8/22/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "SoccerVerificationModel.h"
#include "vpgl_plus.h"
#include "vnl_plus.h"
#include "vgl_plus.h"
#include <vgl/vgl_distance.h>
#include "vil_plus.h"

SoccerVerificationModel::SoccerVerificationModel(double w, double h)
{
    field_width_ = w;
    field_heigh_ = h;
}

SoccerVerificationModel::~SoccerVerificationModel()
{
    
}


void SoccerVerificationModel::calculate_verification_segment(const vpgl_perspective_camera<double> & camera,
                                                             vcl_vector<EdgeLandmark> & landmarks,
                                                             double segment_image_length)
{
    
    vcl_vector<vgl_line_segment_2d<double> > longSegments = SoccerVerificationModel::getLineAndCircleSegment(field_width_, field_heigh_);
    
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    for (int i = 0; i<longSegments.size(); i++) {
        vgl_line_segment_2d<double> sampledSegment;  // sampled line segment must be inside image
        bool isSampled = VpglPlus::sampleLineSegment(camera, w, h, longSegments[i], 0.5, sampledSegment);
        if (!isSampled) {
            continue;
        }
        vgl_point_3d<double> p1 = vgl_point_3d<double>(sampledSegment.point1().x(), sampledSegment.point1().y(), 0.0);
        vgl_point_3d<double> p2 = vgl_point_3d<double>(sampledSegment.point2().x(), sampledSegment.point2().y(), 0.0);
        vgl_point_2d<double> q1 = camera.project(p1);
        vgl_point_2d<double> q2 = camera.project(p2);
        
        // one of the end inside image view
        if (VglPlus::vgl_inside_image(q1, w, h) || VglPlus::vgl_inside_image(q2, w, h)) {
            double distance = vgl_distance(q1, q2);
            int num = distance/segment_image_length;  // the number is decided by its length in image space
            // @todo num may be zero
            for (int j = 0; j<num; j++) {
                double t1 = 1.0 * j / num;
                double t2 = 1.0 * (j+1)/num;
                vgl_point_2d<double> p3 = sampledSegment.point_t(t1);
                vgl_point_2d<double> p4 = sampledSegment.point_t(t2);
                vgl_point_2d<double> q3 = camera.project(vgl_point_3d<double>(p3.x(), p3.y(), 0.0));
                vgl_point_2d<double> q4 = camera.project(vgl_point_3d<double>(p4.x(), p4.y(), 0.0));
                vgl_point_2d<double> q5 = centre(q3, q4);
                if (VglPlus::vgl_inside_image(q5, w, h)) {
                    EdgeLandmark elm;
                    elm.wld_seg_ = vgl_line_segment_2d<double>(p3, p4);
                    elm.img_seg_ = vgl_line_segment_2d<double>(q3, q4);
                    landmarks.push_back(elm);
                }
            }
        }
    }
   // printf("find %lu edge landmarks.\n", landmarks.size());
    return;
}

vcl_vector<double> SoccerVerificationModel::onLineRatio(const vil_image_view<vxl_byte> &image, const vcl_vector<EdgeLandmark> & landmarks,
                                                        double minMagnitude, double edge_width)
{
    vcl_vector<double> ratios;
    
    vil_image_view<double> magnitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, magnitude, grad_i, grad_j);
    for (int i = 0; i<landmarks.size(); i++) {
        double ratio = SoccerVerificationModel::onEdgePixelRatio(magnitude, grad_i, grad_j, landmarks[i].img_seg_, minMagnitude, edge_width);
        ratios.push_back(ratio);
    }
    return ratios;
}

vcl_vector<double> SoccerVerificationModel::onlineRatioScan(const vil_image_view<vxl_byte> &image, const vcl_vector<EdgeLandmark> & landmarks,
                                                            double minMagnitude, double edge_width)
{
    vcl_vector<double> ratios;
    
    vil_image_view<double> magnitude;
    vil_image_view<double> grad_i;
    vil_image_view<double> grad_j;
    VilPlus::vil_gradient(image, magnitude, grad_i, grad_j);
    
    for (int i = 0; i<landmarks.size(); i++) {
        double ratio = SoccerVerificationModel::onEdgePixelRatioScan(magnitude, grad_i, grad_j, landmarks[i].img_seg_, minMagnitude, edge_width);
        ratios.push_back(ratio);
    }
    return ratios;
}


vcl_vector<vgl_line_segment_2d<double> > SoccerVerificationModel::getLineAndCircleSegment(double width, double height)
{
    vcl_vector<vgl_line_segment_2d<double> > segments;
    vcl_vector<vgl_line_segment_2d<double> > linesegments   = SoccerVerificationModel::getWorldLineSegment(width, height);
    vcl_vector<vgl_line_segment_2d<double> > circlesegments = SoccerVerificationModel::getCircleLineSegment(width, height);
    segments.insert(segments.end(), linesegments.begin(), linesegments.end());
    segments.insert(segments.end(), circlesegments.begin(), circlesegments.end());
    return segments;
}

vcl_vector<vgl_line_segment_2d<double> > SoccerVerificationModel::getWorldLineSegment(double width, double height)
{
    
    vcl_vector< vgl_line_segment_2d< double > > markings(18);
    
    // one center line
    markings[0] = vgl_line_segment_2d<double>(vgl_point_2d< double >(0, -height/2), vgl_point_2d< double >(0, height/2));
    
    // 18 yard line
    markings[1] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) );
    markings[2] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) );
    
    // 18 yard line short, horizontal, near end
    markings[3] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), -22 ) );
    markings[4] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), -22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), -22 ) );
    
    // 6 yard
    markings[5] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * (-1), -10 ), vgl_point_2d< double >( (width/2 - 6) * (-1), +10 ) );
    markings[6] = vgl_line_segment_2d< double >( vgl_point_2d< double >( (width/2 - 6) * (+1), -10 ), vgl_point_2d< double >( (width/2 - 6) * (+1), +10 ) );
    
    // 18 yard line short, horizontal, far end
    markings[7] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (-1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (-1), +22 ) );
    markings[8] = vgl_line_segment_2d< double >( vgl_point_2d< double >( width/2 * (+1), +22 ), vgl_point_2d< double >( (width/2 - 18) * (+1), +22 ) );
    
    // part of goal line (left)
    markings[9]  = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-width/2, -12));
    markings[10] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, 12), vgl_point_2d< double >(-width/2, height/2));
    
    // part of goal line (right)
    // a bug
    markings[11] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, -height/2), vgl_point_2d< double >(width/2, -12));
    markings[12] = vgl_line_segment_2d<double>(vgl_point_2d< double >(width/2, 12), vgl_point_2d< double >(width/2, height/2));
    
    // part of far touch line
    markings[13] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-20, height/2), vgl_point_2d< double >(20, height/2));
    
    // part of near touch line
    markings[14] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, -height/2), vgl_point_2d< double >(-20, -height/2));
    markings[15] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20, -height/2), vgl_point_2d< double >(width/2, -height/2));
    
    // part of far touch line
    markings[16] = vgl_line_segment_2d<double>(vgl_point_2d< double >(-width/2, height/2), vgl_point_2d< double >(-20, height/2));
    markings[17] = vgl_line_segment_2d<double>(vgl_point_2d< double >(20,  height/2), vgl_point_2d< double >(width/2, height/2));
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 );
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    return markings;
}

vcl_vector<vgl_line_segment_2d<double> > SoccerVerificationModel::getCircleLineSegment(double width, double height)
{
    vcl_vector< vgl_line_segment_2d< double > > markings;
    
    // center circle
    for ( unsigned int i = 0; i < 16; ++i )
    {
        double range = 2.0 * vnl_math::pi;
        double startTheta = range / 16 * i;
        double stopTheta  = range / 16 * ( i + 1 );
        
        vgl_point_2d<double> p1((10 * cos(startTheta)), 10 * sin(startTheta));
        vgl_point_2d<double> p2((10 * cos(stopTheta)) , 10 * sin(stopTheta));
        markings.push_back( vgl_line_segment_2d< double >( p1, p2));
    }
    
    // left and right penalty circle
    // arc near penelty area
    for (int dx = -1; dx <= 1; dx += 2) {
        for ( unsigned int i = 0; i < 3; ++i )
        {
            double alpha = atan2(-8.0, 6.0);
            double range = 2.0 * fabs(alpha);
            double startTheta = range / 3 * i + alpha;
            double stopTheta  = range / 3 * ( i + 1 ) + alpha;
            
            vgl_point_2d<double> p1((width/2 - 12 - 10 * cos(startTheta)) * dx, 0 + 10 * sin(startTheta));
            vgl_point_2d<double> p2((width/2 - 12 - 10 * cos(stopTheta)) * dx, 0 + 10 * sin(stopTheta));
            
            markings.push_back( vgl_line_segment_2d< double >( p1, p2));
        }
    }
    
    // move original to left bottom, yard to meter
    for ( unsigned int i = 0; i < markings.size(); ++i )
    {
        double x1 = ( (markings[i].point1().x() + width/2) * 0.9144 );
        double y1 = ( (markings[i].point1().y() + height/2) * 0.9144 );
        double x2 = ( (markings[i].point2().x() + width/2) * 0.9144 );
        double y2 = ( (markings[i].point2().y() + height/2) * 0.9144 );
        
        markings[i] = vgl_line_segment_2d< double >( vgl_point_2d< double >( x1, y1 ), vgl_point_2d< double >( x2, y2 ) );
    }
    return markings;
}

double SoccerVerificationModel::onEdgePixelRatio(const vil_image_view<double> & magnitude,
                                         const vil_image_view<double> & grad_i,
                                         const vil_image_view<double> & grad_j,
                                         const vgl_line_segment_2d<double> & lineSeg,
                                         const double min_magnitude,
                                         const int edge_neighbor_size)
{
    assert(magnitude.nplanes() == 1);
    assert(grad_i.ni() == magnitude.ni());
    assert(grad_i.nj() == magnitude.nj());
    assert(grad_j.ni() == magnitude.ni());
    assert(grad_j.nj() == magnitude.nj());
    
    int sz = edge_neighbor_size;
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    
    vgl_point_2d<double> p1 = lineSeg.point1();
    vgl_point_2d<double> p2 = lineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linePts;
    VilPlus::draw_line(p1, p2, linePts, w, h);
    if (linePts.size() < 10) {
        return 0.0;
    }
    
    int cannyDir = SoccerVerificationModel::cannyDirection(p2.x() - p1.x(), p2.y() - p1.y());
    switch (cannyDir) {
        case 0:
            cannyDir = 2;
            break;
        case 1:
            cannyDir = 3;
            break;
        case 2:
            cannyDir = 0;
            break;
        case 3:
            cannyDir = 1;
            break;
        default:
            break;
    }
    
    int num = 0;
    for (int i = 0; i<linePts.size(); i++) {
        int startX = linePts[i].x();
        int startY = linePts[i].y();
        bool isFound = false;
        
        for (int j = -sz/2; j <= sz/2; j++) {
            for (int k = -sz/2; k <= sz/2; k++) {
                int xx = startX + k;
                int yy = startY + j;
                if (xx >= 0 && xx<w &&
                    yy >= 0 && yy<h) {
                    if (magnitude(xx, yy) > min_magnitude &&
                        cannyDir == SoccerVerificationModel::cannyDirection(grad_i(xx, yy), grad_j(xx, yy)))
                    {
                        isFound = true;
                        break;
                    }
                }
            }
            if (isFound) {
                break;
            }
        }
        if (isFound) {
            num++;
        }
    }
    
    double ratio = 1.0 * num /linePts.size();
    return ratio;
}

double SoccerVerificationModel::onEdgePixelRatioScan(const vil_image_view<double> & magnitude,
                                                     const vil_image_view<double> & grad_i,
                                                     const vil_image_view<double> & grad_j,
                                                     const vgl_line_segment_2d<double> & lineSeg,
                                                     const double min_magnitude,
                                                     const int edge_neighbor_size)
{
    assert(magnitude.nplanes() == 1);
    assert(grad_i.ni() == magnitude.ni());
    assert(grad_i.nj() == magnitude.nj());
    assert(grad_j.ni() == magnitude.ni());
    assert(grad_j.nj() == magnitude.nj());
    
    int sz = edge_neighbor_size;
    const int w = magnitude.ni();
    const int h = magnitude.nj();
    
    vgl_point_2d<double> p1 = lineSeg.point1();
    vgl_point_2d<double> p2 = lineSeg.point2();
    vcl_vector<vgl_point_2d<double> > linePts;
    VilPlus::draw_line(p1, p2, linePts, w, h);
    if (linePts.size() < 10) {
        return 0.0;
    }
    
    //   3 2 1
    //   4 * 0
    //   5 6 7
    int dx = 1;
    int dy = 0;
    int octive_dir = VglPlus::octaveDirection(p2.x() - p1.x(), p2.y() - p1.y());
    if (octive_dir == 0 || octive_dir == 4) {
        dx = 0;
        dy = 1;
    }
    else if(octive_dir == 7 || octive_dir == 3)
    {
        dx = 1;
        dy = -1;
    }
    else if(octive_dir == 6 || octive_dir == 2)
    {
        dx = 1;
        dy = 0;
    }
    else if(octive_dir == 5 || octive_dir == 1)
    {
        dx = 1;
        dy = 1;
    }
    else
    {
        assert(0);
    }
    
    vgl_vector_2d<double> line_dir = lineSeg.direction();
    
    const double angle_threshold = acos(45.0/180.0*vnl_math::pi);
    int num = 0;
    for (int i = 0; i<linePts.size(); i += 1) {
        int startX = linePts[i].x();
        int startY = linePts[i].y();
        bool isFound = false;
        
        // forward
        for (int j = 0; j <= sz/2; j++) {
            int xx = startX + dx;
            int yy = startY + dy;
            if (xx >= 0 && xx<w &&
                yy >= 0 && yy<h &&
                magnitude(xx, yy) > min_magnitude)
            {
                vgl_vector_2d<double> mag_dir(grad_i(xx, yy), grad_j(xx, yy));
                double cos_val = fabs(cos_angle(line_dir, mag_dir));
                if (cos_val > angle_threshold) {
                    isFound = true;
                    break;
                }
            }
        }
        
        // backward
        if (!isFound) {
            for (int j = 0; j <= sz/2; j++) {
                int xx = startX - dx;
                int yy = startY - dy;
                if (xx >= 0 && xx<w &&
                    yy >= 0 && yy<h &&
                    magnitude(xx, yy) > min_magnitude)
                {
                    vgl_vector_2d<double> mag_dir(grad_i(xx, yy), grad_j(xx, yy));
                    double cos_val = fabs(cos_angle(line_dir, mag_dir));
                    if (cos_val > angle_threshold) {
                        isFound = true;
                        break;
                    }
                }
            }
        }
        if (isFound) {
            num++;
        }
    }
    
    double ratio = 1.0 * num /linePts.size();
    return ratio;
}

int SoccerVerificationModel::cannyDirection(double dx, double dy)
{
    int dir = 0;
    if(VnlPlus::isEqualZero(dx))
    {
        if(VnlPlus::isEqualZero(dy))
        {
            dir = 0;
        }
        else
        {
            dir = 2;
        }
    }
    else
    {
        // 0.4142 --> 22.5^o, 2.4142--> 657.5^o
        double tanV = dy/dx;
        if(tanV > -0.4142f && tanV <= 0.4142f)
        {
            dir = 0;
        }
        else if(tanV > 0.4142f && tanV < 2.4142f)
        {
            dir = 1;
        }
        else if(abs(tanV) >= 2.4142f)
        {
            dir = 2;
        }
        else if(tanV > -2.4142f && tanV <= -0.4142f)
        {
            dir = 3;
        }
    }
    return dir;
}


