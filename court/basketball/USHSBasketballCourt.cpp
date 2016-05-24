//
//  USHSBasketballCourt.cpp
//  OnlineStereo
//
//  Created by jimmy on 5/2/16.
//  Copyright (c) 2016 Nowhere Planet. All rights reserved.
//

#include "USHSBasketballCourt.h"
#include <vicl/vicl_line_segment.h>
#include "vil_plus.h"
#include "vgl_plus.h"
#include "vpgl_plus.h"

vector< vgl_line_segment_2d< double > >
USBasketballCourt::getAllLineSegments(const double width, const double height)
{
    
    assert(width > 10 && height > 10);
    vector< vgl_line_segment_2d< double > > markings;
    vector<vgl_point_2d<double> > line_end_pts = USBasketballCourt::markingEndPoints(width, height);
    vector<std::pair<int, int> > end_pts_pairs = USBasketballCourt::pointPairsToLineSegments();
    for (int i = 0; i<end_pts_pairs.size(); i++) {
        int s = end_pts_pairs[i].first;
        int e = end_pts_pairs[i].second;
        markings.push_back(vgl_line_segment_2d<double>(line_end_pts[s], line_end_pts[e]));
    }
    
    
    // center circle
    const double foot_to_meter = 0.3048;
    
    {
        const double radius = 6.0 * foot_to_meter;
        vgl_point_2d<double> cp(width/2.0*foot_to_meter, height/2.0*foot_to_meter);
        for ( int i = 0; i < 64; ++i )
        {
            double startTheta = 2 * vnl_math::pi / 64 * i;
            double stopTheta  = 2 * vnl_math::pi / 64 * ( i + 1 );
            vgl_point_2d<double> p1(cos(startTheta) * radius, sin(startTheta) * radius);
            vgl_point_2d<double> p2(cos(stopTheta) * radius, sin(stopTheta)   * radius);
            
            p1 = vgl_point_2d<double>(p1.x() + cp.x(), p1.y() + cp.y());
            p2 = vgl_point_2d<double>(p2.x() + cp.x(), p2.y() + cp.y());
            
            markings.push_back( vgl_line_segment_2d< double >(p1, p2));
        }
    }
    
    // left penalty circle
    {
        const double radius = 6.0 * foot_to_meter;
        vgl_point_2d<double> cp(19.0*foot_to_meter, ((height-12.0)/2.0+6.0)*foot_to_meter);
        for ( int i = -16; i < 16; ++i )
        {
            double startTheta = 2 * vnl_math::pi / 64 * i;
            double stopTheta  = 2 * vnl_math::pi / 64 * ( i + 1 );
            vgl_point_2d<double> p1(cos(startTheta) * radius, sin(startTheta) * radius);
            vgl_point_2d<double> p2(cos(stopTheta) * radius, sin(stopTheta)   * radius);
            
            p1 = vgl_point_2d<double>(p1.x() + cp.x(), p1.y() + cp.y());
            p2 = vgl_point_2d<double>(p2.x() + cp.x(), p2.y() + cp.y());
            
            markings.push_back( vgl_line_segment_2d< double >(p1, p2));
        }
    }
    
    // right penalty circle
    {
        const double radius = 6.0 * foot_to_meter;
        vgl_point_2d<double> cp((width - 19.0)*foot_to_meter, ((height-12.0)/2.0+6.0)*foot_to_meter);
        for ( int i = 16; i < 48; ++i )
        {
            double startTheta = 2 * vnl_math::pi / 64 * i;
            double stopTheta  = 2 * vnl_math::pi / 64 * ( i + 1 );
            vgl_point_2d<double> p1(cos(startTheta) * radius, sin(startTheta) * radius);
            vgl_point_2d<double> p2(cos(stopTheta) * radius, sin(stopTheta)   * radius);
            
            p1 = vgl_point_2d<double>(p1.x() + cp.x(), p1.y() + cp.y());
            p2 = vgl_point_2d<double>(p2.x() + cp.x(), p2.y() + cp.y());
            
            markings.push_back( vgl_line_segment_2d< double >(p1, p2));
        }
    }
    
    // female 3 point line left
    {
        vgl_point_2d<double> cp(63.0/12.0*foot_to_meter, height/2.0*foot_to_meter);
        const double radius = (19+6-63.0/12.0) * foot_to_meter;
        for ( int i = -16; i < 16; ++i )
        {
            double startTheta = 2 * vnl_math::pi / 64 * i;
            double stopTheta  = 2 * vnl_math::pi / 64 * ( i + 1 );
            vgl_point_2d<double> p1(cos(startTheta) * radius, sin(startTheta) * radius);
            vgl_point_2d<double> p2(cos(stopTheta) * radius, sin(stopTheta)   * radius);
            
            p1 = vgl_point_2d<double>(p1.x() + cp.x(), p1.y() + cp.y());
            p2 = vgl_point_2d<double>(p2.x() + cp.x(), p2.y() + cp.y());
            
            markings.push_back( vgl_line_segment_2d< double >(p1, p2));
        }
    }
    
    // female 3 point line right
    {
        vgl_point_2d<double> cp((width-63.0/12.0)*foot_to_meter, height/2.0*foot_to_meter);
        const double radius = (19+6-63.0/12.0) * foot_to_meter;
        for ( int i = 16; i < 48; ++i )
        {
            double startTheta = 2 * vnl_math::pi / 64 * i;
            double stopTheta  = 2 * vnl_math::pi / 64 * ( i + 1 );
            vgl_point_2d<double> p1(cos(startTheta) * radius, sin(startTheta) * radius);
            vgl_point_2d<double> p2(cos(stopTheta) * radius, sin(stopTheta)   * radius);
            
            p1 = vgl_point_2d<double>(p1.x() + cp.x(), p1.y() + cp.y());
            p2 = vgl_point_2d<double>(p2.x() + cp.x(), p2.y() + cp.y());
            
            markings.push_back( vgl_line_segment_2d< double >(p1, p2));
        }
    }    
    return markings;
}


// points that locate in the intersection of field lines
vector<vgl_point_2d<double> >
USBasketballCourt::getCalibratePoints(const double width, const double height)
{
    vector<vgl_point_2d<double> > points = USBasketballCourt::markingEndPoints();
    return points;
    
}

vector<vgl_point_2d<double> >
USBasketballCourt::markingEndPoints(const double width, const double height)
{
    assert(width > 40 && height > 25);
    
    const int pts_num = 30;
    vector<vgl_point_2d<double> > marking_pts;
    marking_pts.resize(pts_num);
    
    marking_pts[0] = vgl_point_2d<double>(0, 0);
    marking_pts[1] = vgl_point_2d<double>(width, 0);
    marking_pts[2] = vgl_point_2d<double>(width, height);
    marking_pts[3] = vgl_point_2d<double>(0, height);
    marking_pts[4] = vgl_point_2d<double>(width/2, 0);
    marking_pts[5] = vgl_point_2d<double>(width/2, height);
    
    // center circle
    marking_pts[6] = vgl_point_2d<double>(width/2, height/2 - 12/2.0);
    marking_pts[7] = vgl_point_2d<double>(width/2, height/2 + 12/2.0);
    
    // left penalty line
    marking_pts[8] = vgl_point_2d<double>(19, (height-12)/2.0);
    marking_pts[9] = vgl_point_2d<double>(19, (height-12)/2.0 + 12.0);
    
    // right penalty line
    marking_pts[10] = vgl_point_2d<double>(width - 19.0, (height-12)/2.0);
    marking_pts[11] = vgl_point_2d<double>(width - 19.0, (height-12)/2.0 + 12.0);
    
    //
    marking_pts[12] = vgl_point_2d<double>(0, (height-12)/2.0);
    marking_pts[13] = vgl_point_2d<double>(0, (height-12)/2.0 + 12.0);
    marking_pts[14] = vgl_point_2d<double>(width, (height-12)/2.0);
    marking_pts[15] = vgl_point_2d<double>(width, (height-12)/2.0 + 12.0);
    
    // virtual points
    marking_pts[16] = vgl_point_2d<double>(19, 0);
    marking_pts[17] = vgl_point_2d<double>(19, height);
    marking_pts[18] = vgl_point_2d<double>(width - 19, 0);
    marking_pts[19] = vgl_point_2d<double>(width - 19, height);
    
    // 3 point line extension, left
    const double radius_three_point = 19+6-63.0/12.0;
    const double dis_to_sideline = (height-2*radius_three_point)/2.0;
    marking_pts[20] = vgl_point_2d<double>(0, dis_to_sideline);
    marking_pts[21] = vgl_point_2d<double>(0, height - dis_to_sideline);
    marking_pts[22] = vgl_point_2d<double>(63.0/12.0, dis_to_sideline);
    marking_pts[23] = vgl_point_2d<double>(63.0/12.0, height - dis_to_sideline);
    
    // 3 point line extension, right
    marking_pts[24] = vgl_point_2d<double>(width, dis_to_sideline);
    marking_pts[25] = vgl_point_2d<double>(width, height - dis_to_sideline);
    marking_pts[26] = vgl_point_2d<double>(width - 63.0/12.0, dis_to_sideline);
    marking_pts[27] = vgl_point_2d<double>(width - 63.0/12.0, height - dis_to_sideline);
    
    // penalty circle and 3 point line intersection
    marking_pts[28] = vgl_point_2d<double>(25.0, height/2.0);
    marking_pts[29] = vgl_point_2d<double>(width - 25.0, height/2.0);
    
    // foot to meter
    const double foot_to_meter = 0.3048;
    for (int i = 0; i<marking_pts.size(); i++) {
        double x = marking_pts[i].x() * foot_to_meter;
        double y = marking_pts[i].y() * foot_to_meter;
        marking_pts[i] = vgl_point_2d<double>(x, y);
    }
    
    return marking_pts;
}

vector<std::pair<int, int> > USBasketballCourt::pointPairsToLineSegments()
{
    int pair [][2] = {
        0, 4, 0, 12,
        1, 4, 1, 14,
        2, 5, 2, 15,
        3, 5, 3, 13,
        4, 6,
        5, 7,
        6, 7,
        8, 9, 8, 12,
        9, 13,
        10, 11, 10, 14,
        11, 15,
        12, 13,
        14, 15,
        20, 22,
        21, 23,
        24, 26,
        25, 27
    };
    
    vector<std::pair<int, int> > ret_pairs;
    for (int i = 0; i<sizeof(pair)/(2*sizeof(pair[0][0])); i++) {
        ret_pairs.push_back(std::pair<int, int>(pair[i][0], pair[i][1]));
    }
    return ret_pairs;
}

void USBasketballCourt::projectCalibPoints(const vpgl_perspective_camera<double> &camera,
                                           int width, int height,
                                           vector<vgl_point_2d<double> > & wld_pts,
                                           vector<vgl_point_2d<double> > & img_pts,
                                           int threshold)
{
    assert(wld_pts.size() == 0);
    assert(img_pts.size() == 0);
    
    double field_width  = this->fieldWidth();
    double field_height = this->fieldHeight();
    vcl_vector<vgl_point_2d<double> > courtPts = this->getCalibratePoints(field_width, field_height);
    
    
    for (int i = 0; i<courtPts.size(); i++) {
        vgl_homg_point_3d<double> p(courtPts[i].x(), courtPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        
        if (VglPlus::vgl_inside_image(q, width, height, threshold))
        {
            wld_pts.push_back(vgl_point_2d<double>(p.x(), p.y()));
            img_pts.push_back(q);
        }
    }
    assert(wld_pts.size() == img_pts.size());
}

void USBasketballCourt::projectCalibPoints(const vpgl_perspective_camera<double> &camera,
                                           int width, int height,
                                           vector<vgl_point_2d<double> > & wld_pts,
                                           vector<vgl_point_2d<double> > & img_pts,
                                           vector<vgl_point_2d<double> > & court_diagram_pts,
                                           int threshold)
{
    assert(wld_pts.size() == 0);
    assert(img_pts.size() == 0);
    
    double field_width  = this->fieldWidth();
    double field_height = this->fieldHeight();
    vcl_vector<vgl_point_2d<double> > courtPts = this->getCalibratePoints(field_width, field_height);
    
    
    for (int i = 0; i<courtPts.size(); i++) {
        vgl_homg_point_3d<double> p(courtPts[i].x(), courtPts[i].y(), 0, 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        
        vgl_point_2d<double> q = vgl_point_2d< double >(camera.project(p));
        
        if (VglPlus::vgl_inside_image(q, width, height, threshold))
        {
            wld_pts.push_back(vgl_point_2d<double>(p.x(), p.y()));
            img_pts.push_back(q);
        }
    }
    assert(wld_pts.size() == img_pts.size());
    
    // from world to image coordinate
    vgl_transform_2d<double> worldTodiagram = this->worldToDiagram();
    for (int i = 0; i<wld_pts.size(); i++) {
        vgl_point_2d<double> p = wld_pts[i];
        vgl_point_2d<double> q = worldTodiagram(p);
        court_diagram_pts.push_back(q);
    }
    
    assert(wld_pts.size() == court_diagram_pts.size());
}

bool USBasketballCourt::projectNode(const vpgl_perspective_camera<double> & camera, int node,
                               vgl_line_segment_2d<double> & wld_seg,
                               vgl_line_segment_2d<double> & img_seg)
{
    // linesegment in world coordinate
    double field_width  = this->fieldWidth();
    double field_height = this->fieldHeight();
    vector< vgl_line_segment_2d< double > > lineSegments;
    vector<vgl_point_2d<double> > line_end_pts = USBasketballCourt::markingEndPoints(field_width, field_height);
    vector<std::pair<int, int> > end_pts_pairs = USBasketballCourt::pointPairsToLineSegments();
    for (int i = 0; i<end_pts_pairs.size(); i++) {
        int s = end_pts_pairs[i].first;
        int e = end_pts_pairs[i].second;
        lineSegments.push_back(vgl_line_segment_2d<double>(line_end_pts[s], line_end_pts[e]));
    }
    
    if (node < 0 || node >= lineSegments.size()) {
        return false;
    }
    wld_seg = lineSegments[node];
    
    vgl_homg_point_3d<double> p1 = vgl_homg_point_3d<double>(wld_seg.point1().x(), wld_seg.point1().y(), 0.0, 1.0);
    vgl_homg_point_3d<double> p2 = vgl_homg_point_3d<double>(wld_seg.point2().x(), wld_seg.point2().y(), 0.0, 1.0);
    
    const int w = camera.get_calibration().principal_point().x() * 2;
    const int h = camera.get_calibration().principal_point().y() * 2;
    
    if (!camera.is_behind_camera(p1) && !camera.is_behind_camera(p2))
    {
        // make sure the sampled segment is inside image
        vgl_line_segment_2d<double> in_image_segment;
        bool isInImage = VpglPlus::sampleLineSegment(camera, w, h, wld_seg, 0.5, in_image_segment);
        if (!isInImage) {
            return false;
        }
        
        vgl_point_2d<double> q1 = camera.project(vgl_point_3d<double>(in_image_segment.point1().x(), in_image_segment.point1().y(), 0.0));
        vgl_point_2d<double> q2 = camera.project(vgl_point_3d<double>(in_image_segment.point2().x(), in_image_segment.point2().y(), 0.0));
        
        img_seg = vgl_line_segment_2d<double>(q1, q2);
        return true;
    }
    return false;
}



/*****************              USHSBasketballCourt                  *****************/
USHSBasketballCourt::USHSBasketballCourt(double width, double height)
{
    field_width_  = width;
    field_height_ = height;
}

USHSBasketballCourt::~USHSBasketballCourt()
{
    
}

void USHSBasketballCourt::courtDiagram(vil_image_view<vxl_byte> & image)
{
    const int w = field_width_ * 12 + 70 * 2;
    const int h = field_height_ * 12 + 70 * 2;
    image = vil_image_view<vxl_byte>(w, h, 3);
    // brown	244 	164 	96
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            image(i, j, 0) = 244;
            image(i, j, 1) = 164;
            image(i, j, 2) = 96;
        }
    }
    
    vgl_transform_2d<double> wld_to_diagram = this->worldToDiagram();
    vcl_vector< vgl_line_segment_2d< double > > courLine = this->getAllLineSegments(field_width_, field_height_);
    for (int i = 0; i<courLine.size(); i++) {
        vgl_point_2d<double> p1 = courLine[i].point1();
        vgl_point_2d<double> p2 = courLine[i].point2();
        
        vgl_point_2d<double> q1 = wld_to_diagram(p1);
        vgl_point_2d<double> q2 = wld_to_diagram(p2);
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), VilPlus::white(), 2);
    }
}


vgl_transform_2d<double> USHSBasketballCourt::diagramToWorld()
{
    double imageH = field_height_ * 12;
    double m1[9] = {
        1, 0, -70,
        0, 1, -70,
        0, 0,  1};
    double m2[9] = {
        1, 0, 0,
        0, -1, imageH,
        0, 0, 1};
    //every 1 pixel is 1 inch, inch to meter
    double m3[9] = {
        1 * 0.0254, 0, 0,
        0, 1 * 0.0254, 0,
        0, 0, 1
    };
    
    vgl_transform_2d< double > model;
    model = vgl_transform_2d<double>(vnl_matrix_fixed<double, 3, 3>(m3) * vnl_matrix_fixed<double, 3, 3>(m2) * vnl_matrix_fixed<double, 3, 3>(m1));
    return model;
}

vgl_transform_2d<double> USHSBasketballCourt::worldToDiagram()
{
    double imageH = field_height_ * 12;
    // meter to inch
    double m1[9] = {
        39.37, 0, 0,
        0, 39.37, 0,
        0, 0, 1};
    
    double m2[9] = {
            1, 0, 0,
            0, -1, imageH,
            0, 0, 1};
    
    double m3[9] = {
        1, 0, 70,
        0, 1, 70,
        0, 0,  1};
    
    vgl_transform_2d< double > model;
    model = vgl_transform_2d<double>(vnl_matrix_fixed<double, 3, 3>(m3) * vnl_matrix_fixed<double, 3, 3>(m2) * vnl_matrix_fixed<double, 3, 3>(m1));
    return model;
}

string USHSBasketballCourt::name()
{
    return string("USHSBasketballCourt");
}


void USHSBasketballCourt::overlayLines(const vpgl_perspective_camera<double> & camera,
                                       vil_image_view<vxl_byte> &image,
                                       const vector<vxl_byte> & color)
{
    assert(image.nplanes() == 3);
    
    vector< vgl_line_segment_2d< double > > markings = getAllLineSegments(field_width_, field_height_);
    
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
}

void USHSBasketballCourt::projectLineSegments(const vpgl_perspective_camera<double> & camera,
                                              vector< vgl_line_segment_2d< double > > &lines)
{
    vector< vgl_line_segment_2d< double > > markings = this->getAllLineSegments(field_width_, field_height_);
    
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

void USHSBasketballCourt::projectPoints(const vpgl_perspective_camera<double> & camera,
                                        vector< vgl_point_2d<double> > & points)
{
    vector< vgl_point_2d<double> > pts = USBasketballCourt::markingEndPoints(field_width_, field_height_);
    
    for ( unsigned int i = 0; i < pts.size(); ++i )
    {
        vgl_homg_point_3d< double > p1( pts[i].x(), pts[i].y(), 0, 1.0 );
        
        if (camera.is_behind_camera(p1)) {
            continue;
        }
        
        vgl_point_2d< double > q = vgl_point_2d< double >(camera.project(p1));
        points.push_back(q);
    }
}


