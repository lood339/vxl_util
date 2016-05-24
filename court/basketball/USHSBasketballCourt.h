//
//  USHSBasketballCourt.h
//  OnlineStereo
//
//  Created by jimmy on 5/2/16.
//  Copyright (c) 2016 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__USHSBasketballCourt__
#define __OnlineStereo__USHSBasketballCourt__

// US high school basketball court
#include <vector>
#include <vgl/vgl_line_segment_2d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_transform_2d.h>

using std::string;
using std::vector;

class USBasketballCourt
{
public:
    USBasketballCourt(){};
    virtual ~USBasketballCourt(){};
    
   
    virtual void courtDiagram(vil_image_view<vxl_byte> & image) = 0;
    virtual vgl_transform_2d<double> diagramToWorld() = 0;
    virtual vgl_transform_2d<double> worldToDiagram() = 0;
    virtual string name() = 0;
    
    virtual void projectCalibPoints(const vpgl_perspective_camera<double> &camera,
                                    int width, int height,
                                    vector<vgl_point_2d<double> > & wld_pts,
                                    vector<vgl_point_2d<double> > & img_pts,
                                    int threshold); // threshold away from the image boundary
    
    // project points from world coordinate to image and diagram
    virtual void projectCalibPoints(const vpgl_perspective_camera<double> &camera,
                                    int width, int height,
                                    vector<vgl_point_2d<double> > & wld_pts,
                                    vector<vgl_point_2d<double> > & img_pts,
                                    vector<vgl_point_2d<double> > & court_diagram_pts,
                                    int threshold); // threshold away from the image boundary
    
    
    // project short line segment
    virtual bool projectNode(const vpgl_perspective_camera<double> & camera, int node,
                             vgl_line_segment_2d<double> & wld_seg,
                             vgl_line_segment_2d<double> & img_seg);
    
    
    virtual double fieldWidth()  = 0;
    virtual double fieldHeight() = 0;
    // each line segment used as a node
    virtual int lineSegmentSize() = 0;
    virtual void overlayLines(const vpgl_perspective_camera<double> & camera,
                              vil_image_view<vxl_byte> &image,
                              const vcl_vector<vxl_byte> & color) = 0;
    
    // width, height in feet
    // output in meter
    // only has 2D lines in the court
    static vector< vgl_line_segment_2d< double > >
    getAllLineSegments(const double width, const double height);
   
    
    // points that locate in the intersection of field lines
    static vector<vgl_point_2d<double> >
    getCalibratePoints(const double width = 84.0, const double height = 50.0);
    
    static vector<vgl_point_2d<double> >
    markingEndPoints(const double width = 84.0, const double height = 50.0);
    
protected:
    
    // point pairs to generate line segments
    static vector<std::pair<int, int> > pointPairsToLineSegments();
    
};

// high school
class USHSBasketballCourt : public USBasketballCourt
{
    double field_width_;
    double field_height_;
public:
    USHSBasketballCourt(double width = 84.0, double height = 50.0);
    ~USHSBasketballCourt();
   
    virtual void courtDiagram(vil_image_view<vxl_byte> & image);
    virtual vgl_transform_2d<double> diagramToWorld();
    virtual vgl_transform_2d<double> worldToDiagram();
    virtual string name();
    virtual double fieldWidth()
    {
        return field_width_;
    }
    virtual double fieldHeight()
    {
        return field_height_;
    }
    virtual int lineSegmentSize()
    {
        return (int)pointPairsToLineSegments().size();
    }
    
    
    // specified method
    void overlayLines(const vpgl_perspective_camera<double> & camera,
                      vil_image_view<vxl_byte> &image,
                      const vector<vxl_byte> & color);
    
    void projectLineSegments(const vpgl_perspective_camera<double> & camera,
                             vector< vgl_line_segment_2d< double > > &lines);
    
    void projectPoints(const vpgl_perspective_camera<double> & camera,
                       vector< vgl_point_2d<double> > & points);
    
    // project point to
    
};






#endif /* defined(__OnlineStereo__USHSBasketballCourt__) */
