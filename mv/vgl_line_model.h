//
//  vgl_line_model.h
//  WireframeModel
//
//  Created by jimmy on 6/19/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __WireframeModel__vgl_line_model__
#define __WireframeModel__vgl_line_model__

// line model from multiple view line reconstruction
#include <vpgl/vpgl_perspective_camera.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vcl_vector.h>

struct LineSegmentWithIndex
{
    int index_; // index in the line map
    vgl_line_segment_2d<double> seg_;
};

class VglLineModel
{
public:
    VglLineModel();
    ~VglLineModel();
    
    void setCameras(const vcl_vector<vpgl_perspective_camera<double> > & cameras){cameras_ = cameras;}
    void setImagenames(const vcl_vector<vcl_string> & names){imageNames_ = names;}
    void setSegments(const vcl_vector<vcl_vector<vgl_line_segment_2d<double> > > & segments){segmentsVec_ = segments;}
    void setUnmatchedSegments(const vcl_vector<vcl_vector<vgl_line_segment_2d<double> > > & segments){unmatchedSegmentsVec_ = segments;}
    void setIndexedSegments(const vcl_vector<vcl_vector< LineSegmentWithIndex > > & segments){indexedSegmentsVec_ = segments;}
    
    vcl_vector<vpgl_perspective_camera<double> > cameras(void) const {return cameras_;}
    vcl_vector<vcl_string> imageNames(void) const {return imageNames_;}
    vcl_vector<vcl_vector<vgl_line_segment_2d<double>> > segments(void) const {return segmentsVec_;}
    vcl_vector<vcl_vector<vgl_line_segment_2d<double>> > unmatchedSegments(void) const {return unmatchedSegmentsVec_;}
    vcl_vector< vcl_vector<LineSegmentWithIndex > > indexedSegments(void)const {return indexedSegmentsVec_;}
    
    void clear();
    bool write(const char *file) const;
    bool read(const char *file);
    
private:
    vcl_vector<vpgl_perspective_camera<double> > cameras_;  // related camera in each view
    vcl_vector<vcl_string> imageNames_;                     // image names of related view
    
    // matched 2D line segment in different views images, order maters
    vcl_vector<vcl_vector<vgl_line_segment_2d<double>> > segmentsVec_;
    
    // un matched 2D line segment, it could be 0 in some views, order not mater
    vcl_vector<vcl_vector<vgl_line_segment_2d<double>> > unmatchedSegmentsVec_;
    
    // line segment with index
    vcl_vector< vcl_vector<LineSegmentWithIndex > > indexedSegmentsVec_;
    
};

#endif /* defined(__WireframeModel__vgl_line_model__) */
