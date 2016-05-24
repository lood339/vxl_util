//
//  vxl_hough_line.cpp
//  OnlineStereo
//
//  Created by jimmy on 2/16/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vxl_hough_line.h"
#include <vil/vil_hough.h>
#include <vgl/vgl_distance.h>
#include "vil_plus.h"

#include <vgl/vgl_fit_line_2d.h>

int VxlHoughLine::oneByOneLineDetection(const vcl_vector<vgl_point_2d<double> > & pts, int imageW, int imageH,
                                        const VxlHoughParameter & para, vcl_vector<vgl_line_2d<double> > & lines)
{
    vil_image_view<vxl_byte> edgeMask(imageW, imageH, 1);
    edgeMask.fill(0);
    
    for (int i = 0; i<pts.size(); i++) {
        edgeMask(int(pts[i].x()), int(pts[i].y())) = 255;
    }
    return VxlHoughLine::oneByOneLineDetection(edgeMask, para, lines);
}

int VxlHoughLine::oneByOneLineDetection(const vil_image_view<vxl_byte> & edgeMask, const VxlHoughParameter & para,
                                        vcl_vector<vgl_line_2d<double> > & lines)
{
    assert(edgeMask.nplanes() == 1 || edgeMask.nplanes() == 3);
    
    const int w = edgeMask.ni();
    const int h = edgeMask.nj();
    
    // set line detection parameters
    const int line_num = para.maxLineNum_;
    const double line_distance_threshold = para.lineWidth_;
    float rhoResolution   = para.rhoResolution_;
    float thetaResolution = para.thetaResolution_;
    double houghThreshold = para.houghThreshold_;
    
    vil_image_view<vxl_byte> curMask;
    curMask.deep_copy(edgeMask);
    for (int i = 0; i<line_num; i++) {
        // convert white color pixel to mask
        vil_image_view<bool> curLineMask(w, h, 1);
        curLineMask.fill(false);
        
        for (int y = 0; y<h; y++) {
            for (int x = 0; x<w; x++) {
                if (curMask(x, y, 0) == 255) {
                    curLineMask(x, y) = true;
                }
            }
        }
      //  VilPlus::vil_save(curMask, "curMask.jpg");
        
        // hough accumulation image
        vil_hough_representation houghRepresentation;
        vil_hough(curLineMask, houghRepresentation, rhoResolution, thetaResolution);
        
        // detection lines
        vcl_vector<vgl_line_2d<double> > curLines;
        vil_hough_detect_lines(houghRepresentation, curLines, houghThreshold);
        
        if (curLines.size() == 0) {
            printf("Warning: can not new find line\n");
            break;
        }
        
        // remove the edge pixels along the line in the image
        int edgePixelNum = 0;
        for (int y = 0; y<h; y++) {
            for (int x = 0; x<w; x++) {
                if (curMask(x, y) == 255) {
                    vgl_point_2d<double> p(x, y);
                    double dis = vgl_distance(curLines[0], p);
                    if (dis <= line_distance_threshold) {
                        for (int k = 0; k<curLineMask.nplanes(); k++) {
                            curMask(x, y, k) = 0;
                        }
                        edgePixelNum++;
                    }
                }
            }
        }
        if (edgePixelNum >= para.minPixeNum_) {
            lines.push_back(curLines[0]);
        }
      //  printf("edge pixel number is %d\n", edgePixelNum);
    }
    return (int)lines.size();
}

bool VxlHoughLine::detectOneLine(const vcl_vector<vgl_point_2d<double> > & pts, int imageW, int imageH,
                                 const VxlHoughParameter & para, vgl_line_2d<double> & line,
                                 vcl_vector<vgl_point_2d<double> > & inlierPts)
{
    assert(para.maxLineNum_ == 1);
    
    vil_image_view<vxl_byte> edgeMask(imageW, imageH, 1);
    edgeMask.fill(0);
    
    for (int i = 0; i<pts.size(); i++) {
        edgeMask(int(pts[i].x()), int(pts[i].y())) = 255;
    }
    
    vcl_vector<vgl_line_2d<double> > lines;
    int num = VxlHoughLine::oneByOneLineDetection(edgeMask, para, lines);
    if (num != 1) {
     //   printf("Warning: Hough line detection failed.\n");
        return false;
    }
    line = lines[0];
    for (int i = 0; i<pts.size(); i++) {
        double dis = vgl_distance(line, pts[i]);
        if (dis <= para.inlierDistance_) {
            inlierPts.push_back(pts[i]);
        }
    }
  //  printf("line feature inlier number, ratio is %lu, %f\n", pts.size(), 1.0*inlierPts.size()/pts.size());
    return true;
}
/*
bool VxlHoughLine::estimateCenterLine(const vcl_vector<vgl_point_2d<double> > & line1Pts, const vcl_vector<vgl_point_2d<double> > & line2Pts,
                                      vcl_vector<vgl_point_2d<double> > & centerPts, vgl_line_2d<double> & centerLine)
{
    assert(line1Pts.size() >= line2Pts.size());
   // VxlANNQuadTree(int k, int dim);
    VxlANNQuadTree quadTree(1, 2);
    quadTree.set_tree(line1Pts);
    
    for (int i = 0; i<line2Pts.size(); i++) {
        vgl_point_2d<double> p = line2Pts[i];
        vgl_point_2d<double> q = quadTree.nearest(p);
        double x = (q.x() + p.x())/2.0;
        double y = (q.y() + p.y())/2.0;        
        centerPts.push_back(vgl_point_2d<double>(x, y));
    }
    
    if (centerPts.size() < 5) {
        return false;
    }
    
    centerLine = vgl_fit_line_2d(centerPts);
    return true;
}
 */
















