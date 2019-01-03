//
//  opengl_utils.cpp
//  PlanarAlign
//
//  Created by Jimmy Chen LOCAL on 1/21/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "opengl_utils.h"
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>

void OpenglUtils::drawCross(const vector<vgl_point_2d<double> > &pts, float width)
{
    glBegin(GL_LINES);
    for (unsigned int i = 0; i<pts.size(); i++)
    {
        //center point
        double px = pts[i].x();
        double py = pts[i].y();
        
        vgl_point_2d<double> p1, p2, p3, p4;
        
        double h_l = width;
        p1.set(px - h_l, py);
        p2.set(px + h_l, py);
        p3.set(px, py - h_l);
        p4.set(px, py + h_l);
        
        //p1 --> p2, p3 --> p4
        glVertex3f(p1.x(), p1.y(), 0.0);
        glVertex3f(p2.x(), p2.y(), 0.0);
        glVertex3f(p3.x(), p3.y(), 0.0);
        glVertex3f(p4.x(), p4.y(), 0.0);
    }
    glEnd();
}

void OpenglUtils::drawLineSegment(const vector< vgl_line_segment_2d< double > > &lines)
{
    glBegin(GL_LINES);
    for (unsigned int i = 0; i<lines.size(); i++) {
        vgl_point_2d<double> p1 = lines[i].point1();
        vgl_point_2d<double> p2 = lines[i].point2();
        
        glVertex3f(p1.x(), p1.y(), 0.0);
        glVertex3f(p2.x(), p2.y(), 0.0);
    }
    glEnd();
    
}

void OpenglUtils::drawCircle(const vector<vgl_point_2d<double> > & centers, float radius)
{
    const float DEG2RAD = 3.14159/180;
    for (unsigned int i = 0; i<centers.size(); i++) {
        glBegin(GL_LINE_LOOP);
        for (unsigned int j = 0; j<360; j += 30) {
            float degInRad = j * DEG2RAD;
            glVertex2f(cosf(degInRad) * radius + centers[i].x(), sinf(degInRad) * radius + centers[i].y());
        }
        glEnd();
    }
}

void OpenglUtils::drawBox(const vector<vgl_point_2d<double> > &pts, float width)
{
    glBegin(GL_LINES);
    for (unsigned int i = 0; i<pts.size(); i++)
    {
        //center point
        double px = pts[i].x();
        double py = pts[i].y();
        
        vgl_point_2d<double> p1, p2, p3, p4;
        
        double h_l = width/2;
        p1.set(px - h_l, py - h_l);
        p2.set(px + h_l, py - h_l);
        p3.set(px + h_l, py + h_l);
        p4.set(px - h_l, py + h_l);
        
        //p1 --> p2, p2 --> p3, 3---4, 4----1
        glVertex3f(p1.x(), p1.y(), 0.0);  glVertex3f(p2.x(), p2.y(), 0.0);
        glVertex3f(p2.x(), p2.y(), 0.0);  glVertex3f(p3.x(), p3.y(), 0.0);
        glVertex3f(p3.x(), p3.y(), 0.0);  glVertex3f(p4.x(), p4.y(), 0.0);
        glVertex3f(p4.x(), p4.y(), 0.0);  glVertex3f(p1.x(), p1.y(), 0.0);
    }
    glEnd();
}
