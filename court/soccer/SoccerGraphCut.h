//
//  SoccerGraphCut.h
//  OnlineStereo
//
//  Created by jimmy on 2/18/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__SoccerGraphCut__
#define __OnlineStereo__SoccerGraphCut__

// model geometries (lines and circles in the soccer court as a grph)

#include <vcl_vector.h>
#include <vcl_map.h>
#include <vcl_utility.h>

#include <vil/vil_image_view.h>
#include <vgl/vgl_line_2d.h>
#include <vgl/vgl_point_2d.h>
#include <vpgl/vpgl_perspective_camera.h>


#include <vgl/vgl_intersection.h>
#include "WWoSCourtLine.h"
#include "vxl_hough_line.h"
#include "SoccerLineParameters.h"

struct TerminalLink
{
    int node_;
    double source_;
    double sink_;
    
    TerminalLink(){}
    TerminalLink(int n, double sc, double sk)
    {
        node_ = n; source_ = sc; sink_ = sk;
    }
};

struct EdgeLink
{
    int node1_;
    int node2_;
    double cap_;
    
    EdgeLink(){}
    EdgeLink(int n1, int n2, double c1)
    {
        node1_ = n1;
        node2_ = n2;
        cap_ = c1;
    }    
};

class SoccerGraphCut
{
    int geometry_num_;
    
public:
    SoccerGraphCut(int geometry_num);
    ~SoccerGraphCut();    
    
    // sourceSinkLink: int, double, double -> node index (from 0), capacity to souce, capacity to sink. It is terminal link.
    // edges:          int, int, double    -> node1, node2, capacity between them (indirected) assume node1 < node2
    // isSource:       return labels belong to source or sink
    // return: min cut
    double solve_min_flow(const vcl_vector<TerminalLink > & sourceSinkLink,
                          const vcl_vector<EdgeLink> & edges,
                          vcl_vector<vcl_pair<int, bool> > & isSource);
};







#endif /* defined(__OnlineStereo__SoccerGraphCut__) */
