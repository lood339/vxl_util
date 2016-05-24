//
//  SoccerGraphCut.cpp
//  OnlineStereo
//
//  Created by jimmy on 2/18/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "SoccerGraphCut.h"
#include "vxl_plus.h"
#include "graph.h"
#include <vil/vil_crop.h>
#include <vgl/vgl_intersection.h>
#include <vgl/vgl_distance.h>
#include "vil_plus.h"
#include "vxl_hough_line.h"
#include "vil_gmm_util.h"

SoccerGraphCut::SoccerGraphCut(int geometry_num)
{
    geometry_num_ = geometry_num;
}
SoccerGraphCut::~SoccerGraphCut()
{
    
}

double SoccerGraphCut::solve_min_flow(const vcl_vector<TerminalLink > & sourceSinkLink,
                                      const vcl_vector<EdgeLink> & edges,
                                      vcl_vector<vcl_pair<int, bool> > & isSource)
{
    assert(sourceSinkLink.size() == geometry_num_);
    
    int nodeNum = (int)sourceSinkLink.size();
    int edgeNum = (int)edges.size();
    
	Graph<double, double, double> g = Graph<double, double, double>(nodeNum, edgeNum);
    
    // create node
    for (int i = 0; i<sourceSinkLink.size(); i++) {
        g.add_node();
    }
    
    // set terminal link
    for (int i = 0; i<sourceSinkLink.size(); i++) {
        int node = sourceSinkLink[i].node_;
        assert(node < nodeNum);
        g.add_tweights(node, sourceSinkLink[i].source_, sourceSinkLink[i].sink_);
    }
    
    // set edges
    for (int i = 0; i<edges.size(); i++) {
        int node1 = edges[i].node1_;
        int node2 = edges[i].node2_;
        assert(node1 < nodeNum);
        assert(node1 < node2);
        g.add_edge(node1, node2, edges[i].cap_, edges[i].cap_);
    }    
    
	double flow = g.maxflow();    
    isSource.clear();
    for (int i = 0; i<sourceSinkLink.size(); i++) {
        int node = sourceSinkLink[i].node_;
        if (g.what_segment(node) == Graph<double, double, double>::SOURCE) {
            isSource.push_back(vcl_pair<int, bool> (node, true));
        }
        else
        {
            isSource.push_back(vcl_pair<int, bool> (node, false));
        }
    }
    return flow;
}






