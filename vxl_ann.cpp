//
//  vxl_ann.cpp
//  OnlineStereo
//
//  Created by jimmy on 2/21/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vxl_ann.h"
#include <vcl_vector.h>

#define MAX_NODE_NUM 60000

VxlANNQuadTree::VxlANNQuadTree(int k, int dim)
{
	m_nPts = 0;
	m_dataPts = NULL;
	m_queryPt = NULL;
	m_nnIdx = NULL;
	m_dists = NULL;
	m_kdTree = NULL;
	m_k   = k;
	m_dim = dim;
    
	m_queryPt = annAllocPt(m_dim);
	m_dataPts = annAllocPts(MAX_NODE_NUM, m_dim);		//most 40 000 point in the k-d tree
	m_nnIdx = new ANNidx[m_k];
	m_dists = new ANNdist[m_k];
}
VxlANNQuadTree::~VxlANNQuadTree()
{
	if(m_nnIdx)
	{
		delete [] m_nnIdx;
		m_nnIdx = NULL;
	}
	if(m_dists)
	{
		delete [] m_dists;
		m_dists = NULL;
	}
	if(m_kdTree)
	{
		delete m_kdTree;
		m_kdTree = NULL;
	}
	
	annDeallocPt(m_queryPt);
	annDeallocPts(m_dataPts);
	annClose();
}

bool VxlANNQuadTree::set_tree(const vcl_vector<vgl_point_2d<double> > & data)
{
    assert(m_dim == 2);
    
    m_nPts = 0;
    for (int i = 0; i<data.size(); i++) {
        m_dataPts[i][0] = data[i].x();
        m_dataPts[i][1] = data[i].y();
        m_nPts++;
    }
	
//	fprintf(stderr, "ann actual node number = %d\n", m_nPts);
	if (m_nPts <= MAX_NODE_NUM) {
		m_kdTree = new ANNkd_tree(					// build search structure
                                  m_dataPts,					// the data points
                                  m_nPts,						// number of points
                                  m_dim);						// dimension of space
	}
    
	if (!m_kdTree) {
        fprintf(stderr, "VxlANNQuadTree::set_tree \n");
        return false;
	}
    return true;
}

vgl_point_2d<double> VxlANNQuadTree::nearest(const vgl_point_2d<double> & query)
{
    m_queryPt[0] = query.x();
    m_queryPt[1] = query.y();
    
    m_kdTree->annkSearch(                       // search
                         m_queryPt,				// query point
                         m_k,					// number of near neighbors
                         m_nnIdx,				// nearest neighbors (returned)
                         m_dists,				// distance (returned)
                         0);
   // printf("distance = %f \n", m_dists[0]);
    int idx = m_nnIdx[0];
    return vgl_point_2d<double>(m_dataPts[idx][0], m_dataPts[idx][1]);
}
