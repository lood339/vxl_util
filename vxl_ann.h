//
//  vxl_ann.h
//  OnlineStereo
//
//  Created by jimmy on 2/21/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vxl_ann__
#define __OnlineStereo__vxl_ann__

// wrap ANN with vxl
#include <ann/ANN.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>


class VxlANNQuadTree
{
public:
	VxlANNQuadTree(int k, int dim);
	~VxlANNQuadTree();
    
	bool set_tree(const vcl_vector<vgl_point_2d<double> > & data);
    vgl_point_2d<double> nearest(const vgl_point_2d<double> & query);
    
private:
	int					m_nPts;					// actual number of data points
	ANNpointArray		m_dataPts;				// data points
	ANNpoint			m_queryPt;				// query point
	ANNidxArray			m_nnIdx;					// near neighbor indices
	ANNdistArray		m_dists;					// near neighbor distances
	ANNkd_tree*			m_kdTree;					// search structure
    
	int m_dim;
	int m_k;
    
private:	
	void createTree(void);
    
};



#endif /* defined(__OnlineStereo__vxl_ann__) */
