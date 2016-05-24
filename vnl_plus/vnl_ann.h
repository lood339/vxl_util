//
//  vnl_ann.h
//  QuadCopter
//
//  Created by jimmy on 7/31/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef QuadCopter_vnl_ann_h
#define QuadCopter_vnl_ann_h

#include <ann/ANN.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector_fixed.h>

// N is vector dimension
template <unsigned int N>
class VnlANN
{
public:
    VnlANN(const vcl_vector<vnl_vector_fixed<double, N> > & dataset)
    {
        m_nPts = dataset.size();
        m_dataPts = NULL;
        m_queryPt = NULL;
        m_nnIdx = NULL;
        m_dists = NULL;
        m_kdTree = NULL;
        
        m_queryPt = annAllocPt(N);
        m_dataPts = annAllocPts(m_nPts, N);		//most 40 000 point in the k-d tree
        
        bool isSet = set_tree(dataset);
        assert(isSet);
    }
    ~VnlANN()
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
    
    void setK(const unsigned int k)
    {
        m_k = k;
        m_nnIdx = new ANNidx[m_k];
        m_dists = new ANNdist[m_k];
    }
    
    bool nearestNeighbors(const vnl_vector_fixed<double, N> & query,
                          vcl_vector<unsigned int> & nIndex,
                          vcl_vector<double> & nDistance, const double eps = 0.0)
    {
        assert(m_nnIdx);
        assert(m_dists);
        for (int i = 0; i<query.size(); i++) {
            m_queryPt[i] = query[i];
        }
        
        m_kdTree->annkSearch(                       // search
                             m_queryPt,				// query point
                             m_k,					// number of near neighbors
                             m_nnIdx,				// nearest neighbors (returned)
                             m_dists,				// distance (returned)
                             eps);
        nIndex.resize(m_k);
        nDistance.resize(m_k);
        for (int i = 0; i<m_k; i++) {
            nIndex[i] = m_nnIdx[i];
            nDistance[i] = m_dists[i];
        }
        return true;
    }
    
private:
    unsigned int		m_nPts;					// actual number of data points
	ANNpointArray		m_dataPts;				// data points
	ANNpoint			m_queryPt;				// query point
	ANNidxArray			m_nnIdx;				// near neighbor indices
	ANNdistArray		m_dists;				// near neighbor distances
	ANNkd_tree*			m_kdTree;				// search structure
    unsigned int        m_k;                    // k nearest neighbor
    
    bool set_tree(const vcl_vector<vnl_vector_fixed<double, N> > & data)
    {
        for (int i = 0; i<data.size(); i++) {
            for (int j = 0; j<N; j++) {
                m_dataPts[i][j] = data[i][j];
            }
        }
        assert(m_nPts == data.size());
        
     //   fprintf(stderr, "ann actual node number = %d, dimension = %d\n", m_nPts, N);
        m_kdTree = new ANNkd_tree(					// build search structure
                                  m_dataPts,					// the data points
                                  m_nPts,						// number of points
                                  N);						// dimension of space
        
        if (!m_kdTree) {
            fprintf(stderr, "VxlANNQuadTree::set_tree \n");
            return false;
        }
        return true;
    }

    
};


#endif
