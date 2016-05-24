//
//  vnl_flann.h
//  QuadCopter
//
//  Created by jimmy on 9/1/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vnl_flann__
#define __QuadCopter__vnl_flann__

// wrap flann with vnl

#include <vnl/vnl_vector.h>
#include <vcl_vector.h>
#include <flann/flann.hpp>

class vnl_flann
{
    flann::Index<flann::L2<double> > index_;   // store kd tree
    int dim_;
    
public:
    vnl_flann(const flann::IndexParams& params = flann::KDTreeIndexParams(4)):index_(params)
    {
        dim_ = 0;        
    }
    ~vnl_flann()
    {
        
    }
    
    void set_data(const vnl_matrix<double> & data);
    void set_data(const vcl_vector<vnl_vector<double> > & data);
    
    void search(const vnl_matrix<double> & query_data,
                vcl_vector<vcl_vector<int> > & indices,
                vcl_vector<vcl_vector<double> > & dists, int knn) const;
    
    void search(const vcl_vector<vnl_vector<double> > & query_data,
                vcl_vector<vcl_vector<int> > & indices,
                vcl_vector<vcl_vector<double> > & dists, int knn,
                int num_search_leaf = 128) const;
    
    // flann only save index, so it needs original data to construct the whole kd-tree
    void save(const char *index_file);
    void load(const char *index_file, const vcl_vector<vnl_vector<double> > & data);
    
};


#endif /* defined(__QuadCopter__vnl_flann__) */
