//
//  vnl_flann.cpp
//  QuadCopter
//
//  Created by jimmy on 9/1/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vnl_flann.h"
#include <vnl/vnl_matrix.h>

using namespace flann;


void vnl_flann::set_data(const vnl_matrix<double> & data)
{
    vnl_matrix<double> data_copy = data;
    Matrix<double> dataset(data_copy.data_block(), (int)data_copy.rows(), (int)data_copy.cols());
    index_ = Index<L2<double> > (dataset, flann::KDTreeIndexParams(4));
    index_.buildIndex();
    dim_ = (int)data_copy.cols();
}

void vnl_flann::set_data(const vcl_vector<vnl_vector<double> > & data)
{
    assert(data.size() > 0);
    const int dim = (int)data.front().size();
    // wrap dataset
    vnl_matrix<double> data_mat((int)data.size(), (int)data.front().size());
    for (int i = 0; i<data.size(); i++) {
        assert(data[i].size() == dim);
        data_mat.set_row(i, data[i]);
    }
    
    Matrix<double> dataset(data_mat.data_block(), (int)data.size(), dim);
    index_ = Index<L2<double> > (dataset, flann::KDTreeIndexParams(4));
    index_.buildIndex();
    dim_ = dim;
}

void vnl_flann::search(const vnl_matrix<double> & query_data,
                       vcl_vector<vcl_vector<int> > & indices,
                       vcl_vector<vcl_vector<double> > & dists, int knn) const
{
    const int dim = (int)query_data.cols();
    assert(dim == dim_);
    
    Matrix<double> query_data_wrap((double *)query_data.data_block(), (int)query_data.rows(), dim);
    index_.knnSearch(query_data_wrap, indices, dists, knn, flann::SearchParams(128));
}

void vnl_flann::search(const vcl_vector<vnl_vector<double> > & query_data,
                       vcl_vector<vcl_vector<int> > & indices,
                       vcl_vector<vcl_vector<double> > & dists,
                       int knn,
                       int num_search_leaf) const
{
    const int dim = (int)query_data.front().size();
    assert(dim == dim_);
    
    // wrap query data
    vnl_matrix<double> query_data_mat((int)query_data.size(), (int)query_data.front().size(), dim);
    for (int i = 0; i<query_data.size(); i++) {
        query_data_mat.set_row(i, query_data[i]);
    }
    Matrix<double> query_data_wrap(query_data_mat.data_block(), (int)query_data.size(), dim);
    index_.knnSearch(query_data_wrap, indices, dists, knn, flann::SearchParams(num_search_leaf));
}

void vnl_flann::save(const char *file)
{
    assert(file);    
    index_.save(std::string(file));
}


void vnl_flann::load(const char *index_file, const vcl_vector<vnl_vector<double> > & data)
{
    assert(index_file);
    const int dim = (int)data.front().size();
    // wrap dataset
    vnl_matrix<double> data_mat((int)data.size(), (int)data.front().size());
    for (int i = 0; i<data.size(); i++) {
        assert(data[i].size() == dim);
        data_mat.set_row(i, data[i]);
    }
    
    Matrix<double> dataset(data_mat.data_block(), (int)data.size(), dim);
    
    SavedIndexParams index_params((std::string(index_file)));
    index_ =  flann::Index<flann::L2<double>>(dataset, index_params, flann::L2<double>());
    dim_ = dim;
}


