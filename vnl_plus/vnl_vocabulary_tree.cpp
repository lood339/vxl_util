//
//  vnl_vocabulary_tree.cpp
//  QuadCopter
//
//  Created by jimmy on 7/31/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vnl_vocabulary_tree.h"
#include <mbl/mbl_k_means.h>
#include <mbl/mbl_data_array_wrapper.h>

vnl_vocabulary_tree::vnl_vocabulary_tree()
{
    
}

vnl_vocabulary_tree::~vnl_vocabulary_tree()
{
    
}

void vnl_vocabulary_tree::buildTree(const vcl_vector<vnl_vector<double> > & data,
                                    const vcl_vector<unsigned int> & labels,
                                    const vocabulary_tree_parameter & para)
{
    assert(para.k_ >= 2);
    assert(data.size() >= para.k_);
    
    root_ = new vnl_vocabulary_tree_node();
    root_->depth_ = 0;
    this->dim_ = data[0].size();
    this->buildTree(root_, data, labels, para, 0);
}



void vnl_vocabulary_tree::buildTree(vnl_vocabulary_tree_node * node,
                                    const vcl_vector<vnl_vector<double> > & data,
                                    const vcl_vector<unsigned int> & labels,
                                    const vocabulary_tree_parameter & para,
                                    int depth)
{
    assert(node);
    assert(data.size() == labels.size());
    
    if (depth > para.max_depth_ || data.size() <= para.min_leaf_node_) {
        node->isLeaf_ = true;
        node->histgram_ = vnl_vector<double>(para.nLabel_);
        node->histgram_.fill(0);
        for (int i = 0; i<labels.size(); i++) {
            node->histgram_[labels[i]] += 1.0;
        }
        node->histgram_.normalize();
     //   printf("leaf node size %lu\n", labels.size());
        return;
    }
    
    // run k mean
    mbl_data_array_wrapper<vnl_vector<double> > wrappted_data(data);
    vcl_vector<vnl_vector<double> > cluster_centres;
    vcl_vector<unsigned int> partition;
    unsigned int ite = mbl_k_means(wrappted_data, para.k_, &cluster_centres, &partition);
    
    const unsigned int K = (unsigned int) cluster_centres.size();
    //printf("acutal k is %d\n", K);
   
    for (int i = 0; i<K; i++) {
        vnl_vocabulary_tree_node *subNode = new vnl_vocabulary_tree_node();
        assert(subNode);
        subNode->depth_ = depth + 1;
        subNode->cluster_center_ = cluster_centres[i];
        node->subnodes_.push_back(subNode);
    }
    
    // split data
    vcl_vector<vcl_vector<vnl_vector<double> > > splitted_datas(K);
    vcl_vector<vcl_vector<unsigned int> > splitted_labels(K);
    for (int i = 0; i<partition.size(); i++) {
        int idx = partition[i];
        assert(idx < K);
        assert(idx >= 0);
        vnl_vector<double> d = data[i];
        assert(i >=0 && i<data.size());
        assert(i >= 0 && i<labels.size());
        splitted_datas[idx].push_back(data[i]);
        splitted_labels[idx].push_back(labels[i]);
    }
    
    // subdivide
    for (int i = 0; i<node->subnodes_.size() && i<splitted_datas.size() && i<splitted_labels.size(); i++) {
        this->buildTree(node->subnodes_[i], splitted_datas[i], splitted_labels[i], para, depth + 1);      
    }
}

void vnl_vocabulary_tree::query(const vnl_vector<double> & feature, vnl_vector<double> & distribution) const
{
    assert(root_);
    assert(feature.size() == dim_);
    this->query(root_, feature, distribution);
}
void vnl_vocabulary_tree::query(const vnl_vocabulary_tree_node * node,
                                const vnl_vector<double> & feature, vnl_vector<double> & distribution) const
{
    if (node->isLeaf_) {
        distribution = node->histgram_;
        return;
    }
    
    // compare with every cluster center
    double min_dis = INT_MAX;
    int min_index = 0;
    for (int i =0 ; i<node->subnodes_.size(); i++) {
        double dis = vnl_vector_ssd(node->subnodes_[i]->cluster_center_, feature);
        if (dis < min_dis) {
            min_dis = dis;
            min_index = i;
        }
    }
    assert(min_index < node->subnodes_.size());
    
    this->query(node->subnodes_[min_index], feature, distribution);
}







