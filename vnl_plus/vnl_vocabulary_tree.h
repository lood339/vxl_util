//
//  vnl_vocabulary_tree.h
//  QuadCopter
//
//  Created by jimmy on 7/31/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vnl_vocabulary_tree__
#define __QuadCopter__vnl_vocabulary_tree__

// vocabulary tree to classify images
#include <vnl/vnl_vector.h>
#include <vcl_vector.h>

struct vocabulary_tree_parameter
{
    int k_;          // k clusters
    int max_depth_;  // maximum depth of tree
    int nLabel_;     // label number
    int min_leaf_node_;
    
    vocabulary_tree_parameter()
    {
        k_ = 8;
        max_depth_ = 3;
        min_leaf_node_ = 40;
    }
};

class vnl_vocabulary_tree_node;
class vnl_vocabulary_tree
{
    vnl_vocabulary_tree_node *root_;
    int dim_;
    
public:
    vnl_vocabulary_tree();
    ~vnl_vocabulary_tree();
    
    void buildTree(const vcl_vector<vnl_vector<double> > & data,
                   const vcl_vector<unsigned int> & labels,
                   const vocabulary_tree_parameter & para);
    
    // query the distribution of a feature
    void query(const vnl_vector<double> & feature, vnl_vector<double> & distribution) const;
    
private:
    void buildTree(vnl_vocabulary_tree_node * node,
                   const vcl_vector<vnl_vector<double> > & data,
                   const vcl_vector<unsigned int> & labels,
                   const vocabulary_tree_parameter & para, int depth);
    
    void query(const vnl_vocabulary_tree_node * node,
               const vnl_vector<double> & feature, vnl_vector<double> & distribution) const;
    
    
    
};

class vnl_vocabulary_tree_node
{
public:
    vcl_vector<vnl_vocabulary_tree_node * > subnodes_;
    vnl_vector<double> cluster_center_;
    int depth_;
    bool isLeaf_;
    
    vnl_vector<double> histgram_;  // only leaf has
    
    vnl_vocabulary_tree_node()
    {
        depth_ = 0;
        isLeaf_ = false;
    }    
};


#endif /* defined(__QuadCopter__vnl_vocabulary_tree__) */
