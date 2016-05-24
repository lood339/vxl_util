//
//  vxl_evolutionary_clustering.cpp
//  PlayerTracking
//
//  Created by jimmy on 10/9/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_evolutionary_clustering.h"
#include <bsta/bsta_k_means.h>

void VxlEvolutionaryCluster::k_mean_cluster(const vcl_vector<vgl_point_2d<double> > & positions, unsigned int K,
                                            vcl_vector<vgl_point_2d<double> > & centers, vcl_vector<unsigned int> & partitions)
{
    assert(K <= positions.size());
    assert(K > 0);
   
    vcl_vector<vnl_vector<double> > positionVec;
    vcl_vector<vnl_vector<double> > clusterCenter;
    vnl_vector<double> pos(2, 0);
    for (int i = 0; i<positions.size(); i++) {
        pos[0] = positions[i].x();
        pos[1] = positions[i].y();
        positionVec.push_back(pos);
    }
    unsigned int n_iter = bsta_k_means(positionVec, K, &clusterCenter, &partitions);
    assert(clusterCenter.size() == K);
    
    centers.clear();
    for (int i = 0; i<clusterCenter.size(); i++) {
        vgl_point_2d<double> p(clusterCenter[i][0], clusterCenter[i][1]);
        centers.push_back(p);
    }
}