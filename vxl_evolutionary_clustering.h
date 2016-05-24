//
//  vxl_evolutionary_clustering.h
//  PlayerTracking
//
//  Created by jimmy on 10/9/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __PlayerTracking__vxl_evolutionary_clustering__
#define __PlayerTracking__vxl_evolutionary_clustering__

// extend k-mean method to time sequential data

#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>

class VxlEvolutionaryCluster
{
public:
    static void k_mean_cluster(const vcl_vector<vgl_point_2d<double> > & positions, unsigned int K,
                               vcl_vector<vgl_point_2d<double> > & centers, vcl_vector<unsigned int> & partitions);
};


#endif /* defined(__PlayerTracking__vxl_evolutionary_clustering__) */
