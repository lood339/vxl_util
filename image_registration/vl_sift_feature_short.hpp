//
//  vl_sift_feature_short.hpp
//  RGB_RF
//
//  Created by jimmy on 2016-05-10.
//  Copyright © 2016 jimmy. All rights reserved.
//

#ifndef vl_sift_feature_short_cpp
#define vl_sift_feature_short_cpp

// 32 dimension SIFT feature
#include <vnl/vnl_vector_fixed.h>
#include <vnl/vnl_vector_fixed.txx>
#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vl/sift.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include "vl_sift_feature.h"
#include <vector>

using std::vector;

template <unsigned int Dimension>
class vnl_lowe_keypoint
{
public:
    //: Constructor
    vnl_lowe_keypoint(){;}
    
    //: Destructor
    ~vnl_lowe_keypoint(){;}
    
    //: Accessor for the descriptor vector
    const vnl_vector_fixed<double,Dimension>& descriptor() const{return descriptor_;}
    
    //: Accessor for the i location
    double location_i() const {return location_i_; }
    //: Accessor for the j location
    double location_j() const {return location_j_; }
    
    vgl_point_2d<double> location(){return vgl_point_2d<double>(location_i_, location_j_);}
    
    //: Accessor for the scale
    double scale() const {return scale_; }
    //: Accessor for the orientation
    double orientation() const {return orientation_; }
    
    //: Mutator for the i location
    void set_location_i(double i) { location_i_ = i; }
    //: Mutator for the j location
    void set_location_j(double j) { location_j_ = j; }
    //: Mutator for the scale
    void set_scale(double s) { scale_ = s; }
    //: Mutator for the orientation
    void set_orientation(double o) { orientation_ = o; }
    //: Mutator for the descriptor vector
    void set_descriptor(const vnl_vector_fixed<double,Dimension>& descriptor){ descriptor_ = descriptor;}
    
private:
    //: 128-dimensional descriptor vector
    vnl_vector_fixed<double,Dimension> descriptor_;
    
    //: keypoint parameters
    double location_i_;
    double location_j_;
    double scale_;
    double orientation_;
};

typedef vnl_lowe_keypoint<32> lowe_keypoint_32d;

class VlSIFTFeatureShort
{
public:
    static bool vl_keypoint_extractor(const vil_image_view<vxl_byte> & image,
                                      const vl_feat_sift_parameter &parameter,
                                      std::vector<std::shared_ptr<lowe_keypoint_32d> > & keypoints,
                                      bool verbose = true);
};





#endif /* vl_sift_feature_short_cpp */
