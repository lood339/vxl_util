//
//  vil_gmm.h
//  OnlineStereo
//
//  Created by jimmy on 1/29/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vil_gmm__
#define __OnlineStereo__vil_gmm__

// gaussian mixture model in vil
#include <vil/vil_image_view.h>
#include <vpdfl/vpdfl_mixture.h>
#include <vpdfl/vpdfl_mixture_builder.h>
#include <vpdfl/vpdfl_axis_gaussian.h>
#include <vpdfl/vpdfl_axis_gaussian_builder.h>
#include <vpdfl/vpdfl_pc_gaussian.h>
#include <vpdfl/vpdfl_pc_gaussian_builder.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector.h>
#include <vgl/vgl_point_2d.h>

class VilGMM
{
    private:
    unsigned comp_n_;      // component number, is the number of gaussian
    bool verbose_;
    
    // accumulate data
    vcl_vector<vnl_vector<double> > data_;
    
    public:
    vpdfl_pdf_base* gmm_;  // gaussian mixture model
    
    public:
    VilGMM(unsigned gaussian_number, bool verbose = false);
    ~VilGMM();
    
    // train from single image
    bool train(const vil_image_view<vxl_byte> & image, const vil_image_view<vxl_byte> & maskImage, int mask);
    bool train(const vcl_vector<vnl_vector<double> > & data);
    
    void resetData(){data_.clear();}
    // incrementally add image
    bool addOneImage(const vil_image_view<vxl_byte> & image, const vil_image_view<vxl_byte> & maskImage, int mask);
    bool trainOnce();
    
    // predict from a patch area
    // average probability inside the patch
    void probabilty(const vil_image_view<vxl_byte> & image, const vil_image_view<vxl_byte> & maskImage, int mask,
                    vil_image_view<double> & prob, int oddPatchSize);
    
    double probability(const vnl_vector<double> & data);
    
    // visualize mean value of each component (gaussian)
    vil_image_view<vxl_byte> mean_image(int patch_size = 40);
    
    // postfix is ".gmm"
    bool read(const char *fileName);
    bool write(const char *fileName);
};



#endif /* defined(__OnlineStereo__vil_gmm__) */
