//
//  vil_gmm.cpp
//  OnlineStereo
//
//  Created by jimmy on 1/29/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_gmm.h"
#include <vpdfl/vpdfl_axis_gaussian.h>
#include <vpdfl/vpdfl_axis_gaussian_builder.h>
#include <vpdfl/vpdfl_gaussian.h>
#include <vpdfl/vpdfl_gaussian_builder.h>
#include <vpdfl/vpdfl_axis_gaussian.h>
#include <vpdfl/vpdfl_axis_gaussian_builder.h>
#include <mbl/mbl_data_array_wrapper.h>
#include <vsl/vsl_binary_loader.h>
#include <vpdfl/vpdfl_add_all_binary_loaders.h>
#include <vpdfl/vpdfl_calc_mean_var.h>




VilGMM::VilGMM(unsigned gaussian_number, bool verbose)
{
    gmm_ = NULL;
    comp_n_ = gaussian_number;
    verbose_ = verbose;
}

VilGMM::~VilGMM()
{
    if (gmm_) {
        delete gmm_;
        gmm_ = NULL;
    }
}

bool VilGMM::train(const vil_image_view<vxl_byte> & image, const vil_image_view<vxl_byte> & maskImage, int mask)
{
    assert(image.ni() == maskImage.ni());
    assert(image.nj() == maskImage.nj());
    assert(image.nplanes() == 3);
    
    vpdfl_gaussian_builder g_builder;
    vpdfl_mixture_builder builder;
    
    builder.init(g_builder,comp_n_);
    builder.set_weights_fixed(false);
    
    vcl_vector<vnl_vector<double> > data;
    for (int j = 0; j<image.nj(); j++) {
        for (int i = 0; i<image.ni(); i++) {
            if (maskImage(i, j) == mask) {
                vnl_vector<double> color(3);
                for (int k = 0; k<3; k++) {
                    color[k] = image(i, j, k);
                }
                data.push_back(color);
            }
        }
    }
    
    if (data.size() <= comp_n_ * 20) {
        return false;
    }
    
    if (gmm_) {
        delete gmm_;
        gmm_ = NULL;
    }
    gmm_ = builder.new_model();
    
    mbl_data_array_wrapper<vnl_vector<double> > data_array(data);
    builder.build(*gmm_, data_array);
    if (verbose_) {
        vcl_cout<<"training sample number is "<<data.size()<<vcl_endl;
        vcl_cout<<"Probability distribution function is "<<gmm_<<vcl_endl;
        vcl_cout<<"Mean: "<<gmm_->mean()<<vcl_endl;
        vcl_cout<<"Var:  "<<gmm_->variance()<<vcl_endl;
    }
    return true;
}

bool VilGMM::train(const vcl_vector<vnl_vector<double> > & data)
{
    vpdfl_gaussian_builder g_builder;
    vpdfl_mixture_builder builder;
    
    builder.init(g_builder,comp_n_);
    builder.set_weights_fixed(false);    
    
    if (data.size() <= comp_n_ * 20) {
        return false;
    }
    
    if (gmm_) {
        delete gmm_;
        gmm_ = NULL;
    }
    gmm_ = builder.new_model();
    
    mbl_data_array_wrapper<vnl_vector<double> > data_array(data);
    builder.build(*gmm_, data_array);
    if (verbose_) {
        vcl_cout<<"training sample number is "<<data.size()<<vcl_endl;
        vcl_cout<<"Probability distribution function is "<<gmm_<<vcl_endl;
        vcl_cout<<"Mean: "<<gmm_->mean()<<vcl_endl;
        vcl_cout<<"Var:  "<<gmm_->variance()<<vcl_endl;
    }
    return true;
}

bool VilGMM::addOneImage(const vil_image_view<vxl_byte> & image, const vil_image_view<vxl_byte> & maskImage, int mask)
{
    assert(image.ni() == maskImage.ni());
    assert(image.nj() == maskImage.nj());
    assert(image.nplanes() == 3);
    
    for (int j = 0; j<image.nj(); j++) {
        for (int i = 0; i<image.ni(); i++) {
            if (maskImage(i, j) == mask) {
                vnl_vector<double> color(3);
                for (int k = 0; k<3; k++) {
                    color[k] = image(i, j, k);
                }
                data_.push_back(color);
            }           
        }
    }
    return true;
}

bool VilGMM::trainOnce()
{
    vpdfl_gaussian_builder g_builder;
    vpdfl_mixture_builder builder;
    builder.init(g_builder,comp_n_);
    builder.set_weights_fixed(false);
    
    if (gmm_) {
        delete gmm_;
        gmm_ = NULL;
    }
    gmm_ = builder.new_model();
    
    mbl_data_array_wrapper<vnl_vector<double> > data_array(data_);
    
   // vnl_vector<double> mean;
  //  vnl_vector<double> var;
  //  vpdfl_calc_mean_var(mean, var, data_array);
  //  vcl_cout<<"mean is "<<mean<<vcl_endl;
  //  vcl_cout<<"var  is "<<var<<vcl_endl;
    
    builder.build(*gmm_, data_array);
    if (verbose_) {
        vcl_cout<<"training sample number is "<<data_.size()<<vcl_endl;
        vcl_cout<<"Probability distribution function is "<<gmm_<<vcl_endl;
        vcl_cout<<"Mean: "<<gmm_->mean()<<vcl_endl;
        vcl_cout<<"Var:  "<<gmm_->variance()<<vcl_endl;
    }
    
    return true;
}

void VilGMM::probabilty(const vil_image_view<vxl_byte> & image, const vil_image_view<vxl_byte> & maskImage, int mask,
                        vil_image_view<double> & outProb, int patchSize)
{
    assert(image.ni() == maskImage.ni());
    assert(image.nj() == maskImage.nj());
    assert(image.nplanes() == 3);
    assert(gmm_);
    assert(patchSize%2 == 1);
    
    vil_image_view<double> pixelProb(image.ni(), image.nj(), 1);
    pixelProb.fill(0.0);
    
    // probability in each pixel position
    int w = image.ni();
    int h = image.nj();
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            if (maskImage(i, j) == mask) {
                vnl_vector<double> color(3);
                for (int k = 0; k<3; k++) {
                    color[k] = image(i, j, k);
                }
                pixelProb(i, j) = (*gmm_)(color);              
            }
        }
    }
    
    outProb = vil_image_view<double>(w, h, 1);
    outProb.fill(0.0);
    // average probility
    for (int j = patchSize/2; j<h-patchSize/2; j++) {
        for (int i = patchSize/2; i<w-patchSize/w; i++) {
            if (maskImage(i, j) == mask) {
                // average the value inside the patch
                double val = 0;
                int num = 0;
                for (int k = -patchSize/2; k <= patchSize/2; k++) {
                    for (int m = -patchSize/2; m<= patchSize/2; m++) {
                        if (maskImage(i + m, j+ k) == mask) {
                            val += pixelProb(i+m, j+k);
                            num++;
                        }
                    }
                }
                assert(num != 0);
                val /= num;
                outProb(i, j) = val;
            }
        }
    }
}

double VilGMM::probability(const vnl_vector<double> & data)
{
    assert(gmm_);
    
    return (*gmm_)(data);
}

vil_image_view<vxl_byte> VilGMM::mean_image(int patch_size)
{
    vil_image_view<vxl_byte> image(patch_size * comp_n_, patch_size, 3);
    
    // access eam gaussian
    vpdfl_mixture * pmixture = dynamic_cast<vpdfl_mixture *>(gmm_);
    assert(pmixture);
    vcl_vector< vpdfl_pdf_base * > components = pmixture->components();
    assert(components.size() == comp_n_);
    
    vcl_vector<vnl_vector<double> > means;
    for (int i = 0; i<components.size(); i++) {
        vnl_vector<double> m = components[i]->mean();
        assert(m.size() == 3);
        means.push_back(m);
    }
    
    for (int j = 0; j<patch_size; j++) {
        for (int i = 0; i<patch_size * comp_n_; i++) {
            int idx = i/patch_size;
            int r = means[idx][0];
            int g = means[idx][1];
            int b = means[idx][2];
            image(i, j, 0) = r;
            image(i, j, 1) = g;
            image(i, j, 2) = b;
        }
    }
    return image;
}


bool VilGMM::read(const char *fileName)
{
    assert(fileName);
    
    if (gmm_) {
        delete gmm_;
        gmm_ = NULL;
    }
    
    vsl_add_to_binary_loader(vpdfl_pc_gaussian());
    vsl_add_to_binary_loader(vpdfl_axis_gaussian());
    vsl_add_to_binary_loader(vpdfl_mixture());
    vsl_add_to_binary_loader(vpdfl_gaussian());
    
    vsl_b_ifstream bfs_in(fileName);
    vsl_b_read(bfs_in, gmm_);
    bfs_in.close();
    
    if (gmm_->is_a() == vcl_string("vpdfl_mixture")) {
        vpdfl_mixture *pmix = static_cast<vpdfl_mixture *>(gmm_);
        comp_n_ = pmix->n_components();
        verbose_ = true;
    }
    else
    {
        comp_n_ = -1;
        verbose_ = true;
    }
    
    return true;
}

bool VilGMM::write(const char *fileName)
{
    assert(gmm_);
    assert(fileName);
    
    vsl_add_to_binary_loader(vpdfl_pc_gaussian());
    vsl_add_to_binary_loader(vpdfl_mixture());
    vsl_add_to_binary_loader(vpdfl_gaussian());
    
    vsl_b_ofstream bfs_out(fileName);
    vsl_b_write(bfs_out, gmm_);
    bfs_out.close();
    printf("write to %s\n", fileName);
    return true;
}

















