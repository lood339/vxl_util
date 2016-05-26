//
//  vl_sift_feature_short.cpp
//  RGB_RF
//
//  Created by jimmy on 2016-05-10.
//  Copyright © 2016 jimmy. All rights reserved.
//

#include "vl_sift_feature_short.hpp"
#include <vil/vil_convert.h>

bool VlSIFTFeatureShort::vl_keypoint_extractor(const vil_image_view<vxl_byte> & image,
                                               const vl_feat_sift_parameter & param,
                                               std::vector<std::shared_ptr<lowe_keypoint_32d> > & keypoints,
                                               bool verbose)
{
    vil_image_view<vxl_byte> grey;
    if (image.nplanes() == 1) {
        grey.deep_copy(image);
    }
    else
    {
        vil_convert_planes_to_grey(image, grey);
    }
    
    int width  = grey.ni();
    int height = grey.nj();
    int noctaves = param.noctaves; // maximum octatve possible
    int nlevels = 3;
    int o_min = 0;   //first octave index
    
    // create a filter to process the image
    VlSiftFilt *filt = vl_sift_new_short (width, height, noctaves, nlevels, o_min) ;
    
    double   edge_thresh  = param.edge_thresh;
    double   peak_thresh  = param.peak_thresh;
    double   magnif       = param.magnif ;
    double   norm_thresh  = param.norm_thresh;
    double   window_size  = param.window_size;
    
    if (peak_thresh >= 0) vl_sift_set_peak_thresh (filt, peak_thresh) ;
    if (edge_thresh >= 0) vl_sift_set_edge_thresh (filt, edge_thresh) ;
    if (norm_thresh >= 0) vl_sift_set_norm_thresh (filt, norm_thresh) ;
    if (magnif      >= 0) vl_sift_set_magnif      (filt, magnif) ;
    if (window_size >= 0) vl_sift_set_window_size (filt, window_size) ;
    
    // data from image
    vl_sift_pix *fdata = (vl_sift_pix *)malloc(width * height * sizeof (vl_sift_pix));
    for (int y = 0; y<grey.nj(); y++) {
        for (int x = 0; x<grey.ni(); x++) {
            int idx = y * width + x;
            fdata[idx] = grey(x, y, 0);
        }
    }
    
    
    //                                             Process each octave
    
    bool isFirst = true ;
    vl_bool err = VL_ERR_OK;
    
    int nangles = 0;
    double angles[4] = {0.0};
    vnl_vector_fixed<float, 32>  descriptor(0.0f);
    while (1) {
        if (isFirst) {
            isFirst = false;
            err = vl_sift_process_first_octave (filt, fdata) ;
        } else {
            err = vl_sift_process_next_octave  (filt) ;
        }
        if(err == VL_ERR_EOF)
        {
            break;
        }
        
        vl_sift_detect (filt);
        
        VlSiftKeypoint const * keys  = vl_sift_get_keypoints(filt) ;
        int nkeys = vl_sift_get_nkeypoints (filt) ;
        
        for (int i = 0; i<nkeys; i++) {
            VlSiftKeypoint const * curKey = keys + i;
            
            // Depending on the symmetry of the keypoint appearance, determining the orientation can be ambiguous. SIFT detectors have up to four possible orientations
            nangles = vl_sift_calc_keypoint_orientations(filt, angles, curKey) ;
            
            for (int q = 0 ; q < nangles ; q++) {
                vl_sift_calc_keypoint_descriptor_short(filt, &descriptor[0], curKey, angles[q]);
                
                lowe_keypoint_32d *pKeypoint = new lowe_keypoint_32d();
                double x = curKey->x;
                double y = curKey->y;
                double s = curKey->sigma;
                double o = angles[q];
                vnl_vector_fixed<double, 32> des;
                for (int j = 0; j<32; j++) {
                    des[j] = descriptor[j];
                }
                
                pKeypoint->set_location_i(x);
                pKeypoint->set_location_j(y);
                pKeypoint->set_scale(s);
                pKeypoint->set_orientation(o);
                pKeypoint->set_descriptor(des);
                
                keypoints.push_back(std::shared_ptr<lowe_keypoint_32d>(pKeypoint));
            }
        }
    }
    
    vl_sift_delete(filt);
    delete fdata;
    
    if(verbose){
        vcl_cout<<"Found "<<keypoints.size()<<" keypoints."<<vcl_endl;
    }    
    return true;
}