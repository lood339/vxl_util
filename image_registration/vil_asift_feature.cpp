//
//  vil_asift_feature.cpp
//  MAVGoogleImageMatching
//
//  Created by jimmy on 2015-10-30.
//  Copyright © 2015 jimmy. All rights reserved.
//

#include "vil_asift_feature.hpp"
#include <vnl/vnl_inverse.h>
#include "vil_bapl_sift.h"
#include <vnl/vnl_matlab_filewrite.h>
#include "vil_draw.hpp"
#include "vxlOpenCV.h"
#include "vxl_asift_warp.hpp"
#include "vil_util.hpp"

// check boundary and transform the location to the original image
static void boundary_check(vcl_vector<bapl_keypoint_sptr> & keypoints,
                           int w, int h, int threshold,
                           const vnl_matrix<double> & transform,
                           vcl_vector<bool> & isOnBoundary)
{
    assert(transform.rows() == 3 && transform.cols() == 3);   
    
    vnl_matrix<double> loc_mat(3, 1);
    for (int i = 0; i<keypoints.size(); i++) {
        bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoints[i].as_pointer());
        double x = sift->location_i();
        double y = sift->location_j();
        loc_mat(0, 0) = x;
        loc_mat(1, 0) = y;
        loc_mat(2, 0) = 1.0;
        
        vnl_matrix<double> loc_transformed = transform * loc_mat;
        assert(loc_transformed.rows() == 3 && loc_transformed.cols() == 1);
        x = loc_transformed(0, 0)/ loc_transformed(2, 0);
        y = loc_transformed(1, 0)/ loc_transformed(2, 0);
        
        if ( x < threshold || x > w - threshold ||
             y < threshold || y > h - threshold) {
            isOnBoundary.push_back(true);
        }
        else
        {
            isOnBoundary.push_back(false);
        }
        
        // reset location
        sift->set_location_i(x);
        sift->set_location_j(y);
    }
    assert(isOnBoundary.size() == keypoints.size());
}

bool vil_asift_feature::asift_keypoint_extractor(const vil_image_view<vxl_byte> & original_image,
                                                 const vcl_vector<vil_image_view<vxl_byte> > & affine_warped_images,
                                                 const vcl_vector<vnl_matrix_fixed<double, 2, 3> > & affines,
                                                 const vl_feat_sift_parameter & parameter,
                                                 vcl_vector< vcl_vector<bapl_keypoint_sptr> > & keypoints,
                                                 bool verbose)
{
    assert(affine_warped_images.size() == affines.size());
    
    int w = original_image.ni();
    int h = original_image.nj();
    int image_boundary_threshold = 5.0;
    
    // change affines to homography, then inverse homography
    vcl_vector<vnl_matrix<double> > inv_affines;
    for (int i = 0; i<affines.size(); i++) {
        vnl_matrix<double> H(3, 3);
        H.set_identity();
        H.update(affines[i].as_matrix(), 0, 0);
        
        vnl_matrix<double> invH = vnl_inverse(H);
        inv_affines.push_back(invH);
    }
    
    // sift feature from the original image
    vcl_vector<bapl_keypoint_sptr> cur_keypoints;
    VlSIFTFeature::vl_keypoint_extractor(original_image, parameter, cur_keypoints, verbose);
    keypoints.push_back(cur_keypoints);
    
    // from warped image
    for (int i = 0; i<affine_warped_images.size(); i++) {
        vil_image_view<vxl_byte> cur_image = affine_warped_images[i];
        vcl_vector<bapl_keypoint_sptr> cur_keypoints;
        VlSIFTFeature::vl_keypoint_extractor(cur_image, parameter, cur_keypoints, verbose);
        
     //   vcl_vector<vgl_point_2d<double> > pts_warped;
     //   vcl_vector<vgl_point_2d<double> > pts_original;
     //   VilBaplSIFT::getSIFTLocations(cur_keypoints, pts_warped);
        
        // remove boundary pixels
        vcl_vector<bool> is_on_boundary;
        boundary_check(cur_keypoints, w, h, image_boundary_threshold, inv_affines[i], is_on_boundary);
        
     //   VilBaplSIFT::getSIFTLocations(cur_keypoints, pts_original);
        
        /*
        {
            vil_image_view<vxl_byte> matched_image;
            VilDraw::draw_match_vertical(cur_image, original_image, pts_warped, pts_original, matched_image);
            VxlOpenCVImage::imshow(matched_image, "asift_matched");
            
        }
         */
        vcl_vector<bapl_keypoint_sptr> inside_image_keypoints;
        
        for (int j = 0; j<cur_keypoints.size(); j++) {
            if (!is_on_boundary[j]) {
                inside_image_keypoints.push_back(cur_keypoints[j]);
            }
        }
        keypoints.push_back(inside_image_keypoints);
    }
    
    if (verbose) {       
        /*
        vnl_matrix<double> heatmap(h, w, 0);
        for (int i = 0; i<keypoints.size(); i++) {
            vgl_point_2d<double> p = VilBaplSIFT::get_sift_location(keypoints[i]);
            heatmap(p.y(), p.x()) += 1.0;
        }
        
        vnl_matlab_filewrite writer("asif_heat_map.mat");
        writer.write(heatmap, "asift_heat_map");
        printf("save to asif_heat_map.mat\n");
         */
    }
    
    return true;
}


bool vil_asift_feature::asift_keypoint_extractor(const vil_image_view<vxl_byte> & original_image,
                                                 const vcl_vector<vil_image_view<vxl_byte> > & affine_warped_images,
                                                 const vcl_vector<vnl_matrix_fixed<double, 2, 3> > & affines,
                                                 const vl_feat_sift_parameter & parameter,
                                                 vcl_vector<bapl_keypoint_sptr> & keypoints,
                                                 bool verbose)
{
    vcl_vector< vcl_vector<bapl_keypoint_sptr> > each_keypoints;
    vil_asift_feature::asift_keypoint_extractor(original_image, affine_warped_images, affines, parameter, each_keypoints, verbose);
    
    for (int i = 0; i<each_keypoints.size(); i++) {
        keypoints.insert(keypoints.end(), each_keypoints[i].begin(), each_keypoints[i].end());
    }
    if (verbose) {
        printf("ASIFT find %lu key points\n", keypoints.size());
    }
    return true;
}


bool vil_asift_feature::asift_keypoints_extractor(const vil_image_view<vxl_byte> & image,
                                                  const vcl_vector<double> & rotations,
                                                  const vcl_vector<double> & tilts,
                                                  const vl_feat_sift_parameter & param,
                                                  vcl_vector<bapl_keypoint_sptr> & keypoints,
                                                  bool verbose)
{
    vcl_vector<vnl_matrix_fixed<double, 2, 3> > affines;
    vcl_vector<vil_image_view<vxl_byte> > affine_warped_images;
    for (int i = 0; i<rotations.size(); i++) {
        for (int j = 0; j<tilts.size(); j++) {
            double rot = rotations[i];
            double tilt = tilts[j];
            vil_image_view<vxl_byte> warp_image;
            vnl_matrix_fixed<double, 2, 3> affine = vxl_asift_warp::warp_image(image, rot, tilt, warp_image);
            
            affine_warped_images.push_back(warp_image);
            affines.push_back(affine);
        }
    }
    vil_asift_feature::asift_keypoint_extractor(image, affine_warped_images, affines, param, keypoints);
    printf("find %lu ASIFT key points\n", keypoints.size());
    return true;
}


bool vil_asift_feature::asift_keypoints_extractor(const vil_image_view<vxl_byte> & image,
                                                  const vcl_vector<double> & rotations,
                                                  const vcl_vector<double> & tilts,
                                                  const vl_feat_sift_parameter & param,
                                                  vcl_vector< vcl_vector<bapl_keypoint_sptr> > & keypoints,
                                                  bool verbose)
{
    vcl_vector<vnl_matrix_fixed<double, 2, 3> > affines;
    vcl_vector<vil_image_view<vxl_byte> > affine_warped_images;
    for (int i = 0; i<rotations.size(); i++) {
        for (int j = 0; j<tilts.size(); j++) {
            double rot = rotations[i];
            double tilt = tilts[j];
            vil_image_view<vxl_byte> warp_image;
            vnl_matrix_fixed<double, 2, 3> affine = vxl_asift_warp::warp_image(image, rot, tilt, warp_image);
            
            affine_warped_images.push_back(warp_image);
            affines.push_back(affine);
        }
    }
    
    // save affined images for test purpose
    /*
    for (int i = 0; i<affine_warped_images.size(); i++) {
        char buf[1024] = {NULL};
        sprintf(buf, "affine_%d.jpg", i);
        VilUtil::vil_save(affine_warped_images[i], buf);
    }
     */
    vil_asift_feature::asift_keypoint_extractor(image, affine_warped_images, affines, param, keypoints);
    printf("find %lu groups of ASIFT key points\n", keypoints.size());
    return true;
}

