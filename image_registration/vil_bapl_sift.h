//
//  vil_bapl_sift.h
//  QuadCopter
//
//  Created by jimmy on 3/24/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_bapl_sift__
#define __QuadCopter__vil_bapl_sift__

#include <iostream>
#include <vil/vil_image_view.h>
#include <vgl/vgl_point_2d.h>
#include <vcl_vector.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <bapl/bapl_lowe_keypoint.h>
#include <bapl/bapl_bbf_tree.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_keypoint_set.h>

//wrap sift feature in bapl
class VilBaplSIFT
{
public:
    static void getSiftPositions(const vil_image_view<vxl_byte> &image, vcl_vector<vgl_point_2d<double> > &points, double curve_ratio = 10.0);
    static void getSiftDescription(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_lowe_keypoint_sptr> & descriptions);
    
    static void getSiftDescription(const vil_image_view<vxl_byte> &image, const vcl_vector<vgl_point_2d<double> > &pts, vcl_vector<bapl_lowe_keypoint_sptr> & sifts);
    
    static void getSIFT(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_keypoint_sptr> & features);
    
    
    static void getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints, vcl_vector<vgl_point_2d<double> > & pts);
    static void getSIFTDesctription(const vcl_vector<bapl_keypoint_sptr> & keypoints, vcl_vector<vnl_vector_fixed<double, 128> > & descriptions);
    
    static void getMatchingLocations(const vcl_vector<bapl_key_match> & matches, vcl_vector<vgl_point_2d<double> > & pts1, vcl_vector<vgl_point_2d<double> > & pts2);
    
    static vcl_vector<vgl_point_2d<double> > getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints);
    static vcl_vector<vgl_point_2d<double> > getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints, const vcl_vector<bool> & inlier);
    
    static vgl_point_2d<double> get_sift_location(const bapl_keypoint_sptr & keypoint);
    static void set_location(bapl_keypoint_sptr & keypoint, const vgl_point_2d<double> & p);
    
    // generate SIFT from pre-defined location, scale and orientations
    // location_i_(i), location_j_(j), scale_(s), orientation_(o)
    static void getSiftFromDesignatedPositions(const vil_image_view<vxl_byte> & image, vcl_vector<vnl_vector_fixed<double, 4>> & loc_sca_ori,
                                               vcl_vector<bapl_keypoint_sptr> & sifts);
    
    static bool writeSIFT(const char *file, const vcl_vector< bapl_keypoint_sptr > & keypoints);
    static bool readSIFT(const char *file, vcl_vector< bapl_keypoint_sptr > & keypoints);
    
    // image SIFT pair
    static bool writeImageAndSIFT(const char *file, const char *imageName, const char *siftFile);
    static bool readImageAndSIFT(const char *file, vcl_string & imageName, vcl_string & siftFile);    
    
};

#endif /* defined(__QuadCopter__vil_bapl_sift__) */
