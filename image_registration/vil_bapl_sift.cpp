//
//  vil_bapl_sift.cpp
//  QuadCopter
//
//  Created by jimmy on 3/24/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_bapl_sift.h"
#include <bapl/bapl_keypoint_extractor.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_dense_sift_sptr.h>
#include <bapl/bapl_lowe_keypoint_sptr.h>
#include <vil/vil_new.h>
#include <vil/vil_convert.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <bapl/bapl_keypoint_set.h>

void VilBaplSIFT::getSiftPositions(const vil_image_view<vxl_byte> &image, vcl_vector<vgl_point_2d<double> > &points, double curve_ratio)
{
    assert(image.nplanes() == 1 || image.nplanes() == 3);
    assert(points.size() == 0);
    
    vil_image_view<vxl_byte> grey_img;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, grey_img);
    }
    else
    {
        grey_img.deep_copy(image);
    }
    
    vcl_vector<bapl_keypoint_sptr> sift_keypoints;
    vil_image_resource_sptr image_sptr = vil_new_image_resource_of_view(grey_img);
    
    //  float curve_ratio = 2.0f;
    bapl_keypoint_extractor(image_sptr, sift_keypoints, curve_ratio);
    
    
    vcl_vector<bapl_keypoint_sptr>::iterator keypoint_itr, keypoint_end;
    keypoint_end = sift_keypoints.end();
    
    
    for (keypoint_itr = sift_keypoints.begin(); keypoint_itr != keypoint_end; ++keypoint_itr)
    {
        bapl_lowe_keypoint_sptr sift_lowe_keypoint = dynamic_cast<bapl_lowe_keypoint*>((*keypoint_itr).as_pointer());
        double x = sift_lowe_keypoint->location_i();
        double y = sift_lowe_keypoint->location_j();
        
        points.push_back(vgl_point_2d<double>(x, y));
    }
}

void VilBaplSIFT::getSiftDescription(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_lowe_keypoint_sptr> &descriptions)
{
    assert(image.nplanes() == 1 || image.nplanes() == 3);
    
    vil_image_view<vxl_byte> grey_img;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, grey_img);
    }
    else
    {
        grey_img.deep_copy(image);
    }
    
    vcl_vector<bapl_keypoint_sptr> sift_keypoints;
    vil_image_resource_sptr image_sptr = vil_new_image_resource_of_view(grey_img);
    
    bapl_keypoint_extractor(image_sptr, sift_keypoints);
    
    for (vcl_vector<bapl_keypoint_sptr>::iterator itr = sift_keypoints.begin();
         itr != sift_keypoints.end(); itr++) {
        bapl_lowe_keypoint_sptr sift_lowe_keypoint = dynamic_cast<bapl_lowe_keypoint*>((*itr).as_pointer());
        assert(sift_lowe_keypoint);
        descriptions.push_back(sift_lowe_keypoint);
    }
}

void VilBaplSIFT::getSiftDescription(const vil_image_view<vxl_byte> & image, const vcl_vector<vgl_point_2d<double> > & pts, vcl_vector<bapl_lowe_keypoint_sptr> & sifts)
{
    assert(image.nplanes() == 3);
    assert(sifts.size() == 0);
    
    vil_image_view<vxl_byte> grey_img;
    vil_convert_planes_to_grey(image, grey_img);
    
    bapl_lowe_pyramid_set_sptr pyramid_set = new bapl_lowe_pyramid_set(vil_new_image_resource_of_view(grey_img));
    
    for (unsigned int i = 0; i<pts.size(); i++) {
        //    bapl_lowe_keypoint keypoint(pyramid_set, pts[i].x(), pts[i].y());
        bapl_lowe_keypoint_sptr keypoint = new bapl_lowe_keypoint(pyramid_set, pts[i].x(), pts[i].y());
        sifts.push_back(keypoint);
    }
}

void VilBaplSIFT::getSIFT(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_keypoint_sptr> & features)
{
    assert(image.nplanes() == 1 || image.nplanes() == 3);
    assert(features.size() == 0);
    
    vil_image_view<vxl_byte> grey_img;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, grey_img);
    }
    else
    {
        grey_img.deep_copy(image);
    }
    
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_img), features);
}

void VilBaplSIFT::getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints, vcl_vector<vgl_point_2d<double> > & pts)
{
    for (int i = 0; i<keypoints.size(); i++) {
        bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoints[i].as_pointer());
        vgl_point_2d<double> p(sift->location_i(), sift->location_j());
        pts.push_back(p);
    }
}

void VilBaplSIFT::getMatchingLocations(const vcl_vector<bapl_key_match> & matches, vcl_vector<vgl_point_2d<double> > & pts1, vcl_vector<vgl_point_2d<double> > & pts2)
{
    for (int i = 0; i<matches.size(); i++)
    {
        bapl_lowe_keypoint_sptr s1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr s2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(s1->location_i(), s1->location_j());
        vgl_point_2d<double> p2(s2->location_i(), s2->location_j());
        pts1.push_back(p1);
        pts2.push_back(p2);
    }
    
}
vcl_vector<vgl_point_2d<double> > VilBaplSIFT::getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints)
{
    vcl_vector<vgl_point_2d<double> > pts;
    for (int i = 0; i<keypoints.size(); i++) {
        bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoints[i].as_pointer());
        vgl_point_2d<double> p(sift->location_i(), sift->location_j());
        pts.push_back(p);
    }
    return pts;
}

vcl_vector<vgl_point_2d<double> > VilBaplSIFT::getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints, const vcl_vector<bool> & inlier)
{
    assert(inlier.size() == keypoints.size());
    
    vcl_vector<vgl_point_2d<double> > pts;
    for (int i = 0; i<keypoints.size(); i++) {
        if (inlier[i]) {
            bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoints[i].as_pointer());
            vgl_point_2d<double> p(sift->location_i(), sift->location_j());
            pts.push_back(p);
        }
        
    }
    return pts;
}

vgl_point_2d<double> VilBaplSIFT::get_sift_location(const bapl_keypoint_sptr & keypoint)
{
    bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoint.as_pointer());
    return vgl_point_2d<double>(sift->location_i(), sift->location_j());
}
void VilBaplSIFT::set_location(bapl_keypoint_sptr & keypoint, const vgl_point_2d<double> & p)
{
    bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoint.as_pointer());
    sift->set_location_i(p.x());
    sift->set_location_j(p.y());
}


void VilBaplSIFT::getSiftFromDesignatedPositions(const vil_image_view<vxl_byte> & image, vcl_vector<vnl_vector_fixed<double, 4> > & loc_sca_ori,
                                             vcl_vector<bapl_keypoint_sptr> & sifts)
{
   // assert(image.nplanes() == 3);
    assert(sifts.size() == 0);
    
    vil_image_view<vxl_byte> source_grey;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, source_grey);
    }
    else
    {
        source_grey = image;
    }
    
    bapl_lowe_pyramid_set_sptr source_pyramid_set = new bapl_lowe_pyramid_set(vil_new_image_resource_of_view(source_grey));
    
    for (int i = 0; i<loc_sca_ori.size(); i++) {
        bapl_lowe_keypoint *pKeypoint = new bapl_lowe_keypoint(source_pyramid_set, loc_sca_ori[i][0], loc_sca_ori[i][1], loc_sca_ori[i][2], loc_sca_ori[i][3]);
        sifts.push_back(bapl_keypoint_sptr(pKeypoint));
    }
    assert(loc_sca_ori.size() == sifts.size());
}

bool VilBaplSIFT::writeSIFT(const char *file, const vcl_vector< bapl_keypoint_sptr > & keypoints)
{
    FILE *pf = fopen(file, "w");
    if (!pf) {
        printf("Error: canot open %s\n", file);
        return false;
    }
    
    int n = (int)keypoints.size();
    int len = 128;
    fprintf(pf, "%d\t %d\n", n, len);
    for (int i = 0; i<keypoints.size(); i++) {
        bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoints[i].as_pointer());
        double loc_x = sift->location_i();
        double loc_y = sift->location_j();
        double scale = sift->scale();
        double orientation = sift->orientation();
        fprintf(pf, "%lf\t %lf\t %lf\t %lf\n", loc_x, loc_y, scale, orientation);
        
        vnl_vector_fixed<double,128> feat = sift->descriptor();
        for (int j = 0; j<feat.size(); j++) {
            fprintf(pf, "%lf  ", feat[j]);
        }
        fprintf(pf, "\n");
    }
    fclose(pf);
    
    printf("save sift to %s\n", file);
    return true;
}

bool VilBaplSIFT::readSIFT(const char *file, vcl_vector< bapl_keypoint_sptr > & keypoints)
{
    FILE *pf = fopen(file, "r");
    if (!pf) {
        printf("Error: canot open %s\n", file);
        return false;
    }
    int n = 0;
    int len = 0;
    int ret = fscanf(pf, "%d %d ", &n, &len);
    assert(ret == 2);
    assert(len == 128);
    
    for (int i = 0; i < n; i++) {
        double loc_x = 0;
        double loc_y = 0;
        double scale = 0;
        double orientation = 0;
        ret = fscanf(pf, "%lf %lf %lf %lf ", &loc_x, &loc_y, &scale, &orientation);
        assert(ret == 4);
        
        vnl_vector_fixed<double, 128> desc;
        for (int j = 0; j < len; j++) {
            double val = 0;
            ret = fscanf(pf, "%lf", &val);
            desc[j] = val;
        }
        bapl_lowe_pyramid_set_sptr py;
        bapl_lowe_keypoint_sptr kp = new bapl_lowe_keypoint(py, loc_x, loc_y, scale, orientation, desc);
        keypoints.push_back(kp);
    }
    fclose(pf);
    printf("read %d keypoints.\n", n);
    return true;
}

bool VilBaplSIFT::writeImageAndSIFT(const char *file, const char *imageName, const char *siftFile)
{
    FILE *pf = fopen(file, "w");
    assert(pf);
    fprintf(pf, "%s\n", imageName);
    fprintf(pf, "%s\n", siftFile);
    fclose(pf);
    return true;
}

bool VilBaplSIFT::readImageAndSIFT(const char *file, vcl_string & imageName, vcl_string & siftFile)
{
    FILE *pf = fopen(file, "r");
    if (!pf) {
        printf("Error: canot open %s\n", file);
        return false;
    }
    char buf1[1024] = {NULL};
    char buf2[1024] = {NULL};
    fscanf(pf, "%s %s", buf1, buf2);
    imageName = vcl_string(buf1);
    siftFile = vcl_string(buf2);
    fclose(pf);
    return true;    
}

