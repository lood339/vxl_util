//
//  vil_sift_feature.cpp
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vil_sift_feature.h"
#include <bapl/bapl_keypoint_extractor.h>
#include <bapl/bapl_keypoint_sptr.h>
#include <bapl/bapl_dense_sift_sptr.h>
#include <bapl/bapl_lowe_keypoint_sptr.h>
#include <vil/vil_new.h>
#include <vil/vil_convert.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <bapl/bapl_keypoint_set.h>
#include "vxl_plus.h"
#include "basketballCourt.h"
#include "vxl_vrel_plus.h"
#include "vpgl_plus.h"
#include "vil_plus.h"



void VilSIFT::getSiftPositions(const vil_image_view<vxl_byte> &image, vcl_vector<vgl_point_2d<double> > &points, double curve_ratio)
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

void VilSIFT::getSiftDescription(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_lowe_keypoint_sptr> &descriptions)
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

void VilSIFT::getSiftDescription(const vil_image_view<vxl_byte> & image, const vcl_vector<vgl_point_2d<double> > & pts, vcl_vector<bapl_lowe_keypoint_sptr> & sifts)
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

void VilSIFT::getSIFT(const vil_image_view<vxl_byte> &image, vcl_vector<bapl_keypoint_sptr> & features)
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

void VilSIFT::getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints, vcl_vector<vgl_point_2d<double> > & pts)
{
    for (int i = 0; i<keypoints.size(); i++) {
        bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoints[i].as_pointer());
        vgl_point_2d<double> p(sift->location_i(), sift->location_j());
        pts.push_back(p);
    }
}

void VilSIFT::getMatchLocations(const vcl_vector<bapl_key_match> & matches, vcl_vector<vgl_point_2d<double> > & pts1, vcl_vector<vgl_point_2d<double> > & pts2)
{
    vcl_vector<bapl_keypoint_sptr> sift1;
    vcl_vector<bapl_keypoint_sptr> sift2;
    for (int i = 0; i<matches.size(); i++) {
        sift1.push_back(matches[i].first);
        sift2.push_back(matches[i].second);
    }
    VilSIFT::getSIFTLocations(sift1, pts1);
    VilSIFT::getSIFTLocations(sift2, pts2);
}

vcl_vector<vgl_point_2d<double> > VilSIFT::getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints)
{
    vcl_vector<vgl_point_2d<double> > pts;
    for (int i = 0; i<keypoints.size(); i++) {
        bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoints[i].as_pointer());
        vgl_point_2d<double> p(sift->location_i(), sift->location_j());
        pts.push_back(p);
    }
    return pts;
}

vcl_vector<vgl_point_2d<double> > VilSIFT::getSIFTLocations(const vcl_vector<bapl_keypoint_sptr> & keypoints, const vcl_vector<bool> & inlier)
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

vgl_point_2d<double> VilSIFT::sift_location(const bapl_keypoint_sptr & keypoint)
{
    bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoint.as_pointer());
    return vgl_point_2d<double>(sift->location_i(), sift->location_j());
}
void VilSIFT::set_location(bapl_keypoint_sptr & keypoint, const vgl_point_2d<double> & p)
{
    bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keypoint.as_pointer());
    sift->set_location_i(p.x());
    sift->set_location_j(p.y());
}

void VilSIFT::getSiftFromWarpedTopview(const vil_image_view<vxl_byte> &topviewImage, int destWidth, int destHeight,
                                       const vpgl_perspective_camera<double> &keyframeCamera, vcl_vector<bapl_keypoint_sptr> & keyframeSift)
{
    assert(topviewImage.nplanes() == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    assert(keyframeSift.size() == 0);
    
    vil_image_view<vxl_byte> warpedTopview;
    vil_image_view<vxl_byte> alphaMap;    //court area
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    vgl_point_2d<int> starP(30, 30);
    vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
    court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap, starP, endP);
    
    // get features from warpted topview
    vil_image_view<vxl_byte> grey_warpedTopview;
    vil_convert_planes_to_grey(warpedTopview, grey_warpedTopview);
    vcl_vector<bapl_keypoint_sptr> keyframeSiftAll;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_warpedTopview), keyframeSiftAll);
    
    // filter out keypoints near boundary area
    for (int i = 0; i<keyframeSiftAll.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(keyframeSiftAll[i].as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        
        if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
            keyframeSift.push_back(keyframeSiftAll[i]);
        }
    }
    keyframeSiftAll.clear();
}

void VilSIFT::getSiftFromDesignatedPositions(const vil_image_view<vxl_byte> & image, vcl_vector<vnl_vector_fixed<double, 4>> & loc_sca_ori,
                                             vcl_vector<bapl_keypoint_sptr> & sifts)
{
    assert(image.nplanes() == 3);
    assert(sifts.size() == 0);
    
    vil_image_view<vxl_byte> source_grey;
    vil_convert_planes_to_grey(image, source_grey);
    bapl_lowe_pyramid_set_sptr source_pyramid_set = new bapl_lowe_pyramid_set(vil_new_image_resource_of_view(source_grey));
 
    for (int i = 0; i<loc_sca_ori.size(); i++) {
        bapl_lowe_keypoint *pKeypoint = new bapl_lowe_keypoint(source_pyramid_set, loc_sca_ori[i][0], loc_sca_ori[i][1], loc_sca_ori[i][2], loc_sca_ori[i][3]);
        sifts.push_back(bapl_keypoint_sptr(pKeypoint));
    }
    assert(loc_sca_ori.size() == sifts.size());
}

void VilSIFT::getSiftFromWarpedTopview(const vil_image_view<vxl_byte> &topviewImage, int destWidth, int destHeight,
                                       const vpgl_perspective_camera<double> &keyframeCamera, vcl_vector<bapl_keypoint_sptr> & keyframeSift,
                                       vil_image_view<vxl_byte> & warpedTopview, double curve_ratio)
{
    assert(topviewImage.nplanes() == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    assert(keyframeSift.size() == 0);
   
    vil_image_view<vxl_byte> alphaMap;    //court area
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    vgl_point_2d<int> starP(30, 30);
    vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
    court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap, starP, endP);
    
    // get features from warpted topview
    vil_image_view<vxl_byte> grey_warpedTopview;
    vil_convert_planes_to_grey(warpedTopview, grey_warpedTopview);
    vcl_vector<bapl_keypoint_sptr> keyframeSiftAll;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_warpedTopview), keyframeSiftAll, curve_ratio);
    
    // filter out keypoints near boundary area
    for (int i = 0; i<keyframeSiftAll.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(keyframeSiftAll[i].as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        
        if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
            keyframeSift.push_back(keyframeSiftAll[i]);
        }
    }
    keyframeSiftAll.clear();
}

void VilSIFT::geSIFTFromWarpedTopview(const vil_image_view<vxl_byte> &topviewImage, const vil_image_view<vxl_byte> &keyImage,
                                      const vpgl_perspective_camera<double> &keyframeCamera,
                                      vcl_vector<bapl_keypoint_sptr> & warpedTopviewSift,
                                      vcl_vector<bapl_keypoint_sptr> & keyImageSift,
                                      vil_image_view<vxl_byte> & warpedTopview,
                                      double curve_ratio, int patch_size, double ncc_threshold)
{
    assert(topviewImage.nplanes() == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    assert(warpedTopviewSift.size() == 0);
    assert(keyImageSift.size() == 0);
    
    int destWidth  = keyImage.ni();
    int destHeight = keyImage.nj();
    vil_image_view<vxl_byte> alphaMap;    //court area
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    vgl_point_2d<int> starP(30, 30);
    vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
    court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap, starP, endP);
    
    // get features from warpted topview
    vil_image_view<vxl_byte> grey_warpedTopview;
    vil_convert_planes_to_grey(warpedTopview, grey_warpedTopview);
    vcl_vector<bapl_keypoint_sptr> warpedKeyframeSiftAll;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_warpedTopview), warpedKeyframeSiftAll, curve_ratio);
    
    // filter out keypoints near boundary area
    for (int i = 0; i<warpedKeyframeSiftAll.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(warpedKeyframeSiftAll[i].as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        
        if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
            warpedTopviewSift.push_back(warpedKeyframeSiftAll[i]);
        }
    }
    warpedKeyframeSiftAll.clear();
    
    // get feature from keyframes
    vil_image_view<vxl_byte> grey_key_image;
    vil_convert_planes_to_grey(keyImage, grey_key_image);
    vcl_vector<bapl_keypoint_sptr> keyImageSiftAll;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_key_image), keyImageSiftAll, curve_ratio);
    
    // filter sift on foreground objects
    vcl_vector<vgl_point_2d<double> > candidatePositions;
    vcl_vector<int> candidateIndex;
    vcl_vector<double> nccs;
    for (int i = 0; i<keyImageSiftAll.size(); i++) {
        bapl_lowe_keypoint_sptr keypoint = dynamic_cast<bapl_lowe_keypoint *>(keyImageSiftAll[i].as_pointer());
        vgl_point_2d<double> p(keypoint->location_i(), keypoint->location_j());
        
        if (alphaMap((int)p.x(), (int)p.y()) == 255) {
            candidatePositions.push_back(p);
            candidateIndex.push_back(i);
        }
    }
 
    VilPlus::vil_cross_correlation(warpedTopview, keyImage, candidatePositions, candidatePositions, patch_size, nccs);
    for (int i = 0; i<candidateIndex.size(); i++) {
        if (fabs(nccs[i]) >= ncc_threshold) {
            keyImageSift.push_back(keyImageSiftAll[candidateIndex[i]]);
        }
    }
    
    printf("find warpedTopview, orignal image %d %d keypoints\n", (int)warpedTopviewSift.size(), (int)keyImageSift.size());
}

void VilSIFT::getSiftFromBluredKeyframe(const vil_image_view<vxl_byte> &topviewImage, const vil_image_view<vxl_byte> &bluredImage,
                                        const vpgl_perspective_camera<double> & keyframeCamera, vcl_vector<bapl_keypoint_sptr> & keyframeSift,
                                        const int window_size, const double & threshold)
{
    assert(topviewImage.nplanes() == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    assert(keyframeSift.size() == 0);
    assert(bluredImage.nplanes() == 3);
    
    //get features from the image
    vil_image_view<vxl_byte> grey_img;
    vil_convert_planes_to_grey(bluredImage, grey_img);
    vcl_vector<bapl_keypoint_sptr> keyframeSiftAll;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_img), keyframeSiftAll);
    
    const int destWidth  = bluredImage.ni();
    const int destHeight = bluredImage.nj();
    vil_image_view<vxl_byte> alphaMap;
    vil_image_view<vxl_byte> warpedTopview;
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    // get sift inside court
    vgl_point_2d<int> starP(30, 30);
    vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
    court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap);
    
    vcl_vector<bapl_keypoint_sptr> insideCourtSift;
    for (int i = 0; i<keyframeSiftAll.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(keyframeSiftAll[i].as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        
        if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
            insideCourtSift.push_back(keyframeSiftAll[i]);
        }
    }
    keyframeSiftAll.clear();
    
    // filter sift not on the player's body
    // SSD value of each points
    vcl_vector<double> ssds;   // for test
    const int borderSize = window_size/2;
    for (int i = 0; i<insideCourtSift.size(); i++) {
        bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(insideCourtSift[i].as_pointer());
        vgl_point_2d<double> p(sift->location_i(), sift->location_j());
       
        double ssd = 0;
        for (int y = p.y() - borderSize; y <= p.y() + borderSize; y++) {
            for (int x = p.x() - borderSize; x <= p.x() + borderSize; x++) {
                if (x >= 0 && x < destWidth && y >= 0 && y < destHeight) {
                    double dif_r = (bluredImage(x, y, 0) - warpedTopview(x, y, 0))/255.0;
                    double dif_g = (bluredImage(x, y, 1) - warpedTopview(x, y, 1))/255.0;
                    double dif_b = (bluredImage(x, y, 2) - warpedTopview(x, y, 2))/255.0;
                    ssd += dif_r * dif_r;
                    ssd += dif_g * dif_g;
                    ssd += dif_b * dif_b;
                }
            }
        }
        ssd /= window_size * window_size;
        ssds.push_back(ssd);
           
        if (ssd < threshold)
        {
            keyframeSift.push_back(insideCourtSift[i]);
        }
    }
    insideCourtSift.clear();
    
    //for test
    if(0)
    {
        vnl_vector<double> ssds_vec(&ssds[0], (unsigned int)ssds.size());
        vcl_string file = vcl_string("ssds.mat");
        vnl_matlab_filewrite writer(file.c_str());
        writer.write(ssds_vec, "ssds");
        vcl_cout<<"save to: "<<file<<vcl_endl;
        vcl_vector<vxl_byte> color;
        color.push_back(0);
        color.push_back(255);
        color.push_back(0);
        
        vcl_vector<vgl_point_2d<double> > pts;
        for (int i = 0; i<keyframeSift.size(); i++) {
            bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint *>(keyframeSift[i].as_pointer()); //keyframeSift[i];
            pts.push_back(vgl_point_2d<double>(sift->location_i(), sift->location_j()));
        }
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(bluredImage);
        VilPlus::draw_cross(showImage, pts, 5, color);
        VilPlus::vil_save(showImage, "blured_image_sift.jpg");
    }
    
}

void VilSIFT::getSiftMatching(const vil_image_view<vxl_byte> & firstImage, const vil_image_view<vxl_byte> & secondImage,
                              vcl_vector<vgl_point_2d<double> > & pts1, vcl_vector<vgl_point_2d<double> > & pts2, double ratio)
{
    assert(firstImage.nplanes() == 3);
    assert(secondImage.nplanes() == 3);
    
    
    vil_image_view<vxl_byte> grey_image1;
    vil_convert_planes_to_grey(firstImage, grey_image1);
    vcl_vector<bapl_keypoint_sptr> sifts1;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_image1), sifts1);
    
    vil_image_view<vxl_byte> grey_image2;
    vil_convert_planes_to_grey(secondImage, grey_image2);
    vcl_vector<bapl_keypoint_sptr> sifts2;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_image2), sifts2);
    
    bapl_bbf_tree tree(sifts2, 4);
    
    vcl_vector<bapl_key_match> matches;
    for (unsigned int i = 0; i<sifts1.size(); i++) {
        bapl_keypoint_sptr query = sifts1[i];
        vcl_vector<bapl_keypoint_sptr> match;
        tree.n_nearest(query, match, 2, -1);
        if (vnl_vector_ssd(query->descriptor(), match[0]->descriptor() ) <
            vnl_vector_ssd(query->descriptor(), match[1]->descriptor() ) * ratio) {
            bapl_key_match k_p(query, match[0]);
            matches.push_back(k_p);
        }
    }
    
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint *>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint *>(matches[i].second.as_pointer());
        
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        pts1.push_back(p1);
        pts2.push_back(p2);
    }
}

void VilSIFT::matchSIFTfromProjectedTopview(const vil_image_view<vxl_byte> &topviewImage, const vpgl_perspective_camera<double> & keyFrameCamera,
                                            bapl_bbf_tree & tree, double ratio, int destWidth, int destHeight,
                                            vcl_vector<vgl_point_2d<double> > & features1, vcl_vector<vgl_point_2d<double> > & features2)
{
    assert(ratio < 1.0);
    assert(features1.size() == 0);
    assert(features2.size() == 0);
    assert(topviewImage.nplanes() == 3);
    
    vil_image_view<vxl_byte> projectedTopview;
    
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyFrameCamera, destWidth, destHeight, projectedTopview, 20);
    
    // get features from warpted topview
    vil_image_view<vxl_byte> grey_projectedtopview;
    vil_convert_planes_to_grey(projectedTopview, grey_projectedtopview);
    vcl_vector<bapl_keypoint_sptr> sifts;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_projectedtopview), sifts);
    
    //draw features in warped view image
    {
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(projectedTopview);
        
        vcl_vector<vgl_point_2d<double> > pts;
        for (int i = 0; i<sifts.size(); i++) {
            bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(sifts[i].as_pointer());
            vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
            pts.push_back(p1);
        }
        vcl_vector<vxl_byte> colour;
        colour.push_back(0);
        colour.push_back(255);
        colour.push_back(0);
        
        VilPlus::draw_cross(showImage, pts, 4, colour);
        
        VilPlus::vil_save(showImage, "initial_court_marking.jpg");        
    }
    
    vil_image_view<vxl_byte> alphaImage;
    vgl_point_2d<int> starP(30, 30);
    vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
   
    court.getProjectedCourtArea(keyFrameCamera, destWidth, destHeight, alphaImage, starP, endP);
    
    //kd tree match
    vcl_vector<bapl_key_match> matches;
    for (unsigned int i = 0; i<sifts.size(); i++) {
        bapl_keypoint_sptr query = sifts[i];
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(query.as_pointer());
        int px = sift1->location_i();
        int py = sift1->location_j();
        
        if (alphaImage(px, py) == 0)  //in the boundary area of court image
        {
            continue;
        }

        vcl_vector<bapl_keypoint_sptr> match;
        tree.n_nearest(query, match, 2, -1);
        if (vnl_vector_ssd(query->descriptor(), match[0]->descriptor() ) <
            vnl_vector_ssd(query->descriptor(), match[1]->descriptor() ) * ratio) {
            bapl_key_match k_p(query, match[0]);
            matches.push_back(k_p);
        }
    }
    
    vcl_vector<vgl_point_2d<double> > pts1;
    vcl_vector<vgl_point_2d<double> > pts2;
    
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts1.push_back(p1);
        pts2.push_back(p2);
    }
    assert(pts1.size() == pts2.size());
    vcl_cout<<"initial matched number is ---------------------------------- "<<pts1.size()<<vcl_endl;
    
    //with ransac
    vcl_vector<bool> inlier;
    VrelPlus::homography_RANSAC(pts1, pts2, inlier, 2.0);
    assert(inlier.size() == pts1.size());
    
    for (int i = 0; i<inlier.size(); i++) {
        if (inlier[i]) {
            features1.push_back(pts1[i]);
            features2.push_back(pts2[i]);
        }
    }
    assert(features1.size() == features2.size());
    vcl_cout<<"RANSAC matched number is ---------------------------------- "<<features1.size()<<vcl_endl;
}

void VilSIFT::matchSIFTfromKdtree(const vil_image_view<vxl_byte> &image, bapl_bbf_tree & tree, double ratio,
                                  vcl_vector<vgl_point_2d<double> > & features1, vcl_vector<vgl_point_2d<double> > & features2)
{
    assert(ratio < 1.0);
    assert(features1.size() == 0);
    assert(features2.size() == 0);
    assert(image.nplanes() == 3);
    
    // get features from projected topview
    vil_image_view<vxl_byte> grey_image;
    vil_convert_planes_to_grey(image, grey_image);
    vcl_vector<bapl_keypoint_sptr> sifts;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_image), sifts);
    
    //kd tree match
    vcl_vector<bapl_key_match> matches;
    for (unsigned int i = 0; i<sifts.size(); i++) {
        bapl_keypoint_sptr query = sifts[i];
        vcl_vector<bapl_keypoint_sptr> match;
        tree.n_nearest(query, match, 2, -1);
        if (vnl_vector_ssd(query->descriptor(), match[0]->descriptor() ) <
            vnl_vector_ssd(query->descriptor(), match[1]->descriptor() ) * ratio) {
            bapl_key_match k_p(query, match[0]);
            matches.push_back(k_p);
        }
    }
    
    vcl_vector<vgl_point_2d<double> > pts1;
    vcl_vector<vgl_point_2d<double> > pts2;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts1.push_back(p1);
        pts2.push_back(p2);
    }
    assert(pts1.size() == pts2.size());
    vcl_cout<<"initial matched number is ---------------------------------- "<<pts1.size()<<vcl_endl;
    
    //with ransac
    vcl_vector<bool> inlier;
    VrelPlus::homography_RANSAC(pts1, pts2, inlier, 2.0);
    assert(inlier.size() == pts1.size());
    
    for (int i = 0; i<inlier.size(); i++) {
        if (inlier[i]) {
            features1.push_back(pts1[i]);
            features2.push_back(pts2[i]);
        }
    }
    assert(features1.size() == features2.size());
    vcl_cout<<"RANSAC matched number is ---------------------------------- "<<features1.size()<<vcl_endl;
}

vgl_h_matrix_2d<double> VilSIFT::adativeMatchSIFT(const vcl_vector<bapl_keypoint_sptr> & keyframeFeatures,
                               const vcl_vector<bapl_keypoint_sptr> & queryImageFeatures,
                               vcl_vector<vgl_point_2d<double> > & keyframe_pts,
                               vcl_vector<vgl_point_2d<double> > & query_image_pts)
{
    assert(keyframe_pts.size() == 0);
    assert(query_image_pts.size() == 0);
    assert(keyframeFeatures.size() > 10);
    assert(queryImageFeatures.size() > 10);
    
    bapl_bbf_tree tree(queryImageFeatures, 16);
    
    // use two threshold 0.6 and 0.7 too avoid too few matching situation
    // if 0.6 get enough matches, 0.7 and later will be discarded
    vcl_vector<double> ratios;
    ratios.push_back(0.6);
    ratios.push_back(0.7);
    
    vcl_vector< vcl_vector<bapl_key_match> > matchesVec(2);
    for (unsigned int i = 0; i<keyframeFeatures.size(); i++) {
        bapl_keypoint_sptr      query = keyframeFeatures[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        for (int j = 0; j<ratios.size(); j++) {
            if (ssd0 < ssd1 * ratios[j]) {
                matchesVec[j].push_back(k_p);
            }
        }
    }
    
    // decide which threshold 0.6 or 0.7
    vcl_vector<bapl_key_match> matches;
    if (matchesVec[0].size() >= 100) {
        matches = matchesVec[0];
    }
    else
    {
        matches = matchesVec[1];
    }
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_image;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts_keyframe.push_back(p1);
        pts_image.push_back(p2);
    }
    assert(pts_keyframe.size() == pts_image.size());
    vcl_cout<<"initial matched number is     "<<pts_keyframe.size()<<vcl_endl;
    
    vcl_vector<bool> inlier;
    vgl_h_matrix_2d<double> H_keyframeToQuery = VrelPlus::homography_RANSAC(pts_keyframe, pts_image, inlier, 2.0);    //Result vary from time to time
    assert(inlier.size() == pts_keyframe.size());
    
    for (int i = 0; i<inlier.size(); i++) {
        if (inlier[i]) {
            keyframe_pts.push_back(pts_keyframe[i]);
            query_image_pts.push_back(pts_image[i]);
        }
    }
    assert(keyframe_pts.size() == query_image_pts.size());
    vcl_cout<<"matched number after RANSAC is "<<keyframe_pts.size()<<vcl_endl;
    
    return H_keyframeToQuery;
}

void VilSIFT::inlierByAdativeMatch(const vcl_vector<bapl_keypoint_sptr> & keyframeFeatures,
                                   const vcl_vector<bapl_keypoint_sptr> & queryImageFeatures,
                                   vcl_vector<bool> & keyFrameFeatureInlier)
{
    assert(keyframeFeatures.size() > 10);
    assert(queryImageFeatures.size() > 10);
    
    bapl_bbf_tree tree(queryImageFeatures, 16);
    
    // use two threshold 0.6 and 0.7 too avoid too few matching situation
    // if 0.6 get enough matches, 0.7 and later will be discarded
    vcl_vector<double> ratios;
    ratios.push_back(0.6);
    ratios.push_back(0.7);
    
    vcl_vector< vcl_vector<bapl_key_match> > matchesVec(2);
    vcl_vector< vcl_vector<int> > keyframeOriginalIndexVec(2);
    for (unsigned int i = 0; i<keyframeFeatures.size(); i++) {
        bapl_keypoint_sptr      query = keyframeFeatures[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        for (int j = 0; j<ratios.size(); j++) {
            if (ssd0 < ssd1 * ratios[j]) {
                matchesVec[j].push_back(k_p);
                keyframeOriginalIndexVec[j].push_back(i);
            }
        }
    }
    
    // decide which threshold 0.6 or 0.7
    vcl_vector<bapl_key_match> matches;
    vcl_vector<int> keyframeOriginalIndex;
    if (matchesVec[0].size() >= 100) {
        matches = matchesVec[0];
        keyframeOriginalIndex = keyframeOriginalIndexVec[0];
    }
    else
    {
        matches = matchesVec[1];
        keyframeOriginalIndex = keyframeOriginalIndexVec[1];
    }
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_image;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts_keyframe.push_back(p1);
        pts_image.push_back(p2);
    }
    assert(pts_keyframe.size() == pts_image.size());
    vcl_cout<<"initial matched number is     "<<pts_keyframe.size()<<vcl_endl;
    
    // estimate multiple times
    vcl_vector<int> count_inlier(keyframeOriginalIndex.size(), 0);
    for (int k = 0; k<20; k++) {
        vcl_vector<bool> cur_inlier;
        vgl_h_matrix_2d<double> H_keyframeToQuery = VrelPlus::homography_RANSAC(pts_keyframe, pts_image, cur_inlier, 2.0);    //Result vary from time to time
        assert(cur_inlier.size() == pts_keyframe.size());
        
        for (int i = 0; i<cur_inlier.size(); i++) {
            if (cur_inlier[i]) {
                count_inlier[i]++;
            }
        }
    }
    
    keyFrameFeatureInlier = vcl_vector<bool>(keyframeFeatures.size(), false);
    
    // at least 50% of time is inlier
    int inlier_num = 0;
    for (int i = 0; i<count_inlier.size(); i++) {
        if (count_inlier[i] >= 10) {
            keyFrameFeatureInlier[keyframeOriginalIndex[i]] = true;
            inlier_num++;
        }
    }
    printf("total feature number is %d, inlier is %d\n", (int)keyframeFeatures.size(), inlier_num);
    
}

bool VilSIFT::calibFromKeyframe(const vil_image_view<vxl_byte> &topviewImage, const vpgl_perspective_camera<double> &keyframeCamera,
                                const vil_image_view<vxl_byte> &queryImage, vpgl_perspective_camera<double> &queryCamera,
                                int featureMatchingNumThreshold, double projectionErrorThreshold)
{
    assert(topviewImage.nplanes() == 3);
    assert(queryImage.nplanes()  == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    
    int destWidth  = queryImage.ni();
    int destHeight = queryImage.nj();
    
    // feature in warpted topview
    vil_image_view<vxl_byte> warpedTopview;
    vil_image_view<vxl_byte> alphaMap;    //court area
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    vgl_point_2d<int> starP(30, 30);
    vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
    court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap, starP, endP);
    
    // get features from warpted topview
    vil_image_view<vxl_byte> grey_warpedTopview;
    vil_convert_planes_to_grey(warpedTopview, grey_warpedTopview);
    vcl_vector<bapl_keypoint_sptr> keyframeSiftAll;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_warpedTopview), keyframeSiftAll);
    
    // filter out keypoints near boundary area
    vcl_vector<bapl_keypoint_sptr> keyframeSift;
    for (int i = 0; i<keyframeSiftAll.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(keyframeSiftAll[i].as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        
        if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
            keyframeSift.push_back(keyframeSiftAll[i]);
        }
    }
    keyframeSiftAll.clear();
    
    {
        // save keyframe sift
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(warpedTopview);
        
        vcl_vector<vgl_point_2d<double> > pts;
        for (int i = 0; i<keyframeSift.size(); i++) {
            bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keyframeSift[i].as_pointer());
            vgl_point_2d<double> p(sift->location_i(), sift->location_j());
            pts.push_back(p);
        }
        
        vcl_vector<vxl_byte> colour;
        colour.push_back(0);
        colour.push_back(255);
        colour.push_back(0);
        VilPlus::draw_cross(showImage, pts, 4, colour);
        VilPlus::vil_save(showImage, "keyframe_sift.jpg");
    }
    
    // feature in image
    vil_image_view<vxl_byte> grey_queryImage;
    vil_convert_planes_to_grey(queryImage, grey_queryImage);
    vcl_vector<bapl_keypoint_sptr> queryImageSift;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_queryImage), queryImageSift);
    
    bapl_bbf_tree tree(queryImageSift, 16);
    
    // match from warpted topview to image
    // use two threshold 0.6 and 0.7 too avoid too few matching situation
    // if 0.6 get enough matches, 0.7 and later will be discarded
    vcl_vector<double> ratios;
    ratios.push_back(0.6);
    ratios.push_back(0.7);
    
    vcl_vector< vcl_vector<bapl_key_match> > matchesVec(2);
    for (unsigned int i = 0; i<keyframeSift.size(); i++) {
        bapl_keypoint_sptr      query = keyframeSift[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        for (int j = 0; j<ratios.size(); j++) {
            if (ssd0 < ssd1 * ratios[j]) {
                matchesVec[j].push_back(k_p);
            }
        }
    }
    
    // decide which threshold 0.6 or 0.7
    vcl_vector<bapl_key_match> matches;
    if (matchesVec[0].size() >= 100) {
        matches = matchesVec[0];
    }
    else
    {
        matches = matchesVec[1];
    }
    if (matches.size() <= featureMatchingNumThreshold) {
        vcl_cerr<<"match number too few "<<matches.size()<<vcl_endl;
        return false;
    }
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_image;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts_keyframe.push_back(p1);
        pts_image.push_back(p2);
    }
    assert(pts_keyframe.size() == pts_image.size());
    vcl_cout<<"initial matched number is ---------------------------------- "<<pts_keyframe.size()<<vcl_endl;
    
    //ransac 10 times, to get the best one with minimal projection error
    vgl_h_matrix_2d<double> H_topviewTokeyframe;
    court.topviewToCameraHomography(keyframeCamera, H_topviewTokeyframe, destWidth, destHeight);
    
    vcl_vector<double> projectionErrors;
    vcl_vector<vpgl_perspective_camera<double> > optimizedCameras;
    for (int i = 0; i<10; i++) {
        vcl_vector<bool> inlier;  //inlier not used here, only H_keyframeToQuery is used
        vgl_h_matrix_2d<double> H_keyframeToQuery = VrelPlus::homography_RANSAC(pts_keyframe, pts_image, inlier, 2.0);    //Result vary from time to time
        assert(inlier.size() == pts_keyframe.size());
        
        vnl_matrix_fixed<double, 3, 3> topviewToQuery =  H_keyframeToQuery.get_matrix() * H_topviewTokeyframe.get_matrix();
        vgl_h_matrix_2d<double> H_topviewToQuery(topviewToQuery);
        
        //sample positions in the topview
        vcl_vector<vgl_point_2d<double> > courtPoints = court.getCalibPoints();
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints_world;
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints;   // query image sapce
        for (int  j = 0; j<courtPoints.size(); j++) {
            vgl_point_2d<double> p_world   = courtPoints[j];
            vgl_point_2d<double> p_topview = (court.imageToWorld().inverse())(p_world);
            
            vgl_homg_point_2d<double> p(p_topview.x(), p_topview.y(), 1.0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)H_topviewToQuery(p);
            
            if(vgl_inside_image(proj_p, destWidth, destHeight))
            {
                sampledCourtPoints_world.push_back(p_world);  // world coordinate in meter
                sampledCourtPoints.push_back(proj_p);         // pixel position in query image
            }
        }
        
        assert(sampledCourtPoints.size() == sampledCourtPoints_world.size());
        if (sampledCourtPoints.size() < 4) {
            vcl_cerr<<"sampled court point less than 4."<<vcl_endl;
            continue;
        }
        
        vgl_point_2d<double> principlePoint(destWidth/2, destHeight/2);
        vpgl_perspective_camera<double> initCamera;
        bool isCalib = VpglPlus::init_calib(sampledCourtPoints_world, sampledCourtPoints, principlePoint, initCamera);
        
        if (!isCalib) {
            vcl_cerr<<"init calib failed."<<vcl_endl;
            continue;
        }
        vcl_cout<<"init focal length is "<<initCamera.get_calibration().focal_length()<<vcl_endl;
        
        if (sampledCourtPoints_world.size() < 5) {
            vcl_cout<<" sampled points less than 5, camera is not optimized.\n";
            continue;
        }
        
        vpgl_perspective_camera<double> opt_camera;
        bool isOpted = VpglPlus::optimize_perspective_camera(sampledCourtPoints_world, sampledCourtPoints, initCamera, opt_camera);
        if (!isOpted) {
            vcl_cerr<<"optimize camera failed."<<vcl_endl;
            continue;
        }
        
        vcl_cout<<"optimized focal length is "<<opt_camera.get_calibration().focal_length()<<vcl_endl;
        
        double err_avg = 0;  // average projection error
        for (int j = 0; j<sampledCourtPoints_world.size(); j++) {
            vgl_point_3d<double> p(sampledCourtPoints_world[i].x(), sampledCourtPoints_world[i].y(), 0);
            vgl_point_2d<double> proj_p = opt_camera.project(p);
            
            double dx = sampledCourtPoints[i].x() - proj_p.x();
            double dy = sampledCourtPoints[i].y() - proj_p.y();
            err_avg += sqrt(dx * dx + dy * dy);
        }
        err_avg /= sampledCourtPoints_world.size();
        
        if (err_avg <= projectionErrorThreshold) {
            projectionErrors.push_back(err_avg);
            optimizedCameras.push_back(opt_camera);
        }
    }
    
    if (optimizedCameras.size() == 0) {
        vcl_cerr<<"No camera found under projection error threshold.\n";
        return false;
    }
    
    //find the camera with minimum reprojection error;
    double err_min = INT_MAX;
    int idx_min = -1;
    for (int i = 0; i< projectionErrors.size(); i++) {
        if (projectionErrors[i] < err_min) {
            err_min = projectionErrors[i];
            idx_min = i;
        }
    }
    assert(idx_min != -1);
    queryCamera = optimizedCameras[idx_min];
    
    return true;
}

bool VilSIFT::calibFromKeyframe_keypoint_vs_SIFT(const vil_image_view<vxl_byte> &topviewImage, const vpgl_perspective_camera<double> &keyframeCamera,
                                                 const vil_image_view<vxl_byte> &queryImage, vpgl_perspective_camera<double> &queryCamera,
                                                 vpgl_perspective_camera<double> & queryCameraFromSIFT,
                                                 vcl_vector<vgl_point_2d<double> > & sift_pts,
                                                 int featureMatchingNumThreshold, double projectionErrorThreshold)
{
    assert(topviewImage.nplanes() == 3);
    assert(queryImage.nplanes()  == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    
    int destWidth  = queryImage.ni();
    int destHeight = queryImage.nj();
    
    // feature in warpted topview
    vil_image_view<vxl_byte> warpedTopview;
    vil_image_view<vxl_byte> alphaMap;    //court area
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    vgl_point_2d<int> starP(30, 30);
    vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
    court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap, starP, endP);
    
    // get features from warpted topview
    vil_image_view<vxl_byte> grey_warpedTopview;
    vil_convert_planes_to_grey(warpedTopview, grey_warpedTopview);
    vcl_vector<bapl_keypoint_sptr> keyframeSiftAll;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_warpedTopview), keyframeSiftAll);
    
    // filter out keypoints near boundary area
    vcl_vector<bapl_keypoint_sptr> keyframeSift;
    for (int i = 0; i<keyframeSiftAll.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(keyframeSiftAll[i].as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        
        if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
            keyframeSift.push_back(keyframeSiftAll[i]);
        }
    }
    keyframeSiftAll.clear();
    
    {
        // save keyframe sift
        vil_image_view<vxl_byte> showImage;
        showImage.deep_copy(warpedTopview);
        
        vcl_vector<vgl_point_2d<double> > pts;
        for (int i = 0; i<keyframeSift.size(); i++) {
            bapl_lowe_keypoint_sptr sift = dynamic_cast<bapl_lowe_keypoint*>(keyframeSift[i].as_pointer());
            vgl_point_2d<double> p(sift->location_i(), sift->location_j());
            pts.push_back(p);
        }
        
        vcl_vector<vxl_byte> colour;
        colour.push_back(0);
        colour.push_back(255);
        colour.push_back(0);
        VilPlus::draw_cross(showImage, pts, 4, colour, 1);
        VilPlus::vil_save(showImage, "keyframe_sift.jpg");
    }
    
    // feature in image
    vil_image_view<vxl_byte> grey_queryImage;
    vil_convert_planes_to_grey(queryImage, grey_queryImage);
    vcl_vector<bapl_keypoint_sptr> queryImageSift;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_queryImage), queryImageSift);
    
    bapl_bbf_tree tree(queryImageSift, 16);
    
    // match from warpted topview to image
    // use two threshold 0.6 and 0.7 too avoid too few matching situation
    // if 0.6 get enough matches, 0.7 and later will be discarded
    vcl_vector<double> ratios;
    ratios.push_back(0.6);
    ratios.push_back(0.7);
    
    vcl_vector< vcl_vector<bapl_key_match> > matchesVec(2);
    for (unsigned int i = 0; i<keyframeSift.size(); i++) {
        bapl_keypoint_sptr      query = keyframeSift[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        for (int j = 0; j<ratios.size(); j++) {
            if (ssd0 < ssd1 * ratios[j]) {
                matchesVec[j].push_back(k_p);
            }
        }
    }
    
    // decide which threshold 0.6 or 0.7
    vcl_vector<bapl_key_match> matches;
    if (matchesVec[0].size() >= 100) {
        matches = matchesVec[0];
    }
    else
    {
        matches = matchesVec[1];
    }
    if (matches.size() <= featureMatchingNumThreshold) {
        vcl_cerr<<"match number too few "<<matches.size()<<vcl_endl;
        return false;
    }
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_image;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts_keyframe.push_back(p1);
        pts_image.push_back(p2);
    }
    assert(pts_keyframe.size() == pts_image.size());
    vcl_cout<<"initial matched number is ---------------------------------- "<<pts_keyframe.size()<<vcl_endl;
    
    //ransac 10 times, to get the best one with minimal projection error
    vgl_h_matrix_2d<double> H_topviewTokeyframe;
    court.topviewToCameraHomography(keyframeCamera, H_topviewTokeyframe, destWidth, destHeight);
    
    vcl_vector<double> projectionErrors;
    vcl_vector<vpgl_perspective_camera<double> >   optimizedCameras;
    vcl_vector<vcl_vector<vgl_point_2d<double> > > optimized_sifts;
    vcl_vector<vgl_h_matrix_2d<double> >           optimized_h_topviewToQuery;
   
    for (int i = 0; i<10; i++) {
        vcl_vector<bool> inlier;  //inlier not used here, only H_keyframeToQuery is used
        vgl_h_matrix_2d<double> H_keyframeToQuery = VrelPlus::homography_RANSAC(pts_keyframe, pts_image, inlier, 2.0);    //Result vary from time to time
        assert(inlier.size() == pts_keyframe.size());
        
        vnl_matrix_fixed<double, 3, 3> topviewToQuery =  H_keyframeToQuery.get_matrix() * H_topviewTokeyframe.get_matrix();
        vgl_h_matrix_2d<double> H_topviewToQuery(topviewToQuery);
        
        //sample positions in the topview
        vcl_vector<vgl_point_2d<double> > courtPoints = court.getCalibPoints();
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints_world;
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints;   // query image sapce
        for (int  j = 0; j<courtPoints.size(); j++) {
            vgl_point_2d<double> p_world   = courtPoints[j];
            vgl_point_2d<double> p_topview = (court.imageToWorld().inverse())(p_world);
            
            vgl_homg_point_2d<double> p(p_topview.x(), p_topview.y(), 1.0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)H_topviewToQuery(p);
            
            if(vgl_inside_image(proj_p, destWidth, destHeight))
            {
                sampledCourtPoints_world.push_back(p_world);  // world coordinate in meter
                sampledCourtPoints.push_back(proj_p);         // pixel position in query image
            }
        }
        
        assert(sampledCourtPoints.size() == sampledCourtPoints_world.size());
        if (sampledCourtPoints.size() < 4) {
            vcl_cerr<<"sampled court point less than 4."<<vcl_endl;
            continue;
        }
        
        vgl_point_2d<double> principlePoint(destWidth/2, destHeight/2);
        vpgl_perspective_camera<double> initCamera;
        bool isCalib = VpglPlus::init_calib(sampledCourtPoints_world, sampledCourtPoints, principlePoint, initCamera);
        
        if (!isCalib) {
            vcl_cerr<<"init calib failed."<<vcl_endl;
            continue;
        }
        vcl_cout<<"init focal length is "<<initCamera.get_calibration().focal_length()<<vcl_endl;
        
        if (sampledCourtPoints_world.size() < 5) {
            vcl_cout<<" sampled points less than 5, camera is not optimized.\n";
            continue;
        }
        
        vpgl_perspective_camera<double> opt_camera;
        bool isOpted = VpglPlus::optimize_perspective_camera(sampledCourtPoints_world, sampledCourtPoints, initCamera, opt_camera);
        if (!isOpted) {
            vcl_cerr<<"optimize camera failed."<<vcl_endl;
            continue;
        }
        
        vcl_cout<<"optimized focal length is "<<opt_camera.get_calibration().focal_length()<<vcl_endl;
        
        double err_avg = 0;  // average projection error
        for (int j = 0; j<sampledCourtPoints_world.size(); j++) {
            vgl_point_3d<double> p(sampledCourtPoints_world[i].x(), sampledCourtPoints_world[i].y(), 0);
            vgl_point_2d<double> proj_p = opt_camera.project(p);
            
            double dx = sampledCourtPoints[i].x() - proj_p.x();
            double dy = sampledCourtPoints[i].y() - proj_p.y();
            err_avg += sqrt(dx * dx + dy * dy);
        }
        err_avg /= sampledCourtPoints_world.size();
        
        if (err_avg <= projectionErrorThreshold) {
            projectionErrors.push_back(err_avg);
            optimizedCameras.push_back(opt_camera);
            
            //store dada for sift camera calibration
            // for visualization purpose
            {
                vcl_vector<vgl_point_2d<double> > inlier_sift_pts;
                for (int j = 0; j<inlier.size(); j++) {
                    if(inlier[j])
                    {
                        inlier_sift_pts.push_back(pts_image[j]);
                    }
                }
                optimized_sifts.push_back(inlier_sift_pts);
                optimized_h_topviewToQuery.push_back(H_topviewToQuery);
            }
        }
    }
    
    if (optimizedCameras.size() == 0) {
        vcl_cerr<<"No camera found under projection error threshold.\n";
        return false;
    }
    
    //find the camera with minimum reprojection error;
    double err_min = INT_MAX;
    int idx_min = -1;
    for (int i = 0; i< projectionErrors.size(); i++) {
        if (projectionErrors[i] < err_min) {
            err_min = projectionErrors[i];
            idx_min = i;
        }
    }
    assert(idx_min != -1);
    queryCamera = optimizedCameras[idx_min];
    
    // calib from SIFT as a comparison
    {
        vcl_vector<vgl_point_2d<double> > sifts  = optimized_sifts[idx_min];
        vgl_h_matrix_2d<double> H_topviewToQuery = optimized_h_topviewToQuery[idx_min];
        vcl_vector<vgl_point_2d<double> > courtPoints;
        for (int i = 0; i<sifts.size(); i++) {
            vgl_point_2d<double> p = sifts[i];
            vgl_point_2d<double> p_top_view_image = (vgl_point_2d<double>)(H_topviewTokeyframe.get_inverse())(vgl_homg_point_2d<double>(p.x(), p.y(), 1.0));
            vgl_point_2d<double> p_world = (court.imageToWorld())(p_top_view_image);
            courtPoints.push_back(p_world);
        }
        
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints_world;
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints;   // query image sapce
        for (int  j = 0; j<courtPoints.size(); j++) {
            vgl_point_2d<double> p_world   = courtPoints[j];
            vgl_point_2d<double> p_topview = (court.imageToWorld().inverse())(p_world);
            
            vgl_homg_point_2d<double> p(p_topview.x(), p_topview.y(), 1.0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)H_topviewToQuery(p);
            
            if(vgl_inside_image(proj_p, destWidth, destHeight))
            {
                sampledCourtPoints_world.push_back(p_world);  // world coordinate in meter
                sampledCourtPoints.push_back(proj_p);         // pixel position in query image
            }
        }
        
        assert(sampledCourtPoints.size() == sampledCourtPoints_world.size());
        if (sampledCourtPoints.size() >= 4) {
            vgl_point_2d<double> principlePoint(destWidth/2, destHeight/2);
            vpgl_perspective_camera<double> initCamera;
            bool isCalib = VpglPlus::init_calib(sampledCourtPoints_world, sampledCourtPoints, principlePoint, initCamera);
            
            if (isCalib) {
                vcl_cout<<"init focal length is "<<initCamera.get_calibration().focal_length()<<vcl_endl;
                if (sampledCourtPoints_world.size() >= 5) {
                    vpgl_perspective_camera<double> opt_camera;
                    bool isOpted = VpglPlus::optimize_perspective_camera(sampledCourtPoints_world, sampledCourtPoints, initCamera, opt_camera);
                    if (isOpted) {                        
                        queryCameraFromSIFT = opt_camera;
                        sift_pts = sifts;
                        vcl_cout<<"get camera from sift correspondence."<<vcl_endl;
                    }
                }
            }
        }
    }
    
    return true;
}


bool VilSIFT::calibFromKeyframeCached(const vil_image_view<vxl_byte> &topviewImage, const vpgl_perspective_camera<double> &keyframeCamera,
                                      vcl_vector<bapl_keypoint_sptr> * keyframeFeatures,
                                      const vil_image_view<vxl_byte> &queryImage, vpgl_perspective_camera<double> &queryCamera,
                                      int featureMatchingNumThreshold, double projectionErrorThreshold)
{
    assert(topviewImage.nplanes() == 3);
    assert(queryImage.nplanes()  == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    
    int destWidth  = queryImage.ni();
    int destHeight = queryImage.nj();
    
    // feature in warpted topview
    vil_image_view<vxl_byte> warpedTopview;
    vil_image_view<vxl_byte> alphaMap;    //court area
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    vcl_vector<bapl_keypoint_sptr> keyframeSift;
    if(keyframeFeatures == NULL)
    {
        vgl_point_2d<int> starP(30, 30);
        vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
        court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap, starP, endP);
        
        // get features from warpted topview
        vil_image_view<vxl_byte> grey_warpedTopview;
        vil_convert_planes_to_grey(warpedTopview, grey_warpedTopview);
        vcl_vector<bapl_keypoint_sptr> keyframeSiftAll;
        bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_warpedTopview), keyframeSiftAll);
        
        // filter out keypoints near boundary area
        for (int i = 0; i<keyframeSiftAll.size(); i++) {
            bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(keyframeSiftAll[i].as_pointer());
            vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
            
            if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
                keyframeSift.push_back(keyframeSiftAll[i]);
            }
        }
        keyframeSiftAll.clear();
    }
    else
    {
        keyframeSift = *keyframeFeatures;
    }
    
    // feature in image
    vil_image_view<vxl_byte> grey_queryImage;
    vil_convert_planes_to_grey(queryImage, grey_queryImage);
    vcl_vector<bapl_keypoint_sptr> queryImageSift;
    bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_queryImage), queryImageSift);
    
    bapl_bbf_tree tree(queryImageSift, 16);
    
    // match from warpted topview to image
    // use two threshold 0.6 and 0.7 too avoid too few matching situation
    // if 0.6 get enough matches, 0.7 and later will be discarded
    vcl_vector<double> ratios;
    ratios.push_back(0.6);
    ratios.push_back(0.7);
    
    vcl_vector< vcl_vector<bapl_key_match> > matchesVec(2);
    for (unsigned int i = 0; i<keyframeSift.size(); i++) {
        bapl_keypoint_sptr      query = keyframeSift[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        for (int j = 0; j<ratios.size(); j++) {
            if (ssd0 < ssd1 * ratios[j]) {
                matchesVec[j].push_back(k_p);
            }
        }
    }
    
    // decide which threshold 0.6 or 0.7
    vcl_vector<bapl_key_match> matches;
    if (matchesVec[0].size() >= 100) {
        matches = matchesVec[0];
    }
    else
    {
        matches = matchesVec[1];
    }
    if (matches.size() <= featureMatchingNumThreshold) {
        vcl_cerr<<"match number too few "<<matches.size()<<vcl_endl;
        return false;
    }
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_image;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts_keyframe.push_back(p1);
        pts_image.push_back(p2);
    }
    assert(pts_keyframe.size() == pts_image.size());
    vcl_cout<<"initial matched number is ---------------------------------- "<<pts_keyframe.size()<<vcl_endl;
    
    //ransac 10 times, to get the best one with minimal projection error
    vgl_h_matrix_2d<double> H_topviewTokeyframe;
    court.topviewToCameraHomography(keyframeCamera, H_topviewTokeyframe, destWidth, destHeight);
    
    vcl_vector<vpgl_perspective_camera<double> > optimizedCameras;
    for (int i = 0; i<10; i++) {
        vcl_vector<bool> inlier;  //inlier not used here, only H_keyframeToQuery is used
        vgl_h_matrix_2d<double> H_keyframeToQuery = VrelPlus::homography_RANSAC(pts_keyframe, pts_image, inlier, 2.0);    //Result vary from time to time
        assert(inlier.size() == pts_keyframe.size());
        
        vnl_matrix_fixed<double, 3, 3> topviewToQuery =  H_keyframeToQuery.get_matrix() * H_topviewTokeyframe.get_matrix();
        vgl_h_matrix_2d<double> H_topviewToQuery(topviewToQuery);
        
        //sample positions in the topview
        vcl_vector<vgl_point_2d<double> > courtPoints = DisneyWorldBasketballCourt::getCalibPoints();
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints_world;
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints;   // query image sapce
        for (int  j = 0; j<courtPoints.size(); j++) {
            vgl_point_2d<double> p_world   = courtPoints[j];
            vgl_point_2d<double> p_topview = (court.imageToWorld().inverse())(p_world);
            
            vgl_homg_point_2d<double> p(p_topview.x(), p_topview.y(), 1.0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)H_topviewToQuery(p);
            
            if(vgl_inside_image(proj_p, destWidth, destHeight))
            {
                sampledCourtPoints_world.push_back(p_world);  // world coordinate in meter
                sampledCourtPoints.push_back(proj_p);         // pixel position in query image
            }
        }
        
        assert(sampledCourtPoints.size() == sampledCourtPoints_world.size());
        if (sampledCourtPoints.size() < 4) {
            vcl_cerr<<"sampled court point less than 4."<<vcl_endl;
            continue;
        }
        
        vgl_point_2d<double> principlePoint(destWidth/2, destHeight/2);
        vpgl_perspective_camera<double> initCamera;
        bool isCalib = VpglPlus::init_calib(sampledCourtPoints_world, sampledCourtPoints, principlePoint, initCamera);
        
        if (!isCalib) {
            vcl_cerr<<"init calib failed."<<vcl_endl;
            continue;
        }
        vcl_cout<<"init focal length is "<<initCamera.get_calibration().focal_length()<<vcl_endl;
        
        if (sampledCourtPoints_world.size() < 5) {
            vcl_cerr<<" sampled points less than 5, camera is not optimized.\n";
            continue;
        }
        
        vpgl_perspective_camera<double> opt_camera;
        bool isOpted = VpglPlus::optimize_perspective_camera(sampledCourtPoints_world, sampledCourtPoints, initCamera, opt_camera);
        if (!isOpted) {
            vcl_cerr<<"optimize camera failed."<<vcl_endl;
            continue;
        }
        
        vcl_cout<<"optimized focal length is "<<opt_camera.get_calibration().focal_length()<<vcl_endl;
        
        double err_avg = 0;  // average projection error
        for (int j = 0; j<sampledCourtPoints_world.size(); j++) {
            vgl_point_3d<double> p(sampledCourtPoints_world[i].x(), sampledCourtPoints_world[i].y(), 0);
            vgl_point_2d<double> proj_p = opt_camera.project(p);
            
            double dx = sampledCourtPoints[i].x() - proj_p.x();
            double dy = sampledCourtPoints[i].y() - proj_p.y();
            err_avg += sqrt(dx * dx + dy * dy);
        }
        err_avg /= sampledCourtPoints_world.size();
        
        if (err_avg <= projectionErrorThreshold) {
            
            optimizedCameras.push_back(opt_camera);
        }
    }
    
    if (optimizedCameras.size() == 0) {
        vcl_cerr<<"No camera found under projection error threshold.\n";
        return false;
    }
    
    vcl_vector<double> ssds;
    vcl_vector<double> sads;
    court.alignmentQuality(topviewImage, queryImage, optimizedCameras, ssds, sads);
    int idx_min = (int)(vcl_min_element(sads.begin(), sads.end()) - sads.begin());
    queryCamera = optimizedCameras[idx_min];
    
    return true;
}

bool VilSIFT::calibFromKeyframeCachedSIFT(const vil_image_view<vxl_byte> &topviewImage,
                                          const vpgl_perspective_camera<double> &keyframeCamera,
                                          vcl_vector<bapl_keypoint_sptr> * keyframeFeatures,
                                          const vil_image_view<vxl_byte> & queryImage,
                                          vcl_vector<bapl_keypoint_sptr> * queryImageFeatures,
                                          vpgl_perspective_camera<double> &queryCamera,
                                          int featureMatchingNumThreshold)
{
    assert(topviewImage.nplanes() == 3);
    assert(queryImage.nplanes()  == 3);
    assert(topviewImage.ni() == 1268);
    assert(topviewImage.nj() == 740);
    
    int destWidth  = queryImage.ni();
    int destHeight = queryImage.nj();
    
    // feature in warpted topview
    vil_image_view<vxl_byte> warpedTopview;
    vil_image_view<vxl_byte> alphaMap;    //court area
    DisneyWorldBasketballCourt court;
    court.projectTopviewImage(topviewImage, keyframeCamera, destWidth, destHeight, warpedTopview, 20);
    
    vcl_vector<bapl_keypoint_sptr> keyframeSift;
    if(keyframeFeatures == NULL)
    {
        vgl_point_2d<int> starP(30, 30);
        vgl_point_2d<int> endP(topviewImage.ni() - 30, topviewImage.nj() - 70);
        court.getProjectedCourtArea(keyframeCamera, destWidth, destHeight, alphaMap, starP, endP);
        
        // get features from warpted topview
        vil_image_view<vxl_byte> grey_warpedTopview;
        vil_convert_planes_to_grey(warpedTopview, grey_warpedTopview);
        vcl_vector<bapl_keypoint_sptr> keyframeSiftAll;
        bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_warpedTopview), keyframeSiftAll);
        
        // filter out keypoints near boundary area
        for (int i = 0; i<keyframeSiftAll.size(); i++) {
            bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(keyframeSiftAll[i].as_pointer());
            vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
            
            if (alphaMap((int)p1.x(), (int)p1.y()) == 255) {
                keyframeSift.push_back(keyframeSiftAll[i]);
            }
        }
        keyframeSiftAll.clear();
    }
    else
    {
        keyframeSift = *keyframeFeatures;
    }
    
    // feature in image
     vcl_vector<bapl_keypoint_sptr> queryImageSift;
    if (queryImageFeatures == NULL) {
        vil_image_view<vxl_byte> grey_queryImage;
        vil_convert_planes_to_grey(queryImage, grey_queryImage);
        bapl_keypoint_extractor(vil_new_image_resource_of_view(grey_queryImage), queryImageSift);
    }
    else
    {
        //use cashed query image features
        queryImageSift = * queryImageFeatures;
    }
    
    bapl_bbf_tree tree(queryImageSift, 16);
    
    // match from warpted topview to image
    // use two threshold 0.6 and 0.7 too avoid too few matching situation
    // if 0.6 get enough matches, 0.7 and later will be discarded
    vcl_vector<double> ratios;
    ratios.push_back(0.6);
    ratios.push_back(0.7);
    
    vcl_vector< vcl_vector<bapl_key_match> > matchesVec(2);
    for (unsigned int i = 0; i<keyframeSift.size(); i++) {
        bapl_keypoint_sptr      query = keyframeSift[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        tree.n_nearest(query, match, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        bapl_key_match k_p(query, match[0]);
        for (int j = 0; j<ratios.size(); j++) {
            if (ssd0 < ssd1 * ratios[j]) {
                matchesVec[j].push_back(k_p);
            }
        }
    }
    
    // decide which threshold 0.6 or 0.7
    vcl_vector<bapl_key_match> matches;
    if (matchesVec[0].size() >= 100) {
        matches = matchesVec[0];
    }
    else
    {
        matches = matchesVec[1];
    }
    if (matches.size() <= featureMatchingNumThreshold) {
        vcl_cerr<<"match number too few "<<matches.size()<<vcl_endl;
        return false;
    }
    
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_image;
    for (int i = 0; i<matches.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint*>(matches[i].second.as_pointer());
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        
        pts_keyframe.push_back(p1);
        pts_image.push_back(p2);
    }
    assert(pts_keyframe.size() == pts_image.size());
    vcl_cout<<"initial matched number is "<<pts_keyframe.size()<<vcl_endl;
    
    // test
    /*
    static int idx = 0;
    {
        idx++;
        vcl_vector<vgl_point_2d<double>> pts1 = pts_keyframe;
        vcl_vector<vgl_point_2d<double>> pts2 = pts_image;
        vil_image_view<vxl_byte> matchImage;
        VilPlus::draw_match_dense(warpedTopview, queryImage, pts1, pts2, matchImage);
        
        char buf[1024] = {NULL};
        sprintf(buf, "%d.jpg", idx);
        VilPlus::vil_save(matchImage, buf);
    }
     */
    
    //ransac 10 times, to get the best one with minimal projection error
    vgl_h_matrix_2d<double> H_topviewTokeyframe;
    court.topviewToCameraHomography(keyframeCamera, H_topviewTokeyframe, destWidth, destHeight);
    
    vcl_vector<vpgl_perspective_camera<double> > optimizedCameras;
    for (int i = 0; i<10; i++) {
        vcl_vector<bool> inlier;  //inlier not used here, only H_keyframeToQuery is used
        vgl_h_matrix_2d<double> H_keyframeToQuery = VrelPlus::homography_RANSAC(pts_keyframe, pts_image, inlier, 2.0);    //Result vary from time to time
        assert(inlier.size() == pts_keyframe.size());
        
        vnl_matrix_fixed<double, 3, 3> topviewToQuery =  H_keyframeToQuery.get_matrix() * H_topviewTokeyframe.get_matrix();
        vgl_h_matrix_2d<double> H_topviewToQuery(topviewToQuery);
        
        //sample positions in the topview
        vcl_vector<vgl_point_2d<double> > courtPoints = DisneyWorldBasketballCourt::getCalibPoints();
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints_world;
        vcl_vector<vgl_point_2d<double> > sampledCourtPoints;   // query image sapce
        for (int  j = 0; j<courtPoints.size(); j++) {
            vgl_point_2d<double> p_world   = courtPoints[j];
            vgl_point_2d<double> p_topview = (court.imageToWorld().inverse())(p_world);
            
            vgl_homg_point_2d<double> p(p_topview.x(), p_topview.y(), 1.0);
            vgl_point_2d<double> proj_p = (vgl_point_2d<double>)H_topviewToQuery(p);
            
            if(vgl_inside_image(proj_p, destWidth, destHeight, 20))
            {
                sampledCourtPoints_world.push_back(p_world);  // world coordinate in meter
                sampledCourtPoints.push_back(proj_p);         // pixel position in query image
            }
        }
        /*
        {
            //test draw projected point in image space
            vil_image_view<vxl_byte> showImage;
            showImage.deep_copy(queryImage);
            VilPlus::draw_cross(showImage, sampledCourtPoints, 5, VilPlus::green());
            
            char buf[1024] = {NULL};
            sprintf(buf, "%d_projected_keypoint.jpg", i);
            VilPlus::vil_save(showImage, buf);
        }
         */
        
        assert(sampledCourtPoints.size() == sampledCourtPoints_world.size());
        if (sampledCourtPoints.size() < 4) {
        //    vcl_cerr<<"sampled court point less than 4."<<vcl_endl;
            continue;
        }
        
        vgl_point_2d<double> principlePoint(destWidth/2, destHeight/2);
        vpgl_perspective_camera<double> initCamera;
        bool isCalib = VpglPlus::init_calib(sampledCourtPoints_world, sampledCourtPoints, principlePoint, initCamera);
        
        if (!isCalib) {
            vcl_cerr<<"init calib failed."<<vcl_endl;
            continue;
        }
  //      vcl_cout<<"init focal length is "<<initCamera.get_calibration().focal_length()<<vcl_endl;
        
        if (sampledCourtPoints_world.size() < 5) {
        //    vcl_cerr<<" sampled points less than 5, camera is not optimized.\n";
            continue;
        }
        
        vpgl_perspective_camera<double> opt_camera;
        bool isOpted = VpglPlus::optimize_perspective_camera(sampledCourtPoints_world, sampledCourtPoints, initCamera, opt_camera);
        if (!isOpted) {
        //    vcl_cerr<<"optimize camera failed."<<vcl_endl;
            continue;
        }
        
        vcl_cout<<"optimized focal length is "<<opt_camera.get_calibration().focal_length()<<vcl_endl;
        
        double cx = opt_camera.camera_center().x();
        double cy = opt_camera.camera_center().y();
        double cz = opt_camera.camera_center().z();
        // average positions: 12.9456, -14.8695, 6.21215
        if (cx >= 10  && cx <= 20 &&
            cy >= -20 && cy <= -10 &&
            cz >= 5.0 && cz <= 10)
        {
            optimizedCameras.push_back(opt_camera);
        }
        
    }
    
    if (optimizedCameras.size() == 0) {
        vcl_cerr<<"No camera found under projection error threshold.\n";
        return false;
    }
    
    vcl_vector<double> ssds;
    vcl_vector<double> sads;
    court.alignmentQuality(topviewImage, queryImage, optimizedCameras, ssds, sads);
    int idx_min = (int)(vcl_min_element(sads.begin(), sads.end()) - sads.begin());
    queryCamera = optimizedCameras[idx_min];
    
    return true;
}

bool VilSIFT::writeSIFT(const char *file, const vcl_vector< bapl_keypoint_sptr > & keypoints)
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
    
    return true;
}

bool VilSIFT::readSIFT(const char *file, vcl_vector< bapl_keypoint_sptr > & keypoints)
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
    printf("find %d keypoints.\n", n);
    return true;
}




