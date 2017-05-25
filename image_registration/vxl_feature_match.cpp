//
//  vxl_feature_match.cpp
//  OnlineStereo
//
//  Created by jimmy on 1/3/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vxl_feature_match.h"
#include <bapl/bapl_keypoint_set.h>
#include <vpgl/vpgl_fundamental_matrix.h>
#include <vgl/vgl_distance.h>
#include "vxl_vrel_plus.h"
#include "vil_bapl_sift.h"


void VxlFeatureMatch::siftMatchByRatio(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                       const vcl_vector<bapl_keypoint_sptr> & keypointsB,
                                       vcl_vector<bapl_key_match> & matches,
                                       vcl_vector<vcl_pair<int, int> > & matchedIndices,
                                       double ratio, double feature_distance_threshold)
{
    bapl_bbf_tree tree(keypointsB, 4);
    
    for (unsigned int i = 0; i<keypointsA.size(); i++)
    {
        bapl_keypoint_sptr query = keypointsA[i];
        vcl_vector<bapl_keypoint_sptr> match;
        vcl_vector< int > closest_indices;
        tree.n_nearest(query, match, closest_indices, 2, -1);
        
        if (vnl_vector_ssd(query->descriptor(), match[0]->descriptor() ) <
            vnl_vector_ssd(query->descriptor(), match[1]->descriptor() ) * ratio)
        {
            double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
            double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
            bapl_key_match k_p(query, match[0]);
            if (ssd0 < feature_distance_threshold && ssd0 < ssd1 * ratio) {
                matches.push_back(k_p);
                matchedIndices.push_back(vcl_pair<int,int>(i, closest_indices[0]));
            }
        }        
    }
   // printf("find %lu matches from %lu points\n", matches.size(), keypointsA.size());
}

int VxlFeatureMatch::siftMatchAndHomography(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                            const vcl_vector<bapl_keypoint_sptr> & keypointsB,                                            
                                            vcl_vector<vgl_point_2d<double> > & inlierPtsA,
                                            vcl_vector<vgl_point_2d<double> > & inlierPtsB,
                                            double homography_inlier_distance,
                                            double ratio,
                                            double feature_distance_threshold)
{
    vcl_vector<bapl_key_match> matches;
    vcl_vector<vcl_pair<int, int> > matchedIndices;
    VxlFeatureMatch::siftMatchByRatio(keypointsA, keypointsB, matches, matchedIndices, ratio, feature_distance_threshold);
    
    // filter by homography
    vcl_vector<bapl_key_match> finalMatches;
    bool isH = VxlFeatureMatch::filterMatchingByHomography(matches, finalMatches, homography_inlier_distance);
    if (!isH) {
        return 0;
    }
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_query;
    for (int i = 0; i<finalMatches.size(); i++) {
        pts_keyframe.push_back((VilBaplSIFT::get_sift_location(finalMatches[i].first)));
        pts_query.push_back((VilBaplSIFT::get_sift_location(finalMatches[i].second)));
    }
    assert(pts_keyframe.size() >= 4);
    
    inlierPtsA = pts_keyframe;
    inlierPtsB = pts_query;
    
    assert(inlierPtsA.size() == inlierPtsB.size());
    return (int)inlierPtsA.size();
}

int VxlFeatureMatch::siftMatchAndHomography(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                            bapl_bbf_tree *tree,
                                            vcl_vector<vgl_point_2d<double> > & inlierPtsA,
                                            vcl_vector<vgl_point_2d<double> > & inlierPtsB,
                                            double homography_inlier_distance,
                                            double ratio,
                                            double feature_distance_threshold)
{
    vcl_vector<bapl_key_match> matches;
    
    // sift match
    for (unsigned int i = 0; i<keypointsA.size(); i++)
    {
        bapl_keypoint_sptr query = keypointsA[i];
        vcl_vector<bapl_keypoint_sptr> match;
        vcl_vector< int > closest_indices;
        tree->n_nearest(query, match, closest_indices, 2, -1);
        double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
        double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
        if (ssd0 < feature_distance_threshold && ssd0 < ssd1 * ratio)
        {
            bapl_key_match k_p(query, match[0]);
            matches.push_back(k_p);
        }
    }
    if (matches.size() < 4) {
        return 0;
    }
    
    // filter by homography
    vcl_vector<bapl_key_match> finalMatches;
    bool isH = VxlFeatureMatch::filterMatchingByHomography(matches, finalMatches, homography_inlier_distance);
    if (!isH) {
        return 0;
    }
    
    for (int i = 0; i<finalMatches.size(); i++) {
        inlierPtsA.push_back((VilBaplSIFT::get_sift_location(finalMatches[i].first)));
        inlierPtsB.push_back((VilBaplSIFT::get_sift_location(finalMatches[i].second)));
    }
    assert(inlierPtsA.size() >= 4);
    
    assert(inlierPtsA.size() == inlierPtsB.size());
    return (int)inlierPtsA.size();
}

bool VxlFeatureMatch::filterMatchingByHomography(const vcl_vector<bapl_key_match> & initMatches,
                                                 vcl_vector<bapl_key_match> & finalMatches,
                                                 double homography_inlier_distance)
{
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_query;
    for (int i = 0; i<initMatches.size(); i++) {
        pts_keyframe.push_back(VilBaplSIFT::get_sift_location(initMatches[i].first));
        pts_query.push_back(VilBaplSIFT::get_sift_location(initMatches[i].second));
    }
    vcl_vector<bool> inlier;
    vgl_h_matrix_2d<double> H;
    bool isFind = VrelPlus::homography_RANSAC(pts_keyframe, pts_query, inlier, H, homography_inlier_distance);    //Result vary from time to time
    if (!isFind) {
        return false;
    }
    assert(inlier.size() == pts_keyframe.size());
    for (int i = 0; i<inlier.size(); i++) {
        if (inlier[i]) {
            finalMatches.push_back(initMatches[i]);
        }
    }
    return finalMatches.size() >= 4;
}

void VxlFeatureMatch::mutualBestMatching(const vcl_vector<bapl_keypoint_sptr> & keypointsA,
                                            const vcl_vector<bapl_keypoint_sptr> & keypointsB,
                                            vcl_vector<vgl_point_2d<double> > & ptsA,
                                            vcl_vector<vgl_point_2d<double> > & ptsB, double ratio)
{
    // match from B to A
    bapl_bbf_tree treeA(keypointsA, 4);
    bapl_bbf_tree treeB(keypointsB, 4);
    vcl_vector<bapl_key_match> matchesB;
    for (unsigned int i = 0; i<keypointsB.size(); i++) {
        bapl_keypoint_sptr query = keypointsB[i];
        vcl_vector<bapl_keypoint_sptr> match;
        treeA.n_nearest(query, match, 2, -1);
        if (vnl_vector_ssd(query->descriptor(), match[0]->descriptor() ) <
            vnl_vector_ssd(query->descriptor(), match[1]->descriptor() ) * ratio) {
            bapl_key_match k_p(query, match[0]);
            
            //match from A to B
            bapl_keypoint_sptr query2 =  match[0];
            vcl_vector<bapl_keypoint_sptr> match2;
            treeB.n_nearest(query2, match2, 2, -1);
            if ((vnl_vector_ssd(query2->descriptor(), match2[0]->descriptor() ) <
                 vnl_vector_ssd(query2->descriptor(), match2[1]->descriptor() ) * ratio )
                && match2[0] == query) {
                matchesB.push_back(k_p);
            }
        }
    }
    
    for (int i = 0; i<matchesB.size(); i++) {
        bapl_lowe_keypoint_sptr sift1 = dynamic_cast<bapl_lowe_keypoint *>(matchesB[i].first.as_pointer());
        bapl_lowe_keypoint_sptr sift2 = dynamic_cast<bapl_lowe_keypoint *>(matchesB[i].second.as_pointer());
        
        vgl_point_2d<double> p1(sift1->location_i(), sift1->location_j());
        vgl_point_2d<double> p2(sift2->location_i(), sift2->location_j());
        ptsA.push_back(p1);
        ptsB.push_back(p2);
    }
}

class DataIndex
{
public:
    double val_;
    int index_;
    
    DataIndex(double v, int i)
    {
        val_ = v;
        index_ = i;
    }
    
    bool operator < (const DataIndex & other) const
    {
        return val_ < other.val_;
    }
    
};

/*
bool VxlFeatureMatch::epipolarMatch(const vcl_vector<bapl_keypoint_sptr> & leftKeypoints,
                                     const vcl_vector<bapl_keypoint_sptr> & rightKeypoints,
                                     const vpgl_perspective_camera<double> & cl,
                                     const vpgl_perspective_camera<double> & cr,
                                     vcl_vector<vcl_pair<int, int> > & matches,
                                     double min_pts_line_dis,
                                     double sift_ratio)
{
    // fundamental matrix
    vpgl_fundamental_matrix<double> fdm(cr, cl);
    
    vcl_vector<vgl_point_2d<double>> lpts;
    vcl_vector<vgl_point_2d<double>> rpts;
    VilBaplSIFT::getSIFTLocations(leftKeypoints, lpts);
    VilBaplSIFT::getSIFTLocations(rightKeypoints, rpts);
    
    // loop left key points
    for (int i = 0; i<lpts.size(); i++) {
        vgl_point_2d<double> lp = lpts[i];
        
        // obtain candidate points within a band near epipolar line
        vgl_homg_line_2d<double> r_epipolar_line = fdm.r_epipolar_line(vgl_homg_point_2d<double>(lp.x(), lp.y(), 1.0));
        vgl_line_2d<double> line(r_epipolar_line);
        vcl_vector<int> candinate_index;
        for (int j = 0; j<rpts.size(); j++) {
            double dis = vgl_distance(line, rpts[j]);
            if (dis < min_pts_line_dis) {
                candinate_index.push_back(j);
            }
        }
        //  printf("find %d candinate points.\n", (int)candinate_index.size());
        bool isMatched = false;
        if (candinate_index.size() >= 2) {
            // find two candidate with minumum ssd
            bapl_keypoint_sptr query = leftKeypoints[i];
            vcl_vector<DataIndex> ssds;
            for (int j = 0; j<candinate_index.size(); j++) {
                int idx = candinate_index[j];
                double ssd = vnl_vector_ssd(query->descriptor(), rightKeypoints[idx]->descriptor());
                DataIndex di(ssd, idx);
                ssds.push_back(di);
            }
            std::sort(ssds.begin(), ssds.end());
            assert(ssds.size() >= 2);
            if (ssds[0].val_ * sift_ratio < ssds[1].val_) {
                vcl_pair<int, int> match(i, ssds[0].index_);
                matches.push_back(match);
                isMatched = true;
            }
        }
        if (!isMatched) {
            vcl_pair<int, int> match(i, -1);
            matches.push_back(match);
        }
    }
    assert(matches.size() == leftKeypoints.size());
    return true;
}
*/

/*
bool VxlFeatureMatch::epipolarMatch(const vcl_vector<vgl_point_2d<double> > & ptsl,
                                     const vcl_vector<vgl_point_2d<double> > & ptsr,
                                     const vpgl_perspective_camera<double> & cl,
                                     const vpgl_perspective_camera<double> & cr,
                                     vcl_vector<bool> & isMatch,
                                     double min_pts_line_dis)
{
    assert(ptsl.size() == ptsr.size());
    
    vpgl_fundamental_matrix<double> fdm(cr, cl);
    
    // loop left key points
    isMatch.clear();
    for (int i = 0; i<ptsl.size(); i++) {
        vgl_point_2d<double> lp = ptsl[i];
        
        // obtain candidate points within a band near epipolar line
        vgl_homg_line_2d<double> r_epipolar_line = fdm.r_epipolar_line(vgl_homg_point_2d<double>(lp.x(), lp.y(), 1.0));
        vgl_line_2d<double> line(r_epipolar_line);
        double dis = vgl_distance(line, ptsr[i]);
        printf("distance to epipolar line is %f\n", dis);
        
        if (dis < min_pts_line_dis) {
            isMatch.push_back(true);
        }
        else
        {
            isMatch.push_back(false);
        }
    }
    return isMatch.size() == ptsl.size();
}
 */

static int getPatchIndex(const vgl_point_2d<double> & p, int patchSize, int imageW)
{
    int grid_w = imageW/patchSize;
    
    int px = p.x()/patchSize;
    int py = p.y()/patchSize;
    
    return py * grid_w + px;
}

vgl_h_matrix_2d<double> VxlFeatureMatch::geometryAwareSIFTMatch(const vcl_vector<bapl_keypoint_sptr> & leftKeypoints,
                                             const vcl_vector<bapl_keypoint_sptr> & rightKeypoints,
                                             const vgl_h_matrix_2d<double> & initH, int imageW, int imageH,
                                             double feature_distance_threshold,
                                             vcl_vector<bapl_key_match> & matches,
                                             bool ransac)
{
    // group features by location
    const int patchSize = 80;
    const double ratio = 0.6;  //  0.6
    const int grid_w = imageW/patchSize;
    const int grid_h = imageH/patchSize;
 
    vcl_vector<vcl_vector<bapl_keypoint_sptr> > featureVec(grid_h * grid_w);
    
    // put features into different grid
    for (int i = 0; i<rightKeypoints.size(); i++) {
        vgl_point_2d<double> p = VilBaplSIFT::get_sift_location(rightKeypoints[i]);
        int idx = getPatchIndex(p, patchSize, imageW);
        assert(idx < featureVec.size());
        featureVec[idx].push_back(rightKeypoints[i]);
    }
    
    // distable cout in bapl_bbf_tree construction function
    std::streambuf* cout_sbuf = std::cout.rdbuf(); // save original sbuf
    std::ofstream   fout("/dev/null");
    std::cout.rdbuf(fout.rdbuf()); // redirect 'cout' to a 'fout'
    
    // build multiple tree
    vcl_vector<bapl_bbf_tree *> grid_kd_tree;
    for (int i = 0; i<featureVec.size(); i++) {
        //  printf("data number is %d in tree\n", (int)featureVec[i].size());
        if (featureVec[i].size()  >= 2) {
            grid_kd_tree.push_back(new bapl_bbf_tree(featureVec[i], 4));
        }
        else
        {
            grid_kd_tree.push_back(NULL);
        }
    }
    std::cout.rdbuf(cout_sbuf); // restore the original stream buffer
    
    vcl_vector<bapl_key_match> initMatch;
    for (unsigned int i = 0; i<leftKeypoints.size(); i++) {
        bapl_keypoint_sptr      query = leftKeypoints[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        vgl_point_2d<double> p = VilBaplSIFT::get_sift_location(leftKeypoints[i]);
        vgl_point_2d<double> q = initH(vgl_homg_point_2d<double>(p));
        if (q.x() >= 0 && q.x() < imageW && q.y() >= 0 && q.y() < imageH) {
            int idx = getPatchIndex(q, patchSize, imageW);
            assert(idx < grid_kd_tree.size());
            if (grid_kd_tree[idx] != NULL && grid_kd_tree[idx]) {
                
                grid_kd_tree[idx]->n_nearest(query, match, 2, -1);
                double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
                double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
                bapl_key_match k_p(query, match[0]);
                if (ssd0 < feature_distance_threshold && ssd0  < ssd1 * ratio) {
                    initMatch.push_back(k_p);
                }
            }
        }
    }
    
    // release memory
    for (int i = 0; i<grid_kd_tree.size(); i++) {
        if (grid_kd_tree[i]) {
            delete grid_kd_tree[i];
            grid_kd_tree[i] = NULL;
        }
    }
    
    printf("init matched number is %d\n", (int)initMatch.size());
    
    vgl_h_matrix_2d<double> H_keyframeToQuery;
    if (ransac) {
        vcl_vector<vgl_point_2d<double> > pts_keyframe;
        vcl_vector<vgl_point_2d<double> > pts_query;
        for (int i = 0; i<initMatch.size(); i++) {
            pts_keyframe.push_back(VilBaplSIFT::get_sift_location(initMatch[i].first));
            pts_query.push_back(VilBaplSIFT::get_sift_location(initMatch[i].second));
        }
        vcl_vector<bool> inlier;
        H_keyframeToQuery = VrelPlus::homography_RANSAC(pts_keyframe, pts_query, inlier, 2.0);    //Result vary from time to time
        assert(inlier.size() == pts_keyframe.size());
        for (int i = 0; i<inlier.size(); i++) {
            if (inlier[i]) {
                matches.push_back(initMatch[i]);
            }
        }        
    }
    else
    {
        vcl_vector<vgl_homg_point_2d<double> > pts_keyframe;
        vcl_vector<vgl_homg_point_2d<double> > pts_query;
        for (int i = 0; i<initMatch.size(); i++) {
            pts_keyframe.push_back(vgl_homg_point_2d<double>(VilBaplSIFT::get_sift_location(initMatch[i].first)));
            pts_query.push_back(vgl_homg_point_2d<double>(VilBaplSIFT::get_sift_location(initMatch[i].second)));
        }
        H_keyframeToQuery = vgl_h_matrix_2d<double>(pts_keyframe, pts_keyframe);
        matches = initMatch;
    }
    printf("geometry aware matched number is %d\n\n", (int)matches.size());
    return H_keyframeToQuery;    
}
bool VxlFeatureMatch::geometryAwareSIFTMatch(const vcl_vector<bapl_keypoint_sptr> & leftKeypoints,
                                             const vcl_vector<bapl_keypoint_sptr> & rightKeypoints,
                                             const vgl_h_matrix_2d<double> & initH, int imageW, int imageH,
                                             double feature_distance_threshold,
                                             vgl_h_matrix_2d<double> & finalH,
                                             vcl_vector<bapl_key_match> & matches, bool ransac)
{
    // group features by location
    const int patchSize = 40;
    const double ratio = 0.6;
    const int grid_w = imageW/patchSize;
    const int grid_h = imageH/patchSize;
    
    vcl_vector<vcl_vector<bapl_keypoint_sptr> > featureVec(grid_h * grid_w);
    
    // put features into different grid
    for (int i = 0; i<rightKeypoints.size(); i++) {
        vgl_point_2d<double> p = VilBaplSIFT::get_sift_location(rightKeypoints[i]);
        int idx = getPatchIndex(p, patchSize, imageW);
        assert(idx < featureVec.size());
        featureVec[idx].push_back(rightKeypoints[i]);
    }
    
    // distable cout in bapl_bbf_tree construction function
    std::streambuf* cout_sbuf = std::cout.rdbuf(); // save original sbuf
    std::ofstream   fout("/dev/null");
    std::cout.rdbuf(fout.rdbuf()); // redirect 'cout' to a 'fout'
    
    // build multiple tree
    vcl_vector<bapl_bbf_tree *> grid_kd_tree;
    for (int i = 0; i<featureVec.size(); i++) {
        //  printf("data number is %d in tree\n", (int)featureVec[i].size());
        if (featureVec[i].size()  >= 2) {
            grid_kd_tree.push_back(new bapl_bbf_tree(featureVec[i], 4));
        }
        else
        {
            grid_kd_tree.push_back(NULL);
        }
    }
    std::cout.rdbuf(cout_sbuf); // restore the original stream buffer
    
    vcl_vector<bapl_key_match> initMatch;
    for (unsigned int i = 0; i<leftKeypoints.size(); i++) {
        bapl_keypoint_sptr      query = leftKeypoints[i];
        vcl_vector<bapl_keypoint_sptr> match;
        
        vgl_point_2d<double> p = VilBaplSIFT::get_sift_location(leftKeypoints[i]);
        vgl_point_2d<double> q = initH(vgl_homg_point_2d<double>(p));
        if (q.x() >= 0 && q.x() < imageW && q.y() >= 0 && q.y() < imageH) {
            int idx = getPatchIndex(q, patchSize, imageW);
            assert(idx < grid_kd_tree.size());
            if (grid_kd_tree[idx] != NULL && grid_kd_tree[idx]) {
                
                grid_kd_tree[idx]->n_nearest(query, match, 2, -1);
                double ssd0 = vnl_vector_ssd(query->descriptor(), match[0]->descriptor());
                double ssd1 = vnl_vector_ssd(query->descriptor(), match[1]->descriptor());
                bapl_key_match k_p(query, match[0]);
                if (ssd0 < feature_distance_threshold && ssd0  < ssd1 * ratio) {
                    initMatch.push_back(k_p);
                }
            }
        }
    }
    
    // release memory
    for (int i = 0; i<grid_kd_tree.size(); i++) {
        if (grid_kd_tree[i]) {
            delete grid_kd_tree[i];
            grid_kd_tree[i] = NULL;
        }
    }
    
    printf("init matched number is %d\n", (int)initMatch.size());
    if (initMatch.size() < 10) {
        return false;
    }
    
    if (ransac) {
        vcl_vector<vgl_point_2d<double> > pts_keyframe;
        vcl_vector<vgl_point_2d<double> > pts_query;
        for (int i = 0; i<initMatch.size(); i++) {
            pts_keyframe.push_back(VilBaplSIFT::get_sift_location(initMatch[i].first));
            pts_query.push_back(VilBaplSIFT::get_sift_location(initMatch[i].second));
        }
        vcl_vector<bool> inlier;
        bool isFind = VrelPlus::homography_RANSAC(pts_keyframe, pts_query, inlier, finalH, 2.0);    //Result vary from time to time
        if (!isFind) {
            return false;
        }
        assert(inlier.size() == pts_keyframe.size());
        for (int i = 0; i<inlier.size(); i++) {
            if (inlier[i]) {
                matches.push_back(initMatch[i]);
            }
        }
    }
    else
    {
        vcl_vector<vgl_homg_point_2d<double> > pts_keyframe;
        vcl_vector<vgl_homg_point_2d<double> > pts_query;
        for (int i = 0; i<initMatch.size(); i++) {
            pts_keyframe.push_back(vgl_homg_point_2d<double>(VilBaplSIFT::get_sift_location(initMatch[i].first)));
            pts_query.push_back(vgl_homg_point_2d<double>(VilBaplSIFT::get_sift_location(initMatch[i].second)));
        }
        finalH = vgl_h_matrix_2d<double>(pts_keyframe, pts_keyframe);
        matches = initMatch;
    }
    printf("geometry aware matched number is %d\n\n", (int)matches.size());
    
    return true;
}

bool VxlFeatureMatch::KNNBagOfFeatureMatch(const vcl_vector<bapl_keypoint_sptr> & bagOfFeatures,
                                           const vcl_vector<bapl_keypoint_sptr> & queryFeatures,
                                           int nK, vgl_h_matrix_2d<double> & finalH,
                                           vcl_vector<bapl_key_match> & matches, bool ransac)
{
    bapl_bbf_tree tree(bagOfFeatures, 4);
    
    vcl_vector<bapl_key_match> initMatch;
    for (unsigned int i = 0; i<queryFeatures.size(); i++) {
        bapl_keypoint_sptr query = queryFeatures[i];
        vcl_vector<bapl_keypoint_sptr> match;
        tree.n_nearest(query, match, nK, -1);
        
        for (int k = 0; k<nK; k++) {
            vgl_point_2d<double> p = VilBaplSIFT::get_sift_location(match[k]);
         //   printf("feature position is %f %f\n", p.x(), p.y());
        }
        
        // simple check, if the first 3 match is very similar, then they were regarded as constant match
        int matchNum = 0;
        vgl_point_2d<double> p0 = VilBaplSIFT::get_sift_location(match[0]);
        for (int k = 1; k<nK; k++) {
            vgl_point_2d<double> p = VilBaplSIFT::get_sift_location(match[k]);
            double dx = p0.x() - p.x();
            double dy = p0.y() - p.y();
            if (dx * dx + dy * dy < 25) {
                matchNum++;
            }
         //   printf("feature position is %f %f\n", p.x(), p.y());
        }
        if (matchNum >= 4) {
            bapl_key_match k_p(match[0], query);
            initMatch.push_back(k_p);
        }
      //  printf("\n");
    }
    
    printf("init match number is %lu\n", initMatch.size());
    
    if (initMatch.size() < 10) {
        return false;
    }
    
    vcl_vector<vgl_point_2d<double> > pts_keyframe;
    vcl_vector<vgl_point_2d<double> > pts_query;
    for (int i = 0; i<initMatch.size(); i++) {
        pts_keyframe.push_back(VilBaplSIFT::get_sift_location(initMatch[i].first));
        pts_query.push_back(VilBaplSIFT::get_sift_location(initMatch[i].second));
    }
    vcl_vector<bool> inlier;
    bool isFind = VrelPlus::homography_RANSAC(pts_keyframe, pts_query, inlier, finalH, 2.0);    //Result vary from time to time
    if (!isFind) {
        return false;
    }
    assert(inlier.size() == pts_keyframe.size());
    for (int i = 0; i<inlier.size(); i++) {
        if (inlier[i]) {
            matches.push_back(initMatch[i]);
        }
    }
    printf("final match number is %lu\n", matches.size());
    
    return true;
}


