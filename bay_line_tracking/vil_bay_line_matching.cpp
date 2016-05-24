//
//  vil_bay_line_matching.cpp
//  QuadCopter
//
//  Created by jimmy on 6/16/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_bay_line_matching.h"
#include <vil/vil_convert.h>
#include <vgl/vgl_distance.h>
#include "vgl_plus.h"
#include "vil_plus.h"
#include <vcl_queue.h>
#include <vcl_list.h>

void VilBayLineMatching::generateLineAppearance(const vil_image_view<vxl_byte> & image,
                                    const vcl_vector<LSDLineSegment> & line_segments,
                                    double threshold_length, int color_profile_width,
                                    vcl_vector<LineAppearance> & line_seg_appearances)
{
    assert(image.nplanes() == 3);
    assert(color_profile_width >= 5);
    
    const int w = image.ni();
    const int h = image.nj();
    vil_image_view<vxl_byte> grayImage;
    vil_convert_planes_to_grey(image, grayImage);
    
    vil_image_view<double> hsvImage;
    VilPlus::vil_rgb_to_hsv(image, hsvImage);
    
    vnl_vector<int> bin(166);
    // loop all line segment cadidate
    for (int i = 0; i<line_segments.size(); i++) {
        vgl_point_2d<double> p1 = line_segments[i].seg_.point1();
        vgl_point_2d<double> p2 = line_segments[i].seg_.point2();
        double dis = vgl_distance(p1, p2);
        if (dis >= threshold_length) {
            // parallel move the line segment
            double drift_dis = line_segments[i].width_/2.0 + color_profile_width/2.0;  // not close to the edge area
            vgl_line_segment_2d<double> seg1;
            vgl_line_segment_2d<double> seg2;
            VglPlus::parallelMove(line_segments[i].seg_, drift_dis, seg1, seg2);
            
            // pick up pixel position in the color profile area ( rectangular-like area)
            vcl_vector<vgl_point_2d<double> > pixels1;
            vcl_vector<vgl_point_2d<double> > pixels2;
            VilPlus::draw_line_segment(seg1, color_profile_width/2, w, h, pixels1);
            VilPlus::draw_line_segment(seg2, color_profile_width/2, w, h, pixels2);
            
            if (pixels1.size() < 50 || pixels2.size() < 50) {
                printf("Warning: do not have enough pixel number to calculate histogram!\n");
                printf("pixel number is %lu, %lu\n", pixels1.size(), pixels2.size());
                continue;
            }
            // only for test
            if (0)
            {
                vil_image_view<vxl_byte> showImage;
                showImage.deep_copy(image);
                VilPlus::draw_arrow(showImage, line_segments[i].seg_.point1(), line_segments[i].seg_.point2(), VilPlus::green());
                
                VilPlus::draw_arrow(showImage, seg1.point1(), seg1.point2(), VilPlus::red());
                VilPlus::draw_arrow(showImage, seg2.point1(), seg2.point2(), VilPlus::blue());
                
                //
                for (int j = 0; j<pixels1.size(); j++) {
                    int x = pixels1[j].x();
                    int y = pixels1[j].y();
                    showImage(x, y, 0) = 255;
                }
                for (int j = 0; j<pixels2.size(); j++) {
                    int x = pixels2[j].x();
                    int y = pixels2[j].y();
                    showImage(x, y, 2) = 255;
                }
                
                char buf[1024] = {NULL};
                sprintf(buf, "segment_sides_%d.jpg", rand()%200);
                VilPlus::vil_save(showImage, buf);
            }
            
            // generate histogram
            // 18, 3, 3, bins for HSV, 4 for gray level
            // H: [0, 360)
            // S: [0, 1];
            // V: [0, 255]
            LineAppearance la;
            int bright_left = 0;
            int bright_right = 0;
            bin.fill(0);
            {
                for (int j = 0; j<pixels1.size(); j++) {
                    int x = pixels1[j].x();
                    int y = pixels1[j].y();
                    int h = hsvImage(x, y, 0);
                    int s = hsvImage(x, y, 1) * 3; // 0.x --> 0 1 2
                    int v = hsvImage(x, y, 2);
                    int h_idx = h/20;
                    int s_idx = s;
                    int v_idx = v/(255/3+1);
                    if (h_idx == 18) {
                        h_idx = 17;
                    }
                    if (s_idx == 3) {
                        s_idx = 2;
                    }
                    
                    assert(h_idx >= 0 && h_idx < 18);
                    assert(s_idx >= 0 && s_idx < 3);
                    assert(v_idx >= 0 && v_idx < 3);
                    int bin_idx = h_idx * 3 * 3 + s_idx * 3 + v_idx;
                    assert(bin_idx >= 0 && bin_idx < 162);
                    bin[bin_idx]++;
                    
                    // gray scale
                    int gray = grayImage(x, y, 0);
                    int gray_idx = gray/((255/4) + 1);
                    assert(gray_idx >= 0 && gray_idx < 4);
                    bin[18 * 3 * 3 + gray_idx]++;
                    bright_left += gray;
                }
                
                //
                la.left_bin_ = bin; // assume it is the left, will switch left/right later
                la.right_pixel_num_ = (int)pixels1.size();
                bright_left /= pixels1.size();
            }
            
            // right
            bin.fill(0);
            {
                for (int j = 0; j<pixels2.size(); j++) {
                    int x = pixels2[j].x();
                    int y = pixels2[j].y();
                    int h = hsvImage(x, y, 0);
                    int s = hsvImage(x, y, 1) * 3; // 0 1 2
                    int v = hsvImage(x, y, 2);
                    int h_idx = h/20;
                    int s_idx = s;
                    int v_idx = v/(255/3+1);
                    if (h_idx == 18) {
                        h_idx = 17;
                    }
                    if (s_idx == 3) {
                        s_idx = 2;
                    }
                    assert(h_idx >= 0 && h_idx < 18);
                    assert(s_idx >= 0 && s_idx < 3);
                    assert(v_idx >= 0 && v_idx < 3);
                    int bin_idx = h_idx * 3 * 3 + s_idx * 3 + v_idx;
                    assert(bin_idx >= 0 && bin_idx < 162);
                    bin[bin_idx]++;
                    
                    // gray scale
                    int gray = grayImage(x, y, 0);
                    int gray_idx = gray/((255/4) + 1);
                    assert(gray_idx >= 0 && gray_idx < 4);
                    bin[18 * 3 * 3 + gray_idx]++;
                    bright_right += gray;
                }
                
                la.right_bin_ = bin;
                la.right_pixel_num_ = (int)pixels2.size();
                bright_right /= pixels2.size();
            }
            
            // make sure the bright side is in the left side of the line segment
            if (bright_right > bright_left) {
                vcl_swap(la.left_bin_, la.right_bin_);
                vcl_swap(la.left_pixel_num_, la.right_pixel_num_);
                la.seg_ = vgl_line_segment_2d<double>(p1, p2);
            }
            else
            {
                la.seg_ = vgl_line_segment_2d<double>(p2, p1);
            }
            la.getNormBin();
            line_seg_appearances.push_back(la);
        }
    }
    printf("calculate %lu line segment appearances.\n", line_seg_appearances.size());
}

struct ColorDistance
{
    double dis_;
    int idx_;    // index in line segment group
    
    ColorDistance(double dis, int idx)
    {
        dis_ = dis;
        idx_ = idx;
    }
    
    bool operator < (const ColorDistance & other) const
    {
        return dis_ < other.dis_;
    }
};

struct ViolationScore
{
    double score_;
    int idx_;     // index in line segment group
    
    ViolationScore(double sc, int idx)
    {
        score_ = sc;
        idx_ = idx;
    }
    
    bool operator < (const ViolationScore & other) const
    {
        return score_ < other.score_;
    }
};


struct BayLineMatch
{
    int idx1_;   // line index in view 1
    int idx2_;   // line index in view 2
    double appearance_dissimilarity_;  // histogram distance, the small the better
    double violation_score_;           // [0, 1.0], the small the better
    
    BayLineMatch()
    {
        idx1_ = 0;
        idx2_ = 0;
        appearance_dissimilarity_ = 0;
        violation_score_ = 0;
    }
    
    double dismatch_score(void) const
    {
        return appearance_dissimilarity_ + violation_score_;
    }
    
    bool operator < (const BayLineMatch & other) const
    {
        return idx1_ < other.idx1_;
    }
};

bool isHighViolationScore( const BayLineMatch & m)
{
    return m.violation_score_ > 0.15;
}

// matches: input, output matched pairs
// assume matches is ordered
static void pair_feature_filter(const vcl_vector<LineAppearance> & line_seg1, const vcl_vector<LineAppearance> & line_seg2,
                                vcl_list<BayLineMatch> & matches, const double violation_threshold = 0.15)
{
    // second stage, pair sidedness constraint
    while (1) {
        // remove the largest viotation matches once a time
        vcl_list<BayLineMatch>::iterator ite_max = matches.begin();
        double violation_max = 0.0;
        for (vcl_list<BayLineMatch>::iterator ite = matches.begin(); ite != matches.end(); ite++) {
            // too matched line segment
            vgl_line_segment_2d<double> seg1 = line_seg1[ite->idx1_].seg_;
            vgl_line_segment_2d<double> seg2 = line_seg2[ite->idx2_].seg_;
            //loop over all other matches
            double violation = 0.0;
            double total = 0.001; // prevent it will be 0.0
            for (vcl_list<BayLineMatch>::iterator jte = matches.begin(); jte != matches.end(); jte++) {
                if (ite->idx1_ == jte->idx1_ && ite->idx2_ == jte->idx2_) {
                    continue;
                }
                // another pair of matches
                vgl_line_segment_2d<double> seg3 = line_seg1[jte->idx1_].seg_;
                vgl_line_segment_2d<double> seg4 = line_seg2[jte->idx2_].seg_;
                vgl_point_2d<double> c1 = centre(seg3.point1(), seg3.point2());
                vgl_point_2d<double> c2 = centre(seg4.point1(), seg4.point2());
                if (VglPlus::isLeftSide(seg1, c1) != VglPlus::isLeftSide(seg2, c2)) {
                    violation += 1.0;
                }
                total += 1.0;
            }
            ite->violation_score_ = violation/total;
            if (ite->violation_score_ > violation_max) {
                violation_max = ite->violation_score_;
                ite_max = ite;
            }
        }
        
        if (violation_max > violation_threshold) {
            matches.erase(ite_max);
        }
        else
        {
            break;
        }
    }
    
    // remove one --> multiple soft matches, only left one match
    for (vcl_list<BayLineMatch>::iterator ite = matches.begin(); ite != matches.end();) {
        vcl_list<BayLineMatch>::iterator cur = ite;
        vcl_list<BayLineMatch>::iterator next = ++ite;
        if (next == matches.end()) {
            break;
        }
        if (cur->idx1_ != next->idx1_) {
            // do nothing
        }
        else
        {
            // find the one with the lowest "appearance dissimilarity + violation score
            if (cur->dismatch_score() < next->dismatch_score()) {
                next->violation_score_ = 1.0; // raise violation score to make it removed
                vcl_swap(cur, next);  // make the small one to the end, so that it can be compared to the next
            }
            else
            {
                cur->violation_score_ = 1.0;
            }
        }
        
    }
    matches.remove_if(isHighViolationScore);
    
}

void VilBayLineMatching::match_lines(const vcl_vector<LineAppearance> & line_seg1, const vcl_vector<LineAppearance> & line_seg2,
                                     vcl_vector<vcl_pair<int, int> > & matchIndices, const BayLineMatchingParameter & para)
{
    const double distance_threshold  = para.distance_threshold_;
    const double violation_threshold = para.violation_threshold_;
    
    // initial matching, keep 3 lowest
    vcl_list<BayLineMatch> initial_matches;  // matches by color histogram --> violation score
    for (int i = 0; i<line_seg1.size(); i++) {
        vcl_priority_queue<ColorDistance> maxHeap;
        for (int j = 0; j<line_seg2.size(); j++) {
            double dis = line_seg1[i].distance(line_seg2[j]);
            if (dis > distance_threshold) {
                continue;
            }
            if (maxHeap.size() < 3) {
                maxHeap.push(ColorDistance(dis, j));
            }
            else
            {
                ColorDistance top = maxHeap.top();
                if (top.dis_ > dis) {
                    maxHeap.pop();
                    maxHeap.push(ColorDistance(dis, j));
                }
            }
        }
        
        while (!maxHeap.empty()) {
            ColorDistance cd = maxHeap.top();
            maxHeap.pop();
            
            BayLineMatch m;
            m.idx1_ = i;
            m.idx2_ = cd.idx_;
            m.appearance_dissimilarity_ = cd.dis_;
            initial_matches.push_back(m);
        }
    }
    
    // second stage, pair sidedness constraint
    while (1) {
        // remove the largest viotation matches once a time
        vcl_list<BayLineMatch>::iterator ite_max = initial_matches.begin();
        double violation_max = 0.0;
        for (vcl_list<BayLineMatch>::iterator ite = initial_matches.begin(); ite != initial_matches.end(); ite++) {
            // too matched line segment
            vgl_line_segment_2d<double> seg1 = line_seg1[ite->idx1_].seg_;
            vgl_line_segment_2d<double> seg2 = line_seg2[ite->idx2_].seg_;
            //loop over all other matches
            double violation = 0.0;
            double total = 0.001; // prevent it will be 0.0
            for (vcl_list<BayLineMatch>::iterator jte = initial_matches.begin(); jte != initial_matches.end(); jte++) {
                if (ite->idx1_ == jte->idx1_ && ite->idx2_ == jte->idx2_) {
                    continue;
                }
                // another pair of matches
                vgl_line_segment_2d<double> seg3 = line_seg1[jte->idx1_].seg_;
                vgl_line_segment_2d<double> seg4 = line_seg2[jte->idx2_].seg_;
                vgl_point_2d<double> c1 = centre(seg3.point1(), seg3.point2());
                vgl_point_2d<double> c2 = centre(seg4.point1(), seg4.point2());
                if (VglPlus::isLeftSide(seg1, c1) != VglPlus::isLeftSide(seg2, c2)) {
                    violation += 1.0;
                }
                total += 1.0;
            }
            ite->violation_score_ = violation/total;
            if (ite->violation_score_ > violation_max) {
                violation_max = ite->violation_score_;
                ite_max = ite;
            }
        }
        
        if (violation_max > violation_threshold) {
            initial_matches.erase(ite_max);
        }
        else
        {
            break;
        }
    }
    
    // remove one --> multiple soft matches, only left one match
    for (vcl_list<BayLineMatch>::iterator ite = initial_matches.begin(); ite != initial_matches.end();) {
        vcl_list<BayLineMatch>::iterator cur = ite;
        vcl_list<BayLineMatch>::iterator next = ++ite;
        if (next == initial_matches.end()) {
            break;
        }
        if (cur->idx1_ != next->idx1_) {
            // do nothing
        }
        else
        {
            // find the one with the lowest "appearance dissimilarity + violation score
            if (cur->dismatch_score() < next->dismatch_score()) {
                next->violation_score_ = 1.0; // raise violation score to make it removed
                vcl_swap(cur, next);  // make the small one to the end, so that it can be compared to the next
            }
            else
            {
                cur->violation_score_ = 1.0;
            }
        }
        
    }
    initial_matches.remove_if(isHighViolationScore);
    
    if (para.find_more_mathes_) {
        // finding more matches
        vcl_list<BayLineMatch> reference_matches = initial_matches;
        vcl_vector<bool> matchedLines(line_seg1.size(), false);
        for (vcl_list<BayLineMatch>::iterator ite = reference_matches.begin(); ite != reference_matches.end(); ite++) {
            int idx = ite->idx1_;
            assert(idx >= 0 && idx < line_seg1.size());
            matchedLines[idx] = true;
        }
        
        vcl_list<BayLineMatch> waiting_room_matches;
        for (int i = 0; i<line_seg1.size(); i++) {
            if (matchedLines[i]) {
                continue;
            }
            // compare with all other line segment group 2 and keep three lowest violation score
            vcl_priority_queue<ViolationScore> maxHeap;
            for (int j = 0; j<line_seg2.size(); j++) {
                double dis = line_seg1[i].distance(line_seg2[j]);
                if (dis > distance_threshold) {
                    continue;
                }
                // calculate violation score by reference matches
                vgl_line_segment_2d<double> seg1 = line_seg1[i].seg_;
                vgl_line_segment_2d<double> seg2 = line_seg2[j].seg_;
                double violation = 0.0;
                double total = 0.001;
                for (vcl_list<BayLineMatch>::iterator ite = reference_matches.begin(); ite != reference_matches.end(); ite++) {
                    vgl_line_segment_2d<double> seg3 = line_seg1[ite->idx1_].seg_;
                    vgl_line_segment_2d<double> seg4 = line_seg2[ite->idx2_].seg_;
                    vgl_point_2d<double> c1 = centre(seg3.point1(), seg3.point2());
                    vgl_point_2d<double> c2 = centre(seg4.point1(), seg4.point2());
                    if (VglPlus::isLeftSide(seg1, c1) != VglPlus::isLeftSide(seg2, c2)) {
                        violation += 1.0;
                    }
                    total += 1.0;
                }
                
                violation /= total;
                ViolationScore vs(violation, j);
                if (maxHeap.size() < 3) {
                    maxHeap.push(vs);
                }
                else
                {
                    ViolationScore top = maxHeap.top();
                    if (top.score_ > violation) {
                        maxHeap.pop();
                        maxHeap.push(vs);
                    }
                }
            }
            // mostly the size of maxHeap will be 3
            while (!maxHeap.empty()) {
                ViolationScore top = maxHeap.top();
                maxHeap.pop();
                
                BayLineMatch m;
                m.idx1_ = i;
                m.idx2_ = top.idx_;
                m.appearance_dissimilarity_ = line_seg1[i].distance(line_seg2[top.idx_]);
                waiting_room_matches.push_back(m);
            }
        }
        
        // combine reference matches and matches in waiting room
        reference_matches.insert(reference_matches.end(), waiting_room_matches.begin(), waiting_room_matches.end());
        reference_matches.sort();
        
        pair_feature_filter(line_seg1, line_seg2, reference_matches, violation_threshold);
        
        printf("size before/after adding matches: %lu %lu\n", initial_matches.size(), reference_matches.size());
        
        initial_matches = reference_matches;
    }
    
    // epipolar geometry estimation @todo
    for (vcl_list<BayLineMatch>::iterator ite = initial_matches.begin(); ite != initial_matches.end(); ite++) {
        matchIndices.push_back(vcl_pair<int, int>(ite->idx1_, ite->idx2_));
    }
}
