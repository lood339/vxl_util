//
//  vil_fast_feature.cpp
//  VpglPtzOpt
//
//  Created by Jimmy Chen LOCAL on 3/13/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vil_fast_feature.h"

void vil_fast_feature_point(const vil_image_view<vxl_byte> &gray_image,
                                     vcl_vector<vgl_point_2d<double> > &positive,
                                     vcl_vector<vgl_point_2d<double> > &negative,
                                     int pixl_num_threshold,
                                     int intensity_threshold)
{
    assert(gray_image.nplanes() == 1);
    
    const int w = gray_image.ni();
    const int h = gray_image.nj();
    int shift [16][2] = {
        0, -3,
        1, -3,
        2, -2,
        3, -1,
        3, 0,
        3, 1,
        2, 2,
        1, 3,
        0, 3,
        -1, 3,
        -2, 2,
        -3, 1,
        -3, 0,
        -3, -1,
        -2, -2,
        -1, -3
    };
    
    const int board_size = 3;
    for (int y = board_size; y < h - board_size; y++) {
        for (int x = board_size; x< w - board_size; x++) {
            int val = gray_image(x, y, 0);
            int v1  = gray_image(x + shift[0][0],  y  + shift[0][1]);
            int v5  = gray_image(x + shift[4][0],  y  + shift[4][1]);
            int v9  = gray_image(x + shift[8][0],  y  + shift[8][1]);
            int v13 = gray_image(x + shift[12][0], y  + shift[12][1]);
            int pos_num = 0;
            int neg_num = 0;
            const int positive_threshold = val + intensity_threshold;
            const int negtivee_threshold = val - intensity_threshold;
            
            if (v1 > positive_threshold) {
                pos_num++;
            }
            else if(v1 < negtivee_threshold){
                neg_num++;
            }
            
            if (v5 > positive_threshold) {
                pos_num++;
            }
            else if(v5 < negtivee_threshold){
                neg_num++;
            }
            
            if (v9 > positive_threshold) {
                pos_num++;
            }
            else if(v9 < negtivee_threshold){
                neg_num++;
            }
            
            if (v13 > positive_threshold) {
                pos_num++;
            }
            else if(v13 < negtivee_threshold){
                neg_num++;
            }
            
            // fast rejection of continues 12
            if (pos_num < 3 && neg_num < 3) {
                continue;
            }
            if (pos_num >= 3) {
                if (v1 > positive_threshold)
                {
                    int num = 0;
                    //forward
                    for (int i = 0; i<16; i++) {
                        if (gray_image(x + shift[i][0], y + shift[i][1]) > positive_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (num >= pixl_num_threshold) {
                        positive.push_back(vgl_point_2d<double>(x, y));
                    }
                }
                else if (v5 > positive_threshold)
                {
                    int idx = 4;
                    int num = 0;
                    //forward
                    for (int i = idx; i<16; i++) {
                        if (gray_image(x + shift[i][0], y + shift[i][1]) > positive_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    //backward
                    for (int i = idx - 1; i >= 0; i--) {
                        if (gray_image(x + shift[i][0], y + shift[i][1]) > positive_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (num >= pixl_num_threshold) {
                        positive.push_back(vgl_point_2d<double>(x, y));
                    }
                }
            }
            else if (neg_num >= 3)
            {
                if(v1 < negtivee_threshold)
                {
                    int num = 0;
                    for (int i = 0; i<16; i++) {
                        if (gray_image(x + shift[i][0], y + shift[i][1]) < negtivee_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (num >= pixl_num_threshold) {
                        negative.push_back(vgl_point_2d<double>(x, y));
                    }
                }
                else if(v5 < negtivee_threshold)
                {
                    int idx = 4;
                    int num = 0;
                    for (int i = idx; i<16; i++) {
                        if (gray_image(x + shift[i][0], y + shift[i][1]) < negtivee_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    for (int i = idx -1; i >= 0; i--) {
                        if (gray_image(x + shift[i][0], y + shift[i][1]) < negtivee_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (num >= pixl_num_threshold) {
                        negative.push_back(vgl_point_2d<double>(x, y));
                    }
                }
            }
        }
    }
    
    vcl_cout<<"positive feature number "<<positive.size()<<vcl_endl;
    vcl_cout<<"negative feature number "<<negative.size()<<vcl_endl;
}

void vil_fast_feature(const vil_image_view<vxl_byte> &grey_image,
                               vcl_vector<FAST_feature> & positive,
                               vcl_vector<FAST_feature> & negative,
                               int pixel_num_threshold,
                               int intensity_threshold)
{
    assert(grey_image.nplanes() == 1);
    
    const int w = grey_image.ni();
    const int h = grey_image.nj();
    int shift [16][2] = {
        0, -3,
        1, -3,
        2, -2,
        3, -1,
        3, 0,
        3, 1,
        2, 2,
        1, 3,
        0, 3,
        -1, 3,
        -2, 2,
        -3, 1,
        -3, 0,
        -3, -1,
        -2, -2,
        -1, -3
    };
    
    const int board_size = 3;
    for (int y = board_size; y < h - board_size; y++) {
        for (int x = board_size; x< w - board_size; x++) {
            int val = grey_image(x, y, 0);
            int v1  = grey_image(x + shift[0][0],  y  + shift[0][1]);
            int v5  = grey_image(x + shift[4][0],  y  + shift[4][1]);
            int v9  = grey_image(x + shift[8][0],  y  + shift[8][1]);
            int v13 = grey_image(x + shift[12][0], y  + shift[12][1]);
            int pos_num = 0;
            int neg_num = 0;
            const int positive_threshold = val + intensity_threshold;
            const int negtivee_threshold = val - intensity_threshold;
            
            if (v1 > positive_threshold) {
                pos_num++;
            }
            else if(v1 < negtivee_threshold){
                neg_num++;
            }
            
            if (v5 > positive_threshold) {
                pos_num++;
            }
            else if(v5 < negtivee_threshold){
                neg_num++;
            }
            
            if (v9 > positive_threshold) {
                pos_num++;
            }
            else if(v9 < negtivee_threshold){
                neg_num++;
            }
            
            if (v13 > positive_threshold) {
                pos_num++;
            }
            else if(v13 < negtivee_threshold){
                neg_num++;
            }
            
            // fast rejection of continues 12
            if (pos_num < 3 && neg_num < 3) {
                continue;
            }
            if (pos_num >= 3) {
                if (v1 > positive_threshold)
                {
                    int num = 0;
                    //forward
                    for (int i = 0; i<16; i++) {
                        if (grey_image(x + shift[i][0], y + shift[i][1]) > positive_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (num >= pixel_num_threshold) {
                        //collect feature description
                        FAST_feature fast;
                        fast.position = vgl_point_2d<double>(x, y);
                        for (int k = 0; k<16; k++) {
                            fast.description[k] = grey_image(x + shift[k][0], y + shift[k][1]);
                        }
                        positive.push_back(fast);
                    }
                }
                else if (v5 > positive_threshold)
                {
                    int idx = 4;
                    int num = 0;
                    //forward
                    for (int i = idx; i<16; i++) {
                        if (grey_image(x + shift[i][0], y + shift[i][1]) > positive_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    //backward
                    for (int i = idx - 1; i >= 0; i--) {
                        if (grey_image(x + shift[i][0], y + shift[i][1]) > positive_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (num >= pixel_num_threshold) {
                        FAST_feature fast;
                        fast.position = vgl_point_2d<double>(x, y);
                        for (int k = 0; k<16; k++) {
                            fast.description[k] = grey_image(x + shift[k][0], y + shift[k][1]);
                        }
                        positive.push_back(fast);
                    }
                }
            }
            else if (neg_num >= 3)
            {
                if(v1 < negtivee_threshold)
                {
                    int num = 0;
                    for (int i = 0; i<16; i++) {
                        if (grey_image(x + shift[i][0], y + shift[i][1]) < negtivee_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (num >= pixel_num_threshold) {
                        FAST_feature fast;
                        fast.position = vgl_point_2d<double>(x, y);
                        for (int k = 0; k<16; k++) {
                            fast.description[k] = grey_image(x + shift[k][0], y + shift[k][1]);
                        }
                        negative.push_back(fast);
                    }
                }
                else if(v5 < negtivee_threshold)
                {
                    int idx = 4;
                    int num = 0;
                    for (int i = idx; i<16; i++) {
                        if (grey_image(x + shift[i][0], y + shift[i][1]) < negtivee_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    for (int i = idx -1; i >= 0; i--) {
                        if (grey_image(x + shift[i][0], y + shift[i][1]) < negtivee_threshold) {
                            num++;
                        }
                        else
                        {
                            break;
                        }
                    }
                    if (num >= pixel_num_threshold) {
                        FAST_feature fast;
                        fast.position = vgl_point_2d<double>(x, y);
                        for (int k = 0; k<16; k++) {
                            fast.description[k] = grey_image(x + shift[k][0], y + shift[k][1]);
                        }
                        negative.push_back(fast);
                    }
                }
            }
        }
    }
}

void vil_fast_feature_match(const vcl_vector<FAST_feature> & fast1,
                                     const vcl_vector<FAST_feature> & fast2,
                                     vcl_vector<int> &matchIndex)
{
    for (int i = 0; i<fast1.size(); i++) {
        int ssd_min = INT_MAX;
        int idx     = -1;
        for (int j = 0; j<fast2.size(); j++) {
            int ssd = vnl_vector_ssd(fast1[i].description, fast2[j].description);
            if (ssd < ssd_min) {
                ssd_min = ssd;
                idx = j;
            }
        }
        matchIndex.push_back(idx);
    }
}


vcl_vector<vgl_point_2d<double> > vil_fast_feature_point(const vil_image_view<vxl_byte> &gray_image, int threshold)
{
    vcl_vector<vgl_point_2d<double> > positive;
    vcl_vector<vgl_point_2d<double> > negative;
    
    vil_fast_feature_point(gray_image, positive, negative, 12, threshold);
    
    //combine
    for (int i = 0; i<negative.size(); i++) {
        positive.push_back(negative[i]);
    }
    
    return positive;
}

