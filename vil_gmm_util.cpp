//
//  vil_gmm_util.cpp
//  PlayerDetection
//
//  Created by jimmy on 6/14/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_gmm_util.h"
#include "vil_plus.h"
#include "vxl_hough_line.h"
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_matlab_filewrite.h>



/********************************  VilGMMUtil   **********************************************/


static bool isWhiteColor(const vnl_vector<double> & rgb, double thershold)
{
    vnl_vector<double> rgb_n = rgb;
    rgb_n = rgb_n.normalize();
    vnl_vector<double> white(3, 1.0/sqrt(3.0));
    double dot = dot_product(rgb_n, white);
    return dot > thershold;
}

void VilGMMUtil::positiveResponse(const VilGMM &gmm, const vil_image_view<vxl_byte> & image,
                                  vil_image_view<bool> & mask, double pass_proportion)
{
    assert(image.nplanes() == 3);
    const int w = image.ni();
    const int h = image.nj();
    
    double logProb = gmm.gmm_->log_prob_thresh(pass_proportion);
 //   printf("logProb is %f\n", logProb);
    mask = vil_image_view<bool>(w, h, 1);
    mask.fill(false);
    
    vnl_vector<double> color(3);
    for (int j = 0; j<h; j++) {
        for (int i =0; i<w; i++) {
            color[0] = image(i, j, 0);
            color[1] = image(i, j, 1);
            color[2] = image(i, j, 2);
            if (gmm.gmm_->log_p(color) > logProb ) {
                mask(i, j) = true;
            }
        }
    }
}

void VilGMMUtil::binaryClassify(const VilGMM & gmm1, const VilGMM & gmm2, const vil_image_view<vxl_byte> & image,
                                const int label1, const int label2, vil_image_view<vxl_byte> & labelImage)
{
    assert(image.nplanes() == 3);
    const int w = image.ni();
    const int h = image.nj();
    labelImage = vil_image_view<vxl_byte>(w, h, 1);
    labelImage.fill(0);
    
    vnl_vector<double> color(3);
    for (int y = 0; y<image.nj(); y++) {
        for (int x = 0; x<image.ni(); x++) {
            vnl_vector<double> color(3);
            for (int k = 0; k<3; k++) {
                color[k] = image(x, y, k);
            }
            double logp1 = gmm1.gmm_->log_p(color);
            double logp2 = gmm2.gmm_->log_p(color);
            if (logp1 > logp2) {
                labelImage(x, y) = label1;
            }
            else
            {
                labelImage(x, y) = label2;
            }
        }
    }
}

void VilGMMUtil::whiteLinePixelDetection(const VilGMM & green_gmm, const VilGMM & white_gmm, const vil_image_view<vxl_byte> & image,
                                         const VIlGMMUTILGMMParameter & para, vcl_vector<vgl_point_2d<double> > & whitePixels)
{
    assert(image.nplanes() == 3);
    
    double greenColorLogProb = green_gmm.gmm_->log_prob_thresh(para.backgroundGMMProbThreshold_);
    double whiteColorLogProb = white_gmm.gmm_->log_prob_thresh(para.foregroundGMMProbThreshold_);
    
    const int isGreen = 255;
    vil_image_view<vxl_byte> greenMask (image.ni(), image.nj(), 1);
    greenMask.fill(0);
    
    
    for (int y = 0; y<image.nj(); y++) {
        for (int x = 0; x<image.ni(); x++) {
            vnl_vector<double> color(3);
            for (int k = 0; k<3; k++) {
                color[k] = image(x, y, k);
            }
            if (green_gmm.gmm_->log_p(color) > greenColorLogProb) {
                greenMask(x, y) = isGreen;
            }
        }
    }
    
    int board_size = para.neighbor_size_;
    double isWhiteThreshold = para.is_white_threshold_;
    //  classifer pixel by distance to mean white color gmm
    for (int y = board_size; y<image.nj()-board_size; y++) {
        for (int x = board_size; x<image.ni()-board_size; x++) {
            vnl_vector<double> color(3);
            for (int k = 0; k<3; k++) {
                color[k] = image(x, y, k);
            }
            if (white_gmm.gmm_->log_p(color) > whiteColorLogProb &&
                isWhiteColor(color, isWhiteThreshold) &&
                color[2] > para.blue_channel_min_threshold_)
            {
                // check board_size, at least one green in the neighborhood
                bool hasGreen = false;
                for (int k = -board_size/2; k <= board_size/2; k++)
                {
                    for (int m = board_size/2; m<=board_size/2; m++)
                    {
                        if (greenMask(x+m, y+k) == isGreen) {
                            hasGreen = true;
                            break;
                        }
                    }
                    if (hasGreen) {
                        break;
                    }
                }
                if (hasGreen) {
                    whitePixels.push_back(vgl_point_2d<double>(x, y));
                }
            }
        }
    }
}

void VilGMMUtil::grassLandPixelCheck(const VilGMM & green_gmm, const vil_image_view<vxl_byte> & image,
                                     const VIlGMMUTILGMMParameter & para, const vcl_vector<vgl_point_2d<double> > & positions,
                                     vcl_vector<bool> & isInGrassland)
{
    double greenColorLogProb = green_gmm.gmm_->log_prob_thresh(para.backgroundGMMProbThreshold_);
    
    const int isGreen = 255;
    vil_image_view<vxl_byte> greenMask (image.ni(), image.nj(), 1);
    greenMask.fill(0);
    
    for (int y = 0; y<image.nj(); y++) {
        for (int x = 0; x<image.ni(); x++) {
            vnl_vector<double> color(3);
            for (int k = 0; k<3; k++) {
                color[k] = image(x, y, k);
            }
            if (green_gmm.gmm_->log_p(color) > greenColorLogProb) {
                greenMask(x, y) = isGreen;
            }
        }
    }
    
    int board_size = para.neighbor_size_;
    isInGrassland.resize(positions.size());
    for (int i = 0; i<positions.size(); i++) {
        int x = positions[i].x();
        int y = positions[i].y();
        if ( x < board_size || y < board_size ||
            x + board_size >= image.ni() ||
            y + board_size >= image.nj()) {
            isInGrassland[i] = false;
        }
        else
        {
            bool hasGreen = false;
            for (int k = -board_size/2; k <= board_size/2; k++)
            {
                for (int m = board_size/2; m<=board_size/2; m++)
                {
                    if (greenMask(x+m, y+k) == isGreen) {
                        hasGreen = true;
                        break;
                    }
                }
                if (hasGreen) {
                    break;
                }
            }
            if (hasGreen) {
                isInGrassland[i] = true;
            }
        }
    }
}

bool VilGMMUtil::farAdvertisingLineDetection(const VilGMM & green_gmm, const vil_image_view<vxl_byte> & image,
                                             const FarTouchLineParameter & para, vgl_line_2d<double> & line)
{
    vil_image_view<double> mag;
    VilPlus::vil_magnitude(image, mag);
    
    int w = image.ni();
    int h = image.nj();
    
    double greenColorLogProb = green_gmm.gmm_->log_prob_thresh(para.greenGMMProbThreshold_);
    // scan line method
    vcl_vector<vgl_point_2d<double> > pts;
    for (int i = 0; i<w; i++) {
        for (int j = h/2; j >= 0; j--) {
            if (mag(i, j) > para.mag_threshold_) {
                if (j + para.look_up_pixel_ < h) {
                    vnl_vector<double> color(3);
                    for (int k = 0; k<3; k++) {
                        color[k] = image(i, j + para.look_up_pixel_, k);
                    }
                    // it is green so this is not board line
                    if (green_gmm.gmm_->log_p(color) > greenColorLogProb) {
                        continue;
                    }
                    else
                    {
                        pts.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
            }
        }
    }
    
    //
    vcl_vector<vgl_line_2d<double> > boardLine;
    vil_image_view<vxl_byte> edgeMask(w, h, 1);
    edgeMask.fill(0);
    for (int i = 0; i<pts.size(); i++) {
        int x = pts[i].x();
        int y = pts[i].y();
        edgeMask(x, y) = 255;
    }
    
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    VxlHoughLine::oneByOneLineDetection(edgeMask, houghPara, boardLine);
    if (boardLine.size() != 1) {
        printf("Warning: can not find boardline\n");
        return false;
    }
    line = boardLine[0];
    return true;
}


bool VilGMMUtil::farTouchLineDetection(const vil_image_view<vxl_byte> & image, const vgl_line_2d<double> & advLine,
                                       const FarTouchLineParameter & para, vgl_line_2d<double> & line)
{
    vil_image_view<double> mag;
    VilPlus::vil_magnitude(image, mag);
    
    int w = image.ni();
    int h = image.nj();
    
    vil_image_view<vxl_byte> maskImage(w, h, 3);
    maskImage.fill(0);
    VilPlus::draw_line(maskImage,advLine, VilPlus::white(), para.lineWidth_);
    
    // scan and accumualate average gradient magnitude
    vcl_vector<vgl_point_2d<double>>  pts;
    double lambda = para.lambda_;
    for (int i = 0; i<w; i++) {
        double totalMag = 0.0;
        int magNum = 0;
        for (int j = h/2; j >= 0; j--) {
            if (maskImage(i, j, 0) == 255) {
                if (magNum > 0 && mag(i, j) > para.mag_threshold_2_
                    && mag(i, j) > lambda * totalMag/magNum) {
                    pts.push_back(vgl_point_2d<double>(i, j));
                    break;
                }
                totalMag += mag(i, j);
                magNum++;
            }
        }
    }
    
    vil_image_view<vxl_byte> edgeMask(w, h, 1);
    edgeMask.fill(0);
    for (int i = 0; i<pts.size(); i++) {
        int x = pts[i].x();
        int y = pts[i].y();
        edgeMask(x, y) = 255;
    }
    
    
    VxlHoughParameter houghPara;
    houghPara.maxLineNum_ = 1;
    vcl_vector<vgl_line_2d<double> > lines;
    VxlHoughLine::oneByOneLineDetection(edgeMask, houghPara, lines);
    if (lines.size() == 0) {
        return false;
    }
    line = lines[0];
    return true;
}
/*
bool VilGMMUtil::detectLinePixelByAverageGradient(const vil_image_view<double> & mag, const vil_image_view<vxl_byte> & maskImage,
                                                  const AverageMagnitudeParameter & para, vcl_vector<vgl_point_2d<double> > & edge1Pixels,
                                                  vcl_vector<vgl_point_2d<double> > & edge2Pixels)
{
    assert(mag.ni() == maskImage.ni());
    assert(mag.nj() == maskImage.nj());
    assert(mag.nplanes() == 1);
    
    assert(para.direction_ >= 0 && para.direction_ <= 3);
    
    // int direction_;        // 0 horizontal, 1 vertical, 2, left_top to right_buttom diagonal, 3, left_buttom to right_top diagonal
    const int w = mag.ni();
    const int h = mag.nj();
    if (para.direction_ == 0) {
        
        // for test
        vnl_matrix<double> avgGradientMat(h, w, 0);
        vnl_matrix<double> curGradientMat(h, w, 0);
        // from left to right
        for (int j = 0; j<h; j++) {
            double sumMag = 0.0;
            int num = 0;
            for (int i = 0; i<w; i++) {
                if (maskImage(i, j) == 255) {
                    if (num >= 1) {
                        if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                            edge1Pixels.push_back(vgl_point_2d<double>(i, j));
                            break;
                        }
                    }
                    sumMag += mag(i, j);
                    num++;
                    if (num >= 1) {
                        avgGradientMat(j, i) = sumMag/num;
                        curGradientMat(j, j) = mag(i, j);
                    }
                }
            }
        }
        
        // for test
        if(0)
        {
            vnl_matlab_filewrite writer("avgGradient.mat");
            writer.write(avgGradientMat, "avg_gradient");
            writer.write(curGradientMat, "cur_gradient");
            printf("save to %s\n", "avgGradient.mat");
        }
        
        // from right to left
        for (int j = 0; j<h; j++) {
            double sumMag = 0.0;
            int num = 0;
            for (int i = w-1; i >= 0; i--) {
                if (maskImage(i, j) == 255) {
                    if (num >= 1) {
                        if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                            edge2Pixels.push_back(vgl_point_2d<double>(i, j));
                            break;
                        }
                    }
                    sumMag += mag(i, j);
                    num++;
                }
            }
        }
    }
    else if(para.direction_ == 1)
    {
        // up to down
        for (int i = 0; i<w; i++ ) {
            double sumMag = 0.0;
            int num = 0;
            for (int j = 0; j<h; j++) {
                if (maskImage(i, j) == 255) {
                    if (num >= 1) {
                        if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                            edge1Pixels.push_back(vgl_point_2d<double>(i, j));
                            break;
                        }
                    }
                    sumMag += mag(i, j);
                    num++;
                }
            }
        }
        
        // down to up
        for (int i = 0; i<w; i++) {
            double sumMag = 0.0;
            int num = 0;
            for (int j = h-1; j >= 0; j--) {
                if (maskImage(i, j) == 255) {
                    if (num >= 1) {
                        if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                            edge2Pixels.push_back(vgl_point_2d<double>(i, j));
                            break;
                        }
                    }
                    sumMag += mag(i, j);
                    num++;
                }
            }
        }
    }
    else if(para.direction_ == 2)
    {
        // from left to right
        for (int j = 0; j<h; j++) {
            bool isFirst = false;
            for (int i = 0; i<w; i++) {
                // find first mask position
                if (!isFirst && maskImage(i, j) == 255) {
                    isFirst = true;
                    double sumMag = 0.0;
                    int num = 0;
                    int x = i;
                    int y = j;
                    while (x < w && y >= 0) {
                        if (maskImage(x, y) == 255) {
                            if (num >= 1) {
                                if (mag(x, y) > para.mag_threshold_ && mag(x, y) >= para.lambda_ * sumMag/num) {
                                    edge1Pixels.push_back(vgl_point_2d<double>(x, y));
                                    break;
                                }
                            }
                            sumMag += mag(i, j);
                            num++;
                        }
                        x++;
                        y--;
                    }
                }
                if (isFirst) {
                    break;
                }
            }
        }
        
        // from right to left
        for (int j = 0; j<h; j++) {
            bool isFirst = false;
            for (int i = w-1; i >= 0; i--) {
                // find first mask position
                if (!isFirst && maskImage(i, j) == 255) {
                    isFirst = true;
                    double sumMag = 0.0;
                    int num = 0;
                    int x = i;
                    int y = j;
                    while (x >= 0 && y < h) {
                        if (maskImage(x, y) == 255) {
                            if (num >= 1) {
                                if (mag(x, y) > para.mag_threshold_ && mag(x, y) >= para.lambda_ * sumMag/num) {
                                    edge2Pixels.push_back(vgl_point_2d<double>(x, y));
                                    break;
                                }
                            }
                            sumMag += mag(i, j);
                            num++;
                        }
                        x--;
                        y++;
                    }
                }
                if (isFirst) {
                    break;
                }
            }
        }
    }
    else if(para.direction_ == 3)
    {
        // from left to right
        for (int j = 0; j<h; j++) {
            bool isFirst = false;
            for (int i = 0; i<w; i++) {
                // find first mask position
                if (!isFirst && maskImage(i, j) == 255) {
                    isFirst = true;
                    double sumMag = 0.0;
                    int num = 0;
                    int x = i;
                    int y = j;
                    while (x < w && y < h) {
                        if (maskImage(x, y) == 255) {
                            if (num >= 1) {
                                if (mag(x, y) > para.mag_threshold_ && mag(x, y) >= para.lambda_ * sumMag/num) {
                                    edge1Pixels.push_back(vgl_point_2d<double>(x, y));
                                    break;
                                }
                            }
                            sumMag += mag(i, j);
                            num++;
                        }
                        x++;
                        y++;
                    }
                }
                if (isFirst) {
                    break;
                }
            }
        }
        
        // from right to left
        for (int j = 0; j<h; j++) {
            bool isFirst = false;
            for (int i = w-1; i >= 0; i--) {
                // find first mask position
                if (!isFirst && maskImage(i, j) == 255) {
                    isFirst = true;
                    double sumMag = 0.0;
                    int num = 0;
                    int x = i;
                    int y = j;
                    while (x >= 0 && y >= 0) {
                        if (maskImage(x, y) == 255) {
                            if (num >= 1) {
                                if (mag(x, y) > para.mag_threshold_ && mag(x, y) >= para.lambda_ * sumMag/num) {
                                    edge2Pixels.push_back(vgl_point_2d<double>(x, y));
                                    break;
                                }
                            }
                            sumMag += mag(i, j);
                            num++;
                        }
                        x--;
                        y--;
                    }
                }
                if (isFirst) {
                    break;
                }
            }
        }
    }
    
    return true;
}


bool VilGMMUtil::detectEllipsePixelByAverageGradient(const vil_image_view<double> & mag,
                                                     const vil_image_view<vxl_byte> & mask,
                                                     const vil_image_view<vxl_byte> & nonEllipseMask,
                                                     const AverageMagnitudeParameter & para,
                                                     vcl_vector<vgl_point_2d<double> > & outsideUpPixels,
                                                     vcl_vector<vgl_point_2d<double> > & outsideDownPixels)
{
    const int w = mag.ni();
    const int h = mag.nj();
    assert(w == mask.ni() && h == mask.nj());
    assert(w == nonEllipseMask.ni() && h == nonEllipseMask.nj());
    assert(para.direction_ == 1);
    
    // up to down
    for (int i = 0; i<w; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = 0; j<h; j++) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideUpPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    // down to up
    for (int i = 0 ; i<w; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = h-1; j >= 0; j--) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideDownPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    
    return true;
}

bool VilGMMUtil::detectLeftPenaltyEllipseByAverageGradient(const vil_image_view<double> & mag,
                                                           const vil_image_view<vxl_byte> & mask,
                                                           const vil_image_view<vxl_byte> & nonEllipseMask,
                                                           const AverageMagnitudeParameter & para,
                                                           vcl_vector<vgl_point_2d<double> > & outsideHorizontalPixels,
                                                           vcl_vector<vgl_point_2d<double> > & outsideVerticalPixels)
{
    const int w = mag.ni();
    const int h = mag.nj();
    assert(w == mask.ni() && h == mask.nj());
    assert(w == nonEllipseMask.ni() && h == nonEllipseMask.nj());
    
    // right to left
    for (int j = 0; j<h; j++) {
        double sumMag = 0.0;
        int num = 0;
        for (int i = w-1; i >= 0; i-- ) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideHorizontalPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    
    // down to up
    for (int i = 0; i<w; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = h-1; j >= 0; j--) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideVerticalPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    return true;
}

bool VilGMMUtil::detectRightPenaltyEllipseByAverageGradient(const vil_image_view<double> & mag,
                                                            const vil_image_view<vxl_byte> & mask,
                                                            const vil_image_view<vxl_byte> & nonEllipseMask,
                                                            const AverageMagnitudeParameter & para,
                                                            vcl_vector<vgl_point_2d<double> > & outsideHorizontalPixels,
                                                            vcl_vector<vgl_point_2d<double> > & outsideVerticalPixels)
{
    const int w = mag.ni();
    const int h = mag.nj();
    assert(w == mask.ni() && h == mask.nj());
    assert(w == nonEllipseMask.ni() && h == nonEllipseMask.nj());
    
    // left to right
    for (int j = 0; j<h; j++) {
        double sumMag = 0.0;
        int num = 0;
        for (int i = 0; i < w; i++ ) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideHorizontalPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    
    // down to up
    for (int i = 0; i<w; i++) {
        double sumMag = 0.0;
        int num = 0;
        for (int j = h-1; j >= 0; j--) {
            if (mask(i, j) == 255 && nonEllipseMask(i, j) != 255) {
                if (num >= 1) {
                    if (mag(i, j) > para.mag_threshold_ && mag(i, j) >= para.lambda_ * sumMag/num) {
                        outsideVerticalPixels.push_back(vgl_point_2d<double>(i, j));
                        break;
                    }
                }
                sumMag += mag(i, j);
                num++;
            }
        }
    }
    return true;
}
 
*/
