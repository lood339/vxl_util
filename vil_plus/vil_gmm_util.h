//
//  vil_gmm_util.h
//  PlayerDetection
//
//  Created by jimmy on 6/14/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __PlayerDetection__vil_gmm_util__
#define __PlayerDetection__vil_gmm_util__

#include "vil_gmm.h"
#include "SoccerLineParameters.h"

//
// describe if a pixel is white
// the pixel should be around by green colors (background color)
struct VIlGMMUTILGMMParameter
{
    double backgroundGMMProbThreshold_;  //green
    double foregroundGMMProbThreshold_;  //white
    int neighbor_size_;                 // neighborhood size
    double is_white_threshold_;         // cos(angle_with_(255, 255, 255)), color is normalized
    double blue_channel_min_threshold_;
    
    VIlGMMUTILGMMParameter()
    {
        backgroundGMMProbThreshold_ = 0.95;
        foregroundGMMProbThreshold_ = 0.95;
        neighbor_size_ = 7;
        is_white_threshold_ = 0.98;
        blue_channel_min_threshold_ = 90;
    }
    
    void setGrassLandDetection()
    {
        is_white_threshold_ = 0.0;
        blue_channel_min_threshold_ = 0;
    }
};


class VilGMMUtil
{
public:
    
    // true if pass the proportion
    static void positiveResponse(const VilGMM &gmm, const vil_image_view<vxl_byte> & image,
                                 vil_image_view<bool> & mask, double pass_proportion = 0.99);
    
    // binary classify pixel to two labels
    static void binaryClassify(const VilGMM & gmm1, const VilGMM & gmm2, const vil_image_view<vxl_byte> & image,
                               const int label1, const int label2, vil_image_view<vxl_byte> & labelImage);
    
    // for soccer field white pixel detection
    static void whiteLinePixelDetection(const VilGMM & green_gmm, const VilGMM & white_gmm, const vil_image_view<vxl_byte> & image,
                                        const VIlGMMUTILGMMParameter & para, vcl_vector<vgl_point_2d<double> > & whitePixels);
    
    // setGrassLandDetection
    static void grassLandPixelCheck(const VilGMM & green_gmm, const vil_image_view<vxl_byte> & image,
                                    const VIlGMMUTILGMMParameter & para, const vcl_vector<vgl_point_2d<double> > & positions,
                                    vcl_vector<bool> & isInGrassland);
    
    
    
    // the touch line is assume in the up half of the image
    static bool farAdvertisingLineDetection(const VilGMM & green_gmm, const vil_image_view<vxl_byte> & image,
                                            const FarTouchLineParameter & para, vgl_line_2d<double> & line);
    // detect far touch line by give advertising line
    static bool farTouchLineDetection(const vil_image_view<vxl_byte> & image, const vgl_line_2d<double> & advLine,
                                      const FarTouchLineParameter & para, vgl_line_2d<double> & line);   
    
    
    
};


#endif /* defined(__PlayerDetection__vil_gmm_util__) */
