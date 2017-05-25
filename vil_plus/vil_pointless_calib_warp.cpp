//
//  vil_pointless_calib_warp.cpp
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vil_pointless_calib_warp.h"
#include <vil/vil_quantize.h>
#include <vil/vil_save.h>
#include "vxl_plus.h"
#include <vul/vul_string+.h>
#include "LucasKanade.h"
#include "LucasKanade.txx"
#include "LucasKanadeWarps-NaturalCamera.h"
#include "LucasKanadeGradient.h"
#include <vil/vil_convert.h>
#include "LucasKanadeWarps-Standard.h"
#include "LucasKanadaWarps_PTZCamera.h"
#include "LucasKanadaWarp_2D_translate.h"
#include "vil_plus.h"
//#include "vil_plus_extra.h"

//long gradient pair
struct LongGradientPair {
    vil_image_view<double> image;
    vil_image_view<double> Ix;
    vil_image_view<double> Iy;
    vcl_string gradientCache;
    unsigned int radius;
};

VilPointlessCalibWarp::VilPointlessCalibWarp()
{
    
}

VilPointlessCalibWarp::~VilPointlessCalibWarp()
{
    
}

bool VilPointlessCalibWarp::warpCourtToImage(BasketballCourt * court, const vil_image_view<vxl_byte> &image, int iterator_num, int radius,
                      const vpgl_perspective_camera<double> &initCamera, vpgl_perspective_camera<double> &finalCamera)
{
  //  if (image_gradient_cached_file_.empty()) {
  //      vcl_cout<<"Error: must have cached file, it could be image name.\n";
  //      return false;
  //  }
    
    vil_image_view<vxl_byte> courtImage;
    court->courtImage(courtImage);
    
    assert(courtImage.nplanes() == 1);
    
    vil_image_view<double> court_double = vil_quantize::dequantize<double>(courtImage);
    
    
    //magnitude of courtImage
    vil_image_view<double> court_magnitude;
    VilPlus::vil_magnitude(court_double, court_magnitude);
    //VilPlusExtra::vil_save(court_magnitude, "magnitude_court.jpg");
    
    //gradient of magnitude of court image
    LongGradientPair courtPair;
    {
        vcl_string gradientFileName = court->name() + "_gradient_magnitude_" + vul_string(radius) + ".vnl_mat";
        vil_image_view<double> Ix;
        vil_image_view<double> Iy;
        lkCachedGradient(court_magnitude, gradientFileName, radius, Ix, Iy);
        vcl_cout<<"save to : "<<gradientFileName<<vcl_endl;
        
        courtPair.image = court_magnitude;
        courtPair.Ix = Ix;
        courtPair.Iy = Iy;
        courtPair.gradientCache = gradientFileName;
        courtPair.radius = radius;
        
        vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string(court->name() + "_" + vul_string( radius ) + "_x.jpg"));
        vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string(court->name() + "_" + vul_string( radius ) + "_y.jpg"));
    }
    
    //gradient of magnitude of camera image
    vil_image_view<vxl_byte> grayImage;
    grayImage.deep_copy(image);
    
    if (grayImage.nplanes() != 1) {
        vil_image_view<vxl_byte> temp;
        vil_convert_planes_to_grey(grayImage, temp);
        grayImage = temp;
    }
    
    vil_image_view<double> image_dequantized = vil_quantize::dequantize<double>(image);
    vil_image_view<double> image_magnitude;
    VilPlus::vil_magnitude(image_dequantized, image_magnitude);
    //VilPlusExtra::vil_save(image_magnitude, "magnitude_image.jpg");
    
    LongGradientPair imagePair;
    {
     //   vcl_string gradientFileName = image_gradient_cached_file_ + "_gradient_magnitude_" + vul_string(radius) + ".vnl_mat";
     //   vil_image_view<double> Ix;
    //    vil_image_view<double> Iy;
        
   //     vcl_cout<<"gradientFileName is "<<gradientFileName<<vcl_endl;
        
   //     lkCachedGradient(image_magnitude, gradientFileName, radius, Ix, Iy);
   //     vcl_cout<<"save to : "<<gradientFileName<<vcl_endl;
        
        imagePair.image = image_magnitude;
   //     imagePair.Ix = Ix;
   //     imagePair.Iy = Iy;
   //     imagePair.gradientCache = gradientFileName;
   //     imagePair.radius = radius;
        
   //     vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string(vcl_string("image") + "_" + vul_string( radius ) + "_x.jpg"));
   //     vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string(vcl_string("image") + "_" + vul_string( radius ) + "_y.jpg"));
    }
   
    // weight image
    vil_image_view<double> wtImage;
    court->getWeightImage(initCamera, image.ni(), image.nj(), radius * 2, 100, wtImage);
    //VilPlusExtra::vil_save(wtImage, vcl_string(court->name() + "_weight_image.jpg").c_str());
    
    
    vgl_transform_2d<double> curModel = court->imageToWorld();
    NaturalCameraImageToPitchWarp curWarp(initCamera, curModel);
    
    double iterationError;
    
    lkForwardsAdditive(courtPair.image, imagePair.image,  wtImage, curWarp, courtPair.Ix, courtPair.Iy, &iterationError, iterator_num, 0);
    
    finalCamera = curWarp.camera();
    
    vcl_cout<<"camera focal length "<<finalCamera.get_calibration().focal_length()<<vcl_endl;
    vcl_cout<<"camera x scale "<<finalCamera.get_calibration().x_scale()<<vcl_endl;
    
    return true;
}

bool VilPointlessCalibWarp::warpCourtToImage(SoccerCourt * court, const vil_image_view<vxl_byte> &image, int iterator_num, int radius,
                                             const vpgl_perspective_camera<double> &initCamera, vpgl_perspective_camera<double> &finalCamera)
{
    vil_image_view<vxl_byte> courtImage;
    court->courtImage(courtImage);
    
    assert(courtImage.nplanes() == 1);
    vil_image_view<double> court_double = vil_quantize::dequantize<double>(courtImage);
    
    //magnitude of courtImage
    vil_image_view<double> court_magnitude;
    VilPlus::vil_magnitude(court_double, court_magnitude);
    //VilPlusExtra::vil_save(court_magnitude, "magnitude_court.jpg");
    
    //gradient of magnitude of court image
    LongGradientPair courtPair;
    {
        vcl_string gradientFileName = court->name() + "_gradient_magnitude_" + vul_string(radius) + ".vnl_mat";
        vil_image_view<double> Ix;
        vil_image_view<double> Iy;
        lkCachedGradient(court_magnitude, gradientFileName, radius, Ix, Iy);
        vcl_cout<<"save to : "<<gradientFileName<<vcl_endl;
        
        courtPair.image = court_magnitude;
        courtPair.Ix = Ix;
        courtPair.Iy = Iy;
        courtPair.gradientCache = gradientFileName;
        courtPair.radius = radius;
        
        vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string(court->name() + "_" + vul_string( radius ) + "_x.jpg"));
        vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string(court->name() + "_" + vul_string( radius ) + "_y.jpg"));
    }
    
    //gradient of magnitude of camera image
    vil_image_view<vxl_byte> grayImage;
    grayImage.deep_copy(image);
    
    if (grayImage.nplanes() != 1) {
        vil_image_view<vxl_byte> temp;
        vil_convert_planes_to_grey(grayImage, temp);
        grayImage = temp;
    }
    
    vil_image_view<double> image_dequantized = vil_quantize::dequantize<double>(image);
    vil_image_view<double> image_magnitude;
    VilPlus::vil_magnitude(image_dequantized, image_magnitude);
    //VilPlusExtra::vil_save(image_magnitude, "magnitude_image.jpg");
    
    LongGradientPair imagePair;
    imagePair.image = image_magnitude;
    
    // weight image
    vil_image_view<double> wtImage;
    court->getWeightImage(initCamera, image.ni(), image.nj(), radius * 2, 100, wtImage);
    //VilPlusExtra::vil_save(wtImage, vcl_string(court->name() + "_weight_image.jpg").c_str());
    
    vgl_transform_2d<double> curModel = court->imageToWorld();
    NaturalCameraImageToPitchWarp curWarp(initCamera, curModel);
    
    double iterationError;    
    double tt = clock();
    lkForwardsAdditive(courtPair.image, imagePair.image,  wtImage, curWarp, courtPair.Ix, courtPair.Iy, &iterationError, iterator_num, 0);
    printf("pointless calib cost time %f\n", (clock() - tt)/CLOCKS_PER_SEC);
    
    finalCamera = curWarp.camera();
    vcl_cout<<"camera focal length "<<finalCamera.get_calibration().focal_length()<<vcl_endl;
    vcl_cout<<"camera x scale "<<finalCamera.get_calibration().x_scale()<<vcl_endl;
    
    return true;
}

bool VilPointlessCalibWarp::warpCourtToImageWithLogo(DisneyWorldBasketballCourt * court, const vil_image_view<vxl_byte> &image, int iterator_num, int radius,
                              const vpgl_perspective_camera<double> &initCamera, vpgl_perspective_camera<double> &finalCamera)
{
    if (image_gradient_cached_file_.empty()) {
        vcl_cout<<"Error: must have cached file, it could be image name.\n";
        return false;
    }
    
    vil_image_view<vxl_byte> courtImage;
    court->courtImageWithLogo(courtImage);
    
    assert(courtImage.nplanes() == 1);
    
    vil_image_view<double> court_double = vil_quantize::dequantize<double>(courtImage);
    
    
    //magnitude of courtImage
    vil_image_view<double> court_magnitude;
    VilPlus::vil_magnitude(court_double, court_magnitude);
    //VilPlusExtra::vil_save(court_magnitude, "magnitude_court.jpg");
    
    //gradient of magnitude of court image
    LongGradientPair courtPair;
    {
        vcl_string gradientFileName = court->name() + "_gradient_magnitude_logo_" + vul_string(radius) + ".vnl_mat";
        vil_image_view<double> Ix;
        vil_image_view<double> Iy;
        lkCachedGradient(court_magnitude, gradientFileName, radius, Ix, Iy);
        vcl_cout<<"save to : "<<gradientFileName<<vcl_endl;
        
        courtPair.image = court_magnitude;
        courtPair.Ix = Ix;
        courtPair.Iy = Iy;
        courtPair.gradientCache = gradientFileName;
        courtPair.radius = radius;
        
        vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string(court->name() + "_" + vul_string( radius ) + "_x.jpg"));
        vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string(court->name() + "_" + vul_string( radius ) + "_y.jpg"));
    }
    
    //gradient of magnitude of camera image
    vil_image_view<vxl_byte> grayImage;
    grayImage.deep_copy(image);
    
    if (grayImage.nplanes() != 1) {
        vil_image_view<vxl_byte> temp;
        vil_convert_planes_to_grey(grayImage, temp);
        grayImage = temp;
    }
    
    vil_image_view<double> image_dequantized = vil_quantize::dequantize<double>(image);
    vil_image_view<double> image_magnitude;
    VilPlus::vil_magnitude(image_dequantized, image_magnitude);
    //VilPlusExtra::vil_save(image_magnitude, "magnitude_image.jpg");
    
    LongGradientPair imagePair;
    {
        
       // vcl_string gradientFileName = image_gradient_cached_file_ + "_gradient_magnitude_logo_" + vul_string(radius) + ".vnl_mat";
       // vil_image_view<double> Ix;
       // vil_image_view<double> Iy;
        
       // vcl_cout<<"gradientFileName is "<<gradientFileName<<vcl_endl;
        
      //  lkCachedGradient(image_magnitude, gradientFileName, radius, Ix, Iy);
      //  vcl_cout<<"save to : "<<gradientFileName<<vcl_endl;
        
        imagePair.image = image_magnitude;
     //   imagePair.Ix = Ix;
     //   imagePair.Iy = Iy;
     //   imagePair.gradientCache = gradientFileName;
     //   imagePair.radius = radius;
        
     //   vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string(vcl_string("image") + "_" + vul_string( radius ) + "_x.jpg"));
     //   vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string(vcl_string("image") + "_" + vul_string( radius ) + "_y.jpg"));
    }
    
    // weight image
    vil_image_view<double> wtImage;
    court->getWeightImageWithLogo(initCamera, image.ni(), image.nj(), radius*2, 100, wtImage);
    //VilPlusExtra::vil_save(wtImage, vcl_string(court->name() + "_weight_image.jpg").c_str());
    
    
    vgl_transform_2d<double> curModel = court->imageToWorld();
    NaturalCameraImageToPitchWarp curWarp(initCamera, curModel);
    
    double iterationError;
    
    lkForwardsAdditive(courtPair.image, imagePair.image,  wtImage, curWarp, courtPair.Ix, courtPair.Iy, &iterationError, iterator_num, 0);
    
    finalCamera = curWarp.camera();
    
    return true;
}

double VilPointlessCalibWarp::warp2DPatternToImage(const vil_image_view<vxl_byte> & pattern, const    vil_image_view<vxl_byte> & image, int iterator_num, int radius,
                                                 const vnl_vector_fixed<double, 2> & initTranslate, vnl_vector_fixed<double, 2> & finalTranslate)
{
    assert(pattern.ni() == image.ni());
    assert(pattern.nj() == image.nj());
  //  assert(pattern.nplanes() == 1);
    if (pattern.ni() >= 32 && pattern.nj() >= 32) {
        printf("warning: image size %d %d, may take long time to calcualte gradient\n", pattern.ni(), pattern.nj());
    }
    
    const int w = pattern.ni();
    const int h = pattern.nj();
    
    vil_image_view<vxl_byte> grayPattern;
    grayPattern.deep_copy(pattern);
    if (grayPattern.nplanes() == 3) {
        vil_image_view<vxl_byte> temp;
        vil_convert_planes_to_grey(grayPattern, temp);
        grayPattern = temp;        
    }
    
    vil_image_view<double> pattern_double = vil_quantize::dequantize<double>(pattern);
    
    //magnitude of pattern
    vil_image_view<double> pattern_magnitude;
    VilPlus::vil_magnitude(pattern_double, pattern_magnitude);
    //VilPlusExtra::vil_save(pattern_magnitude, "pattern_court.jpg");
    
    //gradient of magnitude of pattern
    LongGradientPair patternPair;
    {
        vil_image_view<double> Ix;
        vil_image_view<double> Iy;
        lkGradient(pattern_magnitude, Ix, Iy, radius);
        
        patternPair.image = pattern_magnitude;
        patternPair.Ix = Ix;
        patternPair.Iy = Iy;
        patternPair.radius = radius;
        
        vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string("pattern_" + vul_string( radius ) + "_x.jpg"));
        vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string("pattern_" + vul_string( radius ) + "_y.jpg"));
    }
    
    //gradient of magnitude of camera image
    vil_image_view<vxl_byte> grayImage;
    grayImage.deep_copy(image);
    
    if (grayImage.nplanes() != 1) {
        vil_image_view<vxl_byte> temp;
        vil_convert_planes_to_grey(grayImage, temp);
        grayImage = temp;
    }
    
    vil_image_view<double> image_dequantized = vil_quantize::dequantize<double>(image);
    vil_image_view<double> image_magnitude;
    VilPlus::vil_magnitude(image_dequantized, image_magnitude);
    //VilPlusExtra::vil_save(image_magnitude, "magnitude_image.jpg");
    
    LongGradientPair imagePair;
    {
        imagePair.image = image_magnitude;
    }
    
    vil_image_view<double> wtImage(w, h, 1);
    wtImage.fill(0.0);
    // only center image has weight
    for (int j = h/4; j<h/2+h/4; j++) {
        for (int i = h/4; i<w/2+w/4; i++) {
            wtImage(i, j) = 1.0;
        }
    }
   
    Translate_2D_Warp warp2D(initTranslate);
    
    double iterationError;
    lkForwardsAdditive(patternPair.image, imagePair.image,  wtImage, warp2D, patternPair.Ix, patternPair.Iy, &iterationError, iterator_num, 0);
    
    finalTranslate = warp2D.parameters();
    
    return iterationError;
}

bool VilPointlessCalibWarp::homographyWarp(const vil_image_view<vxl_byte> & source, const vil_image_view<vxl_byte> & dest, const vil_image_view<double> & wtImage,
                                           int iterator_num, int radius,
                                           const vgl_h_matrix_2d<double> & initHomo, vgl_h_matrix_2d<double> & finalHomo)
{
    assert(source.nplanes() == 3);
    assert(dest.nplanes() == 3);
    assert(wtImage.nplanes() == 1);
    assert(source.ni() == dest.ni() && source.nj() == dest.nj());
    assert(source.ni() == wtImage.ni() && source.nj() == wtImage.nj());
    
    vil_image_view<double> source_magnitude;
    {
        vil_image_view<vxl_byte> grey;
        vil_convert_planes_to_grey(source, grey);
        vil_image_view<double> source_double = vil_quantize::dequantize<double>(grey);
        
        VilPlus::vil_magnitude(source_double, source_magnitude);
        //VilPlusExtra::vil_save(source_magnitude, "magnitude_source.jpg");
    }
   
    vil_image_view<double> dest_magnitude;
    {
        vil_image_view<vxl_byte> grey;
        vil_convert_planes_to_grey(dest, grey);
        vil_image_view<double> dest_double = vil_quantize::dequantize<double>(grey);
        
        VilPlus::vil_magnitude(dest_double, dest_magnitude);
        //VilPlusExtra::vil_save(dest_magnitude, "magnitude_dest.jpg");
    }
    
    vil_image_view<double> Ix;
    vil_image_view<double> Iy;
    lkGradient(source_magnitude, Ix, Iy, radius);
    
    vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string("source_") + vcl_string(vul_string( radius ) + "_x.jpg"));
    vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string("source_") + vcl_string(vul_string( radius ) + "_y.jpg"));
   

    ProjectiveWarp pointlessHWarp(initHomo.get_inverse());
    double v_error = 0;
    lkForwardsAdditive(source_magnitude, dest_magnitude,  wtImage, pointlessHWarp, Ix, Iy, &v_error, iterator_num, 0);
    
    finalHomo = pointlessHWarp.getiWarpH();
    
    return true;
}

/*
    ******************************************** VilPTZPointlessWarp ************************************************
 */

VilPTZPointlessWarp::VilPTZPointlessWarp()
{
    
}
VilPTZPointlessWarp::~VilPTZPointlessWarp()
{
    
}

bool VilPTZPointlessWarp::warpCourtToImage(BasketballCourt * court, const vil_image_view<vxl_byte> &image,
                                           int iterator_num, int radius, const vpgl_ptz_camera & initCamera, vpgl_ptz_camera & finalCamera)
{
    vil_image_view<vxl_byte> courtImage;
    court->courtImage(courtImage);
    
    assert(courtImage.nplanes() == 1);
    
    vil_image_view<double> court_double = vil_quantize::dequantize<double>(courtImage);
    
    //magnitude of courtImage
    vil_image_view<double> court_magnitude;
    VilPlus::vil_magnitude(court_double, court_magnitude);
    //VilPlusExtra::vil_save(court_magnitude, "magnitude_court.jpg");
    
    //gradient of magnitude of court image
    LongGradientPair courtPair;
    {
        vcl_string gradientFileName = court->name() + "_gradient_magnitude_" + vul_string(radius) + ".vnl_mat";
        vil_image_view<double> Ix;
        vil_image_view<double> Iy;
        lkCachedGradient(court_magnitude, gradientFileName, radius, Ix, Iy);
        vcl_cout<<"save to : "<<gradientFileName<<vcl_endl;
        
        courtPair.image = court_magnitude;
        courtPair.Ix = Ix;
        courtPair.Iy = Iy;
        courtPair.gradientCache = gradientFileName;
        courtPair.radius = radius;
        
        vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string(court->name() + "_" + vul_string( radius ) + "_x.jpg"));
        vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string(court->name() + "_" + vul_string( radius ) + "_y.jpg"));
    }
    
    //gradient of magnitude of camera image
    vil_image_view<vxl_byte> grayImage;
    grayImage.deep_copy(image);
    
    if (grayImage.nplanes() != 1) {
        vil_image_view<vxl_byte> temp;
        vil_convert_planes_to_grey(grayImage, temp);
        grayImage = temp;
    }
    
    vil_image_view<double> image_dequantized = vil_quantize::dequantize<double>(image);
    vil_image_view<double> image_magnitude;
    VilPlus::vil_magnitude(image_dequantized, image_magnitude);
    //VilPlusExtra::vil_save(image_magnitude, "magnitude_image.jpg");
    
    LongGradientPair imagePair;
    imagePair.image = image_magnitude;
    
    // weight image
    vil_image_view<double> wtImage;
    court->getWeightImage(initCamera, image.ni(), image.nj(), radius * 2, 100, wtImage);
    //VilPlusExtra::vil_save(wtImage, vcl_string(court->name() + "_weight_image.jpg").c_str());
    
    
    vgl_transform_2d<double> curModel = court->imageToWorld();
    LucasKanadaPTZCameraWarp ptzWarp(initCamera, curModel);
    
    double iterationError;
    
    lkForwardsAdditive(courtPair.image, imagePair.image,  wtImage, ptzWarp, courtPair.Ix, courtPair.Iy, &iterationError, iterator_num, 0);
    
    finalCamera = ptzWarp.camera();
    
    vcl_cout<<"camera focal length "<<finalCamera.get_calibration().focal_length()<<vcl_endl;
    vcl_cout<<"camera x scale "<<finalCamera.get_calibration().x_scale()<<vcl_endl;

    
    return true;
}



/*
   ******************************************** VilTopviewPointlessCalibWarp ************************************************
 */

VilTopviewPointlessCalibWarp::VilTopviewPointlessCalibWarp()
{
    
}
VilTopviewPointlessCalibWarp::~VilTopviewPointlessCalibWarp()
{
    
}

bool VilTopviewPointlessCalibWarp::warpTopviewToImageCalib(const vil_image_view<vxl_byte> & topview, const vil_image_view<vxl_byte> & image, int iterator_num, int radius, int wt_radius,
                                                           const vpgl_perspective_camera<double> & initCamera, vpgl_perspective_camera<double> & finalCamera)
{
    assert(topview.nplanes() == 3);
    assert(image.nplanes()   == 3);
    
    if (image_gradient_cached_file_.empty()) {
        vcl_cout<<"Error: must have cached file, it could be image name.\n";
        return false;
    }
    
    DisneyWorldBasketballCourt court;
    
    vil_image_view<vxl_byte> grey_topview;
    vil_convert_planes_to_grey(topview, grey_topview);
    vil_image_view<double> topview_double = vil_quantize::dequantize<double>(grey_topview);
    //VilPlusExtra::vil_save(topview_double, "warpTopview_topview_grey.jpg");
    
    //gradient of magnitude of topview image
    LongGradientPair topviewPair;
    {
        vcl_string gradientFileName = court.name() + "_topview_gradient_" + vul_string(radius) + ".vnl_mat";
        vil_image_view<double> Ix;
        vil_image_view<double> Iy;
        lkCachedGradient(topview_double, gradientFileName, radius, Ix, Iy);
        vcl_cout<<"save to : "<<gradientFileName<<vcl_endl;
        
        topviewPair.image = topview_double;
        topviewPair.Ix = Ix;
        topviewPair.Iy = Iy;
        topviewPair.gradientCache = gradientFileName;
        topviewPair.radius = radius;
        
        vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string(court.name() + "_topview_" + vul_string( radius ) + "_x.jpg"));
        vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string(court.name() + "_topview_" + vul_string( radius ) + "_y.jpg"));
    }
    
    //gradient of magnitude of camera image
    vil_image_view<vxl_byte> grey_image;
    vil_convert_planes_to_grey(image, grey_image);
    vil_image_view<double> image_double = vil_quantize::dequantize<double>(grey_image);
    //VilPlusExtra::vil_save(image_double, "warpTopview_grey_image.jpg");
    
    LongGradientPair imagePair;
    {
        vcl_string gradientFileName = image_gradient_cached_file_ + "_grey_image_gradient_" + vul_string(radius) + ".vnl_mat";
        vil_image_view<double> Ix;
        vil_image_view<double> Iy;
        lkCachedGradient(image_double, gradientFileName, radius, Ix, Iy);
        vcl_cout<<"save to : "<<gradientFileName<<vcl_endl;
        
        imagePair.image = image_double;
        imagePair.Ix = Ix;
        imagePair.Iy = Iy;
        imagePair.gradientCache = gradientFileName;
        imagePair.radius = radius;
        
        vil_save(vil_quantize::quantize<vxl_byte>(Ix, true), vcl_string(vcl_string("grey_image_") + vul_string( radius ) + "_x.jpg"));
        vil_save(vil_quantize::quantize<vxl_byte>(Iy, true), vcl_string(vcl_string("grey_image_") + vul_string( radius ) + "_y.jpg"));
    }
    
    vil_image_view<double> wtImage;
    court.getWeightImageWithLogo(initCamera, image.ni(), image.nj(), wt_radius, 100, wtImage);
   // VilPlusExtra::vil_save(wtImage, vcl_string(court.name() + "_weight_image.jpg").c_str());
    
    
    vgl_transform_2d<double> curModel = court.imageToWorld();
    
    NaturalCameraImageToPitchWarp curWarp(initCamera, curModel);
    
    double iterationError;
    
    lkForwardsAdditive(topviewPair.image, imagePair.image,  wtImage, curWarp, topviewPair.Ix, topviewPair.Iy, &iterationError, iterator_num, 0);
    
    finalCamera = curWarp.camera();
    
    return true;
}








