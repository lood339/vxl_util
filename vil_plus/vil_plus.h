//
//  vil_plus.h
//  OnlineStereo
//
//  Created by jimmy on 2/3/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vil_plus__
#define __OnlineStereo__vil_plus__

#include <vil/vil_image_view.h>
#include <vil/vil_load.h>
#include <vcl_vector.h>
#include <vgl/algo/vgl_h_matrix_2d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vgl/vgl_line_segment_3d.h>


class VilPlus
{
    public:
    
    static void vil_load(const char *file_name, unsigned int nChannels, vil_image_view<double> &outImage);
   
    
    static void vil_save(const vil_image_view<vxl_byte> & image, char const* filename, bool print_logo = true);
    static void vil_save(const vil_image_view<bool> &image, char const* filename, bool print_logo = true);
    
    // save to .mat file
    static void vil_save_as_vnl_matrix(const vil_image_view<double> & image, char const* filename, bool print_logo = true);
    
    // gray or rgb data
    static unsigned char * vil_malloc(const vil_image_view<vxl_byte> & image);
    static vil_image_view<vxl_byte> gray2Rgb(const vil_image_view<vxl_byte> & image);    
    
   
    
    // format to double, [0 255)
    static void vil_byteToDouble(const vil_image_view<vxl_byte> & image, vil_image_view<double> & doubleImage);
    
    
    
    // sum of squared distance
    static double vil_ssd(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2);
    
    
  //  static vil_image_view<double> vil_to_double(const vil_image_view<vxl_byte> & image);
    static vil_image_view<vxl_byte> vil_to_gray(const vil_image_view<vxl_byte> & image);
    
    // srcImage and dstImage must has same plane number
    static bool homography_warp_fill(const vil_image_view<vxl_byte>& srcImage,
                                     const vgl_h_matrix_2d<double> &H,
                                     const vil_image_view<vxl_byte> &dstImage,
                                     vil_image_view<vxl_byte> &outImage);
    
    // outImageAlpha: one channel 255-->inside, 0 --> outside
    static bool homography_warp_fill(const vil_image_view<vxl_byte>& srcImage,
                                     const vgl_h_matrix_2d<double> &H,
                                     const vil_image_view<vxl_byte> &dstImage,
                                     vil_image_view<vxl_byte> &outImage,
                                     vil_image_view<vxl_byte> &outImageAlpha);
    
    //  int myints[] = {16,2,77,29}
    
    // static void draw_cross(vil_image_view<vxl_byte> &image, const vcl_vector< vgl_point_2d<double>> &pts, int crossWidth,
    //                        const vcl_vector< vxl_byte >& colour  = vcl_vector<vxl_byte>({0, 255, 0}), int lineWidth = 1);
    
    static void draw_cross(vil_image_view<vxl_byte> &image, const vcl_vector< vgl_point_2d<double>> &pts, int crossWidth,
                           const vcl_vector< vxl_byte >& colour, int lineWidth = 1);
    
    static void draw_cross(vil_image_view<vxl_byte> &image, const vgl_point_2d<double> & pt, int crossWidth,
                           const vcl_vector< vxl_byte >& colour, int lineWidth = 1);
    
    static void draw_dot(vil_image_view<vxl_byte> & image, const vcl_vector< vgl_point_2d<double>> &pts,
                         const vcl_vector< vxl_byte >& colour);
    
    static void draw_edge(vil_image_view<vxl_byte> &image,
                          const vgl_point_2d<double> & startPt,
                          const vgl_point_2d<double> & endPt,
                          const vcl_vector<vxl_byte> & colour,
                          int lineWidth = 1);
    
    static void draw_segment(vil_image_view<vxl_byte> &image,
                             const vgl_point_2d<double> & p1,
                             const vgl_point_2d<double> & p2,
                             const vcl_vector<vxl_byte> & colour,
                             int lineWidth = 1);
    static void draw_segment(vil_image_view<vxl_byte> & image,
                             const vgl_line_segment_2d<double> & seg,
                             const vcl_vector<vxl_byte> & colour,
                             int lineWidth = 1);
    static void draw_segments(vil_image_view<vxl_byte> & image,
                              const vcl_vector<vgl_line_segment_2d<double> > & segments,
                              const vcl_vector<vxl_byte> & colour, int lineWidth = 1);
    
    static void draw_arrow(vil_image_view<vxl_byte> & image,
                           const vgl_point_2d<double> & p1,
                           const vgl_point_2d<double> & p2,
                           const vcl_vector<vxl_byte> & colour,
                           int lineThickness = 1);
    
    static void draw_box(vil_image_view<vxl_byte> & image, const vgl_box_2d<double> &box, const vcl_vector<vxl_byte> & colour, int lineThickness = 2);
    
    static void draw_line(vil_image_view<vxl_byte> & image, const vgl_line_2d<double> & line,
                          const vcl_vector<vxl_byte> & colour, int lineWidth = 2);
    
    static void draw_color_lines(vil_image_view<vxl_byte> & image, vcl_vector<vgl_line_2d<double> > & lines, int lineWidth = 2);    
    
    static void draw_lines(vil_image_view<vxl_byte> & image, const vcl_vector< vgl_line_2d<double> > & lines,
                           const vcl_vector<vxl_byte> & colour, int lineWidth = 2);
    
    
    
    
    
    
    
    static void draw_match(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                           const vcl_vector< vgl_point_2d<double> > & pts1,
                           const vcl_vector< vgl_point_2d<double> > & pts2,
                           vil_image_view<vxl_byte> &matches, const int thickness = 1);
    
    // horizontally combine images
    static void combine_images(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2,
                                vil_image_view<vxl_byte> & combined);
    
    // combine multiple image
    static void combine_images(const vcl_vector<vil_image_view<vxl_byte> > & images, vil_image_view<vxl_byte> & combinedImage, double scale = 1.0);
    
    
    
    static void draw_match_segment(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                                   const vcl_vector< vgl_line_segment_2d<double> > & line1,
                                   const vcl_vector< vgl_line_segment_2d<double> > & line2,
                                   vil_image_view<vxl_byte> &matches, const int thickness = 1);
    
    // draw dense matches by shor lines
    static void draw_match_dense(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                                 const vcl_vector< vgl_point_2d<double> > & pts1,
                                 const vcl_vector< vgl_point_2d<double> > & pts2,
                                 const vcl_vector<bool> & inliers,
                                 vil_image_view<vxl_byte> &matches,
                                 const int thickness = 1);
    
    //sp: start position
    //ep: end position
    static void draw_projected_line(const vpgl_perspective_camera<double> &camera, const vgl_point_3d<double> &sp, const vgl_point_3d<double> &ep,
                                    vil_image_view<vxl_byte> & image, int colorIdx = 2, int lineWidth = 5);
    
    static void draw_projected_box_line(const vpgl_perspective_camera<double> &camera, const vcl_vector<vgl_point_3d<double> > & vertex,
                                        const vcl_vector<vxl_byte> & color, vil_image_view<vxl_byte> & image);
    
    // draw projected line in the image.
    // the original image is in the center of "outImage", with black black ground
    static void draw_line_on_background(const vpgl_perspective_camera<double> & camera,
                                        const vcl_vector<vgl_line_segment_3d<double> > & segs,
                                        const vil_image_view<vxl_byte> & image,
                                        vil_image_view<vxl_byte> & outImage,
                                        const vcl_vector<vxl_byte> & colour, int line_thickness = 2);
    
    // circle parrel with x-y plane
    static void draw_projected_circle(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image,
                                      const vgl_point_2d<double> & center,
                                      const double height = 0.0, const double radius = 0.5, int colorIdx = 2);
    
    static void draw_projected_circle(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image,
                                      const vgl_point_2d<double> & center, const vcl_vector<vxl_byte> & colour,
                                      const double height = 0.0, const double radius = 0.5);
    
    //draw the major and minor axis of an ellipse
    // ellipse in format:
    // a b d
    // b c f
    // d f g  with a = 1.0
    static void draw_ellipse_majorminor_axis(vil_image_view<vxl_byte> & image, const vnl_vector_fixed<double, 5> & ellipseEquation,
                                             const vcl_vector<vxl_byte> & colour);
    
    
    
    
    // project a cylinder (from h1 to h2) into image, calculate projected rectangle area
    static void project_cylinder_rectangle(const vpgl_perspective_camera<double> & camera, const int imageWidth, const int imageHeight,
                                           const vgl_point_2d<double> & center,
                                           const double h1, const double h2, const double radius,
                                           vgl_point_2d<double> & topLeft, vgl_point_2d<double> & bottomRight);
    
    static void draw_rectangle(vil_image_view<vxl_byte> & image, const vgl_point_2d<double> & topLeft, const vgl_point_2d<double> & bottomRight, int colorIdx);
    
    // draw an image sequence into a grid, for visualization purpose
    static void draw_image_grid(const vcl_vector<vil_image_view<vxl_byte> > & images, int nCols, vil_image_view<vxl_byte> & gridImage, int gapWidth = 5);
    
    
    static void visualize_gradient_direction_hsv(vil_image_view<vxl_byte> &image, vil_image_view<vxl_byte> &gradient_direction);
    
    //    h = [0,360], s = [0,1], v = [0,1]
    //    rgb in [0 255]
    static void vil_convert_hsv_to_rgb(const vil_image_view<double> &src, vil_image_view<vxl_byte> &dest);
    
    // rgb to YUV
    // Y [0, 256)
    // U [-128, 128)
    // V [-128, 128)
    // U, V shift to [0, 256)
    static void vil_rgb_to_yuv(const vil_image_view<vxl_byte> & src, vil_image_view<vxl_byte> & dest);
    
    // H: [0, 360)
    // S: [0, 1];
    // V: [0, 255]
    static void vil_rgb_to_hsv(const vil_image_view<vxl_byte> & src, vil_image_view<double> & hsv);
    
    
    //
    static bool homography_warp_alpha(const vil_image_view<vxl_byte>& inImage,
                                      const vgl_h_matrix_2d<double> &H,
                                      int width, int height,
                                      vil_image_view<vxl_byte> &outImageWithAlpha);
    // with black pixels
    static bool homography_warp(const vil_image_view<vxl_byte>& inImage,
                                const vgl_h_matrix_2d<double> &H,
                                int width, int height,
                                vil_image_view<vxl_byte> &outImage);
    
    static bool homography_warp(const vil_image_view<double> & orgImage,
                                const vgl_h_matrix_2d<double> &H,
                                int width, int height,
                                vil_image_view<double> & outImage);
    // H: homography from image A to image B has size (imageWidth, imageHeight)
    // ratio: areas in image (B) is covered by warped image. [0 1)
    // type: 0 --> left, 1 --> right
    static bool warpOverlayRatio(const vgl_h_matrix_2d<double> & H, int imageWidth, int imageHeight, double & ratio, int & type);
    
    //
    static void vil_median_image(const vcl_vector<vil_image_view<vxl_byte> > & images, vil_image_view<vxl_byte> & medianImage);
    
    // images is rgb a
    static void vil_median_image_with_alpha(const vcl_vector<vil_image_view<vxl_byte> > & images, vil_image_view<vxl_byte> & medianImage);
    
    
    
    
 
    
    // find a corresponding patch (locate in initP in kernelImage) in destImage
    // finalP: position in destImage
    // assume kernelImage as same size as destImage, and had similar content around initP
    static bool vil_refine_patch_position(const vil_image_view<vxl_byte> & kernelImage, const vil_image_view<vxl_byte> & destImage,
                                          const vgl_point_2d<double> & initP, int patchSize, int searchSize, vgl_point_2d<double> & finalP);
    
    // find single corresponding patch match from kernalImage to destImage
    static bool vil_refine_patch_position(const vil_image_view<vxl_byte> & kernelImage, const vgl_point_2d<double> & kernelP,
                                          const vil_image_view<vxl_byte> & destImage, const vgl_point_2d<double> & initP,
                                          int patchSize, int searchSize, vgl_point_2d<double> & finalP);
    
    // find a group of corresponding patch matches
    // (-1, -1) in finalP for miss-matching
    // kernalPts: center position of points in kernalImage, the patch should be inside of image
    static bool vil_refine_patch_position(const vil_image_view<vxl_byte> & kernelImage, const vcl_vector<vgl_point_2d<double> > & kernelPts,
                                          const vil_image_view<vxl_byte> & destImage, const vcl_vector<vgl_point_2d<double> > & initPts,
                                          int patchSize, int searchSize, vcl_vector<vgl_point_2d<double> > & finalP);
    
    // collect all connect the pixels into different set
    // minSetSize: minimum set size
    // return: set number < 256
    static int connect_component_label(const vil_image_view<vxl_byte> & biImage, unsigned int minSetSize,
                                       vil_image_view<vxl_byte> & labelImage,
                                       vcl_vector<vcl_vector<vgl_point_2d<double> > > & labelPts);
    
    // draw a line inside of w x h image
    // abandoned functions
    static bool draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts);
    static bool draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts, int w, int h);
    // pixel position in line end point are just reasonable good.
    
    
       // linePts inside of image area
    static bool draw_line_segment(const vgl_line_segment_2d<double> & line_segment, const double thickness,
                                       const int w, const int h,
                                       vcl_vector<vgl_point_2d<double> > & linePts);
    
    static vcl_vector<vxl_byte> blue(void);
    static vcl_vector<vxl_byte> green(void);
    static vcl_vector<vxl_byte> red(void);
    static vcl_vector<vxl_byte> white(void);
    static vcl_vector<vxl_byte> hotPink(void);
    static vcl_vector<vxl_byte> yellow(void);
    static vcl_vector<vxl_byte> purple(void);
    
    
private:
    // linePts may out of image area
    static bool draw_line_segment_unsafe(const vgl_line_segment_2d<double> & line_segment, const double thickness,
                                  const int w, const int h,
                                  vcl_vector<vgl_point_2d<double> > & linePts);
    
    // abadoned
    static bool draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, double width, vcl_vector<vgl_point_2d<double> > & linePts);
    static bool draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, double width, vcl_vector<vgl_point_2d<double> > & linePts, int w, int h);

    
};


#endif /* defined(__OnlineStereo__vil_plus__) */
