//
//  vil_plus.cpp
//  OnlineStereo
//
//  Created by jimmy on 2/3/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vil_plus.h"
#include <vxl_config.h>
#include <vil/vil_image_view.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
#include <vil/vil_convert.h>

#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vgl/vgl_distance.h>

#include <vsl/vsl_binary_io.h>
#include <vgl/io/vgl_io_point_2d.h>

#include <vpgl/algo/vpgl_camera_compute.h>

#include <vil/algo/vil_sobel_3x3.h>


#include <vil/vil_warp.h>
#include <vil/vil_copy.h>
#include <vnl/vnl_math.h>

#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>
#include <vnl/vnl_inverse.h>
#include <vcl_numeric.h>
#include <vnl/io/vnl_io_matrix.h>
#include <vnl/vnl_matlab_read.h>
#include <vnl/vnl_matlab_filewrite.h>
#include <vil/vil_crop.h>

#include <vnl/algo/vnl_matrix_inverse.h>

#include <vnl/algo/vnl_determinant.h>
#include <vil/algo/vil_colour_space.h>
#include <vcl_stack.h>
#include <vil/algo/vil_gauss_filter.h>
#include <vil/algo/vil_colour_space.h>
#include <vcl_algorithm.h>
#include <vil/vil_image_view.h>
#include <vil/vil_bilin_interp.h>
#include <vgl/vgl_box_2d.h>
#include <vgl/vgl_intersection.h>



void VilPlus::vil_load(const char *file_name, unsigned int nChannels, vil_image_view<double> &outImage)
{
    assert(nChannels == 1 || nChannels == 3);
    vil_image_view<vxl_byte> image = ::vil_load(file_name);
    if (image) {
        if (nChannels == 1 && image.nplanes() == 3) {
            vil_image_view<vxl_byte> temp;
            vil_convert_planes_to_grey(image, temp);
            image = temp;
        }
        vil_convert_cast(image, outImage);
    }
}



void VilPlus::vil_save(const vil_image_view<vxl_byte> & image, char const* filename, bool print_logo)
{
    bool isSaveOk = ::vil_save(image, filename);
    if (print_logo && isSaveOk) {
        vcl_cout<<"save to: "<<filename<<vcl_endl;
    }
}

void VilPlus::vil_save(const vil_image_view<bool> &image, char const* filename, bool print_logo)
{
    vil_image_view<vxl_byte> bwImage(image.ni(), image.nj(), 1);
    bwImage.fill(0);
    for (int j = 0; j<image.nj(); j++) {
        for (int i = 0; i<image.ni(); i++) {
            if (image(i, j)) {
                bwImage(i, j) = 255;
            }
        }
    }
    VilPlus::vil_save(bwImage, filename, print_logo);
}


void VilPlus::vil_save_as_vnl_matrix(const vil_image_view<double> & image, char const* filename, bool print_logo)
{
    assert(image.nplanes() == 1);
    vnl_matrix<double> data(image.nj(), image.ni(), 1);
    data.fill(0);
    for (int j = 0; j<image.nj(); j++) {
        for (int i = 0; i<image.ni(); i++) {
            data(j, i) = image(i, j);
        }
    }
    
    vnl_matlab_filewrite awriter(filename);
    awriter.write(data, vcl_string(filename).substr(0, strlen(filename) - 4).c_str());
    
    if (print_logo) {
        printf("save to %s\n", filename);
    }
}

unsigned char * VilPlus::vil_malloc(const vil_image_view<vxl_byte> & image)
{
    unsigned char *data = NULL;
    const int w = image.ni();
    const int h = image.nj();
    const int nPlane = image.nplanes();
    data = new unsigned char[w * h * nPlane];
    assert(data);
    
    for (int j = 0; j<h; j++) {
        for (int i =0 ; i<w; i++) {
            for (int k = 0; k<nPlane; k++) {
                data[j * w + i + k] = image(i, j, k);
            }
        }
    }
    return data;
}

vil_image_view<vxl_byte> VilPlus::gray2Rgb(const vil_image_view<vxl_byte> & image)
{    
    assert(image.nplanes() == 1);
    const int w = image.ni();
    const int h = image.nj();
    
    vil_image_view<vxl_byte> rgbImage = vil_image_view<vxl_byte>(w, h, 3);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            rgbImage(i, j, 0) = image(i, j);
            rgbImage(i, j, 1) = image(i, j);
            rgbImage(i, j, 2) = image(i, j);
        }
    }
    return rgbImage;
}


void VilPlus::vil_byteToDouble(const vil_image_view<vxl_byte> & image, vil_image_view<double> & doubleImage)
{
    // rgb to gray
    vil_image_view<vxl_byte> gray;
    if (image.nplanes() == 3) {
        gray = VilPlus::vil_to_gray(image);
    }
    else
    {
        gray = image;
    }
    
    doubleImage = vil_image_view<double>(gray.ni(), gray.nj(), 1);
    for (int j = 0; j<gray.nj(); j++) {
        for (int i = 0; i<gray.ni(); i++) {
            doubleImage(i, j) = gray(i, j);
        }
    }
}



double VilPlus::vil_ssd(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2)
{
    assert(image1.ni() == image2.ni());
    assert(image1.nj() == image2.nj());
    
    double ssd = 0;
    for (int j = 0; j<image1.nj(); j++) {
        for (int i = 0; i<image1.ni(); i++) {
            for (int p = 0; p<image1.nplanes(); p++) {
                double dif = image1(i, j, p) - image2(i, j, p);
                ssd += dif * dif;
            }
        }
    }
    return ssd;
}


vil_image_view<vxl_byte> VilPlus::vil_to_gray(const vil_image_view<vxl_byte> & image)
{
    assert(image.nplanes() == 3);
    
    vil_image_view<vxl_byte> grayImage;
    vil_convert_planes_to_grey(image, grayImage);
    return grayImage;
}



//inverse H mapping from court (world) to image space
class VxlPlusInvHwarp
{
    public:
    VxlPlusInvHwarp(const vgl_h_matrix_2d<double> &invH)
    {
        invH_ = invH;
    }
    void operator()(const double ox, const double oy, double &ix, double &iy)
    {
        vgl_homg_point_2d<double> p = invH_((vgl_homg_point_2d<double>(ox, oy)));
        ix = p.x()/p.w();
        iy = p.y()/p.w();
    }
    private:
    vgl_h_matrix_2d<double> invH_;
    
};


//bilinear interperation
static vxl_byte InterpolatorFunc( vil_image_view< vxl_byte > const& view, double x, double y, unsigned p )
{
	return vil_bilin_interp_safe( view, x, y, p);
}

static double InterpolatorFuncDouble(vil_image_view<double> const &view, double x, double y, unsigned p)
{
    return vil_bilin_interp_safe( view, x, y, p);
}


bool VilPlus::homography_warp_fill(const vil_image_view<vxl_byte>& srcImage,
                                   const vgl_h_matrix_2d<double> &H,
                                   const vil_image_view<vxl_byte> &destImage,
                                   vil_image_view<vxl_byte> &outImage)
{
    assert(srcImage.nplanes() == destImage.nplanes());
    assert(srcImage.nplanes() == outImage.nplanes());
    
    vgl_h_matrix_2d<double> invH = H.get_inverse();
    
    vil_warp(srcImage, outImage, VxlPlusInvHwarp(invH), InterpolatorFunc);
    
    for (int h = 0; h<outImage.nj(); h++) {
        for (int w =0; w<outImage.ni(); w++) {
            vgl_homg_point_2d<double> p = invH(vgl_homg_point_2d<double>(w, h, 1.0));
            double px = p.x()/p.w();
            double py = p.y()/p.w();
            
            //out of image
            if ((px < 0 || px >= srcImage.ni()) || (py < 0 || py >= srcImage.nj())) {
                for (int i = 0; i<destImage.nplanes(); i++) {
                    outImage(w, h, i) = destImage(w, h, i);
                }
            }
        }
    }
    return true;
}

bool VilPlus::homography_warp_fill(const vil_image_view<vxl_byte>& srcImage,
                                   const vgl_h_matrix_2d<double> &H,
                                   const vil_image_view<vxl_byte> &destImage,
                                   vil_image_view<vxl_byte> &outImage,
                                   vil_image_view<vxl_byte> &outImageAlpha)
{
    assert(srcImage.nplanes() == destImage.nplanes());
    assert(srcImage.nplanes() == outImage.nplanes());
    
    vgl_h_matrix_2d<double> invH = H.get_inverse();
    vil_warp(srcImage, outImage, VxlPlusInvHwarp(invH), InterpolatorFunc);
    
    outImageAlpha = vil_image_view<vxl_byte>(outImage.ni(), outImage.nj(), 1);
    outImageAlpha.fill(255);
    
    for (int h = 0; h<outImage.nj(); h++) {
        for (int w =0; w<outImage.ni(); w++) {
            vgl_homg_point_2d<double> p = invH(vgl_homg_point_2d<double>(w, h, 1.0));
            double px = p.x()/p.w();
            double py = p.y()/p.w();
            
            //out of image
            if ((px < 0 || px >= srcImage.ni()) || (py < 0 || py >= srcImage.nj())) {
                for (int i = 0; i<destImage.nplanes(); i++) {
                    outImage(w, h, i) = destImage(w, h, i);
                }
                outImageAlpha(w, h) = 0;
            }
        }
    }
    return true;
}

static void vicl_overlay_line_segment( vil_image_view< vxl_byte >& image, vgl_line_segment_2d< double > const& segment,
                                      vcl_vector< vxl_byte > const& colour, const unsigned int thickness = 1)
{
    printf("Warning: vicl_overlay_line_segment do no thing. Draw lines are not supported.\n");
}

void VilPlus::draw_cross(vil_image_view<vxl_byte> &image, const vcl_vector< vgl_point_2d<double>> &pts, int crossWidth, const vcl_vector< vxl_byte >& colour, int lineWidth)
{
    assert(image.nplanes() == 3);
    assert(colour.size() == 3);
    
    for (unsigned int i = 0; i<pts.size(); i++)
    {
        //center point
        double px = pts[i].x();
        double py = pts[i].y();
        
        vgl_point_2d<double> p1, p2, p3, p4;
        
        double h_l = crossWidth;
        p1.set(px - h_l, py);
        p2.set(px + h_l, py);
        p3.set(px, py - h_l);
        p4.set(px, py + h_l);
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), colour, lineWidth);
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p3, p4), colour, lineWidth);
    }
}

void VilPlus::draw_cross(vil_image_view<vxl_byte> &image, const vgl_point_2d<double> & pt, int crossWidth,
                         const vcl_vector< vxl_byte >& colour, int lineWidth)
{
    assert(image.nplanes() == 3);
    assert(colour.size() == 3);
    
    //center point
    double px = pt.x();
    double py = pt.y();
    
    vgl_point_2d<double> p1, p2, p3, p4;
    
    double h_l = crossWidth;
    p1.set(px - h_l, py);
    p2.set(px + h_l, py);
    p3.set(px, py - h_l);
    p4.set(px, py + h_l);
    
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), colour, lineWidth);
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p3, p4), colour, lineWidth);
}

void VilPlus::draw_dot(vil_image_view<vxl_byte> & image, const vcl_vector< vgl_point_2d<double>> & pts, const vcl_vector< vxl_byte >& colour)
{
    assert(image.nplanes() == 3);
    assert(colour.size() == 3);
    
    for (unsigned int i = 0; i<pts.size(); i++)
    {
        //center point
        double px = pts[i].x();
        double py = pts[i].y();
        image(px, py, 0) = colour[0];
        image(px, py, 1) = colour[1];
        image(px, py, 2) = colour[2];
    }
}

void VilPlus::draw_edge(vil_image_view<vxl_byte> &image,
                        const vgl_point_2d<double> & startPt,
                        const vgl_point_2d<double> & endPt,
                        const vcl_vector<vxl_byte> & colour,
                        int lineWidth)
{
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(startPt, endPt), colour, lineWidth);
}

void VilPlus::draw_segment(vil_image_view<vxl_byte> &image,
                           const vgl_point_2d<double> & p1,
                           const vgl_point_2d<double> & p2,
                           const vcl_vector<vxl_byte> & colour,
                           int lineWidth)
{
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), colour, lineWidth);
}

void VilPlus::draw_segment(vil_image_view<vxl_byte> & image,
                           const vgl_line_segment_2d<double> & seg,
                           const vcl_vector<vxl_byte> & colour,
                           int lineWidth)
{
    vicl_overlay_line_segment(image, seg, colour, lineWidth);
}

void VilPlus::draw_segments(vil_image_view<vxl_byte> & image,
                            const vcl_vector<vgl_line_segment_2d<double> > & segments,
                            const vcl_vector<vxl_byte> & colour, int lineWidth)
{
    for (int i = 0; i<segments.size(); i++) {
        vicl_overlay_line_segment(image, segments[i], colour, lineWidth);
    }
}

void VilPlus::draw_arrow(vil_image_view<vxl_byte> & image,
                         const vgl_point_2d<double> & p1,
                         const vgl_point_2d<double> & p2,
                         const vcl_vector<vxl_byte> & colour,
                         int lineThickness)
{
    vgl_line_segment_2d<double> seg(p1, p2);
    vicl_overlay_line_segment(image, seg, colour, lineThickness);
    
    // CCW may not right
    vgl_vector_2d<double> dir = rotated(seg.direction(), (180 - 45.0/2)/180.0*vnl_math::pi);
    dir = normalize(dir);
    
    vgl_vector_2d<double> dp = 10.0 * dir;
    vgl_point_2d<double> p3(p2.x() + dp.x(), p2.y() + dp.y());
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p2, p3), colour, 1);
    
    dir = rotated(seg.direction(), (180 + 45.0/2)/180.0*vnl_math::pi);
    dp = 10.0 * dir;
    p3 = vgl_point_2d<double>(p2.x() + dp.x(), p2.y() + dp.y());
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p2, p3), colour, 1);
}

void VilPlus::draw_box(vil_image_view<vxl_byte> & image, const vgl_box_2d<double> &box, const vcl_vector<vxl_byte> & colour, int lineThickness)
{
    double x1 = box.min_x();
    double y1 = box.min_y();
    double x2 = box.max_x();
    double y2 = box.max_y();
    
    vgl_point_2d<double> p1 = vgl_point_2d<double>(x1, y1);
    vgl_point_2d<double> p2 = vgl_point_2d<double>(x2, y1);
    vgl_point_2d<double> p3 = vgl_point_2d<double>(x2, y2);
    vgl_point_2d<double> p4 = vgl_point_2d<double>(x1, y2);
    
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), colour, lineThickness);
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p2, p3), colour, lineThickness);
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p3, p4), colour, lineThickness);
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p4, p1), colour, lineThickness);
}


void VilPlus::draw_line(vil_image_view<vxl_byte> & image, const vgl_line_2d<double> & line,
                        const vcl_vector<vxl_byte> & colour, int lineWidth)
{
    assert(image.nplanes() == 3);
    
    int w = image.ni();
    int h = image.nj();
    vgl_point_2d<double> p1(0, 0);
    vgl_point_2d<double> p2(w, 0);
    vgl_point_2d<double> p3(w, h);
    vgl_point_2d<double> p4(0, h);
    
    vgl_line_2d<double> line1(p1, p2);
    vgl_line_2d<double> line2(p2, p3);
    vgl_line_2d<double> line3(p3, p4);
    vgl_line_2d<double> line4(p4, p1);
    
    vcl_vector<vgl_point_2d<double> > pts;
    vgl_point_2d<double> p;
    // horizontal
    bool isIntersect = vgl_intersection(line1, line, p);
    if (isIntersect && p.x() >= 0 && p.x() < w) {
        pts.push_back(p);
    }
    
    // vertical
    isIntersect = vgl_intersection(line2, line, p);
    if (isIntersect && p.y() >= 0 && p.y() < h) {
        pts.push_back(p);
    }
    
    // horizontal
    isIntersect = vgl_intersection(line3, line, p);
    if (isIntersect && p.x() >= 0 && p.x() < w) {
        pts.push_back(p);
    }
    
    // vertical
    isIntersect = vgl_intersection(line4, line, p);
    if (isIntersect && p.y() >= 0 && p.y() < h) {
        pts.push_back(p);
    }
    
    if (pts.size() == 2) {
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(pts[0], pts[1]), colour, lineWidth);
    }
    else
    {
        printf("Warning: can not draw 2d line from ax + by + c = 0\n");
    }
}

void VilPlus::draw_color_lines(vil_image_view<vxl_byte> & image, vcl_vector<vgl_line_2d<double> > & lines, int lineWidth)
{
    vcl_vector<vcl_vector<vxl_byte> > colours;
    colours.push_back(VilPlus::red());
    colours.push_back(VilPlus::green());
    colours.push_back(VilPlus::blue());
    colours.push_back(VilPlus::yellow());
    colours.push_back(VilPlus::white());
    colours.push_back(VilPlus::hotPink());
    
    for (int i = 0; i<lines.size(); i++) {
        if (i < colours.size()) {
            VilPlus::draw_line(image, lines[i], colours[i]);
        }
        else
        {
            VilPlus::draw_line(image, lines[i], VilPlus::green());
        }
    }
    
}

void VilPlus::draw_lines(vil_image_view<vxl_byte> & image, const vcl_vector< vgl_line_2d<double> > & lines,
                         const vcl_vector<vxl_byte> & colour, int lineWidth)
{
    for (int i = 0; i<lines.size(); i++) {
        VilPlus::draw_line(image, lines[i], colour, lineWidth);
    }
}




void VilPlus::draw_match(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                         const vcl_vector< vgl_point_2d<double> > & pts1,
                         const vcl_vector< vgl_point_2d<double> > & pts2,
                         vil_image_view<vxl_byte> &matches, const int thickness)
{
    assert(image1.nplanes() == 3);
    assert(image2.nplanes() == 3);
    assert(pts1.size() == pts2.size());
    
    int gap_width = 10;
    int x_shift = gap_width + image1.ni();
    int width  = image2.ni() + x_shift;
    int height = vcl_max(image1.nj(), image2.nj());
    
    matches = vil_image_view<vxl_byte>(width, height, 3);
    matches.fill(0);
    
    //copy image 1 to left
    vil_copy_to_window(image1, matches, 0, 0);
    
    //copy image 2 to right
    vil_copy_to_window(image2, matches, x_shift, 0);
    
    for (int i = 0; i<pts1.size(); i++) {
       // int r = rand()%255;
       // int g = rand()%255;
      //  int b = rand()%255;
        int r = 0;
        int g = 255;
        int b = 0;
        
        vcl_vector< vxl_byte> colour;
        colour.push_back(r);
        colour.push_back(g);
        colour.push_back(b);
        
        vgl_point_2d<double> p1 = pts1[i];
        vgl_point_2d<double> p2(vgl_point_2d<double>(pts2[i].x() + x_shift, pts2[i].y()));
        
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(p1, p2), colour, thickness);
        
        // cross in left
        
        vgl_point_2d<double> q1, q2, q3, q4;
        double h_l = 5;
        q1.set(p1.x() - h_l, p1.y());
        q2.set(p1.x() + h_l, p1.y());
        q3.set(p1.x(), p1.y() - h_l);
        q4.set(p1.x(), p1.y() + h_l);
        
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(q1, q2), colour, 2);
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(q3, q4), colour, 2);
        
        // cross in right
        q1.set(p2.x() - h_l, p2.y());
        q2.set(p2.x() + h_l, p2.y());
        q3.set(p2.x(), p2.y() - h_l);
        q4.set(p2.x(), p2.y() + h_l);
        
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(q1, q2), colour, 2);
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(q3, q4), colour, 2);
    }
}

void VilPlus::combine_images(const vil_image_view<vxl_byte> & image1, const vil_image_view<vxl_byte> & image2,
                              vil_image_view<vxl_byte> & combined)
{
    assert(image1.nplanes() == image2.nplanes());
    
    int gap_width = 10;
    int x_shift = gap_width + image1.ni();
    int width  = image2.ni() + x_shift;
    int height = vcl_max(image1.nj(), image2.nj());
    
    combined = vil_image_view<vxl_byte>(width, height, 3);
    combined.fill(0);
    
    //copy image 1 to left
    vil_copy_to_window(image1, combined, 0, 0);
    
    //copy image 2 to right
    vil_copy_to_window(image2, combined, x_shift, 0);
}

void VilPlus::combine_images(const vcl_vector<vil_image_view<vxl_byte> > & images, vil_image_view<vxl_byte> & combinedImage, double scale)
{
    assert(images.size() >= 2);
    
    vil_image_view<vxl_byte> leftImage;
    leftImage.deep_copy(images[0]);
    for (int i = 1; i<images.size(); i++) {
        vil_image_view<vxl_byte> rightImage = images[i];
        VilPlus::combine_images(leftImage, rightImage, combinedImage);
        leftImage.deep_copy(combinedImage);
    }
    combinedImage.deep_copy(leftImage);
}

void VilPlus::draw_match_segment(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                                 const vcl_vector< vgl_line_segment_2d<double> > & line1,
                                 const vcl_vector< vgl_line_segment_2d<double> > & line2,
                                 vil_image_view<vxl_byte> &matches, const int thickness)
{
    assert(image1.nplanes() == 3);
    assert(image2.nplanes() == 3);
    assert(line1.size() == line2.size());
    
    int gap_width = 10;
    int x_shift = gap_width + image1.ni();
    int width  = image1.ni() + x_shift;
    int height = vcl_max(image1.nj(), image2.nj());
    
    matches = vil_image_view<vxl_byte>(width, height, 3);
    matches.fill(0);
    
    //copy image 1 to left
    vil_copy_to_window(image1, matches, 0, 0);
    //copy image 2 to right
    vil_copy_to_window(image2, matches, x_shift, 0);
    
    // loop over all line segment
    for (int i = 0; i<line1.size(); i++) {
      //  int r = rand()%128 + 128;
      //  int g = rand()%128 + 128;
      //  int b = rand()%128 + 128;
        int r = 0;
        int g = 255;
        int b = 0;
        vcl_vector< vxl_byte> colour;
        colour.push_back(r);
        colour.push_back(g);
        colour.push_back(b);
        
        // draw segment in left image
        VilPlus::draw_arrow(matches, line1[i].point1(), line1[i].point2(), colour);
        
        // draw segment in right image
        vgl_point_2d<double> p1(line2[i].point1().x() + x_shift, line2[i].point1().y());
        vgl_point_2d<double> p2(line2[i].point2().x() + x_shift, line2[i].point2().y());
        VilPlus::draw_arrow(matches, p1, p2, colour);
        
        // draw a link from left to right image
        vgl_point_2d<double> m1 = centre(line1[i].point1(), line1[i].point2());
        vgl_point_2d<double> m2 = centre(line2[i].point1(), line2[i].point2());
        m2.set(m2.x() + x_shift, m2.y());
        
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(m1, m2), colour, 1);
    }
}

void VilPlus::draw_match_dense(const vil_image_view<vxl_byte> &image1, const vil_image_view<vxl_byte> &image2,
                               const vcl_vector< vgl_point_2d<double> > & pts1,
                               const vcl_vector< vgl_point_2d<double> > & pts2,
                               const vcl_vector<bool> & inliers,
                               vil_image_view<vxl_byte> &matches,
                               const int thickness)
{
    assert(image1.nplanes() == 3);
    assert(image2.nplanes() == 3);
    assert(pts1.size() == pts2.size());
    assert(inliers.size() == pts1.size());
    
    int gap_width = 10;
    int x_shift = gap_width + image1.ni();
    int width  = image1.ni() + x_shift;
    int height = vcl_max(image1.nj(), image2.nj());
    
    matches = vil_image_view<vxl_byte>(width, height, 3);
    matches.fill(0);
    
    //copy image 1 to left
    vil_copy_to_window(image1, matches, 0, 0);
    
    //copy image 2 to right
    vil_copy_to_window(image2, matches, x_shift, 0);
    
    for (int i = 0; i<pts1.size(); i++) {
        if (!inliers[i]) {
            continue;
        }
        int r = rand()%255;
        int g = rand()%255;
        int b = rand()%255;
        
        vcl_vector< vxl_byte> colour;
        colour.push_back(r);
        colour.push_back(g);
        colour.push_back(b);
        
        vgl_point_2d<double> p1 = pts1[i];
        vgl_point_2d<double> p2(vgl_point_2d<double>(pts2[i].x() + x_shift, pts2[i].y()));
        
        //    vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(p1, p2), colour, thickness);
        
        // cross in left
        vgl_point_2d<double> q1, q2, q3, q4;
        double h_l = 5;
        q1.set(p1.x() - h_l, p1.y());
        q2.set(p1.x() + h_l, p1.y());
        q3.set(p1.x(), p1.y() - h_l);
        q4.set(p1.x(), p1.y() + h_l);
        
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(q1, q2), colour);
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(q3, q4), colour);
        
        // cross in right
        q1.set(p2.x() - h_l, p2.y());
        q2.set(p2.x() + h_l, p2.y());
        q3.set(p2.x(), p2.y() - h_l);
        q4.set(p2.x(), p2.y() + h_l);
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(q1, q2), colour);
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(q3, q4), colour);
        
        // segment in the left
        vgl_point_2d<double> p3, p4;
        p3.set(p1.x() + (p2.x() - p1.x()) * 0.1, p1.y() + (p2.y() - p1.y()) * 0.1);
        p4.set(p1.x() + (p2.x() - p1.x()) * 0.9, p1.y() + (p2.y() - p1.y()) * 0.9);
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(p1, p3), colour);
        vicl_overlay_line_segment(matches, vgl_line_segment_2d<double>(p2, p4), colour);
    }
}

void VilPlus::draw_projected_line(const vpgl_perspective_camera<double> &camera, const vgl_point_3d<double> &sp, const vgl_point_3d<double> &ep,
                                  vil_image_view<vxl_byte> & image, int colorIdx, int lineWidth)
{
    assert(image.nplanes() == 3);
    assert(colorIdx >= 0 && colorIdx <= 2);
    
    vgl_homg_point_3d< double > p1( sp.x(), sp.y(), sp.z(), 1.0 );
    vgl_homg_point_3d< double > p2( ep.x(), ep.y(), ep.z(), 1.0 );
    
    if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
        return;
    }
    
    vgl_point_2d<double> q1 = (vgl_point_2d<double>)camera.project(p1);
    vgl_point_2d<double> q2 = (vgl_point_2d<double>)camera.project(p2);
    
    //   vcl_cout<<"q1 "<<q1<<vcl_endl;
    //   vcl_cout<<"q2 "<<q2<<vcl_endl<<vcl_endl;
    vcl_vector<vxl_byte> red(3, 0);
    vcl_vector<vxl_byte> green(3, 0);
    vcl_vector<vxl_byte> blue(3, 0);
    red[0]   = 255;
    green[1] = 255;
    blue[2]  = 255;
    
    vcl_vector<vcl_vector< vxl_byte >> colors;
    colors.push_back(red);
    colors.push_back(green);
    colors.push_back(blue);
    
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), colors[colorIdx], lineWidth);
}

void VilPlus::draw_projected_box_line(const vpgl_perspective_camera<double> &camera, const vcl_vector<vgl_point_3d<double> > & vertex,
                                      const vcl_vector<vxl_byte> & color, vil_image_view<vxl_byte> & image)
{
    assert(vertex.size() == 8);
    
    int vertexPair[12][2] = {0, 1,
        1, 2,
        2, 3,
        3, 0,
        4, 5,   5, 6,  6,7, 7,4,
        0,4,  1,5,  2,6,  3,7};
    
    for (int i = 0; i<12; i++) {
        vgl_point_3d<double> p1 = vertex[vertexPair[i][0]];
        vgl_point_3d<double> p2 = vertex[vertexPair[i][1]];
        
        vgl_homg_point_3d<double> q1(p1.x(), p1.y(), p1.z(), 1.0);
        vgl_homg_point_3d<double> q2(p2.x(), p2.y(), p2.z(), 1.0);
        
        if (camera.is_behind_camera(q1) || camera.is_behind_camera(q2)) {
            continue;
        }
        vgl_point_2d<double> p3 = (vgl_point_2d<double>)camera.project(q1);
        vgl_point_2d<double> p4 = (vgl_point_2d<double>)camera.project(q2);
        
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p3, p4), color, 2);
    }
}

void VilPlus::draw_line_on_background(const vpgl_perspective_camera<double> & camera,
                                      const vcl_vector<vgl_line_segment_3d<double> > & segs,
                                      const vil_image_view<vxl_byte> & image,
                                      vil_image_view<vxl_byte> & outImage,
                                      const vcl_vector<vxl_byte> & colour, int line_thickness)
{
    const int w = image.ni();
    const int h = image.nj();
    
    outImage = vil_image_view<vxl_byte>(w*2, h*2, 3);
    outImage.fill(0);
    vil_copy_to_window(image, outImage, w/2, h/2);
    
    vgl_vector_2d<double> displacement(w/2, h/2);
    for (int i = 0; i<segs.size(); i++) {
        vgl_homg_point_3d<double> p1 = vgl_homg_point_3d<double>(segs[i].point1());
        vgl_homg_point_3d<double> p2 = vgl_homg_point_3d<double>(segs[i].point2());
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        vgl_point_2d<double> p3 = camera.project(segs[i].point1());
        vgl_point_2d<double> p4 = camera.project(segs[i].point2());
        
        p3 += displacement;
        p4 += displacement;
        vicl_overlay_line_segment(outImage, vgl_line_segment_2d<double>(p3, p4), colour, line_thickness);
    }
    
    
    
    
}

void VilPlus::draw_projected_circle(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image, const vgl_point_2d<double> & center,
                                    const double height, const double radius, int colorIdx)
{
    assert(image.nplanes() == 3);
    assert(colorIdx >= 0 && colorIdx <= 2);
    
    //construct circle
    vcl_vector<vgl_point_3d<double> > pts;
    const int segNum = 32;
    for ( unsigned int i = 0; i <= segNum; ++i )
    {
        double theta =  2 * vnl_math::pi / segNum * i;
        
        double x = center.x() + cos(theta) * radius;
        double y = center.y() + sin(theta) * radius;
        
        pts.push_back(vgl_point_3d<double>(x, y, height));
    }
    
    //project 3d line segment to image space
    vcl_vector<vxl_byte> red(3, 0);
    vcl_vector<vxl_byte> green(3, 0);
    vcl_vector<vxl_byte> blue(3, 0);
    red[0]   = 255;
    green[1] = 255;
    blue[2]  = 255;
    vcl_vector<vcl_vector< vxl_byte >> colors;
    colors.push_back(red);
    colors.push_back(green);
    colors.push_back(blue);
    
    assert(pts.size() == segNum + 1);
    for (int i = 0; i<segNum; i++) {
        vgl_homg_point_3d<double> p1(pts[i].x(), pts[i].y(), pts[i].z(), 1.0);
        vgl_homg_point_3d<double> p2(pts[i+1].x(), pts[i+1].y(), pts[i+1].z(), 1.0);
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        vgl_point_2d<double> q1 = (vgl_point_2d<double>)camera.project(p1);
        vgl_point_2d<double> q2 = (vgl_point_2d<double>)camera.project(p2);
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), colors[colorIdx], 2);
    }
}

void VilPlus::draw_projected_circle(const vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image,
                                    const vgl_point_2d<double> & center, const vcl_vector<vxl_byte> & colour,
                                    const double height, const double radius)
{
    assert(image.nplanes() == 3);
    
    //construct circle
    vcl_vector<vgl_point_3d<double> > pts;
    const int segNum = 32;
    for ( unsigned int i = 0; i <= segNum; ++i )
    {
        double theta =  2 * vnl_math::pi / segNum * i;
        
        double x = center.x() + cos(theta) * radius;
        double y = center.y() + sin(theta) * radius;
        
        pts.push_back(vgl_point_3d<double>(x, y, height));
    }
    
    //project 3d line segment to image space
    
    assert(pts.size() == segNum + 1);
    for (int i = 0; i<segNum; i++) {
        vgl_homg_point_3d<double> p1(pts[i].x(), pts[i].y(), pts[i].z(), 1.0);
        vgl_homg_point_3d<double> p2(pts[i+1].x(), pts[i+1].y(), pts[i+1].z(), 1.0);
        if (camera.is_behind_camera(p1) || camera.is_behind_camera(p2)) {
            continue;
        }
        vgl_point_2d<double> q1 = (vgl_point_2d<double>)camera.project(p1);
        vgl_point_2d<double> q2 = (vgl_point_2d<double>)camera.project(p2);
        vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(q1, q2), colour, 1);
    }
}

//draw the major and minor axis of an ellipse
void VilPlus::draw_ellipse_majorminor_axis(vil_image_view<vxl_byte> & image, const vnl_vector_fixed<double, 5> & ellipse,
                                           const vcl_vector<vxl_byte> & colour)
{
	double a = 1.0;
	double b = ellipse[0];
	double c = ellipse[1];
	double d = ellipse[2];
	double f = ellipse[3];
	double g = ellipse[4];
	//ellipse center
	double x0 = (c*d-b*f)/(b*b-a*c);
	double y0 = (a*f-b*d)/(b*b-a*c);
    
	//angle between major axis and x axis
	double theta = 0;
	if(b == 0 && a<c)
	{
		theta = 0;
	}
	else if(b == 0 && a>c)
	{
		theta = vnl_math::pi/2.0;
	}
	else if(b != 0 && a<c)
	{
		theta = 0.5*(atan(2*b/(a-c)));
	}
	else if(b != 0 && a>c)
	{
		theta = 0.5 * vnl_math::pi + 0.5*atan(2*b/(a-c));
	}
	else
	{
		printf("Error: ellipse theta error.\n");
        return;
	}
	
    
	double temp = 2.0 * (a*f*f+c*d*d+g*b*b-2.0*b*d*f-a*c*g);
	double temp2 = sqrt((a-c)*(a-c)+4*b*b);
	double axisMajor = sqrt(temp/((b*b-a*c)*(temp2 - (a+c))));
	double axisMinor = sqrt(temp/((b*b-a*c)*(-temp2 - (a+c))));
	double xMajor = x0 + axisMajor * cos(theta);
	double yMajor = y0 + axisMajor * sin(theta);
	double xMajor2 = x0 - axisMajor * cos(theta);
	double yMajor2 = y0 - axisMajor * sin(theta);
	
	//draw major axis
//	cvLine(image, cvPoint((int)xMajor2, (int)yMajor2), cvPoint((int)xMajor, (int)yMajor), cvScalar(255, 0, 0), 2);
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(vgl_point_2d<double>(xMajor, yMajor), vgl_point_2d<double>(xMajor2, yMajor2)), colour, 2.0);
	
	double xMinor = x0 + axisMinor * cos(vnl_math::pi/2.0 + theta);
	double yMinor = y0 + axisMinor * sin(vnl_math::pi/2.0 + theta);
	double xMinor2 = x0 - axisMinor * cos(vnl_math::pi/2.0 + theta);
	double yMinor2 = y0 - axisMinor * sin(vnl_math::pi/2.0 + theta);
    
	//draw minor axis
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(vgl_point_2d<double>(xMinor, yMinor), vgl_point_2d<double>(xMinor2, yMinor2)), colour, 2.0);
//	cvLine(image, cvPoint((int)xMinor2, (int)yMino2r), cvPoint((int)xMinor, (int)yMinor), cvScalar(255, 0, 0), 2);
//	cvShowImage("major minor axis", image);
}




void VilPlus::draw_rectangle(vil_image_view<vxl_byte> & image, const vgl_point_2d<double> & topLeft, const vgl_point_2d<double> & bottomRight, int colorIdx)
{
    assert(colorIdx >= 0 && colorIdx <= 2);
    
    vcl_vector<vxl_byte> red(3, 0);
    vcl_vector<vxl_byte> green(3, 0);
    vcl_vector<vxl_byte> blue(3, 0);
    red[0]   = 255;
    green[1] = 255;
    blue[2]  = 255;
    vcl_vector<vcl_vector< vxl_byte >> colors;
    colors.push_back(red);
    colors.push_back(green);
    colors.push_back(blue);
    
    vgl_point_2d<double> p1 = topLeft;
    vgl_point_2d<double> p2 = vgl_point_2d<double>(bottomRight.x(), topLeft.y());
    vgl_point_2d<double> p3 = vgl_point_2d<double>(topLeft.x(), bottomRight.y());
    vgl_point_2d<double> p4 = bottomRight;
    
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p1, p2), colors[colorIdx], 2);
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p2, p4), colors[colorIdx], 2);
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p4, p3), colors[colorIdx], 2);
    vicl_overlay_line_segment(image, vgl_line_segment_2d<double>(p3, p1), colors[colorIdx], 2);
}


void VilPlus::project_cylinder_rectangle(const vpgl_perspective_camera<double> & camera, const int imageWidth, const int imageHeight,
                                         const vgl_point_2d<double> & center,
                                         const double h1, const double h2, const double radius,
                                         vgl_point_2d<double> & topLeft, vgl_point_2d<double> & bottomRight)
{
    vcl_vector<vgl_point_3d<double> > pts;
    const int segNum = 32;
    for ( unsigned int i = 0; i < segNum; ++i )
    {
        double theta =  2 * vnl_math::pi / segNum * i;
        
        double x = center.x() + cos(theta) * radius;
        double y = center.y() + sin(theta) * radius;
        
        pts.push_back(vgl_point_3d<double>(x, y, h1));
        pts.push_back(vgl_point_3d<double>(x, y, h2));
    }
    
    double x_min = INT_MAX;
    double y_min = INT_MAX;
    double x_max = 0;
    double y_max = 0;
    // project pts to image space
    for (int i = 0; i<pts.size(); i++) {
        vgl_homg_point_3d<double> p(pts[i].x(), pts[i].y(), pts[i].z(), 1.0);
        if (camera.is_behind_camera(p)) {
            continue;
        }
        
        vgl_point_2d<double> q = (vgl_point_2d<double>)camera.project(p);
        double x = q.x();
        double y = q.y();
        if (x < 0 || x >= imageWidth || y< 0 || y >= imageHeight) {
            continue;
        }
        x_min = vcl_min(x_min, x);
        x_max = vcl_max(x_max, x);
        y_min = vcl_min(y_min, y);
        y_max = vcl_max(y_max, y);
    }
    topLeft = vgl_point_2d<double>(x_min, y_min);
    bottomRight = vgl_point_2d<double>(x_max, y_max);
}

/*
void VilPlus::visualize_gradient_direction_hsv(vil_image_view<vxl_byte> &image, vil_image_view<vxl_byte> &gradient_direction)
{
    assert(image.nplanes() == 3);
    
    vil_image_view<vxl_byte> grayImage;
    vil_convert_planes_to_grey(image, grayImage);
    
    vil_image_view<double> grayImage_01 = vil_quantize::dequantize<double>(grayImage);
    
    vil_image_view<double> Ix;
    vil_image_view<double> Iy;
    vil_sobel_3x3(grayImage_01, Ix, Iy);
    
    int width = image.ni();
    int height = image.nj();
    
    vil_image_view<double> hsv(width, height, 3);
    gradient_direction = vil_image_view<vxl_byte>(width, height, 3);
    
    for (int y = 0; y<height; y++) {
        for (int x = 0; x<width; x++) {
            double dx = Ix(x, y);
            double dy = Iy(x, y);
            double mag = sqrt(dx * dx + dy * dy);
            if (mag != 0.0) {
                dx /=mag;
                dy /=mag;
            }
            
            double angle = atan2(dy, dx) * 180.0/vnl_math::pi;
            int H = angle + 180;
            if (H < 0) {
                H = 0;
            }
            if (H > 360) {
                H = 360;
            }
            hsv(x, y, 0) = H;
            hsv(x, y, 1) = mag;
            hsv(x, y, 2) = 1.0;
        }
    }
    
    VilPlus::vil_convert_hsv_to_rgb(hsv, gradient_direction);
}
*/
// http://www.cs.rit.edu/~ncs/color/t_convert.html
void VilPlus::vil_convert_hsv_to_rgb(const vil_image_view<double> &src, vil_image_view<vxl_byte> &dest)
{
    assert(src.nplanes() == 3);
    int width = src.ni();
    int height = src.nj();
    
    dest = vil_image_view<vxl_byte>(width, height, 3);
    for (int y = 0; y<height; y++) {
        for (int x = 0; x<width; x++) {
            float h = src(x, y, 0);
            float s = src(x, y, 1);
            float v = src(x, y, 2);
            
            
            float f, p, q, t;
            float r = 0, g = 0, b = 0;
            
            if( s == 0 ) {
                // achromatic (grey)
                r = g = b = v;
            }
            else
            {
                int idx = 0;
                h /= 60;			// sector 0 to 5
                idx = floor( h );
                f = h - idx;			// factorial part of h
                p = v * ( 1 - s );
                q = v * ( 1 - s * f );
                t = v * ( 1 - s * ( 1 - f ) );
                
                switch( idx ) {
                    case 0:
                    r = v;
                    g = t;
                    b = p;
                    break;
                    case 1:
                    r = q;
                    g = v;
                    b = p;
                    break;
                    case 2:
                    r = p;
                    g = v;
                    b = t;
                    break;
                    case 3:
                    r = p;
                    g = q;
                    b = v;
                    break;
                    case 4:
                    r = t;
                    g = p;
                    b = v;
                    break;
                    default:		// case 5:
                    r = v;
                    g = p;
                    b = q;
                    break;
                }
            }
            
            dest(x, y, 0) = r * 255;
            dest(x, y, 1) = g * 255;
            dest(x, y, 2) = b * 255;
        }
    }
}

void VilPlus::vil_rgb_to_yuv(const vil_image_view<vxl_byte> & src, vil_image_view<vxl_byte> & dest)
{
    assert(src.nplanes() == 3);
    
    dest = vil_image_view<vxl_byte>(src.ni(), src.nj(), 3);
    double srcPixel[3];
    double destPixel[3];
    for (int j = 0; j<src.nj(); j++) {
        for (int i = 0; i<src.ni(); i++) {
            srcPixel[0] = src(i, j, 0);
            srcPixel[1] = src(i, j, 1);
            srcPixel[2] = src(i, j, 2);
            
            vil_colour_space_RGB_to_YUV(srcPixel, destPixel);
            destPixel[1] += 128;
            destPixel[2] += 128;
            
            int y = destPixel[0];
            int u = destPixel[1];
            int v = destPixel[2];
            dest(i, j, 0) = y;
            dest(i, j, 1) = u;
            dest(i, j, 2) = v;
        }
    }
}

void VilPlus::vil_rgb_to_hsv(const vil_image_view<vxl_byte> & src, vil_image_view<double> & hsv)
{
    assert(src.nplanes() == 3);
    
    const int w = src.ni();
    const int h = src.nj();
    hsv = vil_image_view<double>(w, h, 3);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            double r = src(i, j, 0);
            double g = src(i, j, 1);
            double b = src(i, j, 2);
            double h = 0;
            double s = 0;
            double v = 0;
            vil_colour_space_RGB_to_HSV(r, g, b, &h,  &s, &v);
            hsv(i, j, 0) = h;
            hsv(i, j, 1) = s;
            hsv(i, j, 2) = v;
            
         //   printf("h s v are  %f\t %f\t %f\n", h, s, v);
        }
    }
    
}

bool VilPlus::homography_warp_alpha(const vil_image_view<vxl_byte>& inImage,
                                    const vgl_h_matrix_2d<double> &H,
                                    int width, int height,
                                    vil_image_view<vxl_byte> &outImageWithAlpha)
{
    assert(inImage.nplanes() == 3);
    
    vil_image_view<vxl_byte> temp(width, height, 3);
    temp.fill(0);
    
    outImageWithAlpha = vil_image_view<vxl_byte>(width, height, 4);
    outImageWithAlpha.fill(0);
    
    vgl_h_matrix_2d<double> invH = H.get_inverse();
    
    vil_warp(inImage, temp, VxlPlusInvHwarp(invH), InterpolatorFunc);
    
    for (int y = 0; y<temp.nj(); y++) {
        for (int x =0; x<temp.ni(); x++) {
            vgl_homg_point_2d<double> p = invH(vgl_homg_point_2d<double>(x, y, 1.0));
            double px = p.x()/p.w();
            double py = p.y()/p.w();
            
            //out of image or in the boundary of image
            if ((px <= 0 || px >= inImage.ni() - 1) || (py <= 0 || py >= inImage.nj() - 1)) {
                outImageWithAlpha(x, y, 0) = 0;
                outImageWithAlpha(x, y, 1) = 0;
                outImageWithAlpha(x, y, 2) = 0;
                outImageWithAlpha(x, y, 3) = 0;
            }
            else
            {
                outImageWithAlpha(x, y, 0) = temp(x, y, 0);
                outImageWithAlpha(x, y, 1) = temp(x, y, 1);
                outImageWithAlpha(x, y, 2) = temp(x, y, 2);
                outImageWithAlpha(x, y, 3) = 255;
            }
        }
    }
    
    return true;
}

bool VilPlus::homography_warp(const vil_image_view<vxl_byte>& inImage,
                              const vgl_h_matrix_2d<double> &H,
                              int width, int height,
                              vil_image_view<vxl_byte> &outImage)
{
    assert(inImage.nplanes() == 3 || inImage.nplanes() == 1);
    
    outImage = vil_image_view<vxl_byte>(width, height, inImage.nplanes());
    outImage.fill(0);  
    
    vgl_h_matrix_2d<double> invH = H.get_inverse();
    vil_warp(inImage, outImage, VxlPlusInvHwarp(invH), InterpolatorFunc);
    
    return true;
}

bool VilPlus::homography_warp(const vil_image_view<double> & orgImage,
                              const vgl_h_matrix_2d<double> & H,
                              int width, int height,
                              vil_image_view<double> & outImage)
{
    outImage = vil_image_view<double>(width, height, orgImage.nplanes());
    vgl_h_matrix_2d<double> invH = H.get_inverse();
    vil_warp(orgImage, outImage, VxlPlusInvHwarp(invH), InterpolatorFuncDouble);
    return true;
}

bool VilPlus::warpOverlayRatio(const vgl_h_matrix_2d<double> & H, int imageWidth, int imageHeight, double & ratio, int & type)
{
    vil_image_view<vxl_byte> imageA(imageWidth, imageHeight, 1);
    imageA.fill(255);
    vil_image_view<vxl_byte> imageB;
    VilPlus::homography_warp(imageA, H, imageWidth, imageHeight, imageB);
    int leftNum  = 0;
    int rightNum = 0;
    for (int j = 0; j<imageHeight; j++) {
        for (int i = 0; i<imageWidth; i++) {
            if (imageB(i, j) == 255) {
                (i < imageWidth/2) ? leftNum++: rightNum++;
            }
        }
    }
    ratio = 1.0 * (leftNum + rightNum)/(imageWidth * imageHeight);
    type  = (leftNum>rightNum) ? 0 : 1;
  //  VilPlus::vil_save(imageB, "21600_mask_right.png");
    return true;
}


void VilPlus::vil_median_image(const vcl_vector<vil_image_view<vxl_byte> > & images, vil_image_view<vxl_byte> & medianImage)
{
    assert(images.size() > 0);
    
    int w = images[0].ni();
    int h = images[0].nj();
    
    medianImage = vil_image_view<vxl_byte>(w, h, 3);
    medianImage.fill(0);
    
    //for each pixel loop over all image
    for (int y = 0; y<medianImage.nj(); y++) {
        for (int x = 0; x<medianImage.ni(); x++) {
            //each pixel
            vcl_vector<vxl_byte> r_v;
            vcl_vector<vxl_byte> g_v;
            vcl_vector<vxl_byte> b_v;
            
            for (int k = 0; k<images.size(); k++) {
                r_v.push_back(images[k](x, y, 0));
                g_v.push_back(images[k](x, y, 1));
                b_v.push_back(images[k](x, y, 2));
            }
            
            size_t half = r_v.size()/2;
            vcl_nth_element(r_v.begin(), r_v.begin() + half, r_v.end());
            vcl_nth_element(g_v.begin(), g_v.begin() + half, g_v.end());
            vcl_nth_element(b_v.begin(), b_v.begin() + half, b_v.end());
            
            medianImage(x, y, 0) = r_v[half];
            medianImage(x, y, 1) = g_v[half];
            medianImage(x, y, 2) = b_v[half];
        }
    }
}

void VilPlus::vil_median_image_with_alpha(const vcl_vector<vil_image_view<vxl_byte> > & images, vil_image_view<vxl_byte> & medianImage)
{
    assert(images.size() > 0);
    
    int w = images[0].ni();
    int h = images[0].nj();
    
    medianImage = vil_image_view<vxl_byte>(w, h, 4);
    medianImage.fill(0);
    
    //for each pixel loop over all image
    for (int y = 0; y<medianImage.nj(); y++) {
        for (int x = 0; x<medianImage.ni(); x++) {
            //each pixel
            vcl_vector<vxl_byte> r_v;
            vcl_vector<vxl_byte> g_v;
            vcl_vector<vxl_byte> b_v;
            
            for (int k = 0; k<images.size(); k++) {
                // alpha channel
                if (images[k](x, y, 3) != 0) {
                    r_v.push_back(images[k](x, y, 0));
                    g_v.push_back(images[k](x, y, 1));
                    b_v.push_back(images[k](x, y, 2));
                }
            }
            
            if (r_v.size() != 0) {
                size_t half = r_v.size()/2;
                vcl_nth_element(r_v.begin(), r_v.begin() + half, r_v.end());
                vcl_nth_element(g_v.begin(), g_v.begin() + half, g_v.end());
                vcl_nth_element(b_v.begin(), b_v.begin() + half, b_v.end());
                
                medianImage(x, y, 0) = r_v[half];
                medianImage(x, y, 1) = g_v[half];
                medianImage(x, y, 2) = b_v[half];
                medianImage(x, y, 3) = 255;
            }
            else
            {
                medianImage(x, y, 3) = 0;
            }
        }
    }
}

void VilPlus::draw_image_grid(const vcl_vector<vil_image_view<vxl_byte> > & images, int nCols, vil_image_view<vxl_byte> & gridImage, int gapWidth)
{
    assert(nCols > 0);
    assert(images.size() > 0);
    
    int nRows = (int)images.size()/nCols;
    if (images.size()%nCols) {
        nRows += 1;
    }
    
    // size of the output image
    //int gapWidth = 5;
    const int imageWidth = images[0].ni();
    const int imageHeight = images[0].nj();
    const int w = imageWidth * nCols + (nCols - 1) * gapWidth;
    const int h = imageHeight * nRows + (nRows - 1) * gapWidth;
    
    gridImage = vil_image_view<vxl_byte>(w, h, images[0].nplanes());
    gridImage.fill(255);
    
    for (int i = 0; i<images.size(); i++) {
        vil_image_view<vxl_byte> curImage = images[i];
        int row = i/nCols;
        int col = i%nCols;
        int x = imageWidth * col  + col * gapWidth;
        int y = imageHeight * row + row * gapWidth;
        
        
        vil_image_view<vxl_byte> subImage = vil_crop(gridImage, x, imageWidth, y, imageHeight);
        subImage.deep_copy(curImage);
    }
}




/*
bool VilPlus::vil_refine_patch_position(const vil_image_view<vxl_byte> & kernelImage, const vil_image_view<vxl_byte> & destImage,
                                        const vgl_point_2d<double> & initP, int patchSize, int searchSize, vgl_point_2d<double> & finalP)
{
    assert(kernelImage.nplanes() == destImage.nplanes());
    assert(kernelImage.ni() == destImage.ni());
    assert(kernelImage.nj() == destImage.nj());
    assert(patchSize < searchSize);
    
    int destWidth  = destImage.ni();
    int destHeight = destImage.nj();
    
    vil_image_view<double> source = vil_quantize::dequantize<double>(kernelImage);
    vil_image_view<double> dest   = vil_quantize::dequantize<double>(destImage);
    
    // grab patch center at initP in warped image
    vgl_point_2d<double> c = initP;
    vil_image_view<double> patch     = vil_crop(source, c.x() - patchSize/2, patchSize, c.y() - patchSize/2, patchSize);
    vil_image_view<double> destPatch = vil_crop(dest, c.x() - searchSize/2, searchSize, c.y() - searchSize/2, searchSize);
    vil_image_view<double> ncc    = vil_normalised_cross_correlation(destPatch, patch);
    assert(ncc.nplanes() == 1);
    
    // max value in ncc
    double val_max = -1.0;
    int idx_x_ = -1;
    int idx_y_ = -1;
    for (int i = 0; i<ncc.ni(); i++) {
        for (int j = 0; j<ncc.nj(); j++) {
            double val = ncc(i, j, 0);
            if (val > val_max) {
                val_max = val;
                idx_x_ = i;
                idx_y_ = j;
            }
        }
    }
    int x_center = idx_x_ + patchSize/2;  // ncc position is in the left top, but patch center is in middle
    int y_center = idx_y_ + patchSize/2;
    int x_offset = c.x() - searchSize/2;  // ncc coordinate to destImage coordinate
    int y_offset = c.y() - searchSize/2;
    
    finalP.set(x_center + x_offset  ,  y_center + y_offset);
    return finalP.x() >= 0 && finalP.x() < destWidth && finalP.y() >= 0 && finalP.y() < destHeight;
}
 */

/*

bool VilPlus::vil_refine_patch_position(const vil_image_view<vxl_byte> & kernelImage, const vgl_point_2d<double> & kernelP,
                                        const vil_image_view<vxl_byte> & destImage, const vgl_point_2d<double> & initP,
                                        int patchSize, int searchSize, vgl_point_2d<double> & finalP)
{
    assert(kernelImage.nplanes() == destImage.nplanes());
    assert(patchSize < searchSize);
    
    int destWidth  = destImage.ni();
    int destHeight = destImage.nj();
    
    vil_image_view<double> source = vil_quantize::dequantize<double>(kernelImage);
    vil_image_view<double> dest   = vil_quantize::dequantize<double>(destImage);
    
    // grab patch center at initP in warped image
    vil_image_view<double> patch     = vil_crop(source, kernelP.x() - patchSize/2, patchSize, kernelP.y() - patchSize/2, patchSize);
    
    int x = initP.x() - searchSize/2;
    int y = initP.y() - searchSize/2;
    if (x < 0) {
        x = 0;
    }
    if (y < 0) {
        y = 0;
    }
    int w = searchSize;
    int h = searchSize;
    if (x + w > destWidth) {
        w = destWidth - x;
    }
    if (y + h > destHeight) {
        h = destHeight - y;
    }
    if (w < patchSize || h < patchSize) {
        return false;
    }
    
    vil_image_view<double> destPatch = vil_crop(dest, x, w, y, h);
    
    vil_image_view<double> ncc    = vil_normalised_cross_correlation(destPatch, patch);
    assert(ncc.nplanes() == 1);
    
    // max value in ncc
    double val_max = -1.0;
    int idx_x_ = -1;
    int idx_y_ = -1;
    for (int i = 0; i<ncc.ni(); i++) {
        for (int j = 0; j<ncc.nj(); j++) {
            double val = fabs(ncc(i, j, 0));
            if (val > val_max) {
                val_max = val;
                idx_x_ = i;
                idx_y_ = j;
            }
        }
    }
    int x_center = idx_x_ ;      // ncc position is in the left top, but patch center is in middle
    int y_center = idx_y_ ;
    int x_offset = x + patchSize/2;  // ncc coordinate to destImage coordinate
    int y_offset = y + patchSize/2;
    
    finalP.set(x_center + x_offset,  y_center + y_offset);
    bool isInside = finalP.x() >= 0 && finalP.x() < destWidth && finalP.y() >= 0 && finalP.y() < destHeight;
    
    if (!isInside) {
        printf("out of dest image %f %f\n", finalP.x(), finalP.y());
    }
    return isInside;
}
 */

/*
bool VilPlus::vil_refine_patch_position(const vil_image_view<vxl_byte> & kernelImage, const vcl_vector<vgl_point_2d<double> > & kernelPts,
                                        const vil_image_view<vxl_byte> & destImage, const vcl_vector<vgl_point_2d<double> > & initPts,
                                        int patchSize, int searchSize, vcl_vector<vgl_point_2d<double> > & finalP)
{
    assert(kernelImage.nplanes() == destImage.nplanes());
    assert(patchSize < searchSize);
    assert(kernelPts.size() == initPts.size());
    assert(finalP.size() == 0);
    
    int destWidth  = destImage.ni();
    int destHeight = destImage.nj();
    vil_image_view<double> source = vil_quantize::dequantize<double>(kernelImage);
    vil_image_view<double> dest   = vil_quantize::dequantize<double>(destImage);
    
    // loop over all initial correspondence
    for (int i = 0; i<kernelPts.size(); i++) {
        vgl_point_2d<double> kp = kernelPts[i];  // kernal position
        vgl_point_2d<double> ip = initPts[i];    // init position
        vgl_point_2d<double> cp;  // corresponding position
        
        vil_image_view<double> patch     = vil_crop(source, kp.x() - patchSize/2, patchSize, kp.y() - patchSize/2, patchSize);
        
        // search top-left position
        int x = ip.x() - searchSize/2;
        int y = ip.y() - searchSize/2;
        if (x < 0) {
            x = 0;
        }
        if (y < 0) {
            y = 0;
        }
        // search size
        int w = searchSize;
        int h = searchSize;
        if (x + w > destWidth) {
            w = destWidth - x;
        }
        if (y + h > destHeight) {
            h = destHeight - y;
        }
        // search size smaller than patch size
        if (w < patchSize || h < patchSize) {
            finalP.push_back(vgl_point_2d<double>(-1, -1));
            continue;
        }
        
        vil_image_view<double> destPatch = vil_crop(dest, x, w, y, h);
        
        vil_image_view<double> ncc    = vil_normalised_cross_correlation(destPatch, patch);
        assert(ncc.nplanes() == 1);
        
        // max value in ncc
        double val_max = -1.0;
        int idx_x_ = -1;
        int idx_y_ = -1;
        for (int i = 0; i<ncc.ni(); i++) {
            for (int j = 0; j<ncc.nj(); j++) {
                double val = fabs(ncc(i, j, 0));
                if (val > val_max) {
                    val_max = val;
                    idx_x_ = i;
                    idx_y_ = j;
                }
            }
        }
        int x_center = idx_x_ ;      // ncc position is in the left top, but patch center is in middle
        int y_center = idx_y_ ;
        int x_offset = x + patchSize/2;  // ncc coordinate to destImage coordinate
        int y_offset = y + patchSize/2;
        
        cp.set(x_center + x_offset,  y_center + y_offset);
        bool isInside = cp.x() >= 0 && cp.x() < destWidth && cp.y() >= 0 && cp.y() < destHeight;
        
        if (!isInside) {
            finalP.push_back(vgl_point_2d<double>(-1, -1));
            continue;
        }
        finalP.push_back(cp);
    }
    assert(kernelPts.size() == finalP.size());
    
    return true;
}
 */

//Connected-component labeling
int VilPlus::connect_component_label(const vil_image_view<vxl_byte> & biImage, unsigned int minSetSize,
                                     vil_image_view<vxl_byte> & labelImage,
                                     vcl_vector<vcl_vector<vgl_point_2d<double> > > & labelPts)
{
    assert(biImage.nplanes() == 1);
    
	
	int w = biImage.ni();
	int h = biImage.nj();
    
	labelImage = vil_image_view<vxl_byte>(w, h, 1);
    labelImage.fill(0);
	int label = 0;
    
	//8 neighbor position
	const int neighbor[16] = {-1,-1, 0,-1, 1,-1,
                            -1,0, 1,0,
                            -1,1, 0,1, 1,1};
	for (int y = 0; y<h; ++y)
	{
		for (int x = 0; x<w; ++x)
		{
			//not labeled
			if (biImage(x, y) == 255 && labelImage(x, y) == 0)
			{
				label++;
				vcl_stack<vgl_point_2d<double> > points; // deep first travel
				points.push(vgl_point_2d<double>(x, y));
                
				labelImage(x, y) = label;
                
				vcl_vector<vgl_point_2d<double> > lPt; // store all the pixels with the same label
                
				while(!points.empty())
				{
					vgl_point_2d<double> p = points.top();
					lPt.push_back(p);
					points.pop();
					for (int i = 0; i<8; ++i)
					{
						int nx = p.x() + neighbor[2*i];
						int ny = p.y() + neighbor[2*i+1];
						if (nx>=0 && nx<w &&ny>=0&&ny<h &&
                            biImage(nx, ny) == 255 &&
                            labelImage(nx, ny) == 0)
						{
							points.push(vgl_point_2d<double>(nx, ny));
							labelImage(nx, ny) = label;
						}
					}
				}
                if (lPt.size() >= minSetSize) {
                    labelPts.push_back(lPt);
                }				
			}
		}
	}
	return label;
}

// http://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
// Bresenham's line algorithm
bool VilPlus::draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts)
{
    /*
    function line(x0, x1, y0, y1)
    real deltax := x1 - x0
    real deltay := y1 - y0
    real error := 0
    real deltaerr := abs (deltay / deltax)    // Assume deltax != 0 (line is not vertical),
    // note that this division needs to be done in a way that preserves the fractional part
    int y := y0
    for x from x0 to x1
        plot(x,y)
        error := error + deltaerr
        while error ≥ 0.5 then
            plot(x, y)
            y := y + sign(y1 - y0)
            error := error - 1.0
     */
    double x0 = p0.x();
    double y0 = p0.y();
    double x1 = p1.x();
    double y1 = p1.y();
    if (p0.x() > p1.x()) {
        vcl_swap(x0, x1);
        vcl_swap(y0, y1);
    }
    
    double deltaX = x1 - x0;
    double deltaY = y1 - y0;
    double error = 0;
    if (fabs(deltaX) < 0.5) {
        // vertical line
        if (y0 > y1) {
            vcl_swap(y0, y1);
        }
        int x = (x0 + x1)/2.0;
        for (int y = y0; y <= y1; y++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
        }
    }
    else if(fabs(deltaY) < 0.5)
    {
        // horizontal line
        int y = (y0 + y1)/2.0;
        for (int x = x0; x <= x1; x++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
        }
    }
    else
    {
        double deltaErr = fabs(deltaY/deltaX);
        int y = (int)y0;
        int sign = y1 > y0 ? 1:-1;
        for (int x = x0; x <= x1; x++) {
            linePts.push_back(vgl_point_2d<double>(x, y));
            error += deltaErr;
            while (error >= 0.5) {
                linePts.push_back(vgl_point_2d<double>(x, y));  // may have duplicated (x, y)
                y += sign;
                error -= 1.0;
            }
        }
    }
    return true;
}

// http://members.chello.at/~easyfilter/bresenham.html
#if 0
void plotLineWidth(int x0, int y0, int x1, int y1, float wd)
{                                    /* plot an anti-aliased line of width wd */
    int dx = abs(x1-x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1-y0), sy = y0 < y1 ? 1 : -1;
    int err = dx-dy, e2, x2, y2;                           /* error value e_xy */
    float ed = dx+dy == 0 ? 1 : sqrt((float)dx*dx+(float)dy*dy);
    
    for (wd = (wd+1)/2; ; ) {                                    /* pixel loop */
        setPixelColor(x0, y0, max(0,255*(abs(err-dx+dy)/ed-wd+1)));
        e2 = err; x2 = x0;
        if (2*e2 >= -dx) {                                            /* x step */
            for (e2 += dy, y2 = y0; e2 < ed*wd && (y1 != y2 || dx > dy); e2 += dx)
                setPixelColor(x0, y2 += sy, max(0,255*(abs(e2)/ed-wd+1)));
            if (x0 == x1) break;
            e2 = err; err -= dy; x0 += sx;
        }
        if (2*e2 <= dy) {                                             /* y step */
            for (e2 = dx-e2; e2 < ed*wd && (x1 != x2 || dx < dy); e2 += dy)
                setPixelColor(x2 += sx, y0, max(0,255*(abs(e2)/ed-wd+1)));
            if (y0 == y1) break;
            err += dx; y0 += sy;
        }
    }
}
#endif

// http://members.chello.at/~easyfilter/bresenham.html
bool VilPlus::draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, double width, vcl_vector<vgl_point_2d<double> > & linePts)
{
    assert(width >= 0.5);
    
    int x0 = p0.x();
    int y0 = p0.y();
    int x1 = p1.x();
    int y1 = p1.y();
    float wd = (float)width;
    
    // plot an anti-aliased line of width wd
    int dx = abs(x1-x0), sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1-y0), sy = y0 < y1 ? 1 : -1;
    int err = dx-dy;
    int e2 = 0;
    int x2 = 0;
    int y2 = 0;                           // error value e_xy
    float ed = dx+dy == 0 ? 1 : sqrt((float)dx*dx+(float)dy*dy);
    
    for (wd = (wd+1)/2; ; )
    {                                    // pixel loop
        linePts.push_back(vgl_point_2d<double>(x0, y0));
        e2 = err; x2 = x0;
        if (2*e2 >= -dx)
        {                                            // x step
            for (e2 += dy, y2 = y0; e2 < ed*wd && (y1 != y2 || dx > dy); e2 += dx)
            {
             //   setPixelColor(x0, y2 += sy, max(0,255*(abs(e2)/ed-wd+1)));
                linePts.push_back(vgl_point_2d<double>(x0, y2 += sy));
            }
            if (x0 == x1)
            {
                break;
            }
            e2 = err;
            err -= dy;
            x0 += sx;
        }
        if (2*e2 <= dy)
        {                                             // y step
            for (e2 = dx-e2; e2 < ed*wd && (x1 != x2 || dx < dy); e2 += dy)
            {
                linePts.push_back(vgl_point_2d<double>(x2 += sx, y0));
            }
            
            if (y0 == y1)
            {
                break;
            }
            err += dx;
            y0 += sy;
        }
    }    
    return true;
}


static long int vil_vcl_round( double x )
{
    return (long int)( x > 0.0 ? x + 0.5 : x - 0.5 );
}

bool VilPlus::draw_line_segment_unsafe(const vgl_line_segment_2d<double> & segment, const double thickness,
                                const int w, const int h, vcl_vector<vgl_point_2d<double> > & linePts)
{
    vgl_point_2d< double > p1 = segment.point1();
	vgl_point_2d< double > p2 = segment.point2();
    
	double slope = vcl_abs( segment.slope_degrees() );
    
	if ( slope > 45.0 && slope < 135.0 )
    {
		int min_j = vcl_max(                 0, (int)vil_vcl_round( vcl_min( p1.y(), p2.y() ) ) );
		int max_j = vcl_min( (int)h-1,          (int)vil_vcl_round( vcl_max( p1.y(), p2.y() ) ) );
        
		for ( int j = min_j; j <= max_j; j++ )
		{
            if ( ( j < 0 ) || ( j >= (int)h ) ) continue;
            
			int i = (int)vil_vcl_round( -( segment.b() / segment.a() ) * j - ( segment.c() / segment.a() ) );
            
			if ( ( i < 0 ) || ( i >= (int)w ) )
				continue;
            
		//	hilitePixel( image, j, i, colour );
            linePts.push_back(vgl_point_2d<double>(i, j));
			for (unsigned int t = 1; t < thickness; t++)
			{
				if (i > 0 && i - t > 0 && i-t < (int)w){
				//	hilitePixel( image, j, i-t, colour );
                    linePts.push_back(vgl_point_2d<double>(i-t, j));
                }
				if (i < (int)w && i+t < (int)w){
				//	hilitePixel( image, j, i+t, colour );
                    linePts.push_back(vgl_point_2d<double>(i+t, j));
                }
			}
		}
        
	}
    else
    {
		int min_i = vcl_max(                 0, (int)vil_vcl_round( vcl_min( p1.x(), p2.x() ) ) );
		int max_i = vcl_min( (int)w-1,          (int)vil_vcl_round( vcl_max( p1.x(), p2.x() ) ) );
        
		for ( int i = min_i; i <= max_i; i++ )
		{
            if ( ( i < 0 ) || ( i >= (int)w ) ) continue;
            
			int j = (int)vil_vcl_round( -( segment.a() / segment.b() ) * i - ( segment.c() / segment.b() ) );
            
			if ( ( j < 0 ) || ( j >= (int)h ) ) continue;
            
		//	hilitePixel( image, j, i, colour );
            linePts.push_back(vgl_point_2d<double>(i, j));
			for (unsigned int t = 1; t < thickness; t++)
			{
				if (j > 0 && j-t > 0)
                {
                    
				//	hilitePixel( image, j-t, i, colour );
                    linePts.push_back(vgl_point_2d<double>(i, j-t));
                }
				if (j < (int)h && j+t < (int)h)
                {
				//	hilitePixel( image, j+t, i, colour );
                    linePts.push_back(vgl_point_2d<double>(i, j+t));
                }
			}
		}
	}
    
    return true;
}

bool VilPlus::draw_line_segment(const vgl_line_segment_2d<double> & line_segment, const double thickness,
                            const int w, const int h,
                            vcl_vector<vgl_point_2d<double> > & linePts)
{
    vcl_vector<vgl_point_2d<double> > pts;
    VilPlus::draw_line_segment_unsafe(line_segment, thickness, w, h, pts);
    for (int i = 0; i<pts.size(); i++) {
        vgl_point_2d<double> p = pts[i];
        if (p.x() >= 0 && p.x() < w && p.y() >= 0 && p.y() < h) {
            linePts.push_back(p);
        }
    }
    return true;    
}

bool VilPlus::draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, vcl_vector<vgl_point_2d<double> > & linePts, int w, int h)
{
    vcl_vector<vgl_point_2d<double> > pts;
    VilPlus::draw_line(p0, p1, pts);
    for (int i = 0; i<pts.size(); i++) {
        vgl_point_2d<double> p = pts[i];
        if (p.x() >= 0 && p.x() < w && p.y() >= 0 && p.y() < h) {
            linePts.push_back(p);
        }
    }
    return true;
}

bool VilPlus::draw_line(const vgl_point_2d<double> & p0, const vgl_point_2d<double> & p1, double width, vcl_vector<vgl_point_2d<double> > & linePts, int w, int h)
{
    vcl_vector<vgl_point_2d<double> > pts;
    VilPlus::draw_line(p0, p1, width, pts);
    for (int i = 0; i<pts.size(); i++) {
        vgl_point_2d<double> p = pts[i];
        if (p.x() >= 0 && p.x() < w && p.y() >= 0 && p.y() < h) {
            linePts.push_back(p);
        }
    }
    return true;
}

vcl_vector<vxl_byte> VilPlus::blue(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(0);
    color.push_back(0);
    color.push_back(255);
    return color;
}

vcl_vector<vxl_byte> VilPlus::green(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(0);
    color.push_back(255);
    color.push_back(0);
    return color;
}

vcl_vector<vxl_byte> VilPlus::red(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(255);
    color.push_back(0);
    color.push_back(0);
    return color;
}

vcl_vector<vxl_byte> VilPlus::white(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(255);
    color.push_back(255);
    color.push_back(255);
    return color;
}

vcl_vector<vxl_byte> VilPlus::hotPink(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(255);
    color.push_back(105);
    color.push_back(180);
    return color;
}

vcl_vector<vxl_byte> VilPlus::yellow()
{
    vcl_vector<vxl_byte> color;
    color.push_back(255);
    color.push_back(255);
    color.push_back(0);
    return color;
}

vcl_vector<vxl_byte> VilPlus::purple(void)
{
    vcl_vector<vxl_byte> color;
    color.push_back(128);
    color.push_back(0);
    color.push_back(128);
    return color;
}


