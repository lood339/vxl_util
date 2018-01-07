//
//  vgl_cylinder_mapping.cpp
//  OnlineStereo
//
//  Created by jimmy on 1/15/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vgl_cylinder_mapping.h"
#include <vnl/vnl_math.h>
#include "vnl_plus.h"
#include <vil/vil_bilin_interp.h>
#include <vil/vil_warp.h>
#include <vgl/vgl_plane_3d.h>
#include <vgl/vgl_intersection.h>
#include "vpgl_plus.h"
#include "vil_plus.h"

vcl_vector<vgl_line_segment_3d<double> > VglCylinderMapping::verticalBarLineSegments(const unsigned int sampleNum)
{
    vcl_vector<vgl_line_segment_3d<double> > segments;
    
    vgl_vector_3d<double> ori = this->cylinder_.orientation();
    assert(VnlPlus::isEqual(ori.x(), 0));
    assert(VnlPlus::isEqual(ori.y(), 0));
    assert(VnlPlus::isEqual(ori.z(), 1));
    
    double len = this->cylinder_.length();
    vgl_point_3d<double> c1 = this->cylinder_.center();
    vgl_point_3d<double> c2(c1.x() + ori.x() * len, c1.y() + ori.y() * len, c1.z() + ori.z() * len);
    double rad = cylinder_.radius();
    
    // vertical bars in the side of cylinder
    for (unsigned i = 0; i<sampleNum; i++) {
        double alpha = 2.0 * vnl_math::pi * i/sampleNum;
     //   double beta  = 2.0 * vnl_math::pi * (i+1)/sampleNum;
        
        double x1 = rad * cos(alpha);
        double y1 = rad * sin(alpha);
    //    double x2 = rad * cos(beta);
     //   double y2 = rad * sin(beta);
        
        vgl_point_3d<double> p1(c1.x() + x1, c1.y() + y1, c1.z());
        vgl_point_3d<double> p3(c2.x() + x1, c2.y() + y1, c2.z());
        
        segments.push_back(vgl_line_segment_3d<double>(p1, p3));
    }
    
    return segments;
}

vil_image_view<vxl_byte> VglCylinderMapping::blankImage()
{
    assert(pan_max_ > pan_min_);
    assert(z_max_ > 0);
    
    vil_image_view<vxl_byte> image((pan_max_ - pan_min_) * pixel_per_degree_in_pan_, z_max_ * pixel_per_meter_in_z, 3);
    image.fill(0);
    
    return image;
}

vgl_point_3d<double> VglCylinderMapping::imageLocation2World(const unsigned x, const unsigned y, const int imageW, const int imageH)
{
    double theta = (90.0 - (x - imageW/2.0)/pixel_per_degree_in_pan_)/180.0 * vnl_math::pi;
    double rad = cylinder_.radius();
    double px = rad * cos(theta);
    double py = rad * sin(theta);
    double pz = 1.0 * (imageH - y) / pixel_per_meter_in_z;
    
    //transform from cylinder coordinate to world coordinate
    px += cylinder_.center().x();
    py += cylinder_.center().y();
    pz += cylinder_.center().z();
    
    return vgl_point_3d<double>(px, py, pz);
}

vcl_vector<vgl_point_3d<double> > VglCylinderMapping::worldLocations(const int imageW, const int imageH)
{
    vcl_vector<vgl_point_3d<double> > points(imageW * imageH);
    for (int j = 0; j<imageH; j++) {
        for (int i = 0; i<imageW; i++) {
            int idx = j * imageW + i;
            vgl_point_3d<double> p = this->imageLocation2World(i, j, imageW, imageH);
            points[idx] = p;
        }
    }
    return points;
}


//inverse H mapping from court (world) to image space
class VxlCylinderWarp
{
    public:
    VxlCylinderWarp(const vpgl_perspective_camera<double> & camera, const vcl_vector<vgl_point_3d<double> > & points, int w, int h)
    {
        camera_ = camera;
        points_ = points;
        w_ = w;
        h_ = h;
        assert(points_.size() == w_ * h_);
    }
    void operator()(const double ox, const double oy, double &ix, double &iy)
    {
     //   vgl_homg_point_2d<double> p = invH_((vgl_homg_point_2d<double>(ox, oy)));
     //   ix = p.x()/p.w();
     //   iy = p.y()/p.w();
        int idx = (int)oy*w_ + (int)ox;
        ix = -1;
        iy = -1;
        if (idx >= 0 && idx < w_ * h_) {
            vgl_point_3d<double> p = points_[idx];
            if (!camera_.is_behind_camera(vgl_homg_point_3d<double>(p.x(), p.y(), p.z(), 1.0))) {
                vgl_point_2d<double> q = camera_.project(p);
                ix = q.x();
                iy = q.y();
            }
        }
    }
    private:
    vpgl_perspective_camera<double> camera_;
    vcl_vector<vgl_point_3d<double> > points_;
    int w_;
    int h_;
};


//bilinear interperation
static vxl_byte InterpolatorFunc( vil_image_view< vxl_byte > const& view, double x, double y, unsigned p )
{
	return vil_bilin_interp_safe( view, x, y, p);
}


vil_image_view<vxl_byte> VglCylinderMapping::warp(const vcl_vector<vgl_point_3d<double> > & points,
                                                  const vpgl_perspective_camera<double> & camera,
                                                  const vil_image_view<vxl_byte> & srcImage,
                                                  const int imageW, const int imageH)
{
    vil_image_view<vxl_byte> destImage = vil_image_view<vxl_byte>(imageW, imageH, 3);
    destImage.fill(0);
    
    vil_warp(srcImage, destImage, VxlCylinderWarp(camera, points, imageW, imageH), InterpolatorFunc);
    
    return destImage;
}

/************************         CylinderImageGenerator              ***********************/
void CylinderImageGenerator::addImage(const vil_image_view<vxl_byte> & image, const vil_image_view<bool> & mask)
{
    int w = image.ni();
    int h = image.nj();
    assert(w == red_channel_.ni() && w == green_channel_.ni() && w == blue_channel_.ni());
    assert(h == red_channel_.nj() && h == green_channel_.nj() && h == blue_channel_.nj());
    
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            if (mask(i, j)) {
                int r = image(i, j, 0);
                int g = image(i, j, 1);
                int b = image(i, j, 2);
                red_channel_(i, j, r)   += 1;
                green_channel_(i, j, g) += 1;
                blue_channel_(i, j, b)  += 1;
            }
        }
    }
}

// final composition result
vil_image_view<vxl_byte> CylinderImageGenerator::mostFrequentImage()
{
    int w = red_channel_.ni();
    int h = red_channel_.nj();
    
    vil_image_view<vxl_byte> image(w, h, 3);
    image.fill(0);
    
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            // red channel
            int idx = 0;    // most frequent pixel value appears
            int max_val = 0;
            for (int k = 0; k<256; k++) {
                if (red_channel_(i, j, k) > max_val) {
                    max_val = red_channel_(i, j, k);
                    idx = k;
                }
            }
            image(i, j, 0) = idx;
            
            // green channel
            idx = 0;
            max_val = 0;
            for (int k = 0; k<256; k++) {
                if (green_channel_(i, j, k) > max_val) {
                    max_val = green_channel_(i, j, k);
                    idx = k;
                }
            }
            image(i, j, 1) = idx;
            
            // blue channel
            idx = 0;
            max_val = 0;
            for (int k = 0; k<256; k++) {
                if (blue_channel_(i, j, k) > max_val) {
                    max_val = blue_channel_(i, j, k);
                    idx = k;
                }
            }
            image(i, j, 2) = idx;
        }
        printf("j is %d\n", j);
    }
    
    return image;
}

static vxl_byte medianPixel(const vil_image_view<vxl_int_32> & image, int x, int y)
{
    assert(image.nplanes() == 256);
    vxl_byte pixel = 0;
    int total_num = 0;
    for (int i = 0; i<256; i++) {
        total_num += image(x, y, i);
    }
    total_num /= 2;
    for (int i = 0; i<256; i++) {
        total_num -= image(x, y, i);
        if (total_num <= 0) {
            pixel = i;
            break;
        }
    }
    return pixel;
}

vil_image_view<vxl_byte> CylinderImageGenerator::medianImage()
{
    int w = red_channel_.ni();
    int h = red_channel_.nj();
    
    vil_image_view<vxl_byte> image(w, h, 3);
    image.fill(0);
    
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            image(i, j, 0) = medianPixel(red_channel_, i, j);
            image(i, j, 1) = medianPixel(green_channel_, i, j);
            image(i, j, 2) = medianPixel(blue_channel_, i, j);
        }
        printf("j is %d\n", j);
    }
    return image;
}


/************************         VglCylinderPanorama              ***********************/

VglCylinderPanorama::VglCylinderPanorama()
{
    printf("in complete version!");
    assert(0);
}

VglCylinderPanorama::~VglCylinderPanorama()
{
    
}

bool VglCylinderPanorama::setCenterCamera(const vpgl_perspective_camera<double> & camera)
{
    vgl_homg_plane_3d<double> pc_plane = camera.principal_plane();
    vnl_matrix_fixed<double, 3, 4> P   = camera.get_matrix();
    vnl_vector<double> P_row1          = P.get_row(0);  // plane pass camera center and project to line x = 0 in image
    assert(P_row1.size() == 4);
    
    vgl_plane_3d<double> P_x(P_row1[0], P_row1[1], P_row1[2], P_row1[3]);
    vgl_plane_3d<double> P_z(pc_plane);
    vgl_line_3d_2_points<double> line;
    bool isInterect =  vgl_intersection(P_x, P_z, line);
    if (!isInterect) {
        return false;
    }
    // vcl_cout<<"cylinder direction is "<<line.direction()<<vcl_endl;
    cylinder_.set_center(camera.camera_center());
    cylinder_.set_radius(camera.get_calibration().focal_length());
    cylinder_.set_orientation(line.direction());
    cylinder_.set_length(1000); // the length is not important
    camera_ = camera;
    return true;
}

bool VglCylinderPanorama::setCenterImage(const vil_image_view<vxl_byte> & image)
{
    assert(image.nplanes() == 3);
    panoramaImage_.deep_copy(image);
    return true;
}

vil_image_view<vxl_byte> VglCylinderPanorama::projectToVirtualCamera(const vpgl_perspective_camera<double> & camera,
                                                                     int imageW, int imageH)
{
    vil_image_view<vxl_byte> image = vil_image_view<vxl_byte>(imageW, imageH, 3);
    image.fill(0);
    
    //
    vgl_point_3d<double> c1 = camera.get_camera_center();
    vgl_point_3d<double> c2 = camera_.get_camera_center();
    double dx = c1.x() - c2.x();
    double dy = c1.y() - c2.y();
    double dz = c1.z() - c2.z();
    printf("distance between camera center is %f %f %f\n", dx, dy, dz);    
    
    vgl_h_matrix_2d<double> H = VpglPlus::homographyFromCameraToCamera(camera_, camera);
    vil_image_view<vxl_byte> outImage;
    outImage.deep_copy(image);
    bool isWarp = VilPlus::homography_warp_fill(panoramaImage_, H, image, outImage);
    assert(isWarp);
    
    return outImage;
}



