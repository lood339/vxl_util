//
//  vgl_cylinder_mapping.h
//  OnlineStereo
//
//  Created by jimmy on 1/15/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vgl_cylinder_mapping__
#define __OnlineStereo__vgl_cylinder_mapping__

#include <vgl/vgl_cylinder.h>
#include <vcl_vector.h>
#include <vgl/vgl_line_segment_3d.h>
#include <vil/vil_image_view.h>
#include <vpgl/vpgl_perspective_camera.h>

class VglCylinderMapping
{
    private:
    vgl_cylinder<double> cylinder_;
    
    // pan: zero from the center of the cylinder image
    // z:   height with 0 in the image bottom
    double pan_min_;  // in degree
    double pan_max_;
    double z_max_;
    
    // resolution of the cylinder image
    int pixel_per_degree_in_pan_;
    int pixel_per_meter_in_z;
    
    
    
    void set_resolution()
    {
        pan_min_ = -40.0;
        pan_max_ =  40.0;
        z_max_ = 5.0;
        
        pixel_per_degree_in_pan_ = 40;
        pixel_per_meter_in_z = 200;
    }
    
    public:
    
    VglCylinderMapping()
    {
        set_resolution();
    }
    // only for Wild World of Sports basketball court
    VglCylinderMapping(const vpgl_perspective_camera<double> & camera)
    {
        vgl_point_3d<double> cc = camera.get_camera_center();
        // vgl_cylinder(Type cx, Type cy, Type cz, Type rad, Type len)
        cylinder_ = vgl_cylinder<double>(cc.x(), cc.y(), 0.0, 15.24 - cc.y(), cc.z());
        set_resolution();
    }
    
    // camera center, assume vgl_point_3d 12.9673,-14.6313,6.15212
    VglCylinderMapping(const vgl_point_3d<double> & cc)
    {
        cylinder_ = vgl_cylinder<double>(cc.x(), cc.y(), 0.0, 15.24 - cc.y(), cc.z());
        set_resolution();
    }
    
    ~VglCylinderMapping()
    {
        
    }
    
    void setCylinder(const vgl_cylinder<double> & cylinder){ cylinder_ = cylinder;}
    
    vil_image_view<vxl_byte> blankImage();
    
    // change from pixel to meter
    vgl_point_3d<double> imageLocation2World(const unsigned x, const unsigned y, const int imageW, const int imageH);
    
    vcl_vector<vgl_point_3d<double> > worldLocations(const int imageW, const int imageH);
    
    // warp the original image to cylinder image by projecting points
    vil_image_view<vxl_byte> warp(const vcl_vector<vgl_point_3d<double> > & points,
                                  const vpgl_perspective_camera<double> & camera,
                                  const vil_image_view<vxl_byte> & originalImage,
                                  const int imageW, const int imageH);
    
    // bars along in the cylinder surface, with the same direction of orientation
    vcl_vector<vgl_line_segment_3d<double> > verticalBarLineSegments(const unsigned int sampleNum);
};

// generate cylinder image from long image sequence
class CylinderImageGenerator
{
    // pixel value mapping table
    vil_image_view<vxl_int_32> red_channel_;
    vil_image_view<vxl_int_32> green_channel_;
    vil_image_view<vxl_int_32> blue_channel_;
    
    public:
    CylinderImageGenerator(int w, int h)
    {
        red_channel_   = vil_image_view<vxl_int_32>(w, h, 256);
        green_channel_ = vil_image_view<vxl_int_32>(w, h, 256);
        blue_channel_  = vil_image_view<vxl_int_32>(w, h, 256);
    }
    ~CylinderImageGenerator(){}
    
    void addImage(const vil_image_view<vxl_byte> & image, const vil_image_view<bool> & mask);
    
    // final composition result
    vil_image_view<vxl_byte> mostFrequentImage();
    vil_image_view<vxl_byte> medianImage();
};

// panorama from two static cameras
// the cylinder is centered at the left camera center, the right image is aligned to the left image
// the virtual camera look at the cylinder in 3D and fetch the pixels in a cylinder panorama
// un conplete version
class VglCylinderPanorama
{
    vgl_cylinder<double>     cylinder_;       // cylinder length is infinite
    vpgl_perspective_camera<double> camera_;  // camera centered at the cylinder
    vil_image_view<vxl_byte> panoramaImage_;  // image from the view of camera
public:
    VglCylinderPanorama();
    ~VglCylinderPanorama();
    
    bool setCenterCamera(const vpgl_perspective_camera<double> & camera);
    
    // image
    bool setCenterImage(const vil_image_view<vxl_byte> & image);
    
    // assume the camera center is approximatelly the same as the cylinder center 
    vil_image_view<vxl_byte> projectToVirtualCamera(const vpgl_perspective_camera<double> & camera,
                                                    int imageW, int imageH);
};

#endif /* defined(__OnlineStereo__vgl_cylinder_mapping__) */
