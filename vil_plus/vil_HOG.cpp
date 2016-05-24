//
//  vil_HOG.cpp
//  PlayerTracking
//
//  Created by Jimmy Chen LOCAL on 4/13/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vil_HOG.h"
#include <vil/vil_convert.h>
#include <vil/vil_crop.h>
#include <mipa/mipa_orientation_histogram.h>
#include <mipa/mipa_orientation_histogram.txx>
#include <vcl_istream.h>
#include <vnl/vnl_math.h>
#include <vicl/vicl_line_segment.h>
#include <vnl/vnl_vector.h>

/*
struct HoGDescription
{
    vgl_point_2d<double> position_;
    vil_image_view<double> description_;
    int cellSize_;
};
 */

void VilHOG::getHOGdescriptionNonOverlap(const vil_image_view<vxl_byte> & image, int cellSize, vcl_vector<HoGDescription> & hogs)
{
    assert(image.nplanes() == 3);
    
    vil_image_view<vxl_byte> image_grey;
    vil_convert_planes_to_grey(image, image_grey);
    
    const int w = image_grey.ni();
    const int h = image_grey.nj();
    
    int patchSize = cellSize + 2;
    for (int y = patchSize; y < h - patchSize; y += cellSize/2) {
        for (int x = patchSize; x < w - patchSize; x += cellSize/2) {
            int i0 = x - patchSize/2;
            int j0 = y - patchSize/2;
            
            HoGDescription hog;
            vil_image_view<vxl_byte> patch = vil_crop(image_grey, i0, patchSize, j0, patchSize);
            mipa_orientation_histogram(patch, hog.description_, 9, cellSize, false);
            
            hog.position_ = vgl_point_2d<double>(x, y);
            hog.cellSize_ = cellSize;
            
            hogs.push_back(hog);
        }
    }
}

void VilHOG::getHOGdescriptionWithBlock(const vil_image_view<vxl_byte> & image, int cellSize, vcl_vector<HoGDescription> & hogs)
{
    assert(image.nplanes() == 3);
    
    vil_image_view<vxl_byte> image_grey;
    vil_convert_planes_to_grey(image, image_grey);
    
    const int w = image_grey.ni();
    const int h = image_grey.nj();
    const int block_size = cellSize * 2 + 2;
    for (int y = block_size/2; y < h - block_size/2; y += block_size/2){
        for (int x = block_size/2; x < w - block_size/2; x += block_size/2) {
            int i0 = x - block_size/2;
            int j0 = y - block_size/2;
            
            HoGDescription hog;
            vil_image_view<vxl_byte> patch = vil_crop(image_grey, i0, block_size, j0, block_size);
            mipa_orientation_histogram(patch, hog.description_, 9, cellSize, false);
            
            hog.position_ = vgl_point_2d<double>(x, y);
            hog.cellSize_ = cellSize;
            hogs.push_back(hog);
        }
    }
}

void VilHOG::visualizeHOG(int destWidth, int destHeight, vcl_vector<HoGDescription> & hogs, vil_image_view<vxl_byte> & hogMap)
{
    int scale = 4.0;
    hogMap = vil_image_view<vxl_byte>(destWidth * scale, destHeight * scale, 1);
    hogMap.fill(0);
    
    // maximum value of magnitue for normalization
    double val_max = -1;
    for (int i = 0; i<hogs.size(); i++) {
        vil_image_view<double> desp = hogs[i].description_;
        assert(desp.ni() == 1 && desp.nj() == 1);
        for (int j = 0; j<desp.nplanes(); j++) {
            if (desp(0, 0, j) > val_max) {
                val_max = desp(0, 0, j);
            }
        }
    }
    assert(val_max > 0);
    vcl_cout<<"val_max is "<<val_max<<vcl_endl;
    
    // draw HOG as a "star"
    for (int i = 0; i<hogs.size(); i++) {
        vgl_point_2d<double> p = vgl_point_2d<double>(hogs[i].position_.x() * scale, hogs[i].position_.y() * scale);
        double r = hogs[i].cellSize_/2 * scale;
        for (int j = 0; j<hogs[i].description_.nplanes(); j++) {
            double ang = 1.0 * j/hogs[i].description_.nplanes() * vnl_math::pi + vnl_math::pi/2.0;
            vgl_point_2d<double> q1(p.x() + r * cos(ang), p.y() + r * sin(ang));
            vgl_point_2d<double> q2(p.x() - r * cos(ang), p.y() - r * sin(ang));
            int intensity = hogs[i].description_(0, 0, j)/val_max * 255;
            
            vcl_vector<vxl_byte> color;
            color.push_back(intensity);
            vicl_overlay_line_segment(hogMap, vgl_line_segment_2d<double>(q1, q2), color, 1);
        }
    }
}

void VilHOG::visualizeHOG2X2(int destWidth, int destHeight, vcl_vector<HoGDescription> & hogs, vil_image_view<vxl_byte> & hogMap)
{
    int scale = 4.0;
    hogMap = vil_image_view<vxl_byte>(destWidth * scale, destHeight * scale, 1);
    hogMap.fill(0);
    
    // maximum value of magnitue for normalization
    double val_max = -1;
    for (int k = 0; k<hogs.size(); k++) {
        vil_image_view<double> desp = hogs[k].description_;
        assert(desp.ni() == 2 && desp.nj() == 2);
        for (int j = 0; j<2; j++) {
            for (int i = 0; i<2; i++) {
                for (int m = 0; m<desp.nplanes(); m++) {
                    double val = desp(i, j, m);
                    if (val > val_max) {
                        val_max = val;
                    }
                }
            }
        }
    }
    const int hc = hogs[0].cellSize_/2 * scale;
    const int offset[4][2] = {
        -hc, -hc,
         hc, -hc,
        -hc,  hc,
         hc,  hc
    };
    
    const int r = hc; // radius
    
    // draw HOG in each cell as a 'star'
    for (int k = 0; k<hogs.size(); k++) {
        vil_image_view<double> desp = hogs[k].description_;
        vgl_point_2d<double> bc = vgl_point_2d<double>(hogs[k].position_.x() * scale, hogs[k].position_.y() * scale);        //block center
        
        for (int j = 0; j<2; j++) {
            for (int i = 0; i<2; i++) {
                int idx = j * 2 + i;                
                vgl_point_2d<double> p(bc.x() + offset[idx][0], bc.y() + offset[idx][1]);
                
                for (int m = 0; m<desp.nplanes(); m++) {
                    double ang = 1.0 * m / desp.nplanes() * vnl_math::pi + vnl_math::pi/2.0;
                    
                    vgl_point_2d<double> q1(p.x() + r * cos(ang), p.y() + r * sin(ang));
                    vgl_point_2d<double> q2(p.x() - r * cos(ang), p.y() - r * sin(ang));
                    double val = desp(i, j, m);
                    int intensity = val/val_max * 255;
                    
                    vcl_vector<vxl_byte> color;
                    color.push_back(intensity);
                    vicl_overlay_line_segment(hogMap, vgl_line_segment_2d<double>(q1, q2), color);
                }
            }
        }
    }
}

void VilHOG::hogsToVector(const vcl_vector<HoGDescription> & hogs, vnl_vector<double> & feature)
{
    vcl_vector<double> data;
    for (int k = 0; k<hogs.size(); k++) {
        vil_image_view<double> desp = hogs[k].description_;
        for (int j = 0; j<desp.nj(); j++) {
            for (int i = 0; i < desp.ni(); i++) {
                for (int p = 0; p<desp.nplanes(); p++) {
                    data.push_back(desp(i, j, p));
                }
            }
        }
    }
    
    feature = vnl_vector<double>(&data[0], (unsigned int)data.size());
}








