//
//  vil_HOG.h
//  PlayerTracking
//
//  Created by Jimmy Chen LOCAL on 4/13/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __PlayerTracking__vil_HOG__
#define __PlayerTracking__vil_HOG__

#include <iostream>
#include <vgl/vgl_point_2d.h>
#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vnl/vnl_vector.h>

// histogram of gradient
struct HoGDescription
{
    vgl_point_2d<double> position_;
    vil_image_view<double> description_;
    int cellSize_;
};

class VilHOG
{

public:
    //block size = cell size
    static void getHOGdescriptionNonOverlap(const vil_image_view<vxl_byte> & image, int cellSize, vcl_vector<HoGDescription> & hogs);
    
    //block size = cell_size * 2 + 2;
    static void getHOGdescriptionWithBlock(const vil_image_view<vxl_byte> & image, int cellSize, vcl_vector<HoGDescription> & hogs);
    
    static void visualizeHOG(int destWidth, int destHeight, vcl_vector<HoGDescription> & hogs, vil_image_view<vxl_byte> & hogMap);
    
    static void visualizeHOG2X2(int destWidth, int destHeight, vcl_vector<HoGDescription> & hogs, vil_image_view<vxl_byte> & hogMap);
    
    
    static void hogsToVector(const vcl_vector<HoGDescription> & hogs, vnl_vector<double> & feature);
    
};

#endif /* defined(__PlayerTracking__vil_HOG__) */
