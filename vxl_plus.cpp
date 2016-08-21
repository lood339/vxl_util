//
//  vil_plus.cpp
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vxl_plus.h"
#include <vxl_config.h>
#include <vil/vil_image_view.h>
#include <vil/vil_load.h>
#include <vil/vil_save.h>
//#include <vul/vul_file_list.h>
#include <vsl/vsl_binary_io.h>
#include <vgl/io/vgl_io_point_2d.h>
#include <vnl/io/vnl_io_matrix.h>

#include <vnl/vnl_matlab_filewrite.h>


#include "vil_plus.h"
#include <vnl/vnl_math.h>
//#include "vnl_plus.h"



/*
 *************************** VulPlus ********************************
 */

void VulPlus::readFileNames(const char *folder, vcl_vector<vcl_string> & files)
{
    printf("VulPlus::readFileNames is unfinised.\n");
    assert(0);
    
    /*
    assert(folder);
    vcl_list<vcl_string> fileList = vul_file_list(folder);
    
    files.resize(0);
    for (vcl_list<vcl_string>::iterator ite = fileList.begin(); ite != fileList.end(); ite++) {
        files.push_back(*ite);
    }
    */
}













