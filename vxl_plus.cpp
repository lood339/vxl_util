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
#include <dirent.h>
//#include "vnl_plus.h"



/*
 *************************** VulPlus ********************************
 */

void VulPlus::readFileNames(const char *folder, vcl_vector<vcl_string> & file_names)
{
    const char *post_fix = strrchr(folder, '.');
    vcl_string pre_str(folder);
    pre_str = pre_str.substr(0, pre_str.rfind('/') + 1);
    //printf("pre_str is %s\n", pre_str.c_str());
    
    assert(post_fix);
   // vcl_vector<vcl_string> file_names;
    DIR *dir = NULL;
    struct dirent *ent = NULL;
    if ((dir = opendir (pre_str.c_str())) != NULL) {
        /* print all the files and directories within directory */
        while ((ent = readdir (dir)) != NULL) {
            const char *cur_post_fix = strrchr( ent->d_name, '.');
            if (!cur_post_fix ) {
                continue;
            }
            //printf("cur post_fix is %s %s\n", post_fix, cur_post_fix);
            
            if (!strcmp(post_fix, cur_post_fix)) {
                file_names.push_back(pre_str + vcl_string(ent->d_name));
                //  cout<<file_names.back()<<endl;
            }
            
            //printf ("%s\n", ent->d_name);
        }
        closedir (dir);
    }
    printf("read %lu files\n", file_names.size());
}













