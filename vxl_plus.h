//
//  vxl_plus.h
//  VpglPtzOpt
//
//  Created by jimmy on 11/15/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#ifndef __VpglPtzOpt__vxl_plus__
#define __VpglPtzOpt__vxl_plus__

#include <vil/vil_image_view.h>
#include <vil/vil_load.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vcl_string.h>
#include <vsl/vsl_binary_io.h>
#include <vgl/io/vgl_io_point_2d.h>
#include <vpgl/io/vpgl_io_perspective_camera.h>
#include <vpgl/io/vpgl_io_proj_camera.h>
#include <vgl/io/vgl_io_line_segment_2d.h>
#include <vgl/io/vgl_io_infinite_line_3d.h>
#include <vgl/io/vgl_io_h_matrix_2d.h>
#include <vpgl/vpgl_fundamental_matrix.h>


inline bool vgl_inside_image(const vgl_point_2d<double> &p, int width, int height)
{
    return p.x() >= 0 && p.x() < width && p.y() >= 0 && p.y() < height;
}

inline bool vgl_inside_image(const vgl_point_2d<double> &p, int width, int height, int threshold)
{
    // assert(threshold > 0);
    return p.x() >= (0 - threshold) && p.x() < (width + threshold) && p.y() >= (0 - threshold) && p.y() < (height + threshold);
}

inline bool vgl_inside_rect(const vgl_point_2d<double> &p, int min_x, int min_y, int max_x, int max_y)
{
    assert(min_x < max_x);
    assert(min_y < max_y);
    
    return p.x() >= min_x && p.x() < max_x && p.y() >= min_y && p.y() < max_y;
}


class VxlCoutControl
{
    std::streambuf* cout_sbuf_;
    public:
    VxlCoutControl()
    {
        cout_sbuf_ = NULL;
    }
    ~VxlCoutControl()
    {
        if (cout_sbuf_) {
            std::cout.rdbuf(cout_sbuf_);
        }
    }
    
    void start()
    {
        cout_sbuf_ = std::cout.rdbuf(); // save original sbuf
        std::ofstream   fout("/dev/null");
        std::cout.rdbuf(fout.rdbuf()); // redirect 'cout' to a 'fout'
    }
    
    void end()
    {
        std::cout.rdbuf(cout_sbuf_);
    }
    
};

class VxlPlus
{
public:
    static void splitFilename (const vcl_string& str, vcl_string &path, vcl_string &file)
    {
        assert(!str.empty());
        unsigned int found = (unsigned int )str.find_last_of("/\\");
        path = str.substr(0, found);
        file = str.substr(found + 1);
    }   
    
    
    static bool readTxtToMat(const char *fileName, const int nRow, const int nCol, vnl_matrix<double> & mat)
    {
        FILE *pf = fopen(fileName, "r");
        if (!pf) {
            vcl_cout<<"can not open: "<<fileName<<vcl_endl;
            return false;
        }
        mat = vnl_matrix<double>(nRow, nCol, 0);
        for (int i = 0; i<nRow; i++) {
            for (int j = 0; j<nCol; j++) {
                double v = 0;
                if(fscanf(pf, "%lf", &v) != 1)
                {
                    fclose(pf);
                    return false;
                }
                mat(i, j) = v;
            }
        }
        fclose(pf);
        return true;
    }
   
};

class VulPlus
{
public:
    // folder: /Users/chenjLOCAL/Desktop/*.txt
    static void readFileNames(const char *folder, vcl_vector<vcl_string> & files);
    
    static void splitFilename (const vcl_string& str, vcl_string &path, vcl_string &file)
    {
        assert(!str.empty());
        unsigned int found = (unsigned int )str.find_last_of("/\\");
        path = str.substr(0, found);
        file = str.substr(found + 1);
    }

};


template <class T>
class VglSaveLoadVector
{
public:
    static void save(const char *fileName, const vcl_vector<T> &vgls)
    {
        vsl_b_ofstream bfs_out(fileName);
        vcl_cout<<"save vgl vector, size is "<<vgls.size()<<vcl_endl;
        vsl_b_write(bfs_out, (int)vgls.size());
        
        for (int i = 0; i<vgls.size(); i++) {
            vsl_b_write(bfs_out, vgls[i]);
        }
        bfs_out.close();
    }
    
    static void load(const char *fileName, vcl_vector<T> &vgls)
    {
        vsl_b_ifstream bfs_in(fileName);
        assert(bfs_in.is().good() == true);
        
        int num = 0;
        vsl_b_read(bfs_in, num);
        vgls.resize(num);
        
        // && bfs_in.is().eof() != true not work
        for (int i = 0; i < num ; i++) {
            vsl_b_read(bfs_in, vgls[i]);
            assert(bfs_in.is().good() == true);
        }
        bfs_in.close();
        vcl_cout<<"load vgl vector, size is "<<vgls.size()<<vcl_endl;
    }
    
    // save load pairs
    static void save(const char *fileName, const vcl_vector< vcl_string > & imageNames, const vcl_vector< T > & propertys)
    {
        assert(imageNames.size() == propertys.size());
        
        vsl_b_ofstream bfs_out(fileName);
        vsl_b_write(bfs_out, (int)imageNames.size()); // pair number
        
        for (int i = 0; i<imageNames.size(); i++) {
             vsl_b_write(bfs_out, imageNames[i]);
        }
        for (int i = 0; i<propertys.size(); i++) {
            vsl_b_write(bfs_out, propertys[i]);
        }
        
        bfs_out.close();
    }
    
    static void load(const char *fileName, vcl_vector< vcl_string > & imageNames, vcl_vector< T > & propertys)
    {
        assert(imageNames.size() == 0);
        assert(propertys.size() == 0);
        
        vsl_b_ifstream bfs_in(fileName);
        assert(bfs_in.is().good() == true);
        
        int num = 0;
        vsl_b_read(bfs_in, num);
        imageNames.resize(num);
        propertys.resize(num);
        
        for (int i = 0; i<num; i++){           
            vsl_b_read(bfs_in, imageNames[i]);
        }
        
        for (int i = 0; i<num; i++){
            vsl_b_read(bfs_in, propertys[i]);
        }
        bfs_in.close();
    }

    
    static void save(const char *fileName, const char *imageName, const T &property)
    {
        vsl_b_ofstream bfs_out(fileName);
        
        vsl_b_write(bfs_out, imageName);        
        vsl_b_write(bfs_out, property);
        bfs_out.close();
    }
    
    static void load(const char *fileName, vcl_string &imageName, T &property)
    {
        vsl_b_ifstream bfs_in(fileName);
        
        char strName[1024] = {NULL};
        vsl_b_read(bfs_in, strName);
        imageName = vcl_string(strName);
        
        vsl_b_read(bfs_in, property);
        bfs_in.close();
    }
    
    static void loadPerspectiveCamera(const char *fileName, vcl_string &imageName, vpgl_perspective_camera<double> & camera)
    {
        vsl_b_ifstream bfs_in(fileName);
        
        char strName[1024] = {NULL};
        vsl_b_read(bfs_in, strName);
        imageName = vcl_string(strName);
        
        vsl_b_read(bfs_in, camera);
        
        // bug when load camera directly
        double fl = camera.get_calibration().get_matrix()[0][0];
        vgl_point_2d<double> pp = camera.get_calibration().principal_point();
        vpgl_calibration_matrix<double> K(fl, pp);
        camera.set_calibration(K);
        
        bfs_in.close();
    }
    
    // assume the image in the same directory and has the same name as camera
    //
    static void loadPerspectiveCameraImage(const char *fileName, vpgl_perspective_camera<double> & camera, vil_image_view<vxl_byte> & image)
    {
        vsl_b_ifstream bfs_in(fileName);
        
        // these two line is redundent
        char strName[1024] = {NULL};
        vsl_b_read(bfs_in, strName);
        
        vsl_b_read(bfs_in, camera);
        // bug when load camera directly
        double fl = camera.get_calibration().get_matrix()[0][0];
        vgl_point_2d<double> pp = camera.get_calibration().principal_point();
        vpgl_calibration_matrix<double> K(fl, pp);
        camera.set_calibration(K);
        bfs_in.close();
        
        vcl_string ss(fileName);
        unsigned found = (unsigned)ss.find_last_of(".");
        assert(found + 4 < strlen(fileName));
        ss[found+1] = 'j';
        ss[found+2] = 'p';
        ss[found+3] = 'g';
        ss[found+4] = NULL;
        image = vil_load(ss.c_str());
    }
    
    
};


template <class T1, class T2>
class VglSaveLoadVectorPair
{
public:
    static void save(const char *fileName, const char *imageName, const vcl_vector< T1 > &wldFeatures, const vcl_vector< T2 > &imgFeatures)
    {
        assert(wldFeatures.size() == imgFeatures.size());
        
        vsl_b_ofstream bfs_out(fileName);
        
        vsl_b_write(bfs_out, imageName);          // image name
        vsl_b_write(bfs_out, (int)wldFeatures.size()); // pts number
        
        for (int i = 0; i<wldFeatures.size(); i++) {
            vsl_b_write(bfs_out, wldFeatures[i]);
        }
        for (int i = 0; i<imgFeatures.size(); i++) {
            vsl_b_write(bfs_out, imgFeatures[i]);
        }
        
        bfs_out.close();
    }
    static void load(const char *fileName, vcl_string &imageName, vcl_vector<T1> &wldFeatures, vcl_vector< T2 > &imgFeatures)
    {
        assert(wldFeatures.size() == 0);
        assert(imgFeatures.size() == 0);
        
        vsl_b_ifstream bfs_in(fileName);
        assert(bfs_in.is().good() == true);
        
        char strName[1024] = {NULL};
        vsl_b_read(bfs_in, strName);
        imageName = vcl_string(strName);
        
        int num = 0;
        vsl_b_read(bfs_in, num);
        wldFeatures.resize(num);
        imgFeatures.resize(num);
        
        for (int i = 0; i<num; i++){
            vsl_b_read(bfs_in, wldFeatures[i]);
        }
        
        for (int i = 0; i<num; i++){
            vsl_b_read(bfs_in, imgFeatures[i]);
        }
        bfs_in.close();
    }
};

#include "vxl_plus.txx"



#endif /* defined(__VpglPtzOpt__vxl_plus__) */
