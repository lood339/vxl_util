//
//  vgl_line_model.cpp
//  WireframeModel
//
//  Created by jimmy on 6/19/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include "vgl_line_model.h"

VglLineModel::VglLineModel()
{
    
}
VglLineModel::~VglLineModel()
{
    
}

void VglLineModel::clear()
{
    imageNames_.clear();
    cameras_.clear();
    segmentsVec_.clear();
    unmatchedSegmentsVec_.clear();
    indexedSegmentsVec_.clear();
}

bool VglLineModel::write(const char *file) const
{
    FILE *pf = fopen(file, "w");
    if (!pf) {
        printf("can not write to file %s\n", file);
        return false;
    }
    if(imageNames_.size() != cameras_.size())
    {
        printf("Error: image number should be the same as camera number.\n");
        fclose(pf);
        return false;
    }
    if (imageNames_.size() != segmentsVec_.size()) {
        printf("Error: image number should be the same as segment group number.\n");
        fclose(pf);
        return false;
    }
    if (imageNames_.size() != unmatchedSegmentsVec_.size()) {
        printf("Error: image number should be the same as unmatched segment group number.\n");
        fclose(pf);
        return false;
    }
    if (imageNames().size() != indexedSegmentsVec_.size()) {
        printf("Error: image number should be the same as indexed segment group number.\n");
        fclose(pf);
        return false;
    }
    
    fprintf(pf, "%lu\n", imageNames_.size());
    // wrtie image name and camera
    for (int i = 0; i<imageNames_.size(); i++) {
        vpgl_perspective_camera<double> camera = cameras_[i];
        fprintf(pf, "%s\n", imageNames_[i].c_str());
        double ppx = camera.get_calibration().principal_point().x();
        double ppy = camera.get_calibration().principal_point().y();
        double fl = camera.get_calibration().get_matrix()[0][0];
        double Rx = camera.get_rotation().as_rodrigues()[0];
        double Ry = camera.get_rotation().as_rodrigues()[1];
        double Rz = camera.get_rotation().as_rodrigues()[2];
        double Cx = camera.get_camera_center().x();
        double Cy = camera.get_camera_center().y();
        double Cz = camera.get_camera_center().z();
        fprintf(pf, "%f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\t %f\n", ppx, ppy, fl, Rx, Ry, Rz, Cx, Cy, Cz);
    }
    
    fprintf(pf, "%lu %lu\n", segmentsVec_.size(), segmentsVec_[0].size());
    // write line segments
    for (int i = 0; i<segmentsVec_.size(); i++) {
        vcl_vector<vgl_line_segment_2d<double> > segs = segmentsVec_[i];
        for (int j = 0; j<segs.size(); j++) {
            vgl_point_2d<double> p1 = segs[j].point1();
            vgl_point_2d<double> p2 = segs[j].point2();
            fprintf(pf, "%f %f %f %f\t", p1.x(), p1.y(), p2.x(), p2.y());
        }
        fprintf(pf, "\n");
    }
    
    // write unmatched line segment
    fprintf(pf, "%lu\n", unmatchedSegmentsVec_.size());
    for (int i = 0; i<unmatchedSegmentsVec_.size(); i++) {
        vcl_vector<vgl_line_segment_2d<double> > segs = unmatchedSegmentsVec_[i];
        fprintf(pf, "%lu\n", segs.size());
        for (int j = 0; j<segs.size(); j++) {
            vgl_point_2d<double> p1 = segs[j].point1();
            vgl_point_2d<double> p2 = segs[j].point2();
            fprintf(pf, "%f %f %f %f\n", p1.x(), p1.y(), p2.x(), p2.y());
        }
    }
    // write indexed line segment
    fprintf(pf, "%lu\n", indexedSegmentsVec_.size());
    for (int i = 0; i<indexedSegmentsVec_.size(); i++) {
        vcl_vector<LineSegmentWithIndex> segs = indexedSegmentsVec_[i];
        fprintf(pf, "%lu\n", segs.size());
        for (int j = 0; j<segs.size(); j++) {
            int index = segs[j].index_;
            vgl_point_2d<double> p1 = segs[j].seg_.point1();
            vgl_point_2d<double> p2 = segs[j].seg_.point2();
            fprintf(pf, "%d %f %f %f %f\n", index, p1.x(), p1.y(), p2.x(), p2.y());
        }
    }
    fclose(pf);
    return true;
}

bool VglLineModel::read(const char *file)
{
    this->clear();
    FILE *pf = fopen(file, "r");
    if (!pf) {
        printf("can not read from file %s\n", file);
        return false;
    }
    int num = 0;
    int ret = fscanf(pf, "%d", &num);
    assert(ret == 1);
    for (int i = 0; i<num; i++) {
        char buf[1024] = {NULL};
        ret = fscanf(pf, "%s", buf);
        assert(ret == 1);
        
        double ppx, ppy, fl, rx, ry, rz, cx, cy, cz;
        ret = fscanf(pf, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", &ppx, &ppy, &fl, &rx, &ry, &rz, &cx, &cy, &cz);
        assert(ret == 9);
        
        vpgl_perspective_camera<double> camera;
        vpgl_calibration_matrix<double> K(fl, vgl_point_2d<double>(ppx, ppy));
        vnl_vector_fixed<double, 3> rod(rx, ry, rz);
        vgl_rotation_3d<double> R(rod);
        vgl_point_3d<double> cc(cx, cy, cz);
        
        camera.set_calibration(K);
        camera.set_rotation(R);
        camera.set_camera_center(cc);
        
        imageNames_.push_back(vcl_string(buf));
        cameras_.push_back(camera);
    }
    
    int seg_num = 0;
    int seg_pair_num = 0;
    ret = fscanf(pf, "%d %d", &seg_num, &seg_pair_num);
    assert(ret == 2);
    for (int i = 0; i<seg_num; i++) {
        vcl_vector<vgl_line_segment_2d<double> > segs;
        for (int j = 0; j<seg_pair_num; j++) {
            double x1 = 0;
            double y1 = 0;
            double x2 = 0;
            double y2 = 0;
            ret = fscanf(pf, "%lf %lf %lf %lf", &x1, &y1, &x2, &y2);
            assert(ret == 4);
            vgl_line_segment_2d<double> seg(vgl_point_2d<double>(x1, y1), vgl_point_2d<double>(x2, y2));
            segs.push_back(seg);
        }
        segmentsVec_.push_back(segs);
    }
    // read un-matched line segments
    int unmatched_seg_num = 0;
    ret = fscanf(pf, "%d", &unmatched_seg_num);
    assert(ret == 1);
    for (int i = 0; i<unmatched_seg_num; i++) {
        int seg_num = 0;
        ret = fscanf(pf, "%d", &seg_num);
        assert(ret == 1);
        vcl_vector<vgl_line_segment_2d<double> > segs;
        for (int j = 0; j<seg_num; j++) {
            double x1 = 0;
            double y1 = 0;
            double x2 = 0;
            double y2 = 0;
            ret = fscanf(pf, "%lf %lf %lf %lf", &x1, &y1, &x2, &y2);
            assert(ret == 4);
            vgl_line_segment_2d<double> seg(vgl_point_2d<double>(x1, y1), vgl_point_2d<double>(x2, y2));
            segs.push_back(seg);
        }
        unmatchedSegmentsVec_.push_back(segs);
    }
    // read indexed line segment
    int num_indexed_seg = 0;
    ret = fscanf(pf, "%d", &num_indexed_seg);
    assert(ret == 1);
    for (int i = 0; i<num_indexed_seg; i++) {
        int seg_num = 0;
        ret = fscanf(pf, "%d", &seg_num);
        assert(ret == 1);
        vcl_vector<LineSegmentWithIndex> segs;
        for (int j = 0; j<seg_num; j++) {
            int seg_index = 0;
            double x1 = 0;
            double y1 = 0;
            double x2 = 0;
            double y2 = 0;
            ret = fscanf(pf, "%d %lf %lf %lf %lf", &seg_index, &x1, &y1, &x2, &y2);
            assert(ret == 5);
            vgl_line_segment_2d<double> seg(vgl_point_2d<double>(x1, y1), vgl_point_2d<double>(x2, y2));
            LineSegmentWithIndex lineSeg;
            lineSeg.index_ = seg_index;
            lineSeg.seg_ = seg;
            segs.push_back(lineSeg);
        }
        indexedSegmentsVec_.push_back(segs);
    }
    
    // check the number of segments
    if (imageNames_.size() != segmentsVec_.size()) {
        printf("Error: image number should be the same as segment group number.\n");
        fclose(pf);
        return false;
    }
    
    // check the number of unmatched segments
    if (imageNames_.size() != unmatchedSegmentsVec_.size()) {
        printf("Error: image number should be the same as un matched segment group number.\n");
        fclose(pf);
        return false;
    }
    fclose(pf);    
    return true;
}


