//
//  wwosSyncPanOverview.cpp
//  PlayerTracking
//
//  Created by jimmy on 7/21/15.
//  Copyright (c) 2015 Disney Reasearch. All rights reserved.
//

#include "wwosSyncPanOverview.h"
#include "vxl_plus.h"
#include <vcl_vector.h>
#include "simple_hash_table.h"

WWoSSyncPanOverview::WWoSSyncPanOverview()
{
    vnl_vector_fixed<int, 4> syncFrames = WWoSSyncPanOverview::MarkinCameraSyncFrames();
    ptz1_ = syncFrames[0];
    ptz2_ = syncFrames[1];
    stationary1_ = syncFrames[2];
    stationary2_ = syncFrames[3];    
}

WWoSSyncPanOverview::WWoSSyncPanOverview(const vnl_vector_fixed<int, 4> & syncFrames)
{
    ptz1_ = syncFrames[0];
    ptz2_ = syncFrames[1];
    stationary1_ = syncFrames[2];
    stationary2_ = syncFrames[3];
}

WWoSSyncPanOverview::~WWoSSyncPanOverview()
{
    
}

bool WWoSSyncPanOverview::readObservation(const char *folder, int start_fn, int step)
{
    vcl_vector<vcl_string> image_names;
    VulPlus::readFileNames(folder, image_names);
    
    observations_.clear();
    for (int i = 0; i<image_names.size(); i++) {
        ObservationData od;
        od.fn_ = start_fn + step * i;
        od.image_name_ = image_names[i];
        observations_.push_back(od);
    }
    printf("load %lu observations\n", observations_.size());    
    return true;
}

bool WWoSSyncPanOverview::readPTZCameras(const char *ptzFilename, int dump_lines_num)
{
    assert(ptzFilename);
    
    FILE *pf = fopen(ptzFilename, "r");
    if (!pf) {
        vcl_cout<<"can not open file "<<ptzFilename<<vcl_endl;
        return false;
    }
    
    for (int i = 0; i<dump_lines_num; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf;
    }
    
    ptzs_.clear();
    while (1) {
        PTZData ptz;
        int ret = fscanf(pf, "%d %lf %lf %lf %lf", &ptz.frame_num, &ptz.fl, &ptz.pan, &ptz.tilt, &ptz.ssd);
        if (ret != 5) {
            break;
        }
        ptzs_.push_back(ptz);
    }
    fclose(pf);
    printf("read %lu ptz data.\n", ptzs_.size());
    return true;
}

bool WWoSSyncPanOverview::syncObservationPTZ(vcl_vector<ObservationPTZData> & observation_ptzs)
{
    assert(ptzs_.size() > 0);
    assert(observations_.size() > 0);
    
    vcl_vector<int> ptz_fns;
    for (int i = 0; i<ptzs_.size(); i++) {
        ptz_fns.push_back(ptzs_[i].frame_num);
    }
    
    SimpleHashTable<PTZData> hashtable(ptz_fns, ptzs_);
    // for every observation find a pan angle
    for (int i = 0; i<observations_.size(); i++) {
        int fn25 = observations_[i].fn_;
        int fn60 = this->getFnPTZ(fn25);
        if (fn60 == -1) {
            continue;
        }
        PTZData pd;
        bool isFind = hashtable.find(fn60, pd);
        if (isFind) {
            ObservationPTZData opd;
            opd.fn_ = fn25;
            opd.image_name_ = observations_[i].image_name_;
            opd.pan_ = pd.pan;
            
            observation_ptzs.push_back(opd);
        }
    }
    printf("find %lu feature frames\n", observation_ptzs.size());
    return true;
}

int WWoSSyncPanOverview::getFnPTZ(int fn25)
{
    if (fn25 < stationary1_ || fn25 > stationary2_) {
        return -1;
    }
    
    return 1.0 * (fn25 - stationary1_)/(stationary2_ - stationary1_)*(ptz2_ - ptz1_) + ptz1_;
}

bool WWoSSyncPanOverview::writeObservationPTZ(const char *fileName, const vcl_vector<ObservationPTZData> & observation_ptzs)
{
    FILE *pf = fopen(fileName, "w");
    assert(pf);
    
    fprintf(pf, "%d\n", (int)observation_ptzs.size());
    for (int i = 0; i<observation_ptzs.size(); i++) {
        ObservationPTZData opd = observation_ptzs[i];
        fprintf(pf, "%d\t %f\t %s\n", opd.fn_, opd.pan_, opd.image_name_.c_str());
    }
    fclose(pf);
    printf("write to %s\n", fileName);
    return true;
}

bool WWoSSyncPanOverview::readObservationPTZ(const char *fileName, vcl_vector<ObservationPTZData> & observation_ptzs)
{
    FILE *pf = fopen(fileName, "r");
    if (!pf) {
        printf("can not read from file %s\n", fileName);
    }
    int num = 0;
    int ret = fscanf(pf, "%d", &num);
    assert(ret == 1);
    for (int i = 0; i<num; i++) {
        int fn = 0;
        double pan = 0.0;
        char buf[1024] = {NULL};
        ret = fscanf(pf, "%d %lf %s", &fn, &pan, buf);
        assert(ret == 3);
        ObservationPTZData opd;
        opd.fn_ = fn;
        opd.pan_ = pan;
        opd.image_name_ = vcl_string(buf);
        observation_ptzs.push_back(opd);
    }
    printf("read %lu observation frames\n", observation_ptzs.size());
    return true;
}










