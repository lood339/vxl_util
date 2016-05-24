//
//  BBGameState.cpp
//  PlayerTracking
//
//  Created by Jimmy Chen LOCAL on 4/22/14.
//  Copyright (c) 2014 Disney Reasearch. All rights reserved.
//

#include "BBGameState.h"
#include <stdio.h>
#include <vcl_iostream.h>
#include <vcl_string.h>
#include <assert.h>

bool readGameTimeSlot(const char *file_name, vcl_vector<BBGameTimeSlot> & slots)
{
    FILE *pf = fopen(file_name, "r");
    if (!pf) {
        vcl_cout<<"canot open "<<file_name<<vcl_endl;
        return false;
    }
    for (int i = 0; i<1; i++) {
        char lineBuf[BUFSIZ] = {NULL};
        fgets(lineBuf, sizeof(lineBuf), pf);
        vcl_cout<<lineBuf<<vcl_endl;
    }
    
    /*
     01:00:36 01:00:49 on
     01:00:49 01:00:18 off penalty
     */
    while (1) {
       // BBGameTimeSlot st;
        int h1 = 0, m1 = 0, s1 = 0;
        int h2 = 0, m2 = 0, s2 = 0;
        char st[BUFSIZ] = {NULL};
        int num = fscanf(pf, "%d:%d:%d %d:%d:%d %s", &h1, &m1, &s1,
                         &h2, &m2, &s2, st);
        if (num != 7) {
            break;
        }
        
        BBGameTimeSlot aTimeSlot;
        aTimeSlot.start_ = BBGameStateTime(h1, m1, s1);
        aTimeSlot.end_   = BBGameStateTime(h2, m2, s2);
        vcl_string aState(st);
        if (aState == vcl_string("penalty")) {
            aTimeSlot.state_ = game_penalty;
        }
        else if(aState == vcl_string("pause")){
            aTimeSlot.state_ = game_pause;
        }
        else if(aState == vcl_string("on")){
            aTimeSlot.state_ = game_on;
        }
        else if(aState == vcl_string("rest")){
            aTimeSlot.state_ = game_rest;
        }
        else if(aState == vcl_string("restart"))
        {
            aTimeSlot.state_ = game_restart;
        }
        else {
            vcl_cout<<"un-recognized state "<<aState<<vcl_endl;
            assert(0);
        }
        slots.push_back(aTimeSlot);
    }
    return true;
}

bool isGameOn(const unsigned long nFrame, double frame_rate, const vcl_vector<BBGameTimeSlot> & slots)
{
    assert(frame_rate == 25.0 || frame_rate == 60.0);
    for (int i = 0; i<slots.size(); i++) {
        BBGameStateTime sst = slots[i].start_;
        BBGameStateTime est = slots[i].end_;
        
        unsigned long int startIdx = (sst.h_ * 60 * 60 + sst.m_ * 60 + sst.s_) * frame_rate;
        unsigned long int stopIdx  = (est.h_ * 60 * 60 + est.m_ * 60 + est.s_) * frame_rate;
        if (nFrame >= startIdx && nFrame < stopIdx) {
            if (slots[i].state_ == game_on) {
                return true;
            }
            else {
                return false;
            }
        }
    }
    return false;
}

bool getGameState(const unsigned long nFrame, double frame_rate, const vcl_vector<BBGameTimeSlot> & slots, GameState & gs)
{
    assert(frame_rate == 25.0 || frame_rate == 60.0);
    for (int i = 0; i<slots.size(); i++) {
        BBGameStateTime sst = slots[i].start_;
        BBGameStateTime est = slots[i].end_;
        
        unsigned long int startIdx = (sst.h_ * 60 * 60 + sst.m_ * 60 + sst.s_) * frame_rate;
        unsigned long int stopIdx  = (est.h_ * 60 * 60 + est.m_ * 60 + est.s_) * frame_rate;
        if (nFrame >= startIdx && nFrame < stopIdx) {
            gs = slots[i].state_;
            return true;
        }
    }
    //cann't find game state
    return false;
}














