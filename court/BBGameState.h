//
//  BBGameState.h
//  PlayerTracking
//
//  Created by Jimmy Chen LOCAL on 4/22/14.
//  Copyright (c) 2014 Disney Reasearch. All rights reserved.
//

#ifndef __PlayerTracking__BBGameState__
#define __PlayerTracking__BBGameState__

// basketball game state
#include <iostream>
#include <vcl_vector.h>

struct BBGameStateTime {
    double h_;
    double m_;
    double s_;
    
    BBGameStateTime()
    {}
    
    BBGameStateTime(int h, int m, int s)
    {h_ = h; m_ = m; s_ = s;}
};

enum GameState{game_on, game_penalty, game_rest, game_restart, game_pause} ;

struct BBGameTimeSlot
{
    BBGameStateTime start_;
    BBGameStateTime end_;
    GameState state_;
};


bool readGameTimeSlot(const char *file_name, vcl_vector<BBGameTimeSlot> & slots);

//
bool isGameOn(const unsigned long nFrame, double frame_rate, const vcl_vector<BBGameTimeSlot> & slots);

// it this frame in gamestat (gs
bool getGameState(const unsigned long nFrame, double frame_rate, const vcl_vector<BBGameTimeSlot> & slots, GameState & gs);




#endif /* defined(__PlayerTracking__BBGameState__) */
