//
//  SoccerLineParameters.h
//  OnlineStereo
//
//  Created by jimmy on 6/30/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef OnlineStereo_SoccerLineParameters_h
#define OnlineStereo_SoccerLineParameters_h

enum SoccerLineType{Internal, FarTouchBorder, NearTouchBorder, RightGoalBorder};

struct FarTouchLineParameter
{
    // detect advertising line
    double mag_threshold_;
    double greenGMMProbThreshold_;
    int look_up_pixel_;
    
    // detect touch line
    double mag_threshold_2_;
    double lambda_;
    double lineWidth_;
    FarTouchLineParameter()
    {
        mag_threshold_ = 0.11;
        greenGMMProbThreshold_ = 0.95;
        look_up_pixel_ = 10;   // loop up sevel pixel to make sure it is not in court field
        
        mag_threshold_2_ = 0.02;
        lambda_ = 1.5;         // compare with accumulated other
        lineWidth_ = 20;
    }
};

struct AverageMagnitudeParameter
{
    double mag_threshold_;
    double lambda_;        // times of average of non-edge pixel gradient
    int direction_;        // 0 horizontal, 1 vertical, 2, left_top to right_buttom diagonal, 3, left_buttom to right_top diagonal
    
    AverageMagnitudeParameter()
    {
        mag_threshold_ = 0.05;
        lambda_ = 2.0;
        direction_ = -1;
    }
};

// left sideview border line parameter
struct LeftsideViewBorderLineParameter
{
    double mag_threshold_;
    double greenGMMProbThreshold_;
    double lambda_;
    double lineWidth_;
    double shift_distance_;
    
    LeftsideViewBorderLineParameter()
    {
        mag_threshold_ = 0.015;         // must larger than
        greenGMMProbThreshold_ = 0.95;  // larger than
        lambda_ = 1.5;                  // larger than
        lineWidth_ = 50;               // search scale
        shift_distance_ = 30.0;
    }
    
};

//
struct FarTouchLineLeftSideviewParameter: public LeftsideViewBorderLineParameter
{
    FarTouchLineLeftSideviewParameter()
    {
        mag_threshold_ = 0.015;         // must larger than
        greenGMMProbThreshold_ = 0.95;  // larger than
        lambda_ = 1.5;                  // larger than
        lineWidth_ = 100;               // search scale
        
        shift_distance_ = 30.0;        
    }
};

struct FarCommerticalBoarderLineParameter : public LeftsideViewBorderLineParameter
{
    int grass_mask;
    int cb_mask;
    FarCommerticalBoarderLineParameter()
    {
        mag_threshold_ = 0.05;         // must larger than
        greenGMMProbThreshold_ = 0.95;  // larger than
        lambda_ = 5.0;                  // larger than
        lineWidth_ = 100;               // search scale
        
        shift_distance_ = 30.0;
        
        grass_mask = 0;
        cb_mask = 1;
    }
};

// border line behind right goal
struct RightBorderLine :public LeftsideViewBorderLineParameter
{
    int grass_mask;
    int cb_mask;
    RightBorderLine()
    {
        mag_threshold_ = 0.03;         // must larger than
        greenGMMProbThreshold_ = 0.95;  // larger than
        lambda_ = 5.0;                  // larger than
        lineWidth_ = 100;               // search scale
        
        shift_distance_ = 30.0;
        grass_mask = 0;
        cb_mask = 1;
    }
};




#endif
