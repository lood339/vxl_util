//
//  vil_weak_line_tracking.h
//  QuadCopter
//
//  Created by jimmy on 7/9/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __QuadCopter__vil_weak_line_tracking__
#define __QuadCopter__vil_weak_line_tracking__

#include <vgl/vgl_vector_2d.h>
#include <vnl/vnl_vector_fixed.h>
#include <vil/vil_image_view.h>
#include <vcl_vector.h>
#include <vgl/vgl_point_2d.h>

struct WeakLineTrackingParameter
{
    double mag_threshold_1_;   // mininum threshold
    double mag_threshold_2_;   // maximum threshold
    double lambda_;            // times of average of non-edge pixel gradient
    double pixel_num_ratio_;   // minimum ratio threshold of how many pixels should be found
    int line_width_;           // potential width of the line
    double parallel_line_direction_threshold_;   // cos (angle)
    
    
  //  int scan_line_canny_direction_;
    int edge_neighbor_size_;        // check if a pixel is in the center of the edge or on the edge
    
    double avg_end_point_distance_;
    
    // for test
    int test_node_;
    
    WeakLineTrackingParameter()
    {
        mag_threshold_1_ = 0.05;
        mag_threshold_2_ = 0.2;
        lambda_ = 2.0;
        pixel_num_ratio_ = 0.1;
        
        line_width_ = 5;
        parallel_line_direction_threshold_ = cos(15.0/180*3.14);
        
        edge_neighbor_size_ = 3;
        avg_end_point_distance_ = 15.0;
        test_node_ = 0;
    }
};


// parameters for scan a line to find edge pixel
struct ScanLineParameter
{
    vgl_vector_2d<int> scan_dir_;
    double magnitude_threshold_;
    double lambda_threshold_;    // ratio of magnitude
    int scan_distance_;
   // int imageW_;
   // int imageH_;
    
    // return parameter
    vnl_vector_fixed<double, 2> mag_mean_std_;
    vnl_vector_fixed<double, 2> lambda_mean_std_;
    
    //
    int test_node_;   // for test
    
    ScanLineParameter()
    {
        
    }
};

// weak line tracking. The edge pixel is very week so the method has to optimize the parameters
class VilWeakLineTracking
{
private:
    // scan on one direction to get edge pixels and the edge's magnitude and lambda data
    static bool scan_on_one_direction(const vil_image_view<double> & magnitude,
                                      const vcl_vector<vgl_point_2d<double> > & scanLinePts,
                                      ScanLineParameter & para, vcl_vector<vgl_point_2d<double> > & edgePixels);
    
    // scan on one direction multiple times and get the majority as result
    static bool multiple_scan_one_direction(const vil_image_view<double> & magnitude,
                                            const vcl_vector<vgl_point_2d<double> > & scanLinePts,
                                            const ScanLineParameter & para, vcl_vector<vgl_point_2d<double> > & edgePixels);
    
    
public:
    // refine init line segment by search in two direction. search area can be large
    // assume the ground truth line segment is close to initial line segment but may lay oneside, or cross-laid
    static bool refineLineSegmentByMultipleParameter(const vil_image_view<vxl_byte> & image, // for test only
                                                     const vil_image_view<double> & mag, const vgl_line_segment_2d<double> & initLineSeg,
                                                     const WeakLineTrackingParameter & para, vgl_line_segment_2d<double> & refinedLineSeg);
    
    // assume lineSeg is very close to real line in the image.
    // assume the line is wider in the image
    // greedily use inlier pixel in on side of edge
    static bool trackingLineFromSegmentByMultipleParameter(const vil_image_view<vxl_byte> & image, // for test only
                                                           const vil_image_view<double> & magnitude,
                                                           const vil_image_view<double> & grad_i,
                                                           const vil_image_view<double> & grad_j,
                                                           const vgl_line_segment_2d<double> & lineSeg,
                                                           const WeakLineTrackingParameter & para,
                                                           vgl_line_2d<double> & line,
                                                           vgl_line_segment_2d<double> & refinedLineSeg);
    
    
};


#endif /* defined(__QuadCopter__vil_weak_line_tracking__) */
