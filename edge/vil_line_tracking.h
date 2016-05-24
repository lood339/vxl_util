//
//  vil_line_tracking.h
//  OnlineStereo
//
//  Created by jimmy on 3/6/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vil_line_tracking__
#define __OnlineStereo__vil_line_tracking__

// tracking roughly parallel lines in image

#include <vcl_vector.h>
#include <vil/vil_image_view.h>
#include <vgl/vgl_line_segment_2d.h>
#include <vnl/vnl_vector_fixed.h>

struct LinesegmntTrackingParameter
{
    double mag_threshold_1_;   // mininum threshold
    double mag_threshold_2_;   // maximum threshold
    double lambda_;            // times of average of non-edge pixel gradient
    double pixel_num_ratio_;   // minimum ratio threshold of how many pixels should be found
    int line_width_;           // potential width of the line
    double parallel_line_direction_threshold_;   // cos (angle)
    double online_pixel_ratio_;                   // check how many pixel should be on line
    
    
    int scan_line_canny_direction_;
    int edge_neighbor_size_;        // check if a pixel is in the center of the edge or on the edge
    
    double avg_end_point_distance_;
    
    LinesegmntTrackingParameter()
    {
        mag_threshold_1_ = 0.05;
        mag_threshold_2_ = 0.2;
        lambda_ = 2.0;
        pixel_num_ratio_ = 0.1;
        
        line_width_ = 5;
        parallel_line_direction_threshold_ = cos(30.0/180*3.14);
        online_pixel_ratio_ = 0.5;
        scan_line_canny_direction_ = 0;
        
        edge_neighbor_size_ = 3;        
        avg_end_point_distance_ = 15.0;
    }
};

class VilWeakLineTracking;
class VilLineTracking
{
    friend VilWeakLineTracking;
public:
    // check if detected line segment is on the (center) of the edge
    // by counting the high-gradient pixel number, in edge_neighbor_size area
    static bool isLineSegmentOnEdge(const vil_image_view<vxl_byte> & image, // for test only
                                    const vil_image_view<double> & magnitude,
                                    const vil_image_view<double> & grad_i,
                                    const vil_image_view<double> & grad_j,
                                    const vgl_line_segment_2d<double> & lineSeg,
                                    const LinesegmntTrackingParameter & para, double edgePixelRatioThreshold = 0.5);
    // ratio of pixel on the edge
    static double onEdgePixelRatio(const vil_image_view<double> & magnitude,
                                   const vil_image_view<double> & grad_i,
                                   const vil_image_view<double> & grad_j,
                                   const vgl_line_segment_2d<double> & lineSeg,
                                   const double min_magnitude,
                                   const int edge_neighbor_size);
    
    // refine init line segment by search in two direction. search area can be large
    static bool refineLineSegment(const vil_image_view<vxl_byte> & image, // for test only
                                  const vil_image_view<double> & mag, const vgl_line_segment_2d<double> & initLineSeg,
                                  const LinesegmntTrackingParameter & para, vgl_line_segment_2d<double> & refinedLineSeg);
    
    // check both edge magnitude and direction
    static bool refineLineSegment(const vil_image_view<vxl_byte> & image,
                                  const vil_image_view<double> & mag,
                                  const vil_image_view<double> & grad_i,
                                  const vil_image_view<double> & grad_j,
                                  const vgl_line_segment_2d<double> & initLineSeg,
                                  const LinesegmntTrackingParameter & para,
                                  vgl_line_segment_2d<double> & refinedLineSeg);
    
    // method of "automatic rectification of long image sequences"
    // verify by gradient magnitude and direction
    // this method only for test/comparison purpose
    static bool refineLineSegmentByMagnitudePeak(const vil_image_view<vxl_byte> & image, // for test only
                                                 const vil_image_view<double> & mag,
                                                 const vil_image_view<double> & grad_i,
                                                 const vil_image_view<double> & grad_j,
                                                 const vgl_line_segment_2d<double> & initLineSeg,
                                                 const LinesegmntTrackingParameter & para,
                                                 vgl_line_segment_2d<double> & refinedLineSeg,
                                                 vgl_line_2d<double> & fefinedLine);
    
    struct LineFilterParameter
    {
        unsigned int edge_width_;
        double filter_output_threshold_;
        
        LineFilterParameter()
        {
            edge_width_ = 1;
            filter_output_threshold_ = 0.02;
        }
        
        
    };
    // method of " real-time camera tracking using sports pitch markings"
    // verify line edge pixels by a line fitler
    static bool refineLineSegmentByLineFilter(const vil_image_view<vxl_byte> & image, // for test only
                                              const vil_image_view<double> & magnitude,
                                              const vil_image_view<double> & grad_i,
                                              const vil_image_view<double> & grad_j,
                                              
                                              const vil_image_view<double> & blueComponent,  // not used
                                              const vil_image_view<bool> & isGrassMask,      // not used
                                              
                                              const LinesegmntTrackingParameter & lineTrackingPara,
                                              const LineFilterParameter & filterOutputPara,
                                              
                                              const vgl_line_segment_2d<double> & initLineSeg,
                                              
                                              vgl_line_segment_2d<double> & refinedLineSeg,
                                              vgl_line_2d<double> & fefinedLine);
    
    /*
    // assume lineSeg is very close to real line in the image.
    // assume the line is wider in the image
    static bool trackingLineFromSegment(const vil_image_view<vxl_byte> & image, // for test only
                                        const vil_image_view<double> & magnitude,
                                        const vil_image_view<double> & grad_i,
                                        const vil_image_view<double> & grad_j,
                                        const vgl_line_segment_2d<double> & lineSeg,
                                        const LinesegmntTrackingParameter & para, vgl_line_2d<double> & line, vgl_line_segment_2d<double> & cenerLineSeg);
     */
    // direction: in para
    static bool trackingLinePixelInOneDirection(const vil_image_view<double> & mag, const vgl_line_segment_2d<double> & initLineSeg,
                                                const LinesegmntTrackingParameter & para, vcl_vector<vgl_point_2d<double> > & edgePixels);
    // ratio of magnitude to cumulative average magnitude (RMCAM)
    static bool trackingLinePixelInOneDirectionByPeakRMCAM(const vil_image_view<double> & mag, const vgl_line_segment_2d<double> & initLineSeg,
                                                           const LinesegmntTrackingParameter & para, vcl_vector<vgl_point_2d<double> > & edgePixels);
    
    // by average gradient
    static bool trackingLinePixel(const vil_image_view<double> & mag, const vgl_line_segment_2d<double> & initLineSeg,
                                  const LinesegmntTrackingParameter & para, vcl_vector<vgl_point_2d<double> > & edge1Pixels,
                                  vcl_vector<vgl_point_2d<double> > & edge2Pixels);
    
    
private:
    // 0, 1, 2, 3 of canny drection
    //       2
    //    3     1
    //       0
    static int cannyDirection(double dx, double dy);
};





#endif /* defined(__OnlineStereo__vil_line_tracking__) */
