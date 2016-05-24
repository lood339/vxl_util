//
//  vil_homo_warp.h
//  OnlineStereo
//
//  Created by jimmy on 8/12/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef __OnlineStereo__vil_homo_warp__
#define __OnlineStereo__vil_homo_warp__

#include <vil/vil_image_view.h>
#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_homg_point_2d.h>
#include <vgl/algo/vgl_h_matrix_2d.h>
#include <vil/vil_warp.h>
#include <vil/vil_bilin_interp.h>

class VilHomoWarp
{
private:
    //inverse H mapping from
    class VilInvHwarp
    {
    public:
        VilInvHwarp(const vgl_h_matrix_2d<double> &invH, const vgl_point_2d<double> & shiftPt)
        {
            invH_    = invH;
            shiftPt_ = shiftPt;
        }
        void operator()(const double ox, const double oy, double &ix, double &iy)
        {
            vgl_homg_point_2d<double> p = invH_((vgl_homg_point_2d<double>(ox, oy)));
            ix = p.x()/p.w() + shiftPt_.x();
            iy = p.y()/p.w() + shiftPt_.y();
        }
    private:
        vgl_h_matrix_2d<double> invH_;
        vgl_point_2d<double> shiftPt_;  //
        
    };
    
    
    //bilinear interperation
    template<class T>
    static double InterpolatorFunc( vil_image_view< T > const& view, double x, double y, unsigned p )
    {
        return vil_bilin_interp_safe( view, x, y, p);
    }
    
    // shift pixel position in destination image
    // for example, save pixels that map to x < 0
    template <class sType, class dType, class MapFunctor, class InterpFunctor>
    static void vil_warp_shift_origin(const vil_image_view<sType>& in,
                                      const vgl_point_2d<double> & outOriginPt,  // shift in the "out" image
                                      vil_image_view<dType>& out,
                                      MapFunctor mapper,
                                      InterpFunctor interp)
    {
        unsigned const out_w = out.ni();
        unsigned const out_h = out.nj();
        
        assert(out.nplanes() == in.nplanes());
        
        for (unsigned p = 0; p < out.nplanes(); ++p)
        {
            for (unsigned oy = 0; oy < out_h; ++oy)
            {
                for (unsigned ox = 0; ox < out_w; ++ox)
                {
                    // *** Find (ix, iy) from (ox,oy)
                    double ix, iy;
                    mapper(double(ox - outOriginPt.x()), double(oy - outOriginPt.y()), ix, iy);
                    out(ox, oy, p) = dType(interp(in, ix, iy, p));
                }
            }
        }
    }


public:
    
    template<class T>
    static bool homography_warp(const vil_image_view<T>& inImage,
                                const vgl_h_matrix_2d<double> &H,
                                int width, int height,
                                vil_image_view<T> &outImage)
    {
        outImage = vil_image_view<T>(width, height, inImage.nplanes());
        vgl_h_matrix_2d<double> invH = H.get_inverse();
        vil_warp(inImage, outImage, VilHomoWarp::VilInvHwarp(invH, vgl_point_2d<double>(0, 0)), VilHomoWarp::InterpolatorFunc<T>);
        return true;
    }
    
    // destOriginPt: (0, 0) pixel position in destImage. (w, 0) for
    template<class T>
    static bool homography_wap_shift_origin(const vil_image_view<T> & sourceImage,
                                                 const vgl_h_matrix_2d<double> &H,
                                                 const vgl_point_2d<double> & sourceOriginPt,
                                                 const vgl_point_2d<double> & destOriginPt,
                                                 int width, int height,
                                                 vil_image_view<T> & destImage)
    {
        destImage = vil_image_view<T>(width, height, sourceImage.nplanes());
        vgl_h_matrix_2d<double> invH = H.get_inverse();
        vil_warp_shift_origin(sourceImage, destOriginPt, destImage, VilHomoWarp::VilInvHwarp(invH, sourceOriginPt), VilHomoWarp::InterpolatorFunc<T>);
        return true;
    }
    
};



#endif /* defined(__OnlineStereo__vil_homo_warp__) */
