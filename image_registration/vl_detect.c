//
//  vl_detect.c
//  QuadCopter
//
//  Created by jimmy on 7/18/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include <vl/generic.h>
#include <vl/sift.h>
#include <vl/mathop.h>

void vl_sift_detect_custom(VlSiftFilt * f, double *locations, int nLoc)
{
    vl_sift_pix* dog   = f-> dog ;
    int          s_min = f-> s_min ;
    int          s_max = f-> s_max ;
    int          w     = f-> octave_width ;
    int          h     = f-> octave_height ;
    double       te    = f-> edge_thresh ;
    double       tp    = f-> peak_thresh ;
    
    int const    xo    = 1 ;      /* x-stride */
    int const    yo    = w ;      /* y-stride */
    int const    so    = w * h ;  /* s-stride */
    
    double       xper  = pow (2.0, f->o_cur) ;
    
    int x, y, s, i, ii, jj ;
    vl_sift_pix *pt, v ;
    VlSiftKeypoint *k ;
    
    /* clear current list */
    f-> nkeys = 0 ;
    
    /* compute difference of gaussian (DoG) */
    pt = f-> dog ;
    for (s = s_min ; s <= s_max - 1 ; ++s) {
        vl_sift_pix* src_a = vl_sift_get_octave (f, s    ) ;
        vl_sift_pix* src_b = vl_sift_get_octave (f, s + 1) ;
        vl_sift_pix* end_a = src_a + w * h ;
        while (src_a != end_a) {
            *pt++ = *src_b++ - *src_a++ ;
        }
    }
    
    /* -----------------------------------------------------------------
     *                                          Find local maxima of DoG
     * -------------------------------------------------------------- */
    
    /* start from dog [1,1,s_min+1] */
    pt  = dog + xo + yo + so ;

    // loop each location, and only check the first s
    for ( i = 0; i<nLoc; i++) {
        x = locations[i*2] / xper;
        y = locations[i*2 +1] / xper;
        for (s = s_min + 1; s <= s_max && s < s_min + 2; ++s) {
            if (f->nkeys >= f->keys_res) {
                f->keys_res += 500 ;
                if (f->keys) {
                    f->keys = vl_realloc (f->keys,
                                          f->keys_res *
                                          sizeof(VlSiftKeypoint)) ;
                } else {
                    f->keys = vl_malloc (f->keys_res *
                                         sizeof(VlSiftKeypoint)) ;
                }
            }
            
            k = f->keys + (f->nkeys ++) ;
            
            k-> ix = x ;
            k-> iy = y ;
            k-> is = s ;
        }        
    }
    
    /* -----------------------------------------------------------------
     *                                               Refine local maxima
     * -------------------------------------------------------------- */
    
    /* this pointer is used to write the keypoints back */
    k = f->keys ;
    
    for (i = 0 ; i < f->nkeys ; ++i) {
        
        int x = f-> keys [i] .ix ;
        int y = f-> keys [i] .iy ;
        int s = f-> keys [i]. is ;
        
        
        
        double Dx=0,Dy=0,Ds=0,Dxx=0,Dyy=0,Dss=0,Dxy=0,Dxs=0,Dys=0 ;
        double A [3*3], b [3] ;
        
        int dx = 0 ;
        int dy = 0 ;
        
        int iter, i, j ;
        
        for (iter = 0 ; iter < 5 ; ++iter) {
            
            x += dx ;
            y += dy ;
            
            pt = dog
            + xo * x
            + yo * y
            + so * (s - s_min) ;
            
            /** @brief Index GSS @internal */
#define at(dx,dy,ds) (*( pt + (dx)*xo + (dy)*yo + (ds)*so))
            
            /** @brief Index matrix A @internal */
#define Aat(i,j)     (A[(i)+(j)*3])
            
            /* compute the gradient */
            Dx = 0.5 * (at(+1,0,0) - at(-1,0,0)) ;
            Dy = 0.5 * (at(0,+1,0) - at(0,-1,0));
            Ds = 0.5 * (at(0,0,+1) - at(0,0,-1)) ;
            
            /* compute the Hessian */
            Dxx = (at(+1,0,0) + at(-1,0,0) - 2.0 * at(0,0,0)) ;
            Dyy = (at(0,+1,0) + at(0,-1,0) - 2.0 * at(0,0,0)) ;
            Dss = (at(0,0,+1) + at(0,0,-1) - 2.0 * at(0,0,0)) ;
            
            Dxy = 0.25 * ( at(+1,+1,0) + at(-1,-1,0) - at(-1,+1,0) - at(+1,-1,0) ) ;
            Dxs = 0.25 * ( at(+1,0,+1) + at(-1,0,-1) - at(-1,0,+1) - at(+1,0,-1) ) ;
            Dys = 0.25 * ( at(0,+1,+1) + at(0,-1,-1) - at(0,-1,+1) - at(0,+1,-1) ) ;
            
            /* solve linear system ....................................... */
            Aat(0,0) = Dxx ;
            Aat(1,1) = Dyy ;
            Aat(2,2) = Dss ;
            Aat(0,1) = Aat(1,0) = Dxy ;
            Aat(0,2) = Aat(2,0) = Dxs ;
            Aat(1,2) = Aat(2,1) = Dys ;
            
            b[0] = - Dx ;
            b[1] = - Dy ;
            b[2] = - Ds ;
            
            /* Gauss elimination */
            for(j = 0 ; j < 3 ; ++j) {
                double maxa    = 0 ;
                double maxabsa = 0 ;
                int    maxi    = -1 ;
                double tmp ;
                
                /* look for the maximally stable pivot */
                for (i = j ; i < 3 ; ++i) {
                    double a    = Aat (i,j) ;
                    double absa = vl_abs_d (a) ;
                    if (absa > maxabsa) {
                        maxa    = a ;
                        maxabsa = absa ;
                        maxi    = i ;
                    }
                }
                
                /* if singular give up */
                if (maxabsa < 1e-10f) {
                    b[0] = 0 ;
                    b[1] = 0 ;
                    b[2] = 0 ;
                    break ;
                }
                
                i = maxi ;
                
                /* swap j-th row with i-th row and normalize j-th row */
                for(jj = j ; jj < 3 ; ++jj) {
                    tmp = Aat(i,jj) ; Aat(i,jj) = Aat(j,jj) ; Aat(j,jj) = tmp ;
                    Aat(j,jj) /= maxa ;
                }
                tmp = b[j] ; b[j] = b[i] ; b[i] = tmp ;
                b[j] /= maxa ;
                
                /* elimination */
                for (ii = j+1 ; ii < 3 ; ++ii) {
                    double x = Aat(ii,j) ;
                    for (jj = j ; jj < 3 ; ++jj) {
                        Aat(ii,jj) -= x * Aat(j,jj) ;
                    }
                    b[ii] -= x * b[j] ;
                }
            }
            
            /* backward substitution */
            for (i = 2 ; i > 0 ; --i) {
                double x = b[i] ;
                for (ii = i-1 ; ii >= 0 ; --ii) {
                    b[ii] -= x * Aat(ii,i) ;
                }
            }
            
            /* .......................................................... */
            /* If the translation of the keypoint is big, move the keypoint
             * and re-iterate the computation. Otherwise we are all set.
             */
            
            dx= ((b[0] >  0.6 && x < w - 2) ?  1 : 0)
            + ((b[0] < -0.6 && x > 1    ) ? -1 : 0) ;
            
            dy= ((b[1] >  0.6 && y < h - 2) ?  1 : 0)
            + ((b[1] < -0.6 && y > 1    ) ? -1 : 0) ;
            
            if (dx == 0 && dy == 0) break ;
        }
        
        /* check threshold and other conditions */
        {
            double val   = at(0,0,0)
            + 0.5 * (Dx * b[0] + Dy * b[1] + Ds * b[2]) ;
            double score = (Dxx+Dyy)*(Dxx+Dyy) / (Dxx*Dyy - Dxy*Dxy) ;
            double xn = x + b[0] ;
            double yn = y + b[1] ;
            double sn = s + b[2] ;
            
            /*
            vl_bool good =
            vl_abs_d (val)  > tp                  &&
            score           < (te+1)*(te+1)/te    &&
            score           >= 0                  &&
            vl_abs_d (b[0]) <  1.5                &&
            vl_abs_d (b[1]) <  1.5                &&
            vl_abs_d (b[2]) <  1.5                &&
            xn              >= 0                  &&
            xn              <= w - 1              &&
            yn              >= 0                  &&
            yn              <= h - 1              &&
            sn              >= s_min              &&
            sn              <= s_max ;
             */
            
            vl_bool good =
            xn              >= 0                  &&
            xn              <= w - 1              &&
            yn              >= 0                  &&
            yn              <= h - 1              &&
            sn              >= s_min              &&
            sn              <= s_max ;
            
            if (good) // do not check the quality
            {
                k-> o     = f->o_cur ;
                k-> ix    = x ;
                k-> iy    = y ;
                k-> is    = s ;
                k-> s     = sn ;
                k-> x     = xn * xper ;
                k-> y     = yn * xper ;
                k-> sigma = f->sigma0 * pow (2.0, sn/f->S) * xper ;
                ++ k ;
            }        
            
        } /* done checking */
    } /* next keypoint to refine */
    
    /* update keypoint count */
    f-> nkeys = (int)(k - f->keys) ;
}

/*
 v = *pt ;
 
 #define CHECK_NEIGHBORS(CMP,SGN)                    \
 ( v CMP ## = SGN 0.8 * tp &&                \
 v CMP *(pt + xo) &&                       \
 v CMP *(pt - xo) &&                       \
 v CMP *(pt + so) &&                       \
 v CMP *(pt - so) &&                       \
 v CMP *(pt + yo) &&                       \
 v CMP *(pt - yo) &&                       \
 \
 v CMP *(pt + yo + xo) &&                  \
 v CMP *(pt + yo - xo) &&                  \
 v CMP *(pt - yo + xo) &&                  \
 v CMP *(pt - yo - xo) &&                  \
 \
 v CMP *(pt + xo      + so) &&             \
 v CMP *(pt - xo      + so) &&             \
 v CMP *(pt + yo      + so) &&             \
 v CMP *(pt - yo      + so) &&             \
 v CMP *(pt + yo + xo + so) &&             \
 v CMP *(pt + yo - xo + so) &&             \
 v CMP *(pt - yo + xo + so) &&             \
 v CMP *(pt - yo - xo + so) &&             \
 \
 v CMP *(pt + xo      - so) &&             \
 v CMP *(pt - xo      - so) &&             \
 v CMP *(pt + yo      - so) &&             \
 v CMP *(pt - yo      - so) &&             \
 v CMP *(pt + yo + xo - so) &&             \
 v CMP *(pt + yo - xo - so) &&             \
 v CMP *(pt - yo + xo - so) &&             \
 v CMP *(pt - yo - xo - so) )
 
 if (CHECK_NEIGHBORS(>,+) ||
 CHECK_NEIGHBORS(<,-) )
 */



