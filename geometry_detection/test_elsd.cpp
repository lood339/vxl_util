//
//  test_elsd.cpp
//  OnlineStereo
//
//  Created by jimmy on 8/26/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#if 0

#include "elsd.h"
#include "vil_plus.h"

int main(int argc, char ** argv)
{
    if (argc<2) error("use : ./elsd image_name.jpg");
    double quant = 2.0;       /* Bound to the quantization error on the
                               gradient norm.                                */
    double ang_th = 22.5;     /* Gradient angle tolerance in degrees.           */
    double p = ang_th/180.0;
    double prec = M_PI*ang_th/180.0; /* radian precision */
    double rho = quant/sin(prec);
    double eps = 1; //atof(argv[2]);
    int smooth = 1; //atoi(argv[3]);
    int ell_count = 0, line_count = 0, circ_count = 0;
    image_double image;
    
    vil_image_view<vxl_byte> org_image = vil_load(argv[1]);
    vil_image_view<vxl_byte> gray = VilPlus::vil_to_gray(org_image);
    int w = gray.ni();
    int h = gray.nj();
   // image = read_pgm_image_double(argv[1]);
    image = new_image_double(w, h);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            image->data[i + j * w] = gray(i, j);
        }
    }
    
    
   EllipseDetection(image, rho, prec, p, eps, smooth, &ell_count, &circ_count,
                     &line_count,argv[1]);
    
    EllipseDetection(image, rho, prec, p, eps, smooth, &ell_count, &circ_count,
                     &line_count,argv[1]);
    printf("%s\n", argv[1]);
    printf("%d elliptical arcs, %d circular arcs, %d line segments\n",
           ell_count, circ_count, line_count);
    return 0;
}
#endif

