//
//  vxl_ELSD.cpp
//  OnlineStereo
//
//  Created by jimmy on 8/26/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#include <vil/vil_convert.h>
#include "vxl_ELSD.h"
#include "elsd.h"
#include "process_line.h"
#include "process_curve.h"
#include "valid_curve.h"
#include <vgl/vgl_fit_ellipse_2d.h>
#include <vgl/vgl_distance.h>

// for test
#include "vil_plus.h"



/*----------------------------------------------------------------------------*/

static void ellipeDetection(image_double image, double rho,double prec,double p,
                            double eps,int smooth,
                            vcl_vector<vnl_vector_fixed<double, 5> > & lines,
                            vcl_vector<vnl_vector_fixed<double, 5> > & circles,
                            vcl_vector<vnl_vector_fixed<double, 5> > & ellipses,
                            vcl_vector<vcl_vector<vgl_point_2d<double> > > & linePoints,
                            vcl_vector<vcl_vector<vgl_point_2d<double> > > & ellipsePoints)
{
    image_double angles,gradx,grady,grad,imgauss;
    image_char used;
    void *mem_p;
    int n_bins = 1024;
    double max_grad = 255.0;
    struct coorlist *list_p;
    struct point *reg, *regl;
    struct point3 *regc, *rege;
    struct rect rec;
    int reg_size = 0,regp_size[3];
    unsigned int xsize,ysize; /* image size */
    int i;
    int min_size[3];
 //   FILE *svg;
    double logNT[3]; /* number of tests for the 3 primitive types */
    double nfa[3]; /* NFA value using the discrete formulation for the three primitive types */
    double parame[5], paramc[5]; /* ellipse/circle parameters */
    double lin[5];
    double density_th = 0.7;
    double mlog10eps = - log10(eps);
    double reg_angle;
    int pext[8];
    unsigned int xsz0,ysz0;
    xsz0 = image->xsize;
    ysz0 = image->ysize;
    
    /* perform gaussian smoothing and subsampling */
    if (smooth)
    {
       // imgauss = gaussian_sampler( image, 0.8, 0.6);
        imgauss = gaussian_sampler( image, 1.0, 0.6); // sensitive to Gaussian sampler
        /* compute gradient magnitude and orientation  */
        angles = ll_angle(imgauss,rho,&list_p,&mem_p,&gradx,&grady,&grad,n_bins,max_grad);
    }
    else{
        angles = ll_angle(image,rho,&list_p,&mem_p,&gradx,&grady,&grad,n_bins,max_grad);
    }
    
    xsize = angles->xsize;
    ysize = angles->ysize;
    
    /* display detection result */
  //  svg = init_svg(strcat(fstr,".svg"),xsz0,ysz0);
    
    /* number of tests for elliptical arcs */
    logNT[2] = 4.0 *(log10((double)xsize)+log10((double)ysize)) + log10(9.0) + log10(3.0); /* N^8 */
    /* number of tests for circular arcs */
    logNT[1] = 3.0 *(log10((double)xsize)+log10((double)ysize)) + log10(9.0) + log10(3.0); /* N^6 */
    /* number of tests for line-segments */
    logNT[0] = 5.0 *(log10((double)xsize)+log10((double)ysize))/2.0 + log10(11) + log10(3.0); /* N^5 */
    
    /* thresholds from which an elliptical/circular/linear arc could be meaningful */
    min_size[2] =(int)((-logNT[2]+log10(eps))/log10(p));
    min_size[1] =(int)((-logNT[1]+log10(eps))/log10(p));
    min_size[0] =(int)((-logNT[0]+log10(eps))/log10(p));
    
    /* file to write coordinates of detected ellipses */
    FILE *fe = fopen("ellipses.txt","wt");
    
    /* allocate memory for region lists */
    reg = (struct point *) calloc(xsize * ysize, sizeof(struct point));
    regl = (struct point *) calloc(xsize * ysize, sizeof(struct point));
    regc = (struct point3 *) calloc(xsize * ysize, sizeof(struct point3));
    rege = (struct point3 *) calloc(xsize * ysize, sizeof(struct point3));
    used = new_image_char_ini(xsize,ysize,NOTUSED);
    
    /* init temporary buffers */
    gBufferDouble = (double*)malloc(sizeof(double));
    gBufferInt    = (int*)malloc(sizeof(int));
    
    gSizeBufferDouble = 1;
    gSizeBufferInt    = 1;
    
    /* begin primitive detection */
    for(;list_p; list_p = list_p->next)
    {
        reg_size = 0;
        if(used->data[list_p->y*used->xsize+list_p->x]==NOTUSED &&
           angles->data[list_p->y*angles->xsize+list_p->x] != NOTDEF)
        {
            /* init some variables */
            for (i=0;i<5;i++)
            {
                parame[i] = 0.0; paramc[i] = 0.0;
            }
            nfa[2] = nfa[1] = nfa[0] = mlog10eps;
            reg_size = 1;regp_size[0] = regp_size[1] = regp_size[2] = 0;
            region_grow(list_p->x, list_p->y, angles, reg, &reg_size, &reg_angle, used, prec);
            
            
           
            /*-------- FIT A LINEAR SEGMENT AND VERIFY IF VALID ------------------- */
            valid_line(reg,&reg_size,reg_angle,prec,p,&rec,lin,grad,gradx,grady,
                       used,angles,density_th,logNT[0],mlog10eps,&nfa[0]);
            regp_size[0] = reg_size;
            
            
            for (i=0;i<regp_size[0];i++) {regl[i].x = reg[i].x; regl[i].y = reg[i].y; }
            
            if (reg_size>2)
            {
                
                /*-------- FIT CONVEX SHAPES (CIRCLE/ELLIPSE) AND VERIFY IF VALID -------- */
                valid_curve(reg,&reg_size,prec,p,angles,used,grad,gradx,grady,paramc,parame,
                            rec,logNT,mlog10eps,density_th,min_size,nfa,pext,regc,rege,regp_size);
                
                /* ------ DECIDE IF LINEAR SEGMENT OR CIRCLE OR ELLIPSE BY COMPARING THEIR NFAs -------*/
                if(nfa[2]>mlog10eps && nfa[2]>nfa[0] && nfa[2]>nfa[1] && regp_size[2]>min_size[2]) /* ellipse */
                {
                  
                    /*if (smooth)
                     fprintf(fe,"%f %f %f %f %f \n",parame[0]*1.25,parame[1]*1.25,parame[2]*1.25,parame[3]*1.25,parame[4]);
                     else
                     fprintf(fe,"%f %f %f %f %f \n",parame[0],parame[1],parame[2],parame[3],parame[4]);*/
               //     write_svg_ellipse(fe,svg,parame,pext,smooth);
                    
                    
                    for (i=0;i<regp_size[0];i++){
                        used->data[regl[i].y*used->xsize+regl[i].x] = NOTUSED;
                    }
                    for (i=0;i<regp_size[1];i++){
                        used->data[regc[i].y*used->xsize+regc[i].x] = NOTUSED;
                    }
                    vcl_vector<vgl_point_2d<double> > pts;
                    for (i=0;i<regp_size[2];i++){
                        if (rege[i].z == USEDELL)
                        {
                            used->data[rege[i].y*used->xsize+rege[i].x] = USED;
                            pts.push_back(vgl_point_2d<double>(rege[i].x, rege[i].y));
                            
                        }
                        else{
                            used->data[rege[i].y*used->xsize+rege[i].x] = USEDELLNA;
                        }
                    }
                    // filter too small ellipse
                    if (pts.size() > 20) {
                        vnl_vector_fixed<double, 5> curEllipse(parame);
                        ellipses.push_back(curEllipse);
                       ellipsePoints.push_back(pts);
                    }
                }
                else if(nfa[1]>mlog10eps && nfa[1]>nfa[0] && nfa[1]>nfa[2] && regp_size[1]>min_size[1]) /* circle */
                {
                    /*if (smooth)
                     fprintf(fe,"%f %f %f %f %f \n",paramc[0]*1.25,paramc[1]*1.25,paramc[2]*1.25,paramc[3]*1.25,paramc[4]);
                     else
                     fprintf(fe,"%f %f %f %f %f \n",paramc[0],paramc[1],paramc[2],paramc[3],paramc[4]);*/
                //    write_svg_circle(fe,svg,paramc,pext,smooth);
                    vnl_vector_fixed<double, 5> curCircle(paramc);
                    circles.push_back(curCircle);
                    
                    for (i=0;i<regp_size[0];i++){
                        used->data[regl[i].y*used->xsize+regl[i].x] = NOTUSED;
                    }
                    for (i=0;i<regp_size[2];i++){
                        used->data[rege[i].y*used->xsize+rege[i].x] = NOTUSED;
                    }
                    for (i=0;i<regp_size[1];i++){
                        if (regc[i].z == USEDCIRC){
                            used->data[regc[i].y*used->xsize+regc[i].x] = USED;
                        }
                        else{
                            used->data[regc[i].y*used->xsize+regc[i].x] = USEDCIRCNA;
                        }
                    }
                }
                else if(nfa[0]>mlog10eps && regp_size[0]>min_size[0] && nfa[0]>nfa[1] && nfa[0]>nfa[2]) /* line */
                {
              //      write_svg_line(svg,lin,smooth);
                    vnl_vector_fixed<double, 5> curLine(lin);
                    lines.push_back(curLine);
                    vcl_vector<vgl_point_2d<double> > pts;
		            for (i=0;i<regp_size[1];i++){
                        used->data[regc[i].y*used->xsize+regc[i].x] = NOTUSED;
                    }
                    for (i=0;i<regp_size[2];i++){
                        used->data[rege[i].y*used->xsize+rege[i].x] = NOTUSED;
                    }
                    for (i=0;i<regp_size[0];i++){
                        used->data[regl[i].y*used->xsize+regl[i].x] = USED;
                        pts.push_back(vgl_point_2d<double>(regl[i].x, regl[i].y));
                    }
                    linePoints.push_back(pts);
                }
                else /* no feature */
                {
                    for (i=0;i<regp_size[1];i++){
                        used->data[regc[i].y*used->xsize+regc[i].x] = NOTUSED;
                    }
                    for (i=0;i<regp_size[2];i++){
                        used->data[rege[i].y*used->xsize+rege[i].x] = NOTUSED;
                    }
                    for (i=0;i<regp_size[0];i++){
                        used->data[regl[i].y*used->xsize+regl[i].x] = NOTUSED;
                    }
                }
            }
        }/* IF USED */
    }/* FOR LIST */    
    
    if (smooth)
    {
        free_image_double(imgauss);
    }
    
    free_image_double(gradx);
    free_image_double(grady);
    free_image_double(grad);
    free_image_double(angles);
    free_image_char(used);
    free(reg);
    free(regl);
    free(regc);
    free(rege);
    free(gBufferDouble);
    free(gBufferInt);
    free(mem_p);
    fclose(fe);
  //  fclose_svg(svg);
}

void VxlELSD::detectLineCirleEllipse(const vil_image_view<vxl_byte> & image,
                                     const ELSDLineEllipseParameter & para,
                                     ELSDResult & elsd)
{
    // fixed parameter
    double quant = 2.0;       /* Bound to the quantization error on the
                               gradient norm.                                */
    double ang_th = 22.5;     /* Gradient angle tolerance in degrees.           */
    double p = ang_th/180.0;
    double prec = M_PI*ang_th/180.0; /* radian precision */
    double rho = quant/sin(prec);
    double eps = 1;
    int smooth = 1;   // 1, sub sample image
    
    vil_image_view<vxl_byte> gray;
    if (image.nplanes() == 3) {
        vil_convert_planes_to_grey(image, gray);
    }
    else
    {
        gray = image;
    }
    
    const int w = gray.ni();
    const int h = gray.nj();
    image_double d_image = new_image_double(w, h);
    for (int j = 0; j<h; j++) {
        for (int i = 0; i<w; i++) {
            d_image->data[i + j * w] = gray(i, j);
        }
    }
    
    vcl_vector<vnl_vector_fixed<double, 5> > lines_params;
    vcl_vector<vnl_vector_fixed<double, 5> > circles_params;
    vcl_vector<vnl_vector_fixed<double, 5> > ellipses_params;
    vcl_vector<vcl_vector<vgl_point_2d<double> > > linePoints;
    vcl_vector<vcl_vector<vgl_point_2d<double> > > ellipsePoints;
    ellipeDetection(d_image, rho, prec, p, eps, smooth, lines_params, circles_params, ellipses_params, linePoints, ellipsePoints);
    
    printf("ELSD find %lu lines, %lu circles, %lu ellipses\n", lines_params.size(), circles_params.size(), ellipses_params.size());
    assert(ellipses_params.size() == ellipsePoints.size());
    
    // collect ellipses
    for (int i = 0; i<ellipses_params.size(); i++) {
        if (ellipsePoints[i].size() > para.ellipse_min_pixel_) {
            double x = ellipses_params[i][0];
            double y = ellipses_params[i][1];
            double major = ellipses_params[i][2];
            double minor = ellipses_params[i][3];
            double orientation = ellipses_params[i][4];
            vgl_ellipse_2d<double> ellipse(vgl_point_2d<double>(x, y), major, minor, orientation);
            elsd.ellipses_.push_back(ellipse);
            elsd.ellipse_points_.push_back(ellipsePoints[i]);
        }
    }
    
    // collect line segment
    for (int i = 0; i<lines_params.size(); i++) {
        double x1 = lines_params[i][0];
        double y1 = lines_params[i][1];
        double x2 = lines_params[i][2];
        double y2 = lines_params[i][3];
        vgl_point_2d<double> p1 = vgl_point_2d<double>(x1, y1);
        vgl_point_2d<double> p2 = vgl_point_2d<double>(x2, y2);
        double dis = vgl_distance(p1, p2);
        if (dis > para.line_min_length_) {
            elsd.lines_.push_back(vgl_line_segment_2d<double>(p1, p2));
            elsd.line_points_.push_back(linePoints[i]);
        }
    }
    
    free_image_double(d_image);
    printf("find %lu long lines, %lu large ellipses\n", elsd.lines_.size(), elsd.ellipses_.size());
}

bool VxlELSD::detect_largest_ellipse(const vil_image_view<vxl_byte> & image,
                                     vgl_ellipse_2d<double> & ellipse,
                                     vcl_vector<vgl_point_2d<double> > & ellipse_points)
{
    ELSDLineEllipseParameter para;
    ELSDResult elsd;
    
       
    VxlELSD::detectLineCirleEllipse(image, para, elsd);
    if (elsd.ellipses_.size() == 0) {
        return false;
    }
    int max_num = 0;
    for (int i = 0; i<elsd.ellipse_points_.size(); i++) {
        if (elsd.ellipse_points_[i].size() > max_num) {
            max_num = (int)elsd.ellipse_points_[i].size();
            ellipse = elsd.ellipses_[i];
            ellipse_points = elsd.ellipse_points_[i];
        }
    }
    return true;
}





