/* common definitions for the sgsmooth plugin */
#ifndef _SGSMOOTH_PLUGIN_H
#define _SGSMOOTH_PLUGIN_H

#ifdef __cplusplus
extern "C" 
{
#endif

double *calc_sgsmooth(const int ndat, double *input,
                             const int window, const int order);

double *calc_sgsderiv(const int ndat, double *input,
                             const int window, const int order, 
                             const double delta);

//void sgs_error(const char *errmsg);
    

#ifdef __cplusplus
}
#endif

#endif
