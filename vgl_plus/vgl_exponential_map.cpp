//
//  vgl_exponential_map.cpp
//  OnlineStereo
//
//  Created by jimmy on 12/10/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

#include "vgl_exponential_map.h"

/*
vgl_exponential_map::vgl_exponential_map()
{
    
}
vgl_exponential_map::vgl_exponential_map(const double x, const double y, double z)
{
    
}

vgl_exponential_map::~vgl_exponential_map()
{
    
}
 */



/* vector indices */
#define X 0
#define Y 1
#define Z 2
#define W 3

typedef double Quat[4];

/* crossover point to Taylor Series approximation.  Figuring 16
 * decimal digits of precision for doubles, the Taylor approximation
 * should be indistinguishable (to machine precision) for angles
 * of magnitude less than 1e-4. To be conservative, we add on three
 * additional orders of magnitude.  */
const double MIN_ANGLE = 1e-7;

/* Angle beyond which we perform dynamic reparameterization of a 3 DOF EM */
const double CUTOFF_ANGLE = M_PI;

double V3Magnitude(const double vec[3])
{
    return sqrt(vec[X]*vec[X] + vec[Y]*vec[Y] + vec[Z]*vec[Z]);
}

void V3Scale(const double v1[3], const double s1, double prod[3])
{
    prod[X] = v1[X] * s1;
    prod[Y] = v1[Y] * s1;
    prod[Z] = v1[Z] * s1;
}

/* -----------------------------------------------------------------
 * 'Check_Parameterization' To escape the vanishing derivatives at
 * shells of 2PI rotations, we reparameterize to a rotation of (2PI -
 * theta) about the opposite axis when we get too close to 2PI
 * -----------------------------------------------------------------*/
int Check_Parameterization(double v[3], double *theta)
{
    int     rep = 0;
    *theta = V3Magnitude(v);
    
    if (*theta > CUTOFF_ANGLE){
        double scl = *theta;
        if (*theta > 2*M_PI){	/* first get theta into range 0..2PI */
            *theta = fmod(*theta, 2*M_PI);
            scl = *theta/scl;
            V3Scale(v, scl, v);
            rep = 1;
        }
        if (*theta > CUTOFF_ANGLE){
            scl = *theta;
            *theta = 2*M_PI - *theta;
            scl = 1.0 - 2*M_PI/scl;
            V3Scale(v, scl, v);
            rep = 1;
        }
    }
    
    return rep;
}




/* -----------------------------------------------------------------
 * 'EM_To_Q' Convert a 3 DOF EM vector 'v' into its corresponding
 * quaternion 'q'. If 'reparam' is non-zero, perform dynamic
 * reparameterization, if necessary, storing the reparameterized EM in
 * 'v' and returning 1.  Returns 0 if no reparameterization was
 * performed.
 * -----------------------------------------------------------------*/

int EM_To_Q(double v[3], Quat q, int reparam)
{
    int      rep=0;
    double   cosp, sinp, theta;
    
    
    if (reparam)
        rep = Check_Parameterization(v, &theta);
    else
        theta = V3Magnitude(v);
    
    cosp = cos(.5*theta);
    sinp = sin(.5*theta);
    
    q[W] = cosp;
    if (theta < MIN_ANGLE)
        V3Scale(v, .5 - theta*theta/48.0, q);	/* Taylor Series for sinc */
    else
        V3Scale(v, sinp/theta, q);
    
    return rep;
}





