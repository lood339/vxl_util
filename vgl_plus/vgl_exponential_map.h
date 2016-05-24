//
//  vgl_exponential_map.h
//  OnlineStereo
//
//  Created by jimmy on 12/10/14.
//  Copyright (c) 2014 Nowhere Planet. All rights reserved.
//

// implement "Practical parameterization of rotations using the exponential map" by F.Sebastian Grassia (CMU)

#ifndef __OnlineStereo__vgl_exponential_map__
#define __OnlineStereo__vgl_exponential_map__

#include <vgl/algo/vgl_rotation_3d.h>
#include <vnl/vnl_quaternion.h>
#include <vnl/vnl_vector_fixed.h>

template <class T>
class vgl_exponential_map
{
    vnl_quaternion<T> q_;
    vnl_vector_fixed<T, 3> exp_map_;      // exponential map vector
    
    static const double MIN_ANGLE_;
    static const double CUTOFF_ANGLE_;
    
public:
    vgl_exponential_map():q_(0, 0, 0, 1)
    {
        exp_map_ = quaternion_to_exponential_map(q_);
    }
    vgl_exponential_map(const double x, const double y, double z):exp_map_(x, y, z)
    {
        q_ = exponential_map_to_quaternion(exp_map_);
    }
    
    vgl_exponential_map(const vgl_rotation_3d<double> & r)
    {
        q_ = r.as_quaternion();
        exp_map_ = quaternion_to_exponential_map(q_);
    }
    ~vgl_exponential_map(){};
    
    
    vnl_vector_fixed<T, 3> as_exp_map()
    {
        return exp_map_;
    }
    
    vnl_matrix_fixed<double, 3, 3> as_matrix()
    {
        return q_.rotation_matrix_transpose().transpose();
    }
    
    vnl_quaternion<double> as_quaternion()
    {
        return q_;
    }
    
private:
    // reparam = true when derivative
    static vnl_quaternion<T> exponential_map_to_quaternion(const vnl_vector_fixed<T, 3> & exp_map, bool reparam = false)
    {
        int rep = 0;   // not return at the moment
        double cosp = 0.0;
        double sinp = 0.0;
        double theta = 0.0;
        
        double qw = 0.0;
        
        vnl_vector_fixed<T, 3> v = exp_map;
        if (reparam)
        {
            rep = Check_Parameterization(v, theta);
        }
        else
        {
            theta = v.magnitude();
        }
        
        cosp = cos(0.5*theta);
        sinp = sin(0.5*theta);
  
        qw = cosp;
        vnl_vector_fixed<T, 3> q_xyz;
        if (theta < vgl_exponential_map<T>::MIN_ANGLE_)
        {
            q_xyz = (0.5 - theta*theta/48.0) * v; // Taylor Series for sinc
        }
        else
        {
            q_xyz = sinp/theta * v;
        }
        
        vnl_quaternion<T> q(q_xyz[0], q_xyz[1], q_xyz[2], qw);
        return q;
    }
    
    static vnl_vector_fixed<T, 3> quaternion_to_exponential_map(const vnl_quaternion<double> & q)
    {
        double theta = 2.0 * acos(q[3]);
        double scale = 0.0;
        if (theta < vgl_exponential_map<T>::MIN_ANGLE_) {
            scale = 2.0 * q[3];
        }
        else
        {
            scale = 2*theta*q[3]/sin(theta);
        }
        printf("theta is %f\n", theta);
        
        vnl_vector_fixed<T, 3> exp_map;
        exp_map[0] = q[0] * scale;
        exp_map[1] = q[1] * scale;
        exp_map[2] = q[2] * scale;
        return exp_map;
    }
    
    
    // 'Check_Parameterization' To escape the vanishing derivatives at
    //  shells of 2PI rotations, we reparameterize to a rotation of (2PI -
    //  theta) about the opposite axis when we get too close to 2PI
    static int Check_Parameterization(vnl_vector_fixed<T, 3> & v, double & theta)
    {
        int rep = 0;
        theta = v.magnitude();
        
        if (theta > vgl_exponential_map<T>::CUTOFF_ANGLE_)
        {
            
            double scl = theta;
            if (theta > 2*M_PI)
            {
                //first get theta into range 0..2PI
                theta = fmod(theta, 2*M_PI);
                scl = theta/scl;
                v *= scl;
                rep = 1;
            }
           
            if (theta > vgl_exponential_map<T>::CUTOFF_ANGLE_)
            {
                scl = theta;
                theta = 2*M_PI - theta;
                scl = 1.0 - 2*M_PI/scl;
                v *= scl;
                rep = 1;
            }
        }
        return rep;
    }

};

template<class T>
const double vgl_exponential_map<T>::MIN_ANGLE_ =  1e-7;

template<class T>
const double vgl_exponential_map<T>::CUTOFF_ANGLE_ = M_PI;

//#include "vgl_exponential_map.cpp"


#endif /* defined(__OnlineStereo__vgl_exponential_map__) */
