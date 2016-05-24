//
//  vgl_vnl_operator.h
//  QuadCopter
//
//  Created by jimmy on 6/26/15.
//  Copyright (c) 2015 Nowhere Planet. All rights reserved.
//

#ifndef QuadCopter_vgl_vnl_operator_h
#define QuadCopter_vgl_vnl_operator_h

#include <vgl/vgl_point_2d.h>
#include <vgl/vgl_point_3d.h>
#include <vnl/vnl_matrix_fixed.h>

template <class T>
vgl_point_3d<T> operator * (const vnl_matrix_fixed<T, 3, 3> &m, const vgl_point_3d<T> &p)
{
    vnl_vector_fixed<T, 3> tmp(p.x(), p.y(), p.z());
    vnl_vector_fixed<T, 3> q = m * tmp;
    return vgl_point_3d<T>(q[0], q[1], q[2]);
}

template <class T>
vgl_point_2d<T> operator * (const vnl_matrix_fixed<T, 3, 3> &m, const vgl_point_2d<T> &p)
{
    vnl_vector_fixed<T, 3> tmp(p.x(), p.y(), 1.0);
    vnl_vector_fixed<T, 3> q = m * tmp;
    return vgl_point_2d<T>(q[0]/q[2], q[1]/q[2]);
}

//template <class T>
//vnl_vector_fixed<T, 3> operator - (const vgl_point_3d<T> & p1, const vgl_point_3d<T> & p2)
//{
//    return vnl_vector_fixed<T, 3>( p1.x() - p2.x(), p1.y() - p2.y(), p1.z() - p2.z());
//}


#endif
