/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     point3d.h
* \author   Collin Johnson
*
* Definition of Point3D -- (x,y,z)
*/

#ifndef MATH_POINT3D_H
#define MATH_POINT3D_H

#include "core/point.h"
#include "core/float_comparison.h"

namespace vulcan
{
namespace math
{

/**
* Point3D is a simple 3d point, (x,y,z)
*/
template <typename T>
class Point3D
{
public:

    T x;
    T y;
    T z;

    explicit Point3D(T x = 0, T y = 0, T z = 0)
    : x(x)
    , y(y)
    , z(z)
    {
    }

    // Allow implicit construction of 3d points from 2d points.
    Point3D(const Point<T>& point2d)
    : x(point2d.x)
    , y(point2d.y)
    , z(0)
    {
    }
};

// Various useful operator overloads
template <typename T, typename U>
bool operator==(const Point3D<T>& lhs, const Point3D<U>& rhs)
{
    // If floating point, then use the fuzzy floating point comparison
    if(std::is_floating_point<T>::value)
    {
        return absolute_fuzzy_equal(lhs.x, rhs.x) && absolute_fuzzy_equal(lhs.y, rhs.y) && absolute_fuzzy_equal(lhs.z, rhs.z);
    }
    else
    {
        return (lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z == rhs.z);
    }
}

template <typename T, typename U>
bool operator!=(const Point3D<T>& lhs, const Point3D<U>& rhs)
{
    // Go from bottom left to top right
    return !(lhs == rhs);
}

template <typename T, typename U>
bool operator<(const Point3D<T>& lhs, const Point3D<U>& rhs)
{
    // Go from bottom left to top right
    return (lhs.x < rhs.x)                       ||
           ((lhs.x == rhs.x) && (lhs.y < rhs.y)) ||
           ((lhs.x == rhs.x) && (lhs.y == rhs.y) && (lhs.z < rhs.z));
}

template<typename T, class ostream>
ostream& operator<<(ostream& out, const Point3D<T>& point)
{
    out<<'('<<point.x<<','<<point.y<<','<<point.z<<')';
    return out;
}

template <typename T, typename U>
Point3D<T> operator-(const Point3D<T>& lhs, const Point3D<U>& rhs)
{
    return Point3D<T>(lhs.x - rhs.x,
                            lhs.y - rhs.y,
                            lhs.z - rhs.z);
}

template <typename T, typename U>
Point3D<T> operator+(const Point3D<T>& lhs, const Point3D<U>& rhs)
{
    return Point3D<T>(lhs.x + rhs.x,
                            lhs.y + rhs.y,
                            lhs.z + rhs.z);
}

template <typename T, typename U>
Point3D<T>& operator+=(Point3D<T>& lhs, const Point3D<U>& rhs)
{
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;

    return lhs;
}


template <typename T, typename U>
Point3D<T>& operator-=(Point3D<T>& lhs, const Point3D<U>& rhs)
{
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;

    return lhs;
}

}
}

#endif // MATH_POINT3D_H
