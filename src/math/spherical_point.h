/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     spherical_point.h
 * \author   Collin Johnson
 *
 * Declaration of SphericalPoint.
 */

#ifndef MATH_SPHERICAL_POINT_H
#define MATH_SPHERICAL_POINT_H

#include "math/point3d.h"
#include <cmath>

namespace vulcan
{
namespace math
{

/**
 * SphericalPoint is a simple spherical point, (rho, phi, theta).
 */
struct SphericalPoint
{
    float rho;
    float phi;
    float theta;

    /**
     * Default constructor for SphericalPoint.
     */
    SphericalPoint(float rho = 0, float phi = 0, float theta = 0) : rho(rho), phi(phi), theta(theta) { }

    /**
     * Constructor for SphericalPoint.
     *
     * Converts cartesian point to a spherical point.
     */
    SphericalPoint(const Point3D<float>& cartesian)
    : rho(std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y + cartesian.z * cartesian.z))
    , phi(std::acos(std::sqrt(cartesian.x * cartesian.x + cartesian.y * cartesian.y) / rho))
    , theta(std::atan2(cartesian.y, cartesian.x))
    {
    }

    /**
     * toCartesian converts the point into Cartesian coordinates.
     */
    Point3D<float> toCartesian(void) const
    {
        return Point3D<float>(rho * std::cos(theta) * std::cos(phi),
                              rho * std::sin(theta) * std::cos(phi),
                              rho * std::sin(phi));
    }
};

// Various useful operators
inline bool operator==(const SphericalPoint& lhs, const SphericalPoint& rhs)
{
    // Go from bottom left to top right
    return (lhs.rho == rhs.rho) && (lhs.theta == rhs.theta) && (lhs.phi == rhs.phi);
}

inline bool operator!=(const SphericalPoint& lhs, const SphericalPoint& rhs)
{
    // Go from bottom left to top right
    return !(lhs == rhs);
}

template <class ostream>
ostream& operator<<(ostream& out, const SphericalPoint& point)
{
    out << '(' << point.rho << ',' << point.theta << ',' << point.phi << ')';
    return out;
}

}   // namespace math
}   // namespace vulcan

#endif   // MATH_SPHERICAL_POINT_H
