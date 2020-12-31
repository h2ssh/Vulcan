/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     interpolation.h
 * \author   Collin Johnson
 *
 * Declaration of a variety of interpolation functions:
 *
 *   - unit_bilinear_interpolation
 *   - linear_interpolation
 */

#ifndef MATH_INTERPOLATION_H
#define MATH_INTERPOLATION_H

#include "core/point.h"
#include <cassert>

namespace vulcan
{
namespace math
{

/**
 * unit_bilinear_interpolation implements bilinear interpolation on a unit grid.
 * For unit bilinear interpolation, the function for calculating the desired value
 * is
 *
 *   f(x,y) = f(0,0)(1-x)(1-y) + f(1,0)x(1-y) + f(0,1)(1-x)y + f(1,1)xy
 *
 * The values to use for interpolation are provided in the following order:
 *   (0,0), (1,0), (0,1), (1,1)
 *
 * \param    point           Point to be interpolated
 * \param    values          Values of the points on the grid around the point to interpolate
 * \pre      0 <= point.x <= 1, 0 <= point.y <= 1
 * \return   Value interpolated at point.
 */
float unit_bilinear_interpolation(const Point<float>& point, float values[4]);

/**
 * linear_interpolation performs simple linear interpolation on the provided value.
 *
 *   result = (end - begin) * ratio + begin
 *
 * \pre  0 <= ratio <= 1.0
 * \param    begin           Start of the range being interpolated
 * \param    end             End of the range being interpolated
 * \param    ratio           Interpolation ratio
 * \return   Linear interpolation of range [begin, end].
 */
template <typename T>
T linear_interpolation(T begin, T end, double ratio)
{
    assert(ratio >= 0.0);
    assert(ratio <= 1.0);

    return begin + ((end - begin) * ratio);
}

}   // namespace math
}   // namespace vulcan

#endif   // MATH_INTERPOLATION_H
