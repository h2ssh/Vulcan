/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     shape_intersection_area.h
 * \author   Collin Johnson
 *
 * Declaration of various functions to find the area of the intersection between shapes. Currently supported shapes are:
 *
 *   - circle_rectangle_intersection_area
 *   - circle_circle_intersection_area
 */

#ifndef MATH_GEOMETRY_SHAPE_INTERSECTION_AREA_H
#define MATH_GEOMETRY_SHAPE_INTERSECTION_AREA_H

#include "math/geometry/circle.h"
#include "math/geometry/rectangle.h"

namespace vulcan
{
namespace math
{

/**
 * circle_rectangle_intersection_area calculates the area of intersection between a circle and a rectangle.
 *
 * \pre  circle.radius() > 0
 */
float circle_rectangle_intersection_area(const Circle<float>& circle, const Rectangle<float>& rectangle);

/**
 * circle_circle_intersection_area calculates the area of intersection between two circles.
 *
 * \pre  lhs.radius() > 0
 * \pre  rhs.radius() > 0
 */
float circle_circle_intersection_area(const Circle<float>& lhs, const Circle<float>& rhs);

}   // namespace math
}   // namespace vulcan

#endif   // MATH_GEOMETRY_SHAPE_INTERSECTION_AREA_H
