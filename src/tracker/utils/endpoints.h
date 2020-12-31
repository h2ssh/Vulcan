/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     endpoints.h
 * \author   Collin Johnson
 *
 * Declaration of utility functions for finding the endpoints of various shapes, which are needed for various object
 * and motion tracking classes:
 *
 *   - major_axis_endpoints
 *   - two_circles_endpoints
 */

#ifndef TRACKER_UTILS_ENDPOINTS_H
#define TRACKER_UTILS_ENDPOINTS_H

#include "tracker/boundaries/shapes.h"

namespace vulcan
{
namespace tracker
{

/**
 * major_axis_endpoints calculates the endpoints of a rectangle along the major axis. The major axis is the axis that
 * is the longest.
 *
 * The endpoints are the intersection of the major axis line with the boundary of the rectangle.
 *
 * \param    rect            Rectangle for which to find the endpoints
 * \return   Endpoints of the rectangle.
 */
Endpoints major_axis_endpoints(const Rectangle& rect);

/**
 * circle_rect endpoints calculates the endpoints of the line connecting the center of the rectangle and circle. The
 * endpoints are where the line running through the two centers intersects the circle and rectangle boundaries.
 */
Endpoints circle_rect_endpoints(const CircleRect& circleRect);

/**
 * two_rects_endpoints calculates the endpoints of a the line connecting the center of the two rectangles. The endpoints
 * are where the line through the endpoints intersects the outer boundary of each rectangle.
 *
 * \param    rects       Rectanges for which to find the endpoints
 * \return   Endpoints of the TwoRects.
 */
Endpoints two_rects_endpoints(const TwoRects& rects);

/**
 * two_circles_endpoints finds the endpoints for a boundary defined by two circles. The endpoints for such a boundary
 * are where the line connecting the center of the two circles intersects the boundary of the circles.
 *
 * \param    circles         Circles for which to find the endpoints
 * \return   Endpoints of line connecting the circles.
 */
Endpoints two_circles_endpoints(const TwoCircles& circles);

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_UTILS_ENDPOINTS_H
