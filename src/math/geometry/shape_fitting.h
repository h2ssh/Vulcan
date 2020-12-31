/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     shape_fitting.h
 * \author   Collin Johnson
 *
 * Declaration of functions:
 *
 *   - axis_aligned_bounding_rectangle   : smallest rectangle that fits around all the points while aligned to the axes
 * of the coordinate system
 *   - minimum_area_bounding_rectangle   : smallest rectangle that fits around all the points, not aligned to axes
 *   - minimum_geometric_error_circle    : circle that minimizes error ((point - center) - radius) ^ 2
 *   - mean_position_and_radius_circle   : circle whose center is the mean of the points and radius is the mean of the
 * radius to the points from this center
 */

#ifndef MATH_GEOMETRY_SHAPE_FITTING_H
#define MATH_GEOMETRY_SHAPE_FITTING_H

#include "math/geometry/circle.h"
#include "math/geometry/rectangle.h"
#include <algorithm>
#include <cassert>

namespace vulcan
{
namespace math
{

template <typename T>
class Polygon;

/**
 * axis_aligned_bounding_rectangle calculates a bounding rectangle for the provided points
 * using a rectangle aligned to the axis of the coordinate frame. The vertices of the
 * rectangle will correspond to the minimum and maximum x,y values among the points.
 *
 * \param    points          Points for which the rectangle should be calculated (size >= 3)
 * \return   Axis-aligned bounding rectangle for the provided points.
 */
template <typename T>
Rectangle<T> axis_aligned_bounding_rectangle(typename std::vector<Point<T>>::const_iterator begin,
                                             typename std::vector<Point<T>>::const_iterator end)
{
    // Axis-aligned bounding rectangle places the vertices of the rectangle at (minX, minY), (maxX, minY), etc. So
    // just find the extrema and you're good to go.

    assert(begin != end);

    T minX = begin->x;
    T maxX = begin->x;
    T minY = begin->y;
    T maxY = begin->y;

    std::for_each(begin, end, [&](const Point<T>& point) {
        if (point.x < minX) {
            minX = point.x;
        } else if (point.x > maxX) {
            maxX = point.x;
        }

        if (point.y < minY) {
            minY = point.y;
        } else if (point.y > maxY) {
            maxY = point.y;
        }
    });

    return Rectangle<T>(Point<T>(minX, minY), Point<T>(maxX, maxY));
}

/**
 * minimum_area_bounding_rectangle finds the minimum bounding rectangle that fits around a
 * collection of points. The rectangle has an arbitrary orientation, as opposed to the simpler
 * bounding rectangle function axis_aligned_bounding_rectangle() that simply encloses the region
 * based on the extrema x,y coordinates of the points it contains.
 *
 * The algorithm implemented here comes from: "Solving Geometric Problems with the Rotating Calipers"
 * from Godfried Toussaint in MELECON '83.
 *
 * The key idea is that the minimum area bounding rectangle must share one side with the convex hull
 * of the points. Therefore, find the convex hull. Find the basic bounding rectangle using the extrema.
 * Rotate this rectangle to align to a side of the convex hull. Keep rotating the rectangle and finding
 * the area until the rotation is more than pi/2. At that point, just take the minimum area rectangle
 * that was found.
 *
 * \param    begin           Start of the points vector
 * \param    end             End of the points vector
 * \return   Minimum area bounding rectangle for the provided points.
 */
Rectangle<float> minimum_area_bounding_rectangle(std::vector<Point<float>>::const_iterator begin,
                                                 std::vector<Point<float>>::const_iterator end);

/**
 * minimum_area_bounding_rectangle is an optimization that allows the boundary to be found using an already-computed
 * convex hull. It just skips that step, but otherwise runs the same.
 */
Rectangle<float> minimum_area_bounding_rectangle(std::vector<Point<float>>::const_iterator begin,
                                                 std::vector<Point<float>>::const_iterator end,
                                                 const Polygon<float>& hull);

/**
 * minimum_geometric_error_bounding_rectangle finds the bounding rectangle for the provided points that minimizes the
 * distance between the contained points and the boundary. The algorithm is the same as for finding the minimum bounding
 * area rectangle, but the error between the points and the boundary is minimized, rather than the total area of the
 * rectangle.
 *
 * The minimum geometry error helps in cases where one corner of a rectangle is seen. In this case, the minimum area
 * rectangle is often orthogonal to the corner, going at a diagonal to the actual shape, the minimum error rectangle
 * nestles in nicely to the corner though.
 *
 * \param    begin           Start of the points vector
 * \param    end             End of the points vector
 * \return   Minimum geometric error bounding rectangle.
 */
Rectangle<float> minimum_geometric_error_bounding_rectangle(std::vector<Point<float>>::const_iterator begin,
                                                            std::vector<Point<float>>::const_iterator end);

/**
 * minimum_geometric_error_bounding_rectangle is an optimization that allows the boundary to be found using an
 * already-computed convex hull. It just skips that step, but otherwise runs the same.
 */
Rectangle<float> minimum_geometric_error_bounding_rectangle(std::vector<Point<float>>::const_iterator begin,
                                                            std::vector<Point<float>>::const_iterator end,
                                                            const Polygon<float>& hull);


/**
 * minimum_geometric_error_circle fits a circle minimizing the geometric error for the points  p_0...i for a circle
 * with center c and radius r:
 *
 *   e(i) = dist(c,p_i) - r
 *
 * The approach implemented uses the Levenberg-Marquardt algorithm with the alternate (A,D,theta) coordinate system
 * defined in this paper:
 *
 *   Least squares fitting of circles  by N . Chernov and C. Lesort
 *
 * When looking at the results, the LMA method is used. LMA is chosen because it works best for circles where only a
 * small subset of the points along the circumference are seen. This condition is very true for any measurements taken
 * by a laser rangefinder since it will necessarily see no more than half, but likely much less, the circle.
 *
 * \pre      std::distance(pointsBegin, pointsEnd) >= 3
 * \pre      minRadius > 0.0
 * \pre      maxRadius > minRadius
 * \param    pointsBegin         Start iterator for the points to be fit
 * \param    pointsEnd           One-past-the-end iterator for the other points to be fit
 * \param    minRadius           Minimum radius for the circle (optional, default = 0.01)
 * \param    maxRadius           Maximum radius for the circle (optional, default = 2.0)
 * \param[out]   fitError        Total error of the final fit circle (optional, default = don't report)
 * \return   Circle fitting the points with the minimum geometric error.
 */
Circle<float> minimum_geometric_error_circle(std::vector<Point<float>>::const_iterator pointsBegin,
                                             std::vector<Point<float>>::const_iterator pointsEnd,
                                             double minRadius = 0.01,
                                             double maxRadius = 2.0,
                                             double* fitError = nullptr);

/**
 * mean_position_and_radius_circle is a simple circle fitting that puts the center of the circle at the mean of the
 * points and the radius of the circle is the mean radius from this center to each of the points.
 *
 * \pre      std::distance(pointsBegin, pointsEnd) > 1
 * \param    pointsBegin         Start iterator for the points to be fit
 * \param    pointsEnd           One-past-the-end iterator for the other points to be fit
 * \return   Mean circle fit to the points
 */
Circle<float> mean_position_and_radius_circle(std::vector<Point<float>>::const_iterator pointsBegin,
                                              std::vector<Point<float>>::const_iterator pointsEnd);

}   // namespace math
}   // namespace vulcan

#endif   // MATH_GEOMETRY_SHAPE_FITTING_H
