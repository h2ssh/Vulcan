/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ray_tracing.h
* \author   Collin Johnson
*
* Definition of function templates for performing various ray tracing operations through grids:
*
*   - trace_ray_until_condition : trace a single ray until a termination condition is satisfied
*   - trace_range_until_condition : trace a range of rays, each at a different angle, until termination
*   - is_cell_visible_from_position : check if a check can be seen by attempting to trace a ray to it
*   - find_cells_along_line : finds all cells along a line in the grid
*/

#ifndef UTILS_RAY_TRACING_H
#define UTILS_RAY_TRACING_H

#include <utils/cell_grid_utils.h>
#include <core/line.h>
#include <core/point.h>
#include <boost/algorithm/clamp.hpp>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace vulcan
{
namespace utils
{

struct ray_trace_range_t
{
    double      startAngle;
    double      range;
    double      resolution;
    bool        counterclockwise;      // which direction to scan
    std::size_t numIncrements;

    explicit ray_trace_range_t(double start = 0.0, double range = 2*M_PI, double resolution = 0.5f, bool ccw = true)
    : startAngle(start)
    , range(std::abs(range))
    , resolution(std::abs(resolution))
    , counterclockwise(ccw)
    , numIncrements(((resolution > 0.0) ? range / resolution : 0) + 1)
    {
    }

    ray_trace_range_t(double start, std::size_t numIncrements, double resolution, bool ccw = true)
    : startAngle(start)
    , range(numIncrements*resolution*(ccw ? 1 : -1))
    , resolution(resolution)
    , counterclockwise(ccw)
    , numIncrements(numIncrements)
    {
    }
};

/**
* TerminationCondition is the function that determines when the ray tracing should stop. It accepts the Grid
* and a cell and returns true when the ray tracing is complete.
*/
// template <class Grid>
// using TerminationCondition = std::function<bool(const Grid&, Point<int>)>;

/**
* trace_ray_until_condition traces a ray from the start cell until the termination condition is true or the maximum distance is reached.
*
* \param    start               Start position of the ray in grid coordinates (cells)
* \param    angle               Angle along which to trace the ray
* \param    maxDistance         Maximum distance to trace the ray             (meters)
* \param    grid                Grid in which to trace
* \param    terminationCond     Condition to use for deciding if the ray has reached its end  signature: bool(const Grid&, Point<int>)
* \return   Endpoint of the ray in grid coordinates.
*/
template <class Grid, class TerminationCondition>
Point<double> trace_ray_until_condition(Point<double>  start,
                                              double               angle,
                                              double               maxDistance,
                                              const Grid&          grid,
                                              TerminationCondition terminationCond)
{
    // Short-circuit the calculation if the start satisfies the termination condition or if the start isn't in the grid
    if(!grid.isCellInGrid(start) || terminationCond(grid, start))
    {
        return start;
    }

    /*
    * The below code is based on the ray-tracing approach outlined here:
    *
    *   http://playtechs.blogspot.com/2007/03/raytracing-on-grid.html  The Simplifying, Round One algorithm is used.
    *
    * This algorithm is a slight variant on Bresenham's algorithm.
    *
    */

    double cosAngle = std::cos(angle);
    double sinAngle = std::sin(angle);

    double endX = start.x + (maxDistance * grid.cellsPerMeter() * cosAngle);
    double endY = start.y + (maxDistance * grid.cellsPerMeter() * sinAngle);

    double dx = std::abs(endX - start.x);
    double dy = std::abs(endY - start.y);

    int x = floor(start.x);
    int y = floor(start.y);

    int totalSteps = 1;
    int xIncr = 1;
    int yIncr = 1;
    double error = 0.0;

    if(dx == 0)
    {
        xIncr = 0;
        error = std::numeric_limits<double>::infinity();    // positive error means increment y. pos infinity to only increment y
    }
    else if (endX > start.x)
    {
        xIncr = 1;
        totalSteps += static_cast<int>(floor(endX)) - x;
        error = (floor(start.x) + 1 - start.x) * dy; // floor + 1 instead of ceil in case starting on a boundary
    }
    else
    {
        xIncr = -1;
        totalSteps += x - static_cast<int>(floor(endX));
        error = (start.x - floor(start.x)) * dy;
    }

    if(dy == 0)
    {
        yIncr = 0;
        error -= std::numeric_limits<double>::infinity();  // negative error means increment x. neg infinity to only increment x
    }
    else if (endY > start.y)
    {
        yIncr = 1;
        totalSteps += int(floor(endY)) - y;
        error -= (floor(start.y) + 1 - start.y) * dx;  // floor + 1 instead of ceil in case starting on a boundary
    }
    else
    {
        yIncr = -1;
        totalSteps += y - static_cast<int>(floor(endY));
        error -= (start.y - floor(start.y)) * dx;
    }

    bool lastCrossedHorizontal = false;
    bool wasTerminated = false;     // flag indicating if stopping reason was due to termination cond
    Point<int> rayCell(x, y);

    int numSteps = 0;
    for(; grid.isCellInGrid(rayCell) && (numSteps < totalSteps); ++numSteps)
    {
        if(terminationCond(grid, rayCell))
        {
            wasTerminated = true;
            break;
        }

        if(error > 0)
        {
            rayCell.y += yIncr;
            error -= dx;
            lastCrossedHorizontal = true;
        }
        else
        {
            rayCell.x += xIncr;
            error += dy;
            lastCrossedHorizontal = false;
        }
    }

    // If less than 2 increments were made, then the termination condition triggered immediately, or on the adjacent cell
    // which means we should clamp to the starting position
    if(numSteps < 2)
    {
        return start;
    }

    dx = rayCell.x - x;
    dy = rayCell.y - y;

    // After finding the end cell, want to get a better angular resolution on where the ray hit. The discretized steps
    // mean the angle of the ray won't match the desired angle. Create the ray extending from the start cell in the
    // desired direction.
    // Now, we find where it intersects the boundary of the ending cell found via the discretized steps. Based on how
    // the stepping was happening, we can determine which side of the cell to intersect with.

    double dist = std::sqrt(dx * dx + dy * dy);
    Point<double> hitPosition(start.x + (dist * cosAngle), start.y + (dist * sinAngle));

    // triggered termination condition, so see where exactly in the cell we hit
    if(wasTerminated)
    {
        const double kCellDist = 0.99999; // keep the dist slightly inside the cell, so if truncation is used, the same
        Line<double> raySegment = Line<double>(start, hitPosition);
        Line<double> cellBoundary;

        if(lastCrossedHorizontal)
        {
            // If y is increasing, then we came from the bottom of the cell
            if(endY > start.y)
            {
                cellBoundary = Line<double>(rayCell.x, rayCell.y, rayCell.x + kCellDist, rayCell.y);
            }
            // Else came from the top of the cell
            else
            {
                cellBoundary = Line<double>(rayCell.x, rayCell.y + kCellDist, rayCell.x + kCellDist, rayCell.y + kCellDist);
            }
        }
        else // if(!lastCrossHorizontal)
        {
            // If x is increasing, then we enter from the left side of the cell
            if(endX > start.x)
            {
                cellBoundary = Line<double>(rayCell.x, rayCell.y, rayCell.x, rayCell.y + kCellDist);
            }
            // Otherwise, we enter from the right side of the cell
            else
            {
                cellBoundary = Line<double>(rayCell.x + kCellDist, rayCell.y, rayCell.x + kCellDist, rayCell.y + kCellDist);
            }
        }

        line_intersection_data_t<double> intersection(raySegment, cellBoundary);
        hitPosition = intersection.intersectionPoint();
    }

    // Clamp the values to ensure they are inside the grid
    return Point<double>(boost::algorithm::clamp(hitPosition.x,  0.0, static_cast<double>(grid.getWidthInCells()-1)),
                               boost::algorithm::clamp(hitPosition.y,  0.0, static_cast<double>(grid.getHeightInCells()-1)));
}

/**
* trace_range_until_condition traces along a each in the range until some condition is satisfied. The endpoints of the
* operation are stored and returned.
*
* \param    start               Start position of the ray in grid coordinates (cells)
* \param    range               Range of angles to be checked
* \param    maxDistance         Maximum distance to trace the ray             (meters)
* \param    grid                Grid in which to trace
* \param    terminationCond     Condition to use for deciding if the ray has reached its end  signature: bool(const Grid&, Point<int>)
* \return   Endpoint of the ray in grid coordinates.
*/
template <class Grid, class TerminationCondition>
std::vector<Point<double>> trace_range_until_condition(Point<double>      start,
                                                             const ray_trace_range_t& range,
                                                             double                   maxDistance,
                                                             const Grid&              grid,
                                                             TerminationCondition     terminationCond)
{
    std::vector<Point<double>> endpoints(range.numIncrements);
    trace_range_until_condition(start, range, maxDistance, grid, terminationCond, endpoints.begin());

    return endpoints;
}

/**
* trace_range_until_condition that stores beginning in endpointBegin.
*
* \param    start               Start position of the ray in grid coordinates (cells)
* \param    range               Range of angles along which to trace rays
* \param    maxDistance         Maximum distance to trace the ray             (meters)
* \param    grid                Grid in which to trace
* \param    terminationCond     Condition to use for deciding if the ray has reached its end  signature: bool(const Grid&, Point<int>)
* \param    endpointBegin       Location to being storing the endpoints. StorageIterator must dereference to a Point<double> reference.
*/
template <class Grid, class TerminationCondition, class StorageIterator>
void trace_range_until_condition(Point<double>      start,
                                 const ray_trace_range_t& range,
                                 double                   maxDistance,
                                 const Grid&              grid,
                                 TerminationCondition     terminationCond,
                                 StorageIterator          endpointBegin)
{
    double increment = range.counterclockwise ? range.resolution : -range.resolution;

    for(std::size_t n = 0; n < range.numIncrements; ++n)
    {
        *endpointBegin++ = trace_ray_until_condition(start,
                                                     range.startAngle + n*increment,
                                                     maxDistance,
                                                     grid,
                                                     terminationCond);
    }
}

/**
* is_cell_visible_from_position checks if a particular cell has a line of visibility from the provided position. Both position and cell are in grid coordinates,
* not global coordinates.
*/
template <class Grid, class TerminationCondition>
bool is_cell_visible_from_position(const Point<double>& position,
                                   const Point<int>&    cell,
                                   const Grid&                grid,
                                   TerminationCondition       terminationCond)
{
    bool terminationFired           = false;
    auto cacheTerminationResultFunc = [&terminationFired, terminationCond](const Grid& g, Point<int> c) { return terminationFired |= terminationCond(g, c); };
    trace_ray_until_condition(position,
                              angle_to_point(position, cell),
                              distance_between_points(position, cell)*grid.metersPerCell(),
                              grid,
                              cacheTerminationResultFunc);
    return !terminationFired;
}

/**
* find_cells_along_line finds the cells in the grid along the specified line. The line is in grid coordinates, not global coordinates.
*/
template <class Grid, class OutputIterator>
OutputIterator find_cells_along_line(Line<double> line, const Grid& grid, OutputIterator out)
{
    // Create a termination condition that just saves each cell checked against
    auto saveCellsFunc = [&out](const Grid& g, Point<int> c) -> bool {
        *out++ = c;
        return false;
    };
    trace_ray_until_condition(line.a,
                              angle_to_point(line.a, line.b),
                              length(line)*grid.metersPerCell(),
                              grid,
                              saveCellsFunc);
    return out;
}

} // namespace utils
} // namespace vulcan

#endif // UTILS_RAY_TRACING_H
