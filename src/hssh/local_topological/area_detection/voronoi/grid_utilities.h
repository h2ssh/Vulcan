/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     grid_utilities.h
 * \author   Collin Johnson
 *
 * Contains commmon functions used in a variety of grid processing algorithms
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_GRID_UTILITIES_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_GRID_UTILITIES_H

#include "core/point.h"
#include <cstdint>
#include <vector>

namespace vulcan
{
namespace hssh
{

class VoronoiSkeletonGrid;

struct ray_trace_range_t
{
    // If a complete circle is desired, startAngle = 0, stopAngle = 2*M_PI, counterclockwise = true
    float startAngle;
    float stopAngle;
    float resolution;
    bool counterclockwise;   // which direction to scan

    explicit ray_trace_range_t(float start = 0.0f, float stop = 2 * M_PI, float resolution = 0.5f, bool ccw = true)
    : startAngle(start)
    , stopAngle(stop)
    , resolution(resolution)
    , counterclockwise(ccw)
    {
    }
};

/**
 * trace_ray_to_cell traces a ray starting from the specified point, along the given angle. The ray endpoint is
 * determined when (grid.getClassification() & mask) || endpoint_is_edge_of_map().
 *
 * \param    angle           Direction along which to trace the ray
 * \param    start           Starting point of the ray
 * \param    mask            Mask to be satisfied for the endpoint of the ray
 * \param    grid            Grid through which the ray should be traced
 * \return   Endpoint of the ray.
 */
Point<int> trace_ray_to_cell(float angle, const Point<int>& start, uint8_t mask, const VoronoiSkeletonGrid& grid);

/**
 * trace_rays_in_range traces the rays in the specified range until a non-island mask cell is encountered. This function
 * is for convenience in calling trace_ray_to_cell multiple times.
 *
 * If the defined range is for > 2*pi range, the cutoff will be after 2*pi is angle subtends by ray area.
 *
 * \param    range           Range of rays to be traced
 * \param    start           Start point of the rays
 * \param    mask            Mask specifying the cells of interest
 * \param    grid            Grid in which to trace
 * \return   Vector of cell endpoints for the rays.
 */
std::vector<Point<int>> trace_rays_in_range(const ray_trace_range_t& range,
                                            const Point<int>& start,
                                            uint8_t mask,
                                            const VoronoiSkeletonGrid& grid);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_GRID_UTILITIES_H
