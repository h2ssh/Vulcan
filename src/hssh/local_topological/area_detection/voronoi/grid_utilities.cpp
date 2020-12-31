/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     grid_utilities.cpp
* \author   Collin Johnson
*
* Definition of trace_ray_to_cell.
*/

#include "hssh/local_topological/area_detection/voronoi/island_detector.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "hssh/local_topological/area_detection/voronoi/grid_utilities.h"
#include "core/angle_functions.h"

namespace vulcan
{
namespace hssh
{

float total_angle_subtended(float start, float stop, bool counterclockwise);
bool  cell_is_ray_endpoint (const Point<int>& point, uint8_t mask, const VoronoiSkeletonGrid& grid);
int   clamp_ray_endpoint   (int endpoint, int dimensionSize);


Point<int> trace_ray_to_cell(float                      angle,
                                   const Point<int>&    start,
                                   uint8_t                    mask,
                                   const VoronoiSkeletonGrid& grid)
{
    float deltaX = cos(angle);
    float deltaY = sin(angle);

    float xPosition = start.x;
    float yPosition = start.y;

    Point<int> rayCell(start.x, start.y);

    while(!cell_is_ray_endpoint(rayCell, mask, grid))
    {
        xPosition += deltaX;
        yPosition += deltaY;

        rayCell.x = xPosition;
        rayCell.y = yPosition;
    }

    rayCell.x = clamp_ray_endpoint(rayCell.x, grid.getWidthInCells());
    rayCell.y = clamp_ray_endpoint(rayCell.y, grid.getHeightInCells());

    return rayCell;
}


std::vector<Point<int>> trace_rays_in_range(const ray_trace_range_t&   range,
                                                  const Point<int>&    start,
                                                  uint8_t                    mask,
                                                  const VoronoiSkeletonGrid& grid)
{
    std::vector<Point<int>> endpoints;

    float totalAngleSubtended = total_angle_subtended(range.startAngle, range.stopAngle, range.counterclockwise);
    float angleSubtended      = 0.0f;
    float angleIncrement      = std::abs(range.resolution);

    for(float angle = range.startAngle; angleSubtended < totalAngleSubtended; angleSubtended += angleIncrement)
    {
        endpoints.push_back(trace_ray_to_cell(angle, start, mask, grid));

        angle += range.counterclockwise ? angleIncrement : -angleIncrement;
    }

    return endpoints;
}


float total_angle_subtended(float start, float stop, bool counterclockwise)
{
    // this is the special case for a complete circle
    if(std::abs(stop - start) > 2*M_PI)
    {
        return 2*M_PI;
    }
    else if(counterclockwise)
    {
        return wrap_to_2pi(2*M_PI + stop - start);
    }
    else
    {
        return wrap_to_2pi(2*M_PI + start - stop);
    }
}


bool cell_is_ray_endpoint(const Point<int>& point, uint8_t mask, const VoronoiSkeletonGrid& grid)
{
    return (point.x >= grid.getWidthInCells() || point.y >= grid.getHeightInCells()) ||
           (grid.getClassification(point.x, point.y) & mask);
}


int clamp_ray_endpoint(int endpoint, int dimensionSize)
{
    if(endpoint == dimensionSize)
    {
        return endpoint - 1;
    }
    else if(endpoint > dimensionSize)  // overflowed on the negative side, so go back to 0
    {
        return 0;
    }

    return endpoint;
}

} // namespace hssh
} // namespace vulcan
