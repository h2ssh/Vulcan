/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     obstacle_distance_grid_builder.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of ObstacleDistanceGridBuilder.
 */

#ifndef MPEPC_GRIDS_OBSTACLE_DISTANCE_GRID_BUILDER_H
#define MPEPC_GRIDS_OBSTACLE_DISTANCE_GRID_BUILDER_H

#include <memory>

namespace vulcan
{
namespace hssh
{
class LocalPerceptualMap;
}
namespace mpepc
{

class ObstacleDistanceGrid;
struct obstacle_distance_grid_builder_params_t;

/**
 * ObstacleDistanceGridBuilder builds the grid map of distance-to-the-nearest-obstacle using 8-way wavefront,
 * where the obstacles are static (or unobserved) cells with an option to expand all static obstacles by some radius.
 */
class ObstacleDistanceGridBuilder
{
public:
    /**
     * Constructor for ObstacleDistanceMapBuilder.
     *
     * \param    params      Parameters for the grid construction
     */
    ObstacleDistanceGridBuilder(const obstacle_distance_grid_builder_params_t& params);

    /**
     * Destructor for ObstacleDistanceGridBuilder.
     */
    ~ObstacleDistanceGridBuilder(void);

    /**
     * buildGrid builds the obstacle distance map based on the LPM provided.
     *
     * \param    lpm             Current LPM from sensors
     * \param    distGrid        Distance map to be built
     * \return   true if a new map is built, false otherwise.
     */
    bool buildGrid(const hssh::LocalPerceptualMap& lpm, ObstacleDistanceGrid& distGrid);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_GRIDS_OBSTACLE_DISTANCE_GRID_BUILDER_H
