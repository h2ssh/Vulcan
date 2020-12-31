/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     navigation_grid_builder.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of NavigationGridBuilder.
 */

#ifndef MPEPC_GRID_NAVIGATION_GRID_BUILDER_H
#define MPEPC_GRID_NAVIGATION_GRID_BUILDER_H

#include "mpepc/types.h"
#include <memory>

namespace vulcan
{
namespace mpepc
{

class CostMap;
class NavigationGrid;
struct navigation_grid_builder_params_t;

/**
 * NavigationGridBuilder handles the construction of NavigationGrids. The NavigationGrid is constructed using the
 * wavefront algorithm. The cell containing the goal pose has zero cost and is placed in the queue. From here, each
 * neighboring cell, based on 8-way connectivity, is considered. If the cumulative cost of traveling to the cell and
 * being at the cell is less than the currently assigned cost, replace the cost for the cell and enqueue the cell.
 * Repeat this process until the queue is empty.
 */
class NavigationGridBuilder
{
public:
    /**
     * Constructor for NavigationGridBuilder.
     *
     * \param    params          Parameters for building a navigation grid
     */
    NavigationGridBuilder(const navigation_grid_builder_params_t& params);

    /**
     * Destructor for NavigationGridBuilder.
     */
    ~NavigationGridBuilder(void);

    /**
     * buildGrid builds a NavigationGrid defining a navigation function that carries the robot from the start
     * location to the specified goal location. The navigation function combines the a distance-based metric and
     * a cost-based metric to determine the optimal policy for every cell, subject to heurstic cost defined by the
     * provided cost map.
     *
     * \param    start               Starting pose of the path through the navigation grid
     * \param    goal                Goal pose for the path
     * \param    costMap             Cost map defining the costs associated with occupying cells
     * \param    navGrid             NavigationGrid built using the targets and cost map
     * \return   True if the grid was built. False if no changes were deemed necessary.
     */
    bool buildGrid(const pose_t& start, const pose_t& goal, const CostMap& costMap, NavigationGrid& navGrid);

    /**
     * buildGrid builds a NavigationGrid defining a navigation function that carries the robot from the start
     * location to the specified goal location. The navigation function combines the a distance-based metric and
     * a cost-based metric to determine the optimal policy for every cell, subject to heurstic cost defined by the
     * provided cost map.
     *
     * This version of the navigation function carries the robot through a sequence of intermediate goal locations
     * before reaching the final goal.
     *
     * \param    start               Starting pose for the path through the navigation grid
     * \param    goal                Goal pose for the path
     * \param    intermediate        Intermediate waypoints that define a high-level topological route
     * \param    costMap             Cost map defining the costs associated with occupying cells
     * \param    navGrid             NavigationGrid built using the targets and cost map
     * \return   True if the grid was built. False if no changes were deemed necessary.
     */
    bool buildGrid(const pose_t& start,
                   const pose_t& goal,
                   const std::vector<NavWaypoint>& intermediate,
                   const CostMap& costMap,
                   NavigationGrid& navGrid);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_GRID_NAVIGATION_GRID_BUILDER_H
