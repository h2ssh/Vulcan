/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     social_norm_utils.h
 * \author   Collin Johnson
 *
 * Declaration of utility functions for converting topological structure into cost functions:
 *
 *   - nearest_skeleton_index : nearest skeleton cell to portion of skeleton of interest
 *   - is_right_of_skeleton : is this cell to the right of the skeleton in its direction of motion?
 */

#ifndef MPEPC_COSTS_SOCIAL_NORM_UTILS_H
#define MPEPC_COSTS_SOCIAL_NORM_UTILS_H

#include "hssh/types.h"

namespace vulcan
{
struct motion_state_t;

namespace hssh
{
class LocalTopoMap;
}
namespace hssh
{
class VoronoiSkeletonGrid;
}
namespace mpepc
{

struct topo_agent_t;

/**
 * Check if the agent in question is about to cross a gateway and enter a new area.
 *
 * \param    agent               Agent to check
 * \param    topoMap             Topo map the agent is in
 * \param    transitionTime      Time (sec) threshold from gateway for near-transition (optional)
 * \return   True if a transition to a new area is about to begin.
 */
bool is_about_to_transition(const topo_agent_t& agent, const hssh::LocalTopoMap& topoMap, double transitionTime = 0.5);

/**
 * Convert the motion state for the robot into a topological state represented as a topo_agent_t within the
 * corresponding topological map.
 *
 * \param    robotState          Metrical state of the topo
 * \param    topoMap             Topological map representation of the environment
 * \return   An agent representation of the robot's topological state.
 */
topo_agent_t create_agent_for_robot(const motion_state_t& robotState, const hssh::LocalTopoMap& topoMap);

/**
 * skeleton_cells_along_route finds the reduced skeleton cells along the route from some starting cell in the map to an
 * ending cell. Neither the start nor end has to be a skeleton cell. The returned cells are ONLY of type
 * REDUCED_SKELETON.
 *
 * \param    startCell       Start cell of the route
 * \param    endCell         End cell of the route
 * \param    skeleton        Skeleton grid representation of the environment
 * \return   The route in skeleton cells from the nearest skeleton cell to startCell to endCell.
 */
hssh::CellVector
  skeleton_cells_along_route(hssh::cell_t startCell, hssh::cell_t endCell, const hssh::VoronoiSkeletonGrid& skeleton);

/**
 * nearest_skeleton_index finds the nearest skeleton cell to the provided cell.
 *
 * \param    cell            Cell for which to find the index
 * \param    skeleton        Skeleton cells of interest
 * \param    hintIdx         Hint for where the index might be
 * \return   Index of the nearest cell or -1 if skeleton.empty().
 */
int nearest_skeleton_index(hssh::cell_t cell, const hssh::CellVector& skeleton, std::size_t hintIdx = 0);

/**
 * is_right_of_skeleton checks if a cell is right of the skeleton in the direction of motion for
 * the skeleton, which is defined by the linear ordering of cells in the skeleton vector.
 *
 * \param    cell            Cell to check for position
 * \param    skeletonIndex   Index of the nearest skeleton cell
 * \param    skeleton        Portion of interest for the skeleton
 * \return   True if the cell is to the right of the skeleton. False if on or to the left.
 */
bool is_right_of_skeleton(hssh::cell_t cell, int skeletonIndex, const hssh::CellVector& skeleton);

/**
 * normalized_position_path finds the normalized lateral position of an agent along a path in the agent's direction
 * of motion along the path. The distance is 0 at the left wall and 1 at the right wall.
 *
 * \param    agent       Agent of interest
 * \param    topoMap     Topological map being navigated
 * \return   Normalized distance in range [0, 1]. If not on a path, then negative.
 */
double normalized_position_path(const topo_agent_t& agent, const hssh::LocalTopoMap& topoMap);

/**
 * normalized_position_gateway finds the normalized lateral position of an agent crossing a gateway. The distance is 0
 * at the left endpoint and 1 at the right endpoint.
 *
 * \param    agent       Agent of interest
 * \param    topoMap     Topological map being navigated
 * \return   Normalized distance of gateway crossing in range [0, 1]. Negative if agent gatewayId not in map.
 */
double normalized_position_gateway(const topo_agent_t& agent, const hssh::LocalTopoMap& topoMap);

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_COSTS_SOCIAL_NORM_UTILS_H
