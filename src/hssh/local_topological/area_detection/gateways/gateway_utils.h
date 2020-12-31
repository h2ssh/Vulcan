/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gateway_utils.h
 * \author   Collin Johnson
 *
 * Declaration of utility functions for constructing gateways:
 *
 *   - are_gateways_intersecting
 *   - are_gatway_angle_close
 *   - is_gateway_untraversable
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_GATEWAY_UTILS_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_GATEWAY_UTILS_H

#include "core/line.h"
#include "hssh/types.h"
#include <boost/optional.hpp>

namespace vulcan
{
namespace hssh
{

class Gateway;
class EndpointValidator;
class VoronoiIsovistField;
class VoronoiSkeletonGrid;

/**
 * are_gateways_intersecting checks if the two gateways intersect.
 */
bool are_gateways_intersecting(const Gateway& lhs, const Gateway& rhs);

/**
 * are_gateway_angles_close checks if the angle between the two gateways is less than closeAngleThreshold
 * and their endpoints are less than endpointDistanceThreshold away from each other.
 *
 * \param    lhs                         Gateway to compare
 * \param    rhs                         Gateway to compare
 * \param    closeAngleThreshold         Threshold for the difference in angles for two gateways to be considered the
 * same (optional, default = pi/4) \param    endpointDistanceThreshold   Threshold for the difference in distance for
 * closest gateway endpoints for gateways to be considered the same (optional, default = 0.2m) \return True if the
 * gateway angles and minimum endpoint distance are less than the defined thresholds.
 */
bool are_gateway_angles_close(const Gateway& lhs,
                              const Gateway& rhs,
                              double closeAngleThreshold = 0.7854,   // pi/4
                              double endpointDistanceThreshold = 0.2);

/**
 * is_gateway_traversable checks if the robot can fit on both sides of the gateway. If not, then it can't be an
 * affordance. The check sweeps through the skeleton and checks that there are enough cells on each side of the gateway
 * to ensure that it can be traversed by the robot. The length of the robot is used to determine if the robot can fit or
 * not.
 *
 * \param    gateway             Gateway to check for length
 * \param    robotLength         Length of the robot
 * \param    skeleton            Skeleton of the environment
 * \return   True if the robot can fit on boths sides of the gateway
 */
bool is_gateway_traversable(const Gateway& gateway, double robotLength, const VoronoiSkeletonGrid& skeleton);

/**
 * gateway_boundary_cells extracts the cells in the grid that the gateway boundary passes through.
 *
 * \param[in]    gateway             Gateway to get cells for
 * \param[in]    grid                Grid in which the gateway exists
 * \param[out]   boundaryCells       Cells along the gateway boundary
 */
void gateway_boundary_cells(const Gateway& gateway, const VoronoiSkeletonGrid& grid, CellVector& boundaryCells);

/**
 * gateway_cell_perimeter computes the discretized perimeter of the gateway by considering the adjacency relationship of
 * its cell boundary to determine when two sides vs. one side of a gateway are visible.
 */
float gateway_cell_perimeter(const Gateway& gateway, float metersPerCell);

/**
 * gateway_normal_from_source_cells finds an initial guess for the normal for a gateway based on the source cells
 * associated with it. The selected normal is the normal to the source cells that have the greatest angle separation
 * between them.
 *
 * The function searches through the pairs of source angles to find which have the greatest angular separation and
 * just uses that one for the normal. This function is just an approximation and requires further refinement from other
 * information, so the moderate-to-good answer here is reasonable.
 *
 * \param    cell        Skeleton cell at which the gateway will be located
 * \param    skeleton    Skeleton of the map
 * \return   Normal to the best source cells for the given gateway center cell.
 */
double gateway_normal_from_source_cells(cell_t cell, const VoronoiSkeletonGrid& skeleton);

/**
 * create_gateway_at_cell creates a new gateway by tracing along the orientation orthogonal to the normal in both
 * directions until an occupied or unknown cell is hit.
 *
 * If the provided cell isn't on the reduced skeleton, then a cell along the skeleton will be found for the gateway.
 *
 * Note that this operation can fail if no gateway with both endpoints in the grid or no intersection with the skeleton
 * is encountered.
 *
 * \param    cell            Cell from which to start finding the gateway
 * \param    normal          Direction of the normal to the gateway
 * \param    id              Unique id to assign the gateway
 * \param    skeleton        Skeleton in which to create the gateway
 * \param    maxExtraLength  How much longer can the gateway be than the distance defined by the skeleton?
 * \return   A gateway that runs through cell with a normal in the normal orientation if one exists. none if no gateway
 *   could be created in the desired configuration.
 */
boost::optional<Gateway> create_gateway_at_cell(cell_t cell,
                                                double normal,
                                                int32_t id,
                                                const VoronoiSkeletonGrid& skeleton,
                                                double maxExtraLength = 0.5);

/**
 * gateway_boundary_line_at_cell finds just the gateway boundary endpoints in the map with the provided normal.
 * No other computations are made. This version allows for quickly evaluating what a potential gateway at some location
 * will be like before creating the full gateway with correct normals.
 *
 * \param    cell            Skeleton cell center of the gateway
 * \param    normal          Normal of the gateway
 * \param    skeleton        Grid in which the gateway exists
 * \param    maxExtraLength  How much longer can the gateway be than the distance defined by the skeleton?
 * \return   Boundary line of the gateway, if one exists.
 */
boost::optional<Line<int>> gateway_boundary_line_at_cell(cell_t cell,
                                                         double normal,
                                                         const VoronoiSkeletonGrid& skeleton,
                                                         double maxExtraLength = 0.5);

/**
 * create_gateway_between_sources creates a gateway that runs between the source cells for a skeleton cell. The gateway
 * generated is not guaranteed to to actually pass through the skeleton cell that generated these source cells.
 *
 * The generated gateway (if one exists) will be the gateway closest in length to the optimal length, which should be
 * twice the distance to nearest obstacle for the originating skeleton cell
 *
 * \param    sources         Source cells to create gateway for
 * \param    skeletonCell    Skeleton cell for these source cells
 * \param    id              Id to assign to the created gateway
 * \param    skeleton        Grid in which the gateway will exist
 * \param    isovists        Isovists to use for helping find the gateway normal
 * \return   The gateway created from these sources if one exists.
 */
boost::optional<Gateway> create_gateway_between_sources(const CellVector& sources,
                                                        cell_t skeletonCell,
                                                        int32_t id,
                                                        const VoronoiSkeletonGrid& skeleton,
                                                        const VoronoiIsovistField& isovists);

/**
 * adjust_gateway_for_new_skeleton creates a new gateway from an existing gateway by adjusting the gateway boundary
 * cells and skeleton cells by the minimal amount needed to make them satisfy the gateway properties -- endpoints on
 * occupied/frontier cells and intersecting the skeleton.
 *
 * The adjustment works by finding new boundary cells for the gateway by finding the nearest valid endpoint cell within
 * the given search radius. Then, the two boundaries are connected by a straight line.
 *
 * If the line between the new boundary cells doesn't intersect the skeleton, then no valid gateway exists.
 *
 * \param    gateway             Gateway to be adjusted
 * \param    skeleton            Skeleton in which to create the gateway
 * \param    maxSearchRadius     Maximum number of cells by which the endpoint can be shifted (optional, default = 3)
 * \return   A gateway as close as possible to the provided gateway using the above approaches for creating the new
 *   gateway.
 */
boost::optional<Gateway>
  adjust_gateway_for_new_skeleton(const Gateway& gateway, const VoronoiSkeletonGrid& skeleton, int maxSearchRadius = 3);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_GATEWAY_UTILS_H
