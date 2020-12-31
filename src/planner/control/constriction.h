/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     constriction.h
 * \author   Collin Johnson
 *
 * Declaration of Constriction, find_route_constrictions, and find_map_constrictions.
 */

#ifndef PLANNER_CONTROL_CONSTRICTION_H
#define PLANNER_CONTROL_CONSTRICTION_H

#include "core/position.h"
#include <vector>

namespace vulcan
{
namespace hssh
{
class VoronoiSkeletonGrid;
}
namespace planner
{

class Constriction;
using Constrictions = std::vector<Constriction>;

/**
 * find_map_constrictions finds the constrictions in the current map. Constrictions occur along edges in the Voronoi
 * skeleton. A constriction is defined as the local minima along each edge where the distance between the obstacles
 * falls below some provided threshold. A constriction is bounded by where the distance between the obstacle goes above
 * the maximum constriction width. The constriction has a minimum that specifies the closest point between the
 * obstacles.
 *
 * \param    skeleton                    Skeleton of the environment
 * \param    maxConstrictionWidth        Maximum width of a constriction to worry about
 */
Constrictions find_map_constrictions(const hssh::VoronoiSkeletonGrid& skeleton, double maxConstrictionWidth);

/**
 * find_route_constrictions finds the constrictions along the route from the start to the goal. The route taken from
 * start to goal is assumed to follow, approximately, the Voronoi graph because the optimal safe route when dealing with
 * constrictions is to follow the midline. The estimated route to be followed is determined using an A* search of the
 * Voronoi graph with the following cost:
 *
 *   cost = distance * (1 + constriction_weight*constriction_cost)
 *   constriction_cost = max_constriction_width - constriction_width
 *
 * Thus, there is a tradeoff between the constrictions and the distance. A small detour to avoid a constriction is
 * acceptable, but a large detour is not. Each constriction encountered adds to the cost. In most environments, there
 * won't be any constrictions except when going through doors or between desks in an office.
 *
 * \see find_map_constrictions for details on what counts as a constriction.
 *
 * \param    start                   Starting position of the route
 * \param    goal                    Goal position of the route
 * \param    skeleton                Voronoi skeleton of the environment
 * \param    constrictions           Constrictions in the environment
 * \param    minTraversableWidth     The minimum distance between obstacles that the robot can possibly traverse
 * \return   Constrictions found along the route in the order in which they occur along the route.
 */
Constrictions find_route_constrictions(position_t start,
                                       position_t goal,
                                       const hssh::VoronoiSkeletonGrid& skeleton,
                                       const Constrictions& constrictions,
                                       double minTraversableWidth);

/**
 * Constriction represents a constriction in the map. A constriction is defined as the narrowest distance between two
 * objects along the Voronoi graph. Every constriction has the following:
 *
 *   - a position of the minimum
 *   - a cell for the minimum
 *   - two boundary positions
 *   - a width
 *   - a normal direction
 *
 * The position defines the location of the minimum of the constriction in global coordinates.
 * The map cell defines the cell associated with the constriction in the current grid map of the environment.
 * The boundary positions specify where the obstacles are first fall below the threshold.
 * The width is the distance between the two obstacles at the constriction.
 * The normal direction is the normal of the line connecting the two objects closest to the constriction point.
 */
class Constriction
{
public:
    using Cell = Point<int>;

    /**
     * Constructor for Constriction.
     *
     * \param    minimum         Cell at the minimum distance between the obstacles
     * \param    boundary        Cells at the boundary of where the constriction begins
     * \param    skeleton        Skeleton graph in which the constriction exists
     */
    Constriction(Cell minimum, std::pair<Cell, Cell> boundary, hssh::VoronoiSkeletonGrid& skeleton);

    /**
     * position retrieves the position of the constriction.
     */
    position_t position(void) const { return position_; }

    /**
     * cell retrieves the cell of the constriction in the map in which it was found.
     */
    Point<int> cell(void) const { return cell_; }

    /**
     * boundary retrieves the position of the boundary of the constriction.
     */
    std::pair<position_t, position_t> boundary(void) const { return boundary_; }

    /**
     * width retreives the width of the constriction, which is the distance between the two closest obstacles.
     */
    double width(void) const { return width_; }

    /**
     * normalDirection retrieves the direction of the normal of the constriction in radians, [-pi,pi].
     */
    double normalDirection(void) const { return normal_; }

private:
    std::pair<position_t, position_t> boundary_;
    position_t position_;
    Point<int> cell_;
    double width_;
    double normal_;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_CONTROL_CONSTRICTION_H
