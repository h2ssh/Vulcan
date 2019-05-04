/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_edit_distance.h
* \author   Collin Johnson
*
* Declaration of topological_edit_distance function for finding the edit distance, based on area types, between two
* LocalTopoRoutes.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_EVALUATION_TOPOLOGICAL_EDIT_DISTANCE_H
#define HSSH_LOCAL_TOPOLOGICAL_EVALUATION_TOPOLOGICAL_EDIT_DISTANCE_H

namespace vulcan
{
namespace hssh
{

class LocalTopoRoute;

/**
* topo_edit_dist_t defines the two types of topological edit distance. The original topological edit distance is
* a normalized distance that spaces samples at even intervals along the path along the Voronoi skeleton and then
* divides by the total metric length. The route distance just takes the two routes and compares them
* directly with edit distance with no normalization. A modification cost of 1 is used for the route distance so that
* it directly measures the number of changes required to the route, rather than considering insertions and deletions
* separately.
*/
struct topo_edit_dist_t
{
    double topological;
    double route;
};

/**
* topological_edit_distance calculates the topological edit distance between two routes through a LocalTopoMap. The
* routes can be created in any way and simply describe some trajectory through the environment. The routes can come from
* the sequence of areas detected by the robot as it drives or by a path planner considered the LocalTopoMap of a global
* metric map.
*
* The topological edit distance is defined to be the edit distance using only insertions or deletions between the
* sequence of areas included in the two routes. By converting the routes into strings, where path segment = p, dest = d,
* and decision point = j (for junction), the topological edit distance can be calculated using the standard definition
* of the edit distance between two strings using a weight of 1.0 for insertions and deletions and a weight of infinity
* for substitutions.
*
* An additional parameter controls how the distance the robot travels within an area is incorporated into the edit
* distance calculation. The string defining the route will add one character for each unit of distance traveled within
* a given area type. For example, if the robot drives 5m within a path segment and the routeUnitDistance is 0.5m, then
* 11 'p's will be added to the route string, one for entering and one for each additional 0.5m spent in the path
* segment.
*
* \param    routeA                  One of the routes to compare
* \param    routeB                  The other route to compare
* \param    routeUnitDistance       Unit of distance for converting the route into a route string
* \return   The various measures of topological edit distance as defined in topo_edit_dists_t.
*/
topo_edit_dist_t topological_edit_distance(const LocalTopoRoute& routeA,
                                           const LocalTopoRoute& routeB,
                                           double routeUnitDistance);

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_EVALUATION_TOPOLOGICAL_EDIT_DISTANCE_H
