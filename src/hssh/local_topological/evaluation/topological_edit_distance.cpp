/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_edit_distance.cpp
* \author   Collin Johnson
*
* Definition of topological_edit_distance function.
*/

#include "hssh/local_topological/evaluation/topological_edit_distance.h"
#include "hssh/local_topological/local_topo_route.h"
#include "utils/edit_distance.h"
#include <sstream>

namespace vulcan
{
namespace hssh
{

std::string route_to_route_string(const LocalTopoRoute& route, double routeUnitDistance, bool useRaw);
char area_type_to_char(const AreaType& type);


topo_edit_dist_t topological_edit_distance(const LocalTopoRoute& routeA,
                                           const LocalTopoRoute& routeB,
                                           double routeUnitDistance)
{
    utils::edit_distance_weights_t topoWeight;
    topoWeight.substitution = 10000.0;
    topoWeight.insertion = 1.0;
    topoWeight.deletion = 1.0;

    topo_edit_dist_t dists;

    // Calculate the normalized distance by using the specified routeUnitDistance
    std::string routeAStr = route_to_route_string(routeA, routeUnitDistance, false);
    std::string routeBStr = route_to_route_string(routeB, routeUnitDistance, false);
    dists.topological = utils::edit_distance(routeAStr, routeBStr, topoWeight, false) * 100.0 / routeBStr.length();

    utils::edit_distance_weights_t routeWeight;
    topoWeight.substitution = 1.0;
    topoWeight.insertion = 1.0;
    topoWeight.deletion = 1.0;

    // Now use the raw route.
    routeAStr = route_to_route_string(routeA, 1e7, true);
    routeBStr = route_to_route_string(routeB, 1e7, true);
    dists.route = utils::edit_distance(routeAStr, routeBStr, routeWeight, false);

    return dists;
}


std::string route_to_route_string(const LocalTopoRoute& route, double routeUnitDistance, bool useRaw)
{
    std::string routeString;

    for(const LocalTopoRouteVisit& visit : route)
    {
        if(useRaw)
        {
            routeString += area_type_to_char(visit.area().type());
        }
        else
        {
            std::fill_n(std::back_inserter(routeString),
                        (visit.distance() / routeUnitDistance) + 1,
                        area_type_to_char(visit.area().type()));
        }
    }

    return routeString;
}


char area_type_to_char(const AreaType& type)
{
    switch(type)
    {
    case AreaType::decision_point:
        return 'j';

    case AreaType::destination:
        return 'd';

    case AreaType::path_segment:
        return 'p';

    default:
        std::cerr << "ERROR: topological_edit_distance: Visited area of unknown type:" << type << '\n';
        return 'f';
    }

    return 'f';
}

} // namespace hssh
} // namespace vulcan
