/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     endpoint_validator.h
* \author   Collin Johnson
* 
* Definition of EndpointValidator.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ENDPOINT_VALIDATOR_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ENDPOINT_VALIDATOR_H

#include <hssh/local_topological/area_detection/voronoi/frontiers_set.h>
#include <hssh/local_topological/gateway.h>
#include <hssh/local_topological/params.h>

namespace vulcan
{
namespace hssh
{

/**
* EndpointValidator is a simple class to valid gateway endpoints.
*
* A valid endpoint cannot:
*   - be too far from the end of a frontier
*   - be in free space
*
* The parameters for EndpointValidator are:
*
*   [EndpointValidatorParameters]
*   max_endpoint_dist_from_frontier_edge_m = maximum distance an endpoint can be from the edge of a frontier for it to be considered valid
*/
class EndpointValidator
{
public:

    /**
    * Constructor for EndpointValidator.
    *
    * \param    grid            Grid in which the endpoints will be appearing
    * \param    params          Parameters for the endpoint validation
    */
//     EndpointValidator(const VoronoiSkeletonGrid& grid, const endpoint_validator_params_t& params)
    EndpointValidator(const VoronoiSkeletonGrid& grid)
//     : frontiers(grid, params.maxEndpointDistFromEdge)
    : grid_(&grid)
    {
    }

    /**
    * isValidGateway checks if both endpoints of a gateway are valid.
    *
    * \param    gateway         Gateway to check for validity
    * \return   isValidEndpoint(gateway.cellBoundary().a) && isValidEndpoint(gateway.cellBoundary().b)
    */
    bool isValidGateway(const Gateway& gateway) const
    {
        return isValidEndpoint(gateway.cellBoundary().a) && isValidEndpoint(gateway.cellBoundary().b);
    }

    /**
    * isValidEndpoint checks to see if an endpoint at the given cell is valid given the conditions described in the class description.
    *
    * \param    endpoint        Possible endpoint to be checked for validity
    * \return   True if the endpoint is valid.
    */
    bool isValidEndpoint(cell_t endpoint) const
    { 
        return (grid_->getClassification(endpoint.x, endpoint.y) & (SKELETON_CELL_OCCUPIED | SKELETON_CELL_FRONTIER));
//         return !frontiers.isFarFromOccupied(endpoint)
//             && (grid_->getClassification(endpoint.x, endpoint.y) & (SKELETON_CELL_OCCUPIED | SKELETON_CELL_FRONTIER));
    }
    
private:
    
//     FrontiersSet frontiers;
    const VoronoiSkeletonGrid* grid_;
};
    
}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ENDPOINT_VALIDATOR_H
