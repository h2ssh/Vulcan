/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     filters.h
* \author   Collin Johnson
* 
* Declaration of various filters to apply to weighted gateways:
* 
*   - filter_out_of_map_gateways : remove all gateways with a endpoint that isn't in the map
*   - filter_generated_gateways : apply filters to generated gateway to ensure their uniqueness
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_GATEWAY_FILTERS_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_GATEWAY_FILTERS_H

#include "hssh/local_topological/area_detection/gateways/weighted_gateway.h"
#include <vector>

namespace vulcan
{
namespace hssh 
{
    
class VornoiSkeletonGrid;

/**
* filter_out_of_map_gateways removes all gateways that are no longer contained in the current map.
* 
* \param    gateways        Gateways to filter
* \param    skeleton        Current skeleton grid of the environment
* \return   All gateways that are still in the boundary of the skeleton.
*/
std::vector<WeightedGateway> filter_out_of_map_gateways(const std::vector<WeightedGateway>& gateways,
                                                        const VoronoiSkeletonGrid& skeleton);

/**
* filter_generated_gateways removes gateways that fail the following conditions:
* 
*   - intersects a higher weighted gateway
*   - is similar to a higher weighted gateway
* 
* \param    gateways        Gateways to filter
* \param    skeleton        Current skeleton grid of the environment
* \return   All gateways that pass all the filters.
*/
std::vector<WeightedGateway> filter_generated_gateways(const std::vector<WeightedGateway>& gateways,
                                                        const VoronoiSkeletonGrid& skeleton);


/**
* is_gateway_boundary_valid checks if the gateway has a valid boundary in skeleton.
* 
* A boundary is valid if:
*   - both endpoints are on obstacle cells
*   - the center is on a skeleton cell
*   - all boundary cells are stored
* 
* \param    g           Gateway to check for validity
* \param    skeleton    Skeleton grid in which the gateway needs to exist
* \return   True if the gateway boundary is still valid per the specified conditions.
*/
bool is_gateway_boundary_valid(const Gateway& g, const VoronoiSkeletonGrid& skeleton);

} // namespace hssh 
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_GATEWAY_FILTERS_H
