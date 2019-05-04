/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     isovist_maximum_gateway_generator.h
* \author   Collin Johnson
*
* Declaration of GatewayGenerator interface and create_isovist_maximum_gateway_generator factory.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_MAXIMUM_GATEWAY_GENERATOR_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_MAXIMUM_GATEWAY_GENERATOR_H

#include <hssh/local_topological/area_detection/gateways/weighted_gateway.h>
#include <memory>
#include <string>
#include <vector>

namespace vulcan
{
namespace hssh
{

class EndpointValidator;
class Gateway;
class GatewayGenerator;
class VoronoiSkeletonGrid;
struct gateway_generator_params_t;

/**
* create_isovist_maximum_gateway_generator creates an instance of a subclass of GatewayGenerator using
* the provided type string and parameters stored in the config file.
*
* \param    type        Type of generator to create
* \param    mapName     Name of the map in which the robot is operating
* \param    params      Config file containing configuration for the subclass
* \pre  type is a valid descriptor for an GatewayGenerator subclass.
* \return   Instance of GatewayGenerator.
*/
std::unique_ptr<GatewayGenerator> create_gateway_generator(const std::string& type,
                                                           const std::string& mapName,
                                                           const gateway_generator_params_t& params);

/**
* GatewayGenerator generates Gateways from isovist_local_maximum_t. One Gateway exists for each local maximum.
* The generation process for the isovists guarantees that the gateway crosses the skeleton in the direction of the gradient.
*
* When generating the gateways, gateways for all maxima are generated at the same time to ensure they are consistent with one
* another. In particular, gateways can't be intersecting, so if they intersect, then get rid of them.
*/
class GatewayGenerator
{
public:

    virtual ~GatewayGenerator(void) { }

    /**
    * generateGateways generates gateways for a given skeleton and isovist field. All generated gateways should be 
    * valid per the provided endpoint validator.
    * 
    * \param    priorGateways       Final gateways from the previous gateway generation
    * \param    isovists            Isovists for the skeleton
    * \param    skeleton            Skeleton for the current map
    * \param    validator           Validator to use for determining valid gateways
    * \return   A collection of gateways with associated weights for the current grid.
    */
    virtual std::vector<WeightedGateway> generateGateways(const std::vector<WeightedGateway>& priorGateways,
                                                          const VoronoiIsovistField& isovists,
                                                          const VoronoiSkeletonGrid& skeleton,
                                                          const EndpointValidator& validator) = 0;

};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISOVIST_MAXIMUM_GATEWAY_GENERATOR_H
