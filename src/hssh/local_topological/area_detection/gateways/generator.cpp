/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     generator.cpp
 * \author   Collin Johnson
 *
 * Definition of create_gateway_generator factory.
 */

#include "hssh/local_topological/area_detection/gateways/generator.h"
#include "hssh/local_topological/area_detection/gateways/classifier_based_generator.h"
#include "hssh/local_topological/area_detection/gateways/isovist_orientation_gateway_generator.h"
#include "hssh/local_topological/area_detection/gateways/isovist_voronoi_gateway_generator.h"
#include "hssh/local_topological/params.h"

namespace vulcan
{
namespace hssh
{

std::unique_ptr<GatewayGenerator> create_gateway_generator(const std::string& type,
                                                           const std::string& mapName,
                                                           const gateway_generator_params_t& params)
{
    if (type == kIsovistVoronoiGatewayGeneratorType) {
        return std::make_unique<IsovistVoronoiGatewayGenerator>(params.voronoiParams);
    }
    // map name needed for classifier, so if it isn't available fall back on the next-best generator
    else if ((type == kIsovistOrientationGatewayGeneratorType) || mapName.empty()) {
        return std::make_unique<IsovistOrientationGatewayGenerator>(params.orientationParams);
    } else if (type == kClassifierBasedGeneratorType) {
        return std::make_unique<ClassifierBasedGenerator>(params.classifierParams, mapName);
    }

    std::cerr << "ERROR: create_isovist_maximum_gateway_generator: Attempted to create unknown generator type:" << type
              << std::endl;
    assert(false);

    return std::unique_ptr<GatewayGenerator>();
}

}   // namespace hssh
}   // namespace vulcan
