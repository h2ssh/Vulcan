/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     debug_info.h
* \author   Collin Johnson
*
* Declaration of local_area_debug_info_t.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_DEBUG_INFO_H
#define HSSH_LOCAL_TOPOLOGICAL_DEBUG_INFO_H

#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include "hssh/local_topological/area_detection/labeling/area_proposal.h"
#include "hssh/local_topological/area_detection/labeling/debug.h"
#include "system/message_traits.h"
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/utility.hpp>

namespace vulcan
{
namespace hssh
{

/**
* local_area_debug_info_t contains debugging information relevant to the classification
* of areas within the robot's local surround.
*/
struct local_area_debug_info_t
{
    AreaGraph                    graph;
    std::vector<DebugHypothesis> maximumLikelihoodHypotheses;
    std::vector<DebugHypothesis> unnormalizedHypotheses;
    std::vector<DebugHypothesis> boostingHypotheses;
    std::vector<DebugHypothesis> isovistHypotheses;
};

/**
* gateway_debug_info_t contains debugging information relevant to the generation of gateways.
*
* The intermediateHypotheses contain all valid gateway hypotheses at each intermediate step of the GatewayGenerator
* algorithm. The string is a text description of the which step the hypotheses come from.
*/
struct gateway_debug_info_t
{
    using GatewayVec = std::vector<Gateway>;

    std::vector<Gateway> gateways;
    std::vector<std::pair<std::string, GatewayVec>> intermediateHypotheses;
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, local_area_debug_info_t& info)
{
    ar (info.graph,
        info.maximumLikelihoodHypotheses,
        info.unnormalizedHypotheses,
        info.boostingHypotheses,
        info.isovistHypotheses);
}

template <class Archive>
void serialize(Archive& ar, gateway_debug_info_t& info)
{
    ar (info.gateways,
        info.intermediateHypotheses);
}

} // namespace hssh
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(hssh::local_area_debug_info_t, ("DEBUG_HSSH_LOCAL_AREA_INFO"))
DEFINE_DEBUG_MESSAGE(hssh::gateway_debug_info_t, ("DEBUG_HSSH_GATEWAY_INFO"))

#endif // HSSH_LOCAL_TOPOLOGICAL_DEBUG_INFO_H
