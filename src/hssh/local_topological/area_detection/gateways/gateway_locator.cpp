/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     visibility_gradient_locator.cpp
 * \author   Collin Johnson
 *
 * Definition of GatewayLocator.
 */

#include "hssh/local_topological/area_detection/gateways/gateway_locator.h"
#include "hssh/local_topological/area_detection/gateways/endpoint_validator.h"
#include "hssh/local_topological/area_detection/gateways/filters.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "hssh/local_topological/area_detection/gateways/generator.h"
#include "hssh/local_topological/debug_info.h"
#include "utils/algorithm_ext.h"
#include <algorithm>
#include <cassert>
#include <iostream>

// #define DEBUG_SOURCES
// #define DEBUG_POTENTIAL_MAXIMA
// #define DEBUG_MAXIMA

namespace vulcan
{
namespace hssh
{

const double kTransitionWeight = 1000000.0;
const double kExitedWeight = 100000.0;

std::vector<Gateway> weighted_to_normal(const std::vector<WeightedGateway>& weighted);


GatewayLocator::GatewayLocator(const gateway_locator_params_t& params, const std::string& mapName)
: params_(params)
, generator_(create_gateway_generator(params.gatewayGeneratorType, mapName, params.generatorParams))
{
}


GatewayLocator::GatewayLocator(std::unique_ptr<GatewayGenerator>&& generator) : generator_(std::move(generator))
{
}


GatewayLocator::~GatewayLocator(void)
{
    // For std::unique_ptr
}


void GatewayLocator::locateGateways(const VoronoiSkeletonGrid& grid, const VoronoiIsovistField& isovistField)
{
    priorGateways_ = filter_out_of_map_gateways(finalGateways_, grid);
    bool haveTransition = adjustPriorsForNewMap(grid);

    EndpointValidator validator(grid);   //, params_.validatorParams);

    generatedGateways_ = generator_->generateGateways(priorGateways_, isovistField, grid, validator);
    filteredGateways_ = filter_generated_gateways(generatedGateways_, grid);

    if (haveTransition) {
        bool haveFoundTransition = false;
        for (auto& gwy : filteredGateways_) {
            haveFoundTransition |= gwy.isTransition;
        }

        assert(haveFoundTransition);
    }
}


bool GatewayLocator::isTransitionGatewayValid(void) const
{
    return isTransitionValid_;
}


void GatewayLocator::discardMostRecentTransitionGateway(void)
{
    // Find if a gateway is a transition and nuke it if it is.
    utils::erase_remove_if(finalGateways_, [](const WeightedGateway& g) {
        return g.isTransition;
    });

    // If no transition, then a valid transition
    isTransitionValid_ = true;
}


void GatewayLocator::assignFinalGateways(const std::vector<Gateway>& gateways)
{
    // For the final gateways, remove any gateways that aren't in this final set of gateways
    finalGateways_ = filteredGateways_;
    utils::erase_remove_if(finalGateways_, [&gateways](const WeightedGateway& g) {
        return !utils::contains(gateways, g.gateway);
    });

    for (auto& gwy : gateways) {
        bool inFinal = utils::contains_if(finalGateways_, [&gwy](const auto& finalGwy) {
            return gwy.isSimilarTo(finalGwy.gateway);
        });
        if (!inFinal) {
            std::cout << "Added final gateway: " << gwy.boundary() << '\n';
            finalGateways_.emplace_back(WeightedGateway{gwy, 1.0, false});
        }
    }
}


void GatewayLocator::assignTransitionGateway(const Gateway& transition)
{
    // Only a single transition gateway exists at any given time. Remove any previous transitions.
    int numErased = utils::erase_remove_if(finalGateways_, [](const WeightedGateway& g) {
        return g.isTransition;
    });

    // Add the new transition
    finalGateways_.push_back(WeightedGateway{transition, kTransitionWeight, true});

    std::cout << "Saved transition gateway: " << transition << ". Erased " << numErased << " prior transitions\n";
}


void GatewayLocator::assignExitedAreaGateways(const std::vector<Gateway>& gateways)
{
    for (auto& gwy : gateways) {
        auto finalIt = std::find_if(finalGateways_.begin(), finalGateways_.end(), [&gwy](const auto& finalGwy) {
            return gwy.isSimilarTo(finalGwy.gateway);
        });

        if ((finalIt != finalGateways_.end()) && !finalIt->isTransition) {
            finalIt->gateway = gwy;   // update the gateway position in case it isn't the same
            finalIt->weight = kExitedWeight;
        } else {
            finalGateways_.push_back(WeightedGateway{gwy, kExitedWeight, false});
        }
    }
}


std::vector<Gateway> GatewayLocator::getGateways(void) const
{
    return weighted_to_normal(filteredGateways_);
}


void GatewayLocator::clearGateways(void)
{
    priorGateways_.clear();
    generatedGateways_.clear();
    filteredGateways_.clear();
    finalGateways_.clear();
}


gateway_debug_info_t GatewayLocator::getDebugInfo(void) const
{
    gateway_debug_info_t debugInfo;
    debugInfo.intermediateHypotheses.push_back(std::make_pair("Prior", weighted_to_normal(priorGateways_)));
    debugInfo.intermediateHypotheses.push_back(std::make_pair("Initial", weighted_to_normal(generatedGateways_)));
    debugInfo.intermediateHypotheses.push_back(std::make_pair("Hypotheses", weighted_to_normal(filteredGateways_)));
    debugInfo.intermediateHypotheses.push_back(std::make_pair("Final", weighted_to_normal(finalGateways_)));
    return debugInfo;
}


bool GatewayLocator::adjustPriorsForNewMap(const VoronoiSkeletonGrid& skeleton)
{
    bool haveTransition = false;

    // Transform each gateway that doesn't have a valid boundary into a gateway with a valid boundary
    for (auto& g : priorGateways_) {
        if (!is_gateway_boundary_valid(g.gateway, skeleton)) {
            // Try to create a new gateway boundary
            auto newGateway = adjust_gateway_for_new_skeleton(g.gateway, skeleton, 15);
            // If a new boundary was successfully found, replace the old gateway boundary with the new one
            if (newGateway) {
                g.gateway = *newGateway;

                if (g.isTransition) {
                    std::cout << "INFO: GatewayLocator: Adjusted transition gateway:" << g.gateway
                              << " skel:" << g.gateway.skeletonCell() << '\n';
                    isTransitionValid_ = true;
                    haveTransition = true;
                }
            } else {
                if (g.isTransition) {
                    std::cerr << "WARNING: GatewayLocator: Failed to adjust previous transition gateway:" << g.gateway
                              << " skel:" << g.gateway.skeletonCell() << '\n';
                    isTransitionValid_ = false;
                }
            }
        } else if (g.isTransition) {
            std::cout << "INFO: GatewayLocator: Found transition gateway:" << g.gateway
                      << " skel:" << g.gateway.skeletonCell() << '\n';
            isTransitionValid_ = true;
            haveTransition = true;
        }
    }

    // Erase any gateways that still aren't valid after attempting to fix the boundaries
    utils::erase_remove_if(priorGateways_, [&skeleton](const WeightedGateway& g) {
        return !is_gateway_boundary_valid(g.gateway, skeleton);
    });

    return haveTransition;
}


std::vector<Gateway> weighted_to_normal(const std::vector<WeightedGateway>& weighted)
{
    std::vector<Gateway> gateways;
    for (auto& w : weighted) {
        gateways.emplace_back(w.gateway);
    }

    return gateways;
}

}   // namespace hssh
}   // namespace vulcan
