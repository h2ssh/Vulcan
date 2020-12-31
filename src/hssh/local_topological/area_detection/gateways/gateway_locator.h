/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     visibility_gradient_locator.h
 * \author   Collin Johnson
 *
 * Declaration of GatewayLocator.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_VISIBILITY_GRADIENT_LOCATOR_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_VISIBILITY_GRADIENT_LOCATOR_H

#include "hssh/local_topological/area_detection/gateways/weighted_gateway.h"
#include "hssh/local_topological/params.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

struct gateway_debug_info_t;
class GatewayGenerator;

/**
 * GatewayLocator
 */
class GatewayLocator
{
public:
    /**
     * Constructor for GatewayLocator.
     *
     * \param    params          Parameters controlling the behavior of the locator
     * \param    mapName         Name of the map robot is operating in
     */
    GatewayLocator(const gateway_locator_params_t& params, const std::string& mapName);

    /**
     * Constructor for GatewayLocator.
     *
     * Wrap a GatewayLocator around an instance of GatewayGenerator.
     *
     * \param    generator           Generator instance to use
     */
    GatewayLocator(std::unique_ptr<GatewayGenerator>&& generator);

    /**
     * Destructor for GatewayLocator.
     */
    ~GatewayLocator(void);

    /**
     * locateGateways finds gateways in the map using the Voronoi skeleton and isovists.
     */
    void locateGateways(const VoronoiSkeletonGrid& grid, const VoronoiIsovistField& isovistField);

    /**
     * isTransitionGatewayValid checks if the most recently assigned transition gateway is still a valid gateway
     * after the most recent update to the gateways via locatedGateways.
     */
    bool isTransitionGatewayValid(void) const;

    /**
     * discardMostRecentTransitionGateway tells the locator to discard any previous state it was maintaining about
     * a particular transition gateway because it was deemed unimportant.
     */
    void discardMostRecentTransitionGateway(void);

    /**
     * assignFinalGateways feeds back information on the gateways that ended up being selected during the classification
     * process. The locator use these gateways to ensure that a similar gateway appears during the next update via
     * locateGateways.
     */
    void assignFinalGateways(const std::vector<Gateway>& gateways);

    /**
     * assignTransitionGateway provides a transition gateway to the locator. Transition gateways take precedence over
     * all other types of gateways, as their disappearance can cause serious problems for the event detection.
     *
     * All transition gateways should remain in the final gateway set until the gateway is no longer in the map.
     *
     * \param    transition          Transition gateway to incorporate into the locator's state
     */
    void assignTransitionGateway(const Gateway& transition);

    /**
     * assignExitedAreaGateways specifies the gateways associated with the area that was just exited by the robot.
     * These gateways should appear in the set of gateways on the next update to ensure that exited area always exists
     * given new information in the map. If an exited area gateway can't be created, then the exited area was likely
     * invalid.
     *
     * Maintaining the exited gatweays also make it more likely to detect the same area if the exited area doesn't
     * leave the LPM bounds while the robot is navigating.
     *
     * \param    gateways            Gateways associated with the exited area
     */
    void assignExitedAreaGateways(const std::vector<Gateway>& gateways);

    /**
     * getGateways retrieves the gateways created on the most recent update.
     */
    std::vector<Gateway> getGateways(void) const;

    /**
     * clearGateways clears all gateways from the locator and resets its state.
     */
    void clearGateways(void);

    /**
     * getDebugInfo retrieves debugging information created during the gateway location process.
     *
     * See debug_info.h for details.
     */
    gateway_debug_info_t getDebugInfo(void) const;

private:
    gateway_locator_params_t params_;
    std::unique_ptr<GatewayGenerator> generator_;
    std::vector<WeightedGateway> priorGateways_;
    std::vector<WeightedGateway> generatedGateways_;
    std::vector<WeightedGateway> filteredGateways_;
    std::vector<WeightedGateway> finalGateways_;

    bool isTransitionValid_ = true;

    bool adjustPriorsForNewMap(const VoronoiSkeletonGrid& skeleton);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_VISIBILITY_GRADIENT_LOCATOR_H
