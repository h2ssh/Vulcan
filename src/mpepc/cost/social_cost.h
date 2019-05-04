/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     social_cost.h
* \author   Collin Johnson
*
* Declaration of a cost function for costs associated with learned social norms in the environment.
*/

#ifndef MPEPC_COSTS_LEARNED_NORM_COST_H
#define MPEPC_COSTS_LEARNED_NORM_COST_H

#include <mpepc/social/topo_agent.h>
#include <mpepc/social/topo_situation.h>
#include <mpepc/types.h>
#include <system/message_traits.h>

namespace vulcan
{
namespace hssh { class LocalTopoRoute; }
namespace mpepc
{

class CostMap;
struct dynamic_object_trajectory_t;

/**
* learned_norm_cost_params_t defines the learned model for navigating through an environment.
*/
struct learned_norm_cost_params_t
{
    TopoSituationResponse defaultResponsePlace;     ///< Response to use if a previously-unseen place situation arises
    TopoSituationResponse defaultResponsePath;      ///< Response to use if a previously-unseen path situation arises
    std::vector<TopoSituationResponse> responses;   ///< Learned responses to different topological situations
    int numLateralBins;                             ///< Number of lateral bins for path situations
};

/**
* learned_norm_debug_info_t contains debug info used to generate the learned norms for the current topo route.
*/
struct learned_norm_info_t
{
    std::vector<topo_agent_t> agents;               ///< Topological agents moving through the environment
    std::vector<PathSituation> pathSituations;      ///< Path situations identified for the current route
    std::vector<PlaceSituation> placeSituations;    ///< Place situations identified for the current route
};

/**
* learned_norm_cost computes the social-norm-based costs associated with a particular topological route through
* the environment.
*
* \param[in]    route           Topological route to the goal
* \param[in]    params          Parameters controlling the social norm
* \param[in]    env             Environment robot is operating in
* \param[in]    objects         Objects detected in the environment
* \param[out]   costs           Cost map to add social norm information to
* \param[out]   norms           Location to store the identified norms and situations
*/
void learned_norm_cost(const hssh::LocalTopoRoute& route,
                       const learned_norm_cost_params_t& params,
                       const planning_environment_t& env,
                       const std::vector<dynamic_object_trajectory_t>& objects,
                       CostMap& costs,
                       learned_norm_info_t& norms);


// Serialization support
template <class Archive>
void serialize(Archive& ar, learned_norm_cost_params_t& params, const unsigned int version)
{
    ar(params.defaultResponsePlace,
       params.defaultResponsePath,
       params.responses,
       params.numLateralBins);
}

template <class Archive>
void serialize(Archive& ar, learned_norm_info_t& debug)
{
    ar(debug.agents,
       debug.pathSituations,
       debug.placeSituations);
}

} // namespace mpepc
} // namespace vulcan

DEFINE_DEBUG_MESSAGE(mpepc::learned_norm_info_t, ("DEBUG_LEARNED_NORM_INFO"));

#endif // MPEPC_COSTS_LEARNED_NORM_COST_H
