/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topo_agent.h
* \author   Collin Johnson
*
* Declaration of topo_agent_t and find_topo_agents function.
*/

#ifndef MPEPC_TOPO_AGENT_H
#define MPEPC_TOPO_AGENT_H

#include <mpepc/simulator/dynamic_object_trajectory.h>
#include <boost/optional.hpp>
#include <vector>

namespace vulcan
{
namespace hssh { class LocalTopoMap; }
namespace tracker { class DynamicObjectCollection; }
namespace mpepc
{

/**
* topo_agent_t represents an agent's intention as a topological action. The only action currently supported is
* a transition to a new area. The current area and the gateway are provided with the agent.
*/
struct topo_agent_t
{
    float radius;
    dynamic_object_state_t state;   // only position and velocity matter
    int areaId;
    int gatewayId;
};

// Equal if same position, areaId, and gatewayId
bool operator==(const topo_agent_t& lhs, const topo_agent_t& rhs);
bool operator!=(const topo_agent_t& lhs, const topo_agent_t& rhs);

/**
* find_topo_agents finds all agents in the environment whose estimated goal is a topological action in
* the environment. The goal estimate comes from the DynamicObject itself. If the agent is in an area and the goal
* is a gateway, then it is a topological agent. If the agent doesn't have a topological goal, then it won't be
* transformed into a topo_agent_t.
*
* \param    agents          Estimated state of agents in the environment
* \param    topoMap         Topological map of the environment
* \return   All topo_agent_ts found in the provided collection of agents.
*/
std::vector<topo_agent_t> find_topo_agents(const std::vector<dynamic_object_trajectory_t>& agents,
                                           const hssh::LocalTopoMap& topoMap);
std::vector<topo_agent_t> find_topo_agents(const tracker::DynamicObjectCollection& agents,
                                           const hssh::LocalTopoMap& topoMap);

/**
* Create a topo_agent_t from a DynamicObject.
*
* \param    object          Object to convert to an agent
* \param    topoMap         Map in which the object exists
* \return   An agent representation of the object. If the object isn't in the map or an area in the map
*/
boost::optional<topo_agent_t> convert_to_topo_agent(const tracker::DynamicObject::ConstPtr& object,
                                                    const hssh::LocalTopoMap& topoMap);

// Serialization support
template <class Archive>
void serialize(Archive& ar, topo_agent_t& agent)
{
    ar(agent.state,
       agent.areaId,
       agent.gatewayId);
}

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_TOPO_AGENT_H
