/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     interaction.h
* \author   Collin Johnson
*
* Declaration of interacting_agent_t and interaction_t types for representing state around the robot at any given time.
*
* Function: find_interactions creates the interaction between the robot and dynamic objects around the robot.
*/

#ifndef MPEPC_EVALUATION_INTERACTION_H
#define MPEPC_EVALUATION_INTERACTION_H

#include "mpepc/social/topo_agent.h"
#include "mpepc/social/topo_situation.h"
#include "core/motion_state.h"
#include "tracker/object_state.h"
#include <boost/optional.hpp>
#include <vector>

namespace vulcan
{
namespace hssh { class LocalTopoMap; }
namespace mpepc
{

class MPEPCLog;

/**
* interaction_t describes an interaction between the robot and agents around the robot at a particular time.
*/
struct interaction_t
{
    int64_t timestamp;

    int areaId;
    pose_t pose;
    velocity_t velocity;

    topo_agent_t robotAgent;
    std::vector<topo_agent_t> agents;

    boost::optional<PathSituation> pathSituation;
    boost::optional<PlaceSituation> placeSituation;
};

/**
* find_interactions iterates through a full log and extracts the smallest data necessary to evaluate interactions
* between the robot and agents around it.
*
* An interaction is created for each DynamicObjectCollection in the log.
*
* \param    log                 Log to pull interactions from
* \param    topoMap             Map in which the log was gathered
* \param    numLateralBins      Number of bins to divide the lateral distance into
* \param    maxDistance         Maximum distance from the robot for an interaction
* \param    ignoreConeAngle     Width of the ignore cone behind the robot
* \return   The interactions that occur within the specified boundary around the robot.
*/
std::vector<interaction_t> find_interactions(MPEPCLog& log,
                                             const hssh::LocalTopoMap& topoMap,
                                             int numLateralBins,
                                             double maxDistance,
                                             double ignoreConeAngle);

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_EVALUATION_INTERACTION_H
