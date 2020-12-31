/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     consumers.h
 * \author   Collin Johnson
 *
 * Declaration of GoalInputConsumer and GoalOutputConsumer interfaces.
 */

#ifndef PLANNER_GOAL_CONSUMERS_H
#define PLANNER_GOAL_CONSUMERS_H

#include <string>
#include <vector>

namespace vulcan
{
namespace hssh
{
class TopologicalMap;
}

namespace planner
{

struct goal_route_command_message_t;
struct goal_debug_info_t;

class DecisionTargetSequence;
class DecisionProgress;
class GoalTarget;
class GoalRoute;
class GoalProgress;

/**
 * GoalInputConsumer is an interface to be implemented by all classes that will be handling
 * data for the module. The data are arriving from external modules.
 */
class GoalInputConsumer
{
public:
    virtual ~GoalInputConsumer(void) { }

    virtual void handleData(const hssh::TopologicalMap& topoMap, const std::string& channel) = 0;
    virtual void handleData(const std::vector<hssh::TopologicalMap>& maps, const std::string& channel) = 0;
    virtual void handleData(const GoalTarget& target, const std::string& channel) = 0;
    virtual void handleData(const goal_route_command_message_t& message, const std::string& channel) = 0;
    virtual void handleData(const DecisionProgress& progress, const std::string& channel) = 0;
};

/**
 * GoalOutputConsumer is an interface to be implemented by all classes that handle the data
 * produced by the global_topo_planner module.
 */
class GoalOutputConsumer
{
public:
    virtual ~GoalOutputConsumer(void) { }

    virtual void handleRoute(const GoalRoute& route) = 0;
    virtual void handleProgress(const GoalProgress& progress) = 0;
    virtual void handleTargetSequence(const DecisionTargetSequence& sequence) = 0;
    virtual void handleDebugInfo(const goal_debug_info_t& info) = 0;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_GOAL_CONSUMERS_H
