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
 * Declaration of TopoMapInputConsumer and TopoMapOutputConsumer interfaces.
 */

#ifndef SIMULATOR_TOPO_CONSUMERS_H
#define SIMULATOR_TOPO_CONSUMERS_H

#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{
class TopologicalMap;
class GlobalLocation;

struct local_topology_place_event_t;
struct local_topology_path_event_t;
}   // namespace hssh

namespace planner
{
class GlobalTopoRoute;
class DecisionTarget;
class DecisionTargetSequence;
}   // namespace planner
namespace simulator
{

/**
 * TopoMapInputConsumer defines the interface for classes in the topo map simulator module
 * that handle data from other modules.
 */
class TopoMapInputConsumer
{
public:
    virtual ~TopoMapInputConsumer(void) { }

    virtual void handleData(const hssh::TopologicalMap& map, const std::string& channel) = 0;
    virtual void handleData(const hssh::GlobalLocation& state, const std::string& channel) = 0;
    virtual void handleData(const planner::GoalRoute& plan, const std::string& channel) = 0;
    virtual void handleData(const planner::DecisionTargetSequence& sequence, const std::string& channel) = 0;
};

/**
 * TopoMapOutputConsumer defines the interface for classes in the topo map simulator module
 * that handle the data produced by the module itself for logging or transmission purposes.
 */
class TopoMapOutputConsumer
{
public:
    virtual ~TopoMapOutputConsumer(void) { }

    virtual void handlePlaceEvent(const hssh::local_topology_place_event_t& event) = 0;
    virtual void handlePathEvent(const hssh::local_topology_path_event_t& event) = 0;
    virtual void handleSimulatorState(const hssh::GlobalLocation& state) = 0;
};

}   // namespace simulator
}   // namespace vulcan

#endif   // SIMULATOR_TOPO_CONSUMERS_H
