/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     target_impl.cpp
 * \author   Collin Johnson
 *
 * Definition of LocalAreaTarget and GatewayTarget subclasses of LocalTopoExplorationTarget.
 */

#include "planner/exploration/local_topo/target_impl.h"
#include "hssh/local_topological/event.h"
#include "hssh/local_topological/events/area_transition.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "planner/utils/local_area_tasks.h"
#include "utils/stub.h"

namespace vulcan
{
namespace planner
{

//////////////////////////// LocalAreaTarget definition //////////////////////////////

LocalAreaTarget::LocalAreaTarget(const hssh::LocalArea& area)
: visited_(false)
, areaId_(area.id())
, areaType_(area.type())
, plannerTask_(create_navigation_task_for_local_area(area))
, boundary_(area.polygonBoundary().vertices())
{
}


bool LocalAreaTarget::checkVisited(const hssh::LocalAreaEvent& event)
{
    // If the area has not been visited yet, then check if it is visited by this event
    if (!visited_) {
        event.accept(*this);
    }

    return visited_;
}


std::shared_ptr<mpepc::NavigationTask> LocalAreaTarget::explorationTask(const pose_t& pose) const
{
    return plannerTask_;
}


void LocalAreaTarget::visitAreaTransition(const hssh::AreaTransitionEvent& event)
{
    // Check if the id of the entered area is the same as the target area
    auto enteredArea = event.enteredArea();
    visited_ = enteredArea && (enteredArea->id() == areaId_);
}


void LocalAreaTarget::visitTurnAround(const hssh::TurnAroundEvent& event)
{
    // TurnAroundEvent doesn't affect the LocalAreaTarget
}

//////////////////////////// GatewayTarget definition //////////////////////////////

GatewayTarget::GatewayTarget(const hssh::Gateway& gateway) : visited_(false), gateway_(gateway)
{
    // The boundary extends +/- 0.25m to either side of the gateway so it will show up on the map.
    Point<double> delta(0.25 * std::cos(gateway.direction()), 0.25 * std::sin(gateway.direction()));

    std::vector<Point<double>> vertices(4);
    vertices.push_back(gateway.boundary().a + delta);
    vertices.push_back(gateway.boundary().b + delta);
    vertices.push_back(gateway.boundary().b - delta);
    vertices.push_back(gateway.boundary().a - delta);

    boundary_ = math::Polygon<float>(vertices);
}


bool GatewayTarget::checkVisited(const hssh::LocalAreaEvent& event)
{
    // If the gateway has not been visited yet, then check if it is visited by this event
    if (!visited_) {
        event.accept(*this);
    }

    return visited_;
}


std::shared_ptr<mpepc::NavigationTask> GatewayTarget::explorationTask(const pose_t& pose) const
{
    PRINT_STUB("GatewayTarget::explorationTask");
    return std::make_shared<mpepc::NavigationTask>(pose_t(gateway_.center(), gateway_.direction()));
}


void GatewayTarget::visitAreaTransition(const hssh::AreaTransitionEvent& event)
{
    // If this transition has a gateway, i.e. it isn't the very first area visited, then check and see if it is
    // similar to the gateway for this target. A similar gateway means the gateway is now visited.
    auto gateway = event.transitionGateway();
    if (gateway) {
        visited_ = gateway->isSimilarTo(gateway_);
    }
}


void GatewayTarget::visitTurnAround(const hssh::TurnAroundEvent& event)
{
    // TurnAroundEvent doesn't affect the GatewayTarget
}

}   // namespace planner
}   // namespace vulcan
