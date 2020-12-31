/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision_action.cpp
 * \author   Collin Johnson
 *
 * Definition of DecisionAction.
 */

#include "planner/interface/decision_action.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "planner/interface/decision.h"
#include "system/system_communicator.h"
#include "utils/stub.h"

namespace vulcan
{
namespace planner
{

std::shared_ptr<mpepc::NavigationTask> create_task_for_target(const pose_t& target);


DecisionAction::DecisionAction(const Decision& decision, const pose_t& robotPose)
: decision_(decision)
, task_(create_task_for_target(pose_t(decision.position(), decision.orientation())))
{
}


void DecisionAction::perform(system::SystemCommunicator& communicator) const
{
    communicator.sendSystem(task_);
}


bool DecisionAction::isComplete(const mpepc::metric_planner_status_message_t& status) const
{
    PRINT_STUB("DecisionAction::isComplete");
    return false;
}


std::shared_ptr<mpepc::NavigationTask> create_task_for_target(const pose_t& target)
{
    // A transition affordance defines the target right on the gateway. However, the robot needs to cross the gateway
    // to make progress, thus the target needs to be placed slightly across the gateway to ensure that the transition
    // between areas actually occurs. The target for the affordance defines the correct direction in which to move the
    // target, so it can be used for nudging the target into the adjacent area.

    const float kTargetExtensionLength = 1.0f;   // Move the target one meter into the adjacent area

    pose_t extendedTarget(target.x + std::cos(target.theta) * kTargetExtensionLength,
                          target.y + std::sin(target.theta) * kTargetExtensionLength,
                          target.theta);
    return std::make_shared<mpepc::NavigationTask>(extendedTarget);
}

}   // namespace planner
}   // namespace vulcan
