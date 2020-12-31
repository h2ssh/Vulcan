/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision_action.h
 * \author   Collin Johnson
 *
 * Declaration of DecisionAction.
 */

#ifndef PLANNER_INTERFACE_DECISION_ACTION_H
#define PLANNER_INTERFACE_DECISION_ACTION_H

#include "planner/interface/decision.h"
#include <memory>

namespace vulcan
{
namespace mpepc
{
class MetricPlannerTask;
}
namespace mpepc
{
struct metric_planner_status_message_t;
}
struct pose_t;
namespace system
{
class SystemCommunicator;
}
namespace planner
{

/**
 * DecisionAction
 */
class DecisionAction
{
public:
    /**
     * Constructor for DecisionAction.
     *
     * \param    decision        Decision to be performed
     * \param    robotPose       Current pose of the robot
     */
    DecisionAction(const Decision& decision, const pose_t& robotPose);

    /**
     * perform performs the action.
     *
     * \param    communicator
     */
    void perform(system::SystemCommunicator& communicator) const;

    /**
     * isComplete checks if the action has finished.
     *
     * \param    status          Status from the underlying planner
     */
    bool isComplete(const mpepc::metric_planner_status_message_t& status) const;

    /**
     * decision retrieves the decision being commanded by this action.
     */
    Decision decision(void) const { return decision_; }

private:
    Decision decision_;
    std::shared_ptr<mpepc::MetricPlannerTask> task_;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_INTERFACE_DECISION_ACTION_H
