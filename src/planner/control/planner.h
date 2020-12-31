/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     planner.h
 * \author   Collin Johnson
 *
 * Declaration of ControlPlanner.
 */

#ifndef PLANNER_CONTROL_PLANNER_H
#define PLANNER_CONTROL_PLANNER_H

#include "planner/control/task.h"
#include "planner/control/task_stack.h"

namespace vulcan
{
namespace planner
{

class ControlCommand;
class ControlState;

/**
 * ControlPlanner
 */
class ControlPlanner
{
public:
    /**
     * Constructor for ControlPlanner.
     */
    ControlPlanner(void);

    /**
     * assignTask assigns a new task to be executed by the planner. If a previous task was running, it will no longer
     * be running.
     *
     * \param    command         New commanded task
     * \param    state           Current control state
     */
    void assignTask(const ControlCommand& command, const ControlState& state);

    /**
     * executeTask executes the current task. The task is updated based on the current state. The status of the task,
     * along with any commands to issue to the metric_planner are returned.
     *
     * \param    state           Current control state
     * \return   Current status of the task. The status is both the state of the task -- executing, finished, failed --
     *   and possibly a new task to send to the metric_planner.
     */
    ControlTaskResult executeTask(const ControlState& state);

    /**
     * haveTask checks if a task is currently being executed.
     */
    bool haveTask(void) const;

private:
    std::shared_ptr<ControlTask> currentTask_;
    ControlTaskStack taskStack_;

    void activateTask(ControlTask& task, const ControlState& state);
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_CONTROL_PLANNER_H
