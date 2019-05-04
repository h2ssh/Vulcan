/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     task.h
* \author   Collin Johnson
*
* Declaration of DecisionTask interface.
*/

#ifndef PLANNER_DECISION_TASK_H
#define PLANNER_DECISION_TASK_H

#include <memory>
#include <vector>

namespace vulcan
{
namespace planner
{

class DecisionState;
class DecisionTaskExecutor;

/**
* DecisionTask is an interface for tasks performed by the DecisionPlanner. Each task describes how the robot should
* move through a single area. Each task can create an appropriate TaskExecutor that handles the actual motion through
* the environment.
*
* Some tasks cannot be immediately accomplished in the area in which they are asked to initially execute. The most
* common example is a "Take the next right"-style command. If the robot arrives at a decision point lacking a right
* turn, then the correct action is to do is drive through the intersection, travel along the path segment, and turn
* right at the following decision point (or continue this same behavior until a right turn is encountered).
*
* To maintain the invariant that a Task corresponds to only motion through a single area, but to allow the above
* behavior for tasks, a Task can have prerequisite Tasks. These prerequisites should be preprended to the task sequence.
* Once they complete, the initial Task will be re-evaluated within the new context.
*
* To use a Task:
*
*   - Check if it has prereqs, given the current state
*       * Prepend the prereqs tasks if they exist
*   - Else, create the executor for the task
*/
class DecisionTask
{
public:

    virtual ~DecisionTask(void) {}

    /**
    * hasPrereqs checks if this Task has prerequisite tasks that must be completed before this task can attempt to
    * execute (all tasks can fail, so completion is never guaranteed). The prereq tasks need to be completed
    * successfully before this task can be executed.
    *
    * \param    state           State of the local topological map in which the task will be executed
    * \return   True if there are prerequisite tasks.
    */
    virtual bool hasPrereqs(const DecisionState& state) const = 0;

    /**
    * createPrereqTasks creates the prerequisite tasks for this task.
    *
    * \param    state           State of the local topological map in which the task will be executed
    * \return   The sequence of tasks to be executed before the current task.
    */
    virtual std::vector<std::unique_ptr<DecisionTask>> createPrereqTasks(const DecisionState& state) const = 0;

    /**
    * execute takes the current local topological state and generates a TaskExecutor to determine metric planner tasks
    * to move through the area and to monitor the progress of motion.
    *
    * \param    state           State of the local topological map in which the task will be executed
    * \return   TaskExecutor for navigating in the environment.
    */
    virtual std::unique_ptr<DecisionTaskExecutor> execute(const DecisionState& state) const = 0;
};

}
}

#endif // PLANNER_DECISION_TASK_H
