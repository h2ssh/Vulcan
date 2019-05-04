/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     executor.h
* \author   Collin Johnson
*
* Declaration of DecisionTaskExecutor interface.
*/

#ifndef PLANNER_DECISION_EXECUTOR_H
#define PLANNER_DECISION_EXECUTOR_H

#include <planner/decision/result.h>

namespace vulcan
{
namespace planner
{

class DecisionState;

/**
* DecisionTaskExecutor is an interface for performing a DecisionTask. A TaskExecutor is able to determine when the task
* it is executing has finished and whether or not the task can actually be completed, based on the current state.
*
* Each DecisionTask will create an appropriate DecisionTaskExecutor based on the state of the robot when the task starts
* begin executed. The task executor is then provided with the DecisionState each time it changes to determine the
* action to be taken. The action is provided as a DecisionResult.
*
* General use of the DecisionTaskExecutor should be along the lines of:
*
*   if(hasFailed())
*       - handle failures
*   else if(isComplete())
*       - move to next task
*   else
*       result = updateCommand()
*/
class DecisionTaskExecutor
{
public:

    virtual ~DecisionTaskExecutor(void) {}

    /**
    * updateCommand updates the current command to be issued from the DecisionPlanner. The command will provide the
    * MetricPlanner with an updated task. If the task finishes or fails, the result may send additional messages up the
    * planning chain to the human user or goal planner, as needed.
    *
    * \param    state           Current local topological state of the robot
    * \return   Result encapsulating the current latest command to be issued from the decision planner.
    */
    virtual DecisionResult updateCommand(const DecisionState& state) const = 0;

    /**
    * isComplete checks to see if the Task being executed has completed.
    *
    * \param    state           Current state of the robot
    * \return   True if the task has finished.
    */
    virtual bool isComplete(const DecisionState& state) const = 0;

    /**
    * hasFailed checks to see if the Task being executed has failed. A failed task can happen for a number of reasons,
    * which will be documented classes that implement this interface.
    *
    * \param    state           Current state of the robot
    * \return   True if the task cannot be completed.
    */
    virtual bool hasFailed(const DecisionState& state) const = 0;
};

}
}

#endif // PLANNER_DECISION_EXECUTOR_H
