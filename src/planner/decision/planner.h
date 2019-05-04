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
* Declaration of DecisionPlanner.
*/

#ifndef PLANNER_DECISION_PLANNER_H
#define PLANNER_DECISION_PLANNER_H

#include <planner/decision/result.h>
#include <planner/decision/sequence.h>
#include <memory>

namespace vulcan
{
namespace planner
{

class DecisionState;
class DecisionTask;
class DecisionTaskExecutor;

/**
* DecisionPlanner organizes the execution of a DecisionTaskSequence. The planner runs the provided tasks in order until
* no tasks remain. Each new task yields a DecisionTaskExecutor that is used for determining the task to send to the
* metric_planner and determining when the task is completed.
*
* The tasks to be executed by the DecisionPlanner can be provided via the addTaskToXXXX methods.
*
* Calculating a new result based on changed state in the local topological layer is performed via the plan method.
*/
class DecisionPlanner
{
public:

    /**
    * Constructor for DecisionPlanner.
    */
    DecisionPlanner(void);

    /**
    * Destructor for DecisionPlanner.
    *
    * For std::unique_ptr use.
    */
    ~DecisionPlanner(void);

    // Not a value type
    DecisionPlanner(const DecisionPlanner&) = delete;
    DecisionPlanner(DecisionPlanner&&)      = delete;

    DecisionPlanner& operator=(const DecisionPlanner&) = delete;
    DecisionPlanner& operator=(DecisionPlanner&&)      = delete;

    //////// Task management methods //////////////

    /**
    * addTaskToFront adds a new task to the end of the sequence of tasks for the planner.
    *
    * \param    task            Task to be added to the sequence
    */
    void addTaskToFront(std::unique_ptr<DecisionTask> task);

    /**
    * addTaskToBack adds a new task to the end of the sequence of tasks being executed by the planner.
    *
    * \param    task            Task to be added to the sequence
    */
    void addTaskToBack(std::unique_ptr<DecisionTask> task);

    /**
    * clearTasks erases all tasks currently being executed or waiting to be executed by the planner.
    */
    void clearTasks(void);

    ///////// Planning methods ////////////

    /**
    * plan updates the plan execution based on updated state information. When executing the current plan, a task may
    * finish, in which case, a new task will be put in the queue. If no tasks remain, the robot will be told to stop.
    *
    * \param    state           Current local topological state of the robot
    * \return   Result of the current planning action to be sent away.
    */
    DecisionResult plan(const DecisionState& state);

private:

    DecisionTaskSequence tasks_;
    std::unique_ptr<DecisionTaskExecutor> currentExecutor_;
};

}
}

#endif // PLANNER_DECISION_PLANNER_H
