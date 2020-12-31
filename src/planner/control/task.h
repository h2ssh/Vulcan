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
* Declaration of ControlTask base class.
*/

#ifndef PLANNER_CONTROL_TASK_H
#define PLANNER_CONTROL_TASK_H

#include "planner/control/task_status.h"
#include "planner/control/task_stack.h"

namespace vulcan
{
namespace mpepc { class MetricPlannerTask; }
namespace planner
{
    
class ControlState;

/**
* ControlTaskResult
*/
struct ControlTaskResult
{
    ControlTaskStatus status;
    std::shared_ptr<mpepc::MetricPlannerTask> task;
};

/**
* ControlTask is the base class for the ControlTasks that can be performed by the control planner. The control planner
* executes one control task at a time.
* 
* A ControlTask is responsible for looking at the current state of the environment and issuing the proper commands to
* take the robot to some location. 
* 
* ControlTask is used via the following basic sequence of operations:
* 
*   - Initialization:
*       -- activate : initialize the task with the most recent control state
*       -- hasSubTasks : check if sub-tasks need to be completed first in order for this task to complete
*       -- pushSubTasks : push any required sub-tasks onto the task stack
* 
*   - General use:
*       -- execute : run an iteration of the task. The result is a status indicating if the task is completed, failed,
*           or in-progress, along with the task to issue to the trajectory planner.
*/
class ControlTask
{
public:
    
    /**
    * Destructor for ControlTask.
    */
    virtual ~ControlTask(void) { }
    
    /**
    * activate performs any initialization tasks that need to be handled for the task to start being executed.
    * 
    * \param    state           Current ControlState
    */
    virtual void activate(const ControlState& state) = 0;
    
    /**
    * hasSubTasks checks if any subtasks are needed to complete this task.
    */
    virtual bool hasSubTask(void) const = 0;
    
    /**
    * pushSubTasks pushes all subtasks onto the task stack.
    * 
    * \param    stack           Stack on which to push the sub-tasks
    * \return   Number of subtasks added.
    */
    virtual int pushSubTasks(ControlTaskStack& stack) = 0;
    
    /**
    * execute runs an iteration of the task using the current state.
    * 
    * When the task completes or fails, the parent task will be notified automatically.
    * 
    * \param    state           Current control state
    * \return   Result that indicates current status of the task along with the trajectory planner task to be issued.
    */
    ControlTaskResult execute(const ControlState& state);

    /**
    * id retrieves the id assigned to the task.
    */
    int32_t id(void) const { return id_; }
    
protected:
    
    /**
    * Constructor for ControlTask.
    * 
    * \param    parent          Parent task that spawned this task
    */
    ControlTask(int32_t id = 0, ControlTask* parent = nullptr);
    
    /**
    * doExecute is called by the execute method to run the actual task update. The result is propagated to the parent
    * task, if one exists, before returning to the caller of execute.
    * 
    * \param    state           Current control state
    * \return   Result with the current status of the task and task to send to the metric planner.
    */
    virtual ControlTaskResult doExecute(const ControlState& state) = 0;
    
    /**
    * childIsDone is called by a child task when it completes or fails.
    * 
    * \param    child           Child task that is done
    * \param    progress        Progress indicating how the task finished
    */
    virtual void childIsDone(ControlTask* child, ControlTaskProgress progress) = 0;
    
private:
    
    int32_t id_;
    ControlTask* parent_;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( id_);
    }
};

}
}

#endif // PLANNER_CONTROL_TASK_H
