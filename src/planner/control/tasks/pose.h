/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose.h
* \author   Collin Johnson
* 
* Declaration of PoseTask.
*/

#ifndef PLANNER_CONTROL_TASKS_POSE_H
#define PLANNER_CONTROL_TASKS_POSE_H

#include "planner/control/task.h"
#include "core/pose.h"

namespace vulcan
{
namespace planner
{
namespace detail
{

/**
* PoseTask drives to the specified pose. The PoseTask is a implementation detail of the control planner and shouldn't be
* used by other modules.
* 
* The PoseTask is a low-level task and creates no subtasks. Either the robot can successfully drive to the target or it
* fails. The PoseTask directly interacts with the metric_planner. The success of a PoseTask is tied to the success of
* the associated MetricPlannerTask.
*/
class PoseTask : public ControlTask
{
public:
    
    /**
    * Constructor for PoseTask.
    * 
    * \param    target          Pose task will drive to
    * \param    id              Id of the task
    * \param    parent          Parent of the task
    */
    PoseTask(const pose_t& target, int32_t id, ControlTask* parent);

    // ControlTask interface
    void activate(const ControlState& state) override;
    bool hasSubTask(void) const override;
    int pushSubTasks(ControlTaskStack& stack) override;
    
private:
    
    pose_t target_;
    std::shared_ptr<mpepc::MetricPlannerTask> mpepcTask_;
        
    // ControlTask interface
    void childIsDone(ControlTask* child, ControlTaskProgress progress) override;
    ControlTaskResult doExecute(const ControlState& state) override;
};

}
}
}

#endif // PLANNER_CONTROL_TASKS_POSE_H
