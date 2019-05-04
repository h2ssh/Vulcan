/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     absolute_motion.h
* \author   Collin Johnson
*
* Declaration of AbsoluteMotionTask.
*/

#ifndef PLANNER_CONTROL_TASKS_ABSOLUTE_MOTION_H
#define PLANNER_CONTROL_TASKS_ABSOLUTE_MOTION_H

#include <planner/control/task.h>
#include <core/pose.h>

namespace vulcan
{
namespace planner
{
    
enum class AbsoluteMotion
{
    stop,
    go_straight,
    turn_left,
    turn_right
};

/**
* AbsoluteMotionTask
*/
class AbsoluteMotionTask : public ControlTask
{
public:

    /**
    * Constructor for AbsoluteMotionTask.
    *
    * \param    direction       Direction of motion (straight, left, right, stop)
    * \param    speed           Speed at which to move (m/s or rad/s)
    */
    AbsoluteMotionTask(AbsoluteMotion motion, double speed, int32_t id = 0, ControlTask* parent = nullptr);

    // ControlTask interface
    void activate(const ControlState& state) override;
    bool hasSubTask(void) const override;
    int pushSubTasks(ControlTaskStack& stack) override;

private:
    
    // ControlTask interface
    ControlTaskResult doExecute(const ControlState& state) override;
    void childIsDone(ControlTask* child, ControlTaskProgress progress) override;
    
    ControlTaskStatus taskStatus(const ControlState& state) const;
    
    AbsoluteMotion motion_;
    double speed_;
    std::shared_ptr<mpepc::MetricPlannerTask> outgoingTask_;
    bool sentTask_;
    pose_t startPose_;
    pose_t targetPose_;
};

} // namespace planner
} // namespace vulcan

#endif // PLANNER_CONTROL_TASKS_ABSOLUTE_MOTION_H
