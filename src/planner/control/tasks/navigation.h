/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation.h
* \author   Collin Johnson
*
* Declaration of NavigationControlTask
*/

#ifndef PLANNER_CONTROL_TASKS_NAVIGATION_H
#define PLANNER_CONTROL_TASKS_NAVIGATION_H

#include "planner/control/task.h"
#include "core/pose.h"
#include <cereal/types/base_class.hpp>
#include <vector>

namespace vulcan
{
namespace planner
{

/**
* NavigationControlTask drives the robot to a desired target pose. The shortest path to the target is checked for
* constrictions. If any constrictions are found, then they will be handled accordingly as subtasks. In the normal case
* though, the task creates the appropriate PoseTask and waits until it finishes successfully.
*
* The NavigationControlTask requires the target pose to be in the LPM when the task is activated. If this condition
* doesn't hold, then the task will fail. Exploration is not supported by this task.
*/
class NavigationControlTask : public ControlTask
{
public:

    /**
    * Constructor for NavigationControlTask.
    *
    * \param    target          Target pose for the robot to drive to
    * \param    id              Id of the task
    * \param    parent          Parent of the task
    */
    NavigationControlTask(const pose_t& target, int32_t id, ControlTask* parent);

    // ControlTask interface
    void activate(const ControlState& state) override;
    bool hasSubTask(void) const override;
    int pushSubTasks(ControlTaskStack& stack) override;

private:

    pose_t target_;
    std::vector<std::shared_ptr<ControlTask>> subtasks_;
    std::vector<bool> completedSubtasks_;

    // ControlTask interface
    void childIsDone(ControlTask* child, ControlTaskProgress progress) override;
    ControlTaskResult doExecute(const ControlState& state) override;

    // Serialization support
    NavigationControlTask(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( cereal::base_class<ControlTask>(this),
            target_);
    }
};

}
}

#endif // PLANNER_CONTROL_TASKS_NAVIGATION_H
