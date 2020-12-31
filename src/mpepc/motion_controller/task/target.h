/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     target.h
 * \author   Collin Johnson and Jong Jin Park
 *
 * Declaration of motion_controller_target_t, which specifies the immediate destination for the robot to
 * drive to using the motion controller.
 */

#ifndef MPEPC_MOTION_CONTROLLER_MOTION_TARGET_TASK_H
#define MPEPC_MOTION_CONTROLLER_MOTION_TARGET_TASK_H

#include "mpepc/control/control_law_coordinates.h"
#include "mpepc/motion_controller/task/task.h"
#include <cereal/access.hpp>
#include <cereal/types/base_class.hpp>

namespace vulcan
{

namespace mpepc
{

/**
 * MotionTargetTask is a task telling the motion controller to drive to a target. The target is
 * likely to move as the robot navigates through the world. The MotionTargetTask is the most common
 * target, as the MPEPC planner uses this approach.
 */
class MotionTargetTask : public MotionControllerTask
{
public:
    static const int32_t MOTION_TARGET_TASK_ID = 1;

    /**
     * Constructor for MotionTargetTask.
     *
     * \param    target          Target for robot to drive to
     * \param    timeoutUs       Timeout during which the task is valid
     * \param    timestamp       Timestamp to assign to the task (optional, default = current time)
     */
    MotionTargetTask(const motion_target_t& target, int64_t timeoutUs, int64_t timestamp = 0)
    : MotionControllerTask(timestamp)
    , target_(target)
    , timeoutUs_(timeoutUs)
    {
        assert(timeoutUs > 0);
    }

    /**
     * getTarget retrieves the target to which the robot should be driving.
     */
    motion_target_t getTarget(void) const { return target_; }

    /**
     * getTimeout retrieves the specified timeout value for the motion target.
     */
    int64_t getTimeout(void) const { return timeoutUs_; }

    // MotionTargetTask interface
    virtual int32_t getId(void) const { return MOTION_TARGET_TASK_ID; }
    virtual std::string getDescription(void) const { return std::string("motion target"); }

protected:
    // For serialization support, allow subclasses to default construct
    MotionTargetTask(void) { }

private:
    motion_target_t target_;
    int64_t timeoutUs_;

    // Serialization support
    friend class ::cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar& ::cereal::base_class<MotionControllerTask>(this);
        ar& target_;
        ar& timeoutUs_;
    }
};

}   // namespace mpepc
}   // namespace vulcan

// Serialization support via smart pointer
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::mpepc::MotionTargetTask)

#endif   // MPEPC_MOTION_CONTROLLER_MOTION_TARGET_TASK_H
