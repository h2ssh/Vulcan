/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rotate.h
* \author   Collin Johnson
* 
* Declaration of RotateTask and RotateTaskManifold for getting the robot to turn-in-place to a specific orientation.
*/

#ifndef MPEPC_METRIC_PLANNER_TASKS_ROTATE_H
#define MPEPC_METRIC_PLANNER_TASKS_ROTATE_H

#include "mpepc/types.h"
#include "mpepc/metric_planner/task/task.h"
#include "core/pose.h"
#include <cereal/access.hpp>

namespace vulcan
{
namespace mpepc
{
    
/**
* RotateTask tells the robot to turn-in-place. The task can command a continuous rotation or a rotation to a specific
* orientation. The RotationMode determines the behavior of the robot performing the task.
*/
class RotateTask : public MetricPlannerTask
{
public:
    
    /**
    * Constructor for RotateTask.
    * 
    * Creates a RotateTask where the robot turns to reach some goal orientation
    * 
    * \param    goalOrientation         Desired orientation for the robot to reach
    * \param    maxCompletionError  Maximum difference in robot orientation to goal orientation for the task to be
    *                               considered complete (optional, default = 0.05 (about 3 degrees))
    * \pre  maxCompletionError > 0
    */
    explicit RotateTask(double goalOrientation, double maxCompletionError = 0.05);
    
    /**
    * Constructor for RotateTask.
    * 
    * Creates a RotateTask where the robot will turn-in-place either left or right forever.
    * 
    * \param    turnDirection       The direction in which the robot should continuously turn
    * \pre  turnDirection == RotationMode::turn_left || turnDirection == RotationMode::turn_right
    */
    explicit RotateTask(RotationMode turnDirection);
    
    // MetricPlannerTask interface
    Id id(void) const override { return id_; }
    bool setTaskParameters(const metric_planner_task_params_t& taskParams, 
                           const task_manifold_builder_params_t& builderParams) override;
    std::unique_ptr<TaskManifold> createTaskManifold(void) override;
    bool isSafeToExecute(const ObstacleDistanceGrid& map) const override;
    bool isComplete(const motion_state_t& state) const override;
    
private:
    
    Id id_;
    RotationMode mode_;
    double orientation_;
    double maxError_;
    
    // Serialization support
    friend class cereal::access;

    RotateTask(void) { } // default ctor needed for serialization

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( id_,
            mode_,
            orientation_,
            maxError_
        );
    }
};

} // namespace mpepc
} // namespace vulcan

// Serialization support via smart pointer
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::mpepc::RotateTask)

#endif // MPEPC_METRIC_PLANNER_TASKS_ROTATE_H
