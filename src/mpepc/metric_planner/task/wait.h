/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     wait.h
* \author   Collin Johnson
* 
* Declaration of WaitTask and WaitTaskManifold, which enable a program using just tasks to temporarily halt the robot
* without needing to send a separate message.
*/

#ifndef MPEPC_METRIC_PLANNER_TASKS_WAIT_H
#define MPEPC_METRIC_PLANNER_TASKS_WAIT_H

#include "mpepc/metric_planner/task/task.h"
#include <cereal/access.hpp>

namespace vulcan
{
namespace mpepc
{

/**
* WaitTask is a task that has the robot wait in one place for a specified duration. The wait can be forever, which
* pauses the robot until the next task arrives, or it can be a pause during the execution of a script or other behavior.
* 
* For timed WaitTasks, the timer begins when the manifold associated with the WaitTask is created.
* 
* The id for the task is set to the timestamp on creation.
*/
class WaitTask : public MetricPlannerTask
{
public:
    
    /**
    * Constructor for WaitTask.
    * 
    * Create a WaitTask that will wait for a specific amount of time.
    * 
    * \param    durationMs          Amount of time to wait (optional, default = 0, which is forever)
    * \pre  durationMs >= 0
    */
    explicit WaitTask(int32_t durationMs = 0);
    
    // MetricPlannerTask interface
    Id id(void) const override { return id_; }
    bool setTaskParameters(const metric_planner_task_params_t& taskParams, 
                            const task_manifold_builder_params_t& builderParams) override;
    std::unique_ptr<TaskManifold> createTaskManifold(void) override;
    bool isSafeToExecute(const ObstacleDistanceGrid& map) const override;
    bool isComplete(const motion_state_t& state) const override;
                            
private:
    
    Id id_;
    int64_t duration_;
    int64_t startTime_;
    
    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( id_,
            duration_);
    }
};

} // namespace mpepc
} // namespace vulcan

// Serialization support via smart pointer
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::mpepc::WaitTask)

#endif // MPEPC_METRIC_PLANNER_TASKS_WAIT_H
