/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation.h
* \author   Jong Jin Park
* 
* Declaration of NavigationTask, which encapsulates a single pose target in a map.,
* and NavigationTaskManifold, which encodes cost-to-go toward achieving the task.
*/

#ifndef MPEPC_NAVIGATION_TASK_H
#define MPEPC_NAVIGATION_TASK_H

#include "mpepc/metric_planner/task/task.h"
#include "mpepc/metric_planner/task/params.h"
#include "mpepc/grid/navigation_grid.h"
#include "core/pose.h"
#include "utils/timestamp.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/base_class.hpp>


namespace vulcan
{
namespace mpepc
{

struct motion_target_t;

/**
* NavigationTask defines a task to move to a particular pose or position in the environment. The task can be defined in
* four ways:
* 
*   - Motion to a single pose
*   - Motion to one of a collection of poses
*   - Motion to a single position
*   - Motion to one of a collection of positions
* 
* 
* The id for the task is assigned to the system timestamp at the time of creation. Thus, to ensure the id stays the same
* only copies of a NavigationTask be made, rather than new tasks with the same target type.
*/
class NavigationTask : public MetricPlannerTask
{
public:
    
    /**
    * Constructor for NavigationTask.
    *
    * Creates a NavigationTask that drives to a pose.
    *
    * \param    target          Target to be associated with the task
    * \param    id              Id to assign the task (optional, default is current system time)
    */
    NavigationTask(const pose_t& target, Id id = -1)
    : id_((id == -1) ? utils::system_time_us() : id)
    , target_(target)
    , isPositionTask_(false)
    {
    }
    
    
    /**
    * Constructor for NavigationTask.
    *
    * Creates a NavigationTask that drives to a series of poses.
    *
    * \param    waypointPoses   Poses to be visited
    * \param    id              Id to assign the task (optional, default is current system time)
    */
    NavigationTask(const std::vector<pose_t> waypointPoses, Id id = -1)
    : id_((id == -1) ? utils::system_time_us() : id)
    , waypointPoses_(waypointPoses)
    , isPositionTask_(false)
    {
        assert(!waypointPoses_.empty());
        target_ = waypointPoses_.back();
    }

    /**
    * Constructor for NavigationTask.
    *
    * Creates a NavigationTask that drives to a position.
    *
    * \param    target          Target position for the robot
    * \param    id              Id to assign the task (optional, default is current system time)
    */
    NavigationTask(position_t target, Id id = -1)
    : id_((id == -1) ? utils::system_time_us() : id)
    , target_(target)
    , candidateTargets_({target})
    , isPositionTask_(true)
    {
    }

    /**
    * Constructor for NavigationTask.
    *
    * Creates a NavigationTask that provides an ordered sequence of possible robot positions to drive to. The
    * planner should select the first safe position within the sequence to use for the target.
    *
    * This message is needed when trying to move to an area, but not a specific pose, like a near a desk, within that
    * area. The metric_planner is the only part of the system capable of knowing if the robot can actually reach a
    * given position, so this message provides the planner with all acceptable positions to handle more qualitative
    * navigation.
    *
    * \param    candidateTargets        Ordered sequence of candidate targets with the most preferred target first
    * \param    id              Id to assign the task (optional, default is current system time)
    */
    NavigationTask(std::vector<position_t> candidateTargets, Id id = -1)
    : id_((id == -1) ? utils::system_time_us() : id)
    , candidateTargets_(candidateTargets)
    , isPositionTask_(true)
    {
        assert(!candidateTargets_.empty());

        target_ = pose_t(candidateTargets_.front());
    }

    /**
    * isPoseTarget checks if this task wants to navigate to a pose.
    */
    bool isPoseTarget(void) const { return !isPositionTask_; }

    /**
    * isPositionTarget checks if this task wants to navigate to a position.
    */
    bool isPositionTarget(void) const { return isPositionTask_; }
    
    /**
    * target retrieves the target pose for the task. The target pose might change over time, but it mostly needed for
    * visualization and debugging purposes.
    * 
    * \return   Current pose target.
    */
    pose_t target(void) const { return target_; };
    std::vector<pose_t> getWaypoints(void) const { return waypointPoses_; };
    
    // MetricPlannerTask interface
    Id id(void) const override { return id_; };
    bool setTaskParameters(const metric_planner_task_params_t& taskParams, 
                           const task_manifold_builder_params_t& builderParams) override;
    std::unique_ptr<TaskManifold> createTaskManifold(void) override;
    bool isSafeToExecute(const ObstacleDistanceGrid& map) const override;
    bool isComplete(const motion_state_t& state) const override;
    
protected:
    
    // For serialization support, allow subclasses to default construct
    NavigationTask(void) { }
    
private:
    
    Id id_;
    mutable pose_t target_;
    std::vector<pose_t> waypointPoses_;
    std::vector<position_t> candidateTargets_;
    bool isPositionTask_;
    
    navigation_task_params_t taskParams_;
    navigation_task_manifold_params_t manifoldParams_;
    
    // Serialization support
    friend class ::cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar)
    {
        ar( id_,
            target_,
            waypointPoses_,
            candidateTargets_,
            isPositionTask_);
    }
};

} // namespace mpepc
} // namespace vulcan

// Serialization support via smart pointer
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::mpepc::NavigationTask)

#endif // MPEPC_NAVIGATION_TASK_H