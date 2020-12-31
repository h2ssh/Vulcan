/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     task.h
 * \author   Jong Jin Park and Collin Johnson
 *
 * Declaration of MetricPlannerTask, which encapsuates incoming tasks to be excuted.
 */

#ifndef MPEPC_METRIC_PLANNER_TASK_H
#define MPEPC_METRIC_PLANNER_TASK_H

#include "system/message_traits.h"
#include <cereal/types/memory.hpp>
#include <memory>

namespace vulcan
{
struct motion_state_t;

namespace mpepc
{

class ObstacleDistanceGrid;
class TaskManifold;
struct metric_planner_task_params_t;
struct task_manifold_builder_params_t;
struct motion_target_t;

/**
 * MetricPlannerTask is an interface defining the properties for a planner task.
 * The task interface defines one method identifying a task type, and another two
 * for identifying preconditions and completion:
 *
 *   - isComplete : checks if the task is completed
 *   - isSafeToExecute(map) checks if the task is safe to execute in a given map
 *
 * Derived classes are meant to encapsulate the absolute minimal information
 * pertaining individual metric planner tasks.
 */
class MetricPlannerTask
{
public:
    using Id = int64_t;

    static const Id kInvalidId = -1;

    // NOTE: As soon as the task parameters, i.e. the criteria for evaluating and
    //         determining the completion of the task varies between individual tasks,
    //         individual tasks should be created with the task parameterse.g.:
    //
    //     - NavigationTask(params) constructor with given params

    /**
     * Constructor for MetricPlannerTask.
     */
    MetricPlannerTask(void) { }

    /**
     * Destructor for this abstract base class.
     */
    virtual ~MetricPlannerTask(void) { }

    /**
     * id retrieves the id assigned to the task when it was created.
     */
    virtual Id id(void) const = 0;

    /**
     * setTaskParameters sets the parameters to use for executing the task. These parameters were loaded from a config
     * file when the module executing the task began. If the task is internally storing parameters, then these
     * parameters can be ignored.
     *
     * \param    taskParams          Task-specific parameters
     * \param    manifoldParams      Manifold-specific parameters
     * \return   True if the parameters were accepted. False if the parameters were rejected.
     */
    virtual bool setTaskParameters(const metric_planner_task_params_t& taskParams,
                                   const task_manifold_builder_params_t& builderParams) = 0;

    /**
     * createTaskManifold creates a TaskManifold encoding the navigation function to be used for reaching the target
     * specified by the manifold.
     *
     * \return   An instance of TaskManifold to be used for reaching the target specified by the task. If no parameters
     *   exist for the task because setTaskParameters wasn't called, then a nullptr is returned.
     */
    virtual std::unique_ptr<TaskManifold> createTaskManifold(void) = 0;

    /**
     * isSafeToExecute checks if the task is safe to perform within a given map
     * by simple collision checking. This can also be used as a basic compatibility
     * test between the given map and the task at hand.
     *
     * \param    map             Grid map of distance to the nearest static obstacle.
     * \return   true if safe (no collision at target pose, not out of bound, etc.)
     */
    virtual bool isSafeToExecute(const ObstacleDistanceGrid& map) const = 0;

    /**
     * isComplete checks if the given state of the robot satisfies the completion
     * crietria.
     *
     * \param    state           State of the robot to be checked.
     * \return   true if the task is complete.
     */
    virtual bool isComplete(const motion_state_t& state) const = 0;
};

}   // namespace mpepc
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(std::shared_ptr<mpepc::MetricPlannerTask>, ("MPEPC_METRIC_PLANNER_TASK"))

#endif   // MPEPC_METRIC_PLANNER_TASK_H
