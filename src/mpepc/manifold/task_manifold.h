/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     task_manifold.h
* \author   Jong Jin Park and Collin Johnson
*
* Declaration of TaskManifold interface.
*/

#ifndef MPEPC_MANIFOLDS_TASK_MANIFOLD_H
#define MPEPC_MANIFOLDS_TASK_MANIFOLD_H

#include <mpepc/simulator/dynamic_object_trajectory.h>
#include <core/pose.h>
#include <vector>

namespace vulcan
{
struct motion_state_t;

namespace system { class ModuleCommunicator; }
namespace mpepc
{

struct planning_environment_t;
struct robot_trajectory_info_t;

/**
* Declaration of the TaskManifold, which encapsulates a cost-to-go metric toward
* achieving a task. For navigation, this is a navigation function. For person
* pacing, this is a distance to a series of desired positions.
*
* Namely, a task manifold we think of is a smooth surface in the workspace of the
* robot that has a unique minimum at the goal point. This can be understood as a
* planar expansion of a path, which is usually a line in the workspace. This guides
* the trajectory planner by providing the required cost-to-go metric in achieving
* the goal. The current implementation mandates that this manifold instruction
* updates when: (1) a new task is given; (2) the (perception of) environment has
* changed; and/or (3) at regular time interval designated by the user.
*/
class TaskManifold
{
public:
    /**
    * Constructor for TaskManifold
    */
    TaskManifold(void) { }

    /**
    * Destructor for TaskManifold
    */
    virtual ~TaskManifold(void) { }

    // TaskManifold interface
    /**
    * update updates the task manifold with the newly received information of the
    * task environment, which consists of the static obstacles, dynamic objects
    * and the state of the robot itself.
    *
    * \param    env             Complete maintained state for the robot
    * \param    trajectories    Estimated trajectories of dynamic objects in the environment.
    */
    virtual void update(const planning_environment_t& env,
                        const std::vector<dynamic_object_trajectory_t>& trajectories) = 0;

    /**
    * createTaskSpecificObjects looks at the environment to see if any additional dynamic objects exist based
    * on the route and current environment.
    *
    * This method should be called after the call to update.
    *
    * By default, this method does nothing and only needs to be optionally overridden.
    *
    * \param    env             Complete environment state for the robot
    * \return   Additional objects to consider during the current planning iteration.
    */
    virtual std::vector<dynamic_object_trajectory_t> createTaskSpecificObjects(const planning_environment_t& env)
    {
        return std::vector<dynamic_object_trajectory_t>();
    }

    /**
    * target retrieves the currently estimated target the manifold is directing the robot towards. It might not be the
    * same pose every time, but the nominal, zero-error pose should be returned.
    *
    * \return   Current pose target of the manifold.
    */
    virtual pose_t target(void) const = 0;

    /**
    * calculateNegativeRewardsOverTrajectory computes negative rewards over a trajectory.
    *
    * The reward for each evaluated state is calculated by taking a short trajectory segment, represented by a
    * start state and end state of the trajectory and the time between those states, and checking how much the robot
    * has progressed toward the goal over the trajectory segment. The reward is negative if the robot is making
    * progress, which can be viewed as descreasing the distance to the goal.
    *
    * \param[in]    trajectory      Trajectory of the robot to be evaluated
    * \param[in]    timestep        Time between samples in the trajectory
    * \param[in]    stride          Stride when iterating through the trajectory -- use every Nth state
    * \param[out]   rewards         Calculated rewards for each evaluated state in the trajectory
    */
    virtual void calculateNegativeRewardsOverTrajectory(const std::vector<motion_state_t>& trajectory,
                                                        float timestep,
                                                        std::size_t stride,
                                                        std::vector<float>& rewards) const = 0;

    /**
    * calculateHeuristicCost returns any additional piece of task-related cost to be
    * added to the progress metric.
    *
    * \param    robotTrajectoryInfo      candidate robot trajectory and related info.
    * \return   heuristic evaluation of the trajectory in achieving the task.
    *
    * NOTE: Idealy, the manifold should contain all necessary and sufficient
    * information, and piecewiseNegativeReward should be enough to carry out the
    * task. Then this value can be identifally zero. However, having a heurisitc
    * function like this could be useful for testing new ideas and making quick
    * amends to incomplete manifold definitions, as it gives us more flexibility.
    */
    virtual double calculateHeuristicCost(const robot_trajectory_info_t& robotTrajectoryInfo) const = 0;

    /**
    * sendDebugInfo sends relevant debug information to communication module.
    */
    virtual void sendDebugInfo(system::ModuleCommunicator& transmitter) = 0;
};

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_MANIFOLDS_TASK_MANIFOLD_H
