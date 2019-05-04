/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file      trajectory_planner_info.h
* \author    Jong Jin Park
*
* Declaration of trajectory_planner_output_t and trajectory_planner_debug_info_t.
*/

#ifndef MPEPC_TRAJECTORY_PLANNER_INFO_H
#define MPEPC_TRAJECTORY_PLANNER_INFO_H

#include <mpepc/trajectory/robot_trajectory_info.h>
#include <system/message_traits.h>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{

namespace mpepc
{

/**
* trajectory_planner_output_t is the return value for TrajectoryPlanner, which
* contains the motion target (the new plan) and an indicator if the target is good,
* i.e. the MPEPCOptimizer have successfully found a good solution.
*/
struct trajectory_planner_output_t
{
    // representation of the new plan.
    motion_target_t motionTarget; // motion target (Park, Johnson and Kuipers, IROS-12)

    // indicator for a 'good' target. A bad target forces robot to a stop. This
    // happens when the optmizer is unable to find a good solution, which should
    // be a rare case.
    bool isGood;
};


/**
* trajectory_planner_info_t contains internal debugging state useful for visualization of the
* processing of the trajectory planner, including all candidate trajectories evaluated
* during the planning process. Individual robot_trajectory_info_t contains a comprehesive
* list of information about a candidate trajectories.
*/
struct trajectory_planner_debug_info_t
{
    int iteration; // total number of iterations within this planning cycle

    int64_t updateStartTimeUs; // time when the planning cycle initiated for the trajectory planner
    int64_t updateEndTimeUs;   // time when the planning cycle finalized for the trajectory planner
    int64_t planReleaseTimeUs; // time to output and execute the generated plan

    // distance to the nearest object at the most recent observation.
    float clearanceToStaticObs;
    float clearanceToDynObs;

//     bool haveGoalPose;
//     pose_t goalPose;

    // received motion state and simulated trajectory to simulator initial state,
    // which is at the plan release time, i.e. the beginning of the estimation interval
    motion_state_t              receivedMotionState;
    std::vector<motion_state_t> trajectoryToInitialState;

    // all the trajectories evaluated over the planning cycle
    std::size_t numTrajectories;        // <= trajectories.size() -- memory reused
    std::vector<robot_trajectory_debug_info_t> trajectories;

    // the (optimial) output trajectory found by the trajectory planner
    robot_trajectory_debug_info_t plannedTrajectory;

    robot_trajectory_debug_info_t coarseOptimizerOutput;
    robot_trajectory_debug_info_t localOptimizerOutput;

    void clear(void)
    {
        iteration             = 0;
        updateStartTimeUs     = 0;
        updateEndTimeUs       = 0;
        planReleaseTimeUs     = 0;
        clearanceToStaticObs  = 0;
        clearanceToDynObs     = 0;
        receivedMotionState   = motion_state_t();
        trajectoryToInitialState.clear();

        numTrajectories = 0;
    };

    void addTrajectory(const robot_trajectory_info_t& trajectory)
    {
        if(numTrajectories >= trajectories.size())
        {
            trajectories.resize(numTrajectories + 10);
        }

        trajectories[numTrajectories].assign(trajectory);
        ++numTrajectories;
    }

    // Retrieve trajectory at the back -- not the same as trajectories.back!
    const robot_trajectory_debug_info_t& back(void) const
    {
        assert(numTrajectories > 0);
        return trajectories[numTrajectories - 1];
    }
};

// Serialization support. Only the robot_trajectory_debug_info_t will be communicated between modules.
template <class Archive>
void serialize(Archive& ar, trajectory_planner_debug_info_t& info)
{
    ar (info.iteration,
        info.updateStartTimeUs,
        info.updateEndTimeUs,
        info.planReleaseTimeUs,
        info.clearanceToStaticObs,
        info.clearanceToDynObs,
        info.receivedMotionState,
        info.trajectoryToInitialState,
        info.trajectories,
        info.plannedTrajectory,
        info.coarseOptimizerOutput,
        info.localOptimizerOutput,
        info.numTrajectories);
}

} // mpepc
} // vulcan

DEFINE_DEBUG_MESSAGE(mpepc::trajectory_planner_debug_info_t, ("MPEPC_TRAJECTORY_PLANNER_DEBUG_INFO"))

#endif // MPEPC_TRAJECTORY_PLANNER_INFO_H
