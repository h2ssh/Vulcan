/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_trajectory_info.h
* \author   Jong Jin Park and Collin Johnson
*
* Definition of robot_trajectory_info_t, robot_trajectory_debug_info_t,
* robot_trajectory_debug_cost_type_t, get_trj_debug_cost_text(),
* and robot_trajectory_debug_info_simple_t.
*/

#ifndef MPEPC_ROBOT_TRAJECTORY_INFO_H
#define MPEPC_ROBOT_TRAJECTORY_INFO_H

#include <mpepc/control/control_law_coordinates.h>
#include <core/motion_state.h>
#include <robot/commands.h>
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>

namespace vulcan
{

namespace mpepc
{

/**
* robot_trajectory_info_t stores an explicit trajectory of robot states, its parameterization and full evaluation results.
*/
struct robot_trajectory_info_t
{
    // from trajectory simulator
    // timing information
    int64_t                                 timestamp;             ///< Time stamp at the beginning of the trajectory
    float                                   timestep;              ///< Time between simulated poses

    // states and command trajectory
    std::vector<motion_state_t>      states;                ///< Pose, velocity, acceleration and wheel states
    std::vector<robot::motion_command_t>    commands;              ///< Robot velocity and joystick commands.
    // NOTE: The length of the commands vector will be smaller than that of the states vector by one, as the commands from the final state will not be calculated.

    // trajectory parameterization via graceful kinematic control law (Park and Kuipers, ICRA 2011, Park, Johnson and Kuipers, IROS 2012)
    std::vector<control_law_coordinates_t>  controlLawCoordinates; ///< Tobot trajectory in control law coordinate
    motion_target_t                         motionTarget;          ///< Target pose, approach direction and velocity gain

    // flag for convergence to the target (from the controller)
    bool                                    hasArrivedAtTarget;    ///< Flag indicating this trajectory has reached the motion target


    // from trajectory evaluator
    // evaluated costs of trajectory segments along the trajectory.
    std::vector<float>                      piecewiseSurvivability; ///< Survivability at each trajectory segment evaluated
    std::vector<float>                      piecewiseRawProgress;   ///< Raw progress (not multiplied by the survivability) at each trajectory segement evaluated
    std::vector<float>                      piecewiseCollisionCost; ///< Expected collision cost at each trajectory segment evaluated
    std::vector<float>                      piecewiseActionCost;    ///< Expected action cost at each trajectory segment evaluated

    std::vector<float>                      distanceToStaticObj;
    std::vector<float>                      distanceToDynamicObj;

    // useful (time-related) quantities resulting from trajectory evaluations.
    float                                   expectedCost;           ///< Total expected cost for this trajectory
    float                                   totalSurvivability;     ///< Total survivability of the trajectory, i.e. the final survivability at the end pose of the trajectory
    float                                   expectedProgress;       ///< Total expected progress
    float                                   expectedCollisionCost;  ///< Total expected cost of collision
    float                                   expectedActionCost;     ///< Total expected cost of action

    float                                   heuristicCost;

    // useful (time-independent) quantities resulting from path evaulations.
    float                                   pathLength;             ///< Integrated length of the path, not considering collision
    float                                   pathCost;               ///< Integrated travel cost of the path, not considering collision

    // collision-related flags
    bool                                    hasStaticCollision;     ///< Flag indicating if the trajectory collides with the static world at any point
    bool                                    hasDynamicCollision;    ///< Flag indicating if the robot thinks it will collide with a dynamic object
    float                                   staticCollisionTime;    ///< Time at which the collision occurred relative to the start time of the trajectory
    float                                   dynamicCollisionTime;   ///< Time at which a dynamic collision first occurred relative to the start time of the trajectory

    /**
    * Store all poses for the trajectory in the provided vector. Poses are pushed back, so they'll be appended to any
    * existing data.
    */
    void getPoses(std::vector<pose_t>& poses) const;

    /**
    * Clear memory to enable easy reuse of the trajectory.
    */
    void clear(void);
};

/**
* robot_trajectory_debug_info_t stores a subuset of the full trajectory info needed for debugging/diplaying purposes.
*/
struct robot_trajectory_debug_info_t
{
    motion_target_t motionTarget;

    std::vector<pose_t> poses;

    std::vector<float> piecewiseSurvivability;
    std::vector<float> piecewiseRawProgress;
    std::vector<float> piecewiseCollisionCost;
    std::vector<float> piecewiseActionCost;

    float expectedCost;
    float totalSurvivability;
    float expectedProgress;
    float expectedCollisionCost;
    float expectedActionCost;

    float heuristicCost;

    bool  hasCollision;

    robot_trajectory_debug_info_t(void)
    {
    }

    robot_trajectory_debug_info_t(const robot_trajectory_info_t& trjInfo);

    /**
    * Assign from a robot_trajectory_info_t. Allows for memory reuse.
    */
    void assign(const robot_trajectory_info_t& trjInfo);
};

/**
* robot_trajectory_debug_cost_type_t is an enumeration of cost types in the debug message.
* NOTE: make sure this corresponds to the contents in robot_trajectory_debug_info_t!
*/
enum robot_trajectory_debug_cost_type_t
{
    EXPECTED_COST = 0,
    TOTAL_SURVIVABILITY,
    EXPECTED_PROGRESS,
    EXPECTED_COLLISION_COST,
    EXPECTED_ACTION_COST,
    PIECEWISE_SURVIVABILITY,
    PIECEWISE_PROGRESS, // this will be computed in the display widget
    PIECEWISE_RAW_PROGRESS,
    PIECEWISE_COLLSION_COST,
    PIECEWISE_ACTION_COST,
    NUM_COST_TYPES // this is here to help iterate over this enum. Feels like a hack but it works.
};

/**
* get_trj_debug_cost_text() returns a text description of each cost type in the debug message.
*/
inline std::string get_trj_debug_cost_text(robot_trajectory_debug_cost_type_t costType)
{
    switch(costType)
    {
        case EXPECTED_COST:
            return "Expected cost";

        case TOTAL_SURVIVABILITY:
            return "Survivability";

        case EXPECTED_PROGRESS:
            return "Expected progress";

        case EXPECTED_COLLISION_COST:
            return "Expected collision cost";

        case EXPECTED_ACTION_COST:
            return "Expected action cost";

        case PIECEWISE_SURVIVABILITY:
            return "Piecewise survivability";

        case PIECEWISE_PROGRESS:
            return "Piecewise expected progress";

        case PIECEWISE_RAW_PROGRESS:
            return "Piecewise raw progress";

        case PIECEWISE_COLLSION_COST:
            return "Piecewise collision cost";

        case PIECEWISE_ACTION_COST:
            return "Piecewise action cost";

        case NUM_COST_TYPES: // intentional fall-through
        default:
            return "No description available!";
    }
}


/**
* robot_trajectory_debug_info_simple_t stores a minimal amount of info, used for storage.
*
* NOTE: If needed, it would be possible to store initial state, motion target and
*       other simulator parameters so that the full trajectory and evaluation
*       results can be regenerated from those minimal data. That could be useful
*       if we were to store large amount of trajectory evaluation history.
*/
struct robot_trajectory_debug_info_simple_t
{
    motion_target_t            motionTarget;
    std::vector<pose_t> poses;
    float                      expectedCost;

    robot_trajectory_debug_info_simple_t(void)
    {
    }

    robot_trajectory_debug_info_simple_t(const robot_trajectory_debug_info_t& debugInfo)
    : motionTarget(debugInfo.motionTarget)
    , poses       (debugInfo.poses)
    , expectedCost(debugInfo.expectedCost)
    {
    }
};


// Serialization support. Only the robot_trajectory_debug_info_t will be communicated between modules.
template <class Archive>
void serialize(Archive& ar, robot_trajectory_debug_info_t& info)
{
    ar (info.motionTarget,
        info.poses,
        info.piecewiseSurvivability,
        info.piecewiseRawProgress,
        info.piecewiseCollisionCost,
        info.piecewiseActionCost,
        info.expectedCost,
        info.totalSurvivability,
        info.expectedProgress,
        info.expectedCollisionCost,
        info.expectedActionCost,
        info.heuristicCost,
        info.hasCollision);
}

} // mpepc
} // vulcan

#endif // MPEPC_ROBOT_TRAJECTORY_INFO_H
