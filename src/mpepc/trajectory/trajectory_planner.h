/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file      trajectory_planner.h
* \author    Jong Jin Park
* 
* Declaration of TrajectoryPlanner.
*/

#ifndef MPEPC_TRAJECTORY_PLANNER_H
#define MPEPC_TRAJECTORY_PLANNER_H

#include <mpepc/simulator/robot_simulator.h>
#include <mpepc/trajectory/trajectory_evaluator.h>
#include <mpepc/trajectory//mpepc_optimizer.h>
#include <mpepc/trajectory//mpepc_pose_follower.h>
#include <mpepc/trajectory/params.h>
#include <memory>

namespace vulcan
{

namespace mpepc
{

struct trajectory_planner_output_t;
struct trajectory_planner_debug_info_t;

class TrajectoryPlanner
{
public:
    
    /**
    * Constructor for TrajectoryPlanner.
    *
    * \param    params          Parameters for TrajectoryPlanner
    */
    TrajectoryPlanner(const trajectory_planner_params_t& params);
    
    // set timestep and time horizon for the trajectory planner
    void setTimeStep  (float timeStep)   { simulator_.setTimeStep  (timeStep); };
    void setTimeLength(float timeLength) { simulator_.setTimeLength(timeLength); };
    
    void setModePoseFollowing(bool tf) { isInPoseFollowingMode_ = tf; };
    
    /**
    * run runs the trajectory planner by passing required information to simulator,
    * evaluator and optimizer in correct order. It is also responsible for collecting
    * debug info gathered during the planning process.
    *
    * \param    task                 Task information.
    * \param    robotState           State of the robot.
    * \param    map                  Location and distances to static obstacles.
    * \param    objects              Estimated trajectories of dynamic objects.
    * \param    previousMotionTarget Previous solution executed by the metric planner.
    * \param    planReleaseTimeUs    Scheduled release of the output. Important for simulator.
    * \param    debugInfo            Relevant debug information to be populated by the trajectory planner (output)
    * \return   trajectory planner output which contains the motion target for the controller.
    */
    trajectory_planner_output_t run(const TaskManifold&                             task,
                                    const motion_state_t                     robotState,
                                    const ObstacleDistanceGrid&                     map,
                                    const std::vector<dynamic_object_trajectory_t>& objects,
                                    const motion_target_t&                          previousMotionTarget,
                                    int64_t                                         planReleaseTimeUs,
                                    trajectory_planner_debug_info_t&                debugInfo);
    
private:
    
    RobotSimulator      simulator_;
    TrajectoryEvaluator evaluator_;
    MPEPCOptimizer      optimizer_;
    MPEPCPoseFollower   poseFollower_;
    
    trajectory_planner_params_t params_;
    
    bool isInPoseFollowingMode_;
};  
    
} // mpepc
} // vulcan

#endif // MPEPC_TRAJECTORY_PLANNER_H