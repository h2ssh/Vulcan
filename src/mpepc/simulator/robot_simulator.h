/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_simulator.h
* \author   Jong Jin Park
*
* Declaration of RobotSimulator
*/

#ifndef MPEPC_ROBOT_SIMULATOR_H
#define MPEPC_ROBOT_SIMULATOR_H

#include <mpepc/simulator/params.h>
#include <mpepc/control/kinematic_control_law.h>
#include <mpepc/control/joystick_control_law.h>
#include <robot/model/robot_plant_model.h>

namespace vulcan
{
struct motion_state_t;
struct pose_t;

namespace robot
{
    struct motion_command_t;
}

namespace mpepc
{

struct robot_trajectory_info_t;

/**
* RobotSimulator estimates future trajectories of the robot in free space, based on the current estimate
* of the robot state, the previous motion taget, replanning interval and the next motion target (candidate).
*
* The RobotSimulator has four important internal objects:
* (1) simulatorKinematicControlLaw that outputs reference kinematics, including reference velocities;
* (2) simulatorJoystickControl which transforms the kinematic control output to joystick commands;
* (3) simulatorKinematicRobotPlantModel, which is a kinematic robot plant model for approximate, dynamic-free estimation;
* (4) simulatorDynamicRobotPlantModel, a fully dynamic robot plant model for accurate prediction.
*
* The robot state is estimated recursively, and the RobotSimulator eventually outputs an estimated (simulated)
* trajectory of the robot of some fixed duration or until the target is reached.
*/
class RobotSimulator
{

public:

    /**
    * constructor for RobotSimulator.
    *
    * \param   params           Parameters for the simulator.
    */
    RobotSimulator(const robot_simulator_params_t& params);

    // set internal variables
    void setTimeStep  (float timeStep)   { defaultTimeStep_   = timeStep; };
    void setTimeLength(float timeLength) { defaultTimeLength_ = timeLength; };

    /**
    * setupForOptimization initalizes internal state of the simulator to facillitate optimization.
    *
    * \param    robotState             Time-stamped state of the robot from the state estimator
    * \param    previousMotionTarget   Control law target used when the robotState was estimated
    * \param    startTimeUs            Beginning of the estimation range in microsecond.
    * \return   simulated state vector from the received robot state to the beginning of the simulation range.
    */
    std::vector<motion_state_t> setupForOptimization(const motion_state_t& robotState,
                                                            const motion_target_t&       previousMotionTarget,
                                                            int64_t                      startTimeUs);

    // indicator for state initialization
    bool isInitialized(void) { return isInitialized_; };
    void clear(void)         { isInitialized_ = false; }; // be lazy and just change the tag.

    motion_state_t getInitializedSimulatorState(void) { return initializedSimulatorState_; };

    /**
    * estimateRobotTrajectory estimates trajectory of the robot using the robot model specified in the parameters,
    * either until a given duration (sec), or if no duration is given, until the trajectory reaches the target.
    * If negative value is given for the time horizon, then the trajectory is simulated until it reaches the target.
    *
    * \param    candidateMotionTarget   Motion target to be used in the next planning cycle.
    * \param    timeStep                Time interval between simulated data points. (optional)
    * \param    timeLength              Time duration of the simulation. (optional)
    * \internal initializedPreviousAngularVelocityCommand_, initializedConvergenceStartTimeUs_,
    *           initializedSimulatorStartTimeUs_ and initializedSimulatorState_.
    * \param[out]  trajectory       robot_trajectory_info_t which contains the simulated trajectory.
    */
    void estimateRobotTrajectory(const motion_target_t& candidateMotionTarget,
                                 float timeStep,
                                 float timeLength,
                                 robot_trajectory_info_t& trajectory);
    // Same as estimateRobotTrajectory, but use the values from setTimeStep and setTimeLength
    void estimateRobotTrajectory(const motion_target_t& candidateMotionTarget, robot_trajectory_info_t& trajectory);

    /**
    * estimateRobotTrajectoryFromInitialState estimates trajectory of the robot from the speicifed initial state.
    * This does not use any pre-setting of internal states.*/
    void estimateRobotTrajectoryFromIntialState(const   motion_target_t&       candidateMotionTarget,
                                                const   motion_state_t& initialState,
                                                int64_t initialTimeUs,
                                                float   timeStep,
                                                float   timeLength,
                                                robot_trajectory_info_t& trajectory,
                                                float   prevAngularVelocityCommand = 0.0f,
                                                int64_t convergenceStartTimeUs = -1);

private:

    // controllers
    KinematicControlLaw simulatorKinematicControlLaw_;
    JoystickControlLaw  simulatorJoystickControl_;

    // plant model
    std::unique_ptr<robot::RobotPlantModel> simulatorRobotPlantModel_;

    // time step and time length
    float defaultTimeStep_;
    float defaultTimeLength_;

    // internal storage for efficient initialization during optimization routine.
    bool                  isInitialized_;
    float                 initializedPreviousAngularVelocityCommand_;
    int64_t               initializedConvergenceStartTimeUs_;
    int64_t               initializedSimulatorStartTimeUs_;
    motion_state_t initializedSimulatorState_;

    // parameters
    robot_simulator_params_t params_;
};


} // mpepc
} // vulcan

#endif // MPEPC_SIMULATOR_MOTION_SIMULATOR_H
