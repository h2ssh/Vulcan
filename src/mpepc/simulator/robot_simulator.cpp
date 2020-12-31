/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     robot_simulator.cpp
 * \author   Jong Jin Park
 *
 * Definition of RobotSimulator
 */

#include "mpepc/simulator/robot_simulator.h"
#include "mpepc/trajectory/robot_trajectory_info.h"
#include "utils/timestamp.h"
#include <cmath>

// #define DEBUG_ROBOT_SIMULATOR
// #define DEBUG_ROBOT_SIMULATOR_TIME
// #define DEBUG_ROBOT_SIMULATOR_STATE_INITIALIZATION

namespace vulcan
{

namespace mpepc
{

RobotSimulator::RobotSimulator(const robot_simulator_params_t& params)
: simulatorKinematicControlLaw_(params.kinematicControlLawParams)   // the kinematic control law
, simulatorJoystickControl_(params.joystickControlLawParams,
                            params.robotPlantModelParams.differentialMotorsPlantParams)   // dynamic joystick controller
, simulatorRobotPlantModel_(
    robot::create_robot_plant_model(params.robotPlantModelParams.type,
                                    params.robotPlantModelParams))   // a dynamic robot model of the specified type
, isInitialized_(false)
, initializedPreviousAngularVelocityCommand_(0.0)
, initializedConvergenceStartTimeUs_(-1)
, initializedSimulatorStartTimeUs_(-1)
, params_(params)
{
    // default time step and length for simulation
    setTimeStep(0.05);
    setTimeLength(5.0);
}


std::vector<motion_state_t> RobotSimulator::setupForOptimization(const motion_state_t& robotState,
                                                                 const motion_target_t& previousMotionTarget,
                                                                 int64_t startTimeUs)
{
    if (startTimeUs == initializedSimulatorStartTimeUs_) {
        std::cout << "WARNING: RobotSimulator: Simulator may be reinitalizing to the same state.\n";
    }

#ifdef DEBUG_ROBOT_SIMULATOR_TIME
    int64_t currentTimeUs = utils::system_time_us();
    if (std::abs(startTimeUs - currentTimeUs)
        < utils::sec_to_usec(1))   // only do this when the system time makes sense
    {
        std::cout << "DEBUG: RobotSimulator: relative timestamp (ms): Pose      : "
                  << (robotState.pose.timestamp - currentTimeUs) / 1000 << '\n';
        std::cout << "DEBUG: RobotSimulator: relative timestamp (ms): Start Time: "
                  << (startTimeUs - currentTimeUs) / 1000 << '\n';
    }

    std::cout << "DEBUG: RobotSimulator: timestamp (ms): NextCycleStart   : " << startTimeUs / 1000 << '\n';
    std::cout << "DEBUG: RobotSimulator: timestamp (ms): MotionState      : " << robotState.timestamp / 1000 << '\n';
    std::cout << "DEBUG: RobotSimulator: timestamp (ms): Pose             : " << robotState.pose.timestamp / 1000
              << '\n';
    std::cout << "DEBUG: RobotSimulator: timestamp (ms): PoseDistribution : "
              << robotState.poseDistribution.timestamp / 1000 << '\n';
    std::cout << "DEBUG: RobotSimulator: timestamp (ms): Velocity         : " << robotState.velocity.timestamp / 1000
              << '\n';
    std::cout << "DEBUG: RobotSimulator: timestamp (ms): RightDriveWheel  : "
              << robotState.differentialWheels.rightWheel.timestamp / 1000 << '\n';
    std::cout << "DEBUG: RobotSimulator: timestamp (ms): LeftDriveWheel   : "
              << robotState.differentialWheels.leftWheel.timestamp / 1000 << '\n';
#endif

    // internal states
    motion_state_t simulatorState = robotState;
    motion_target_t motionTarget = previousMotionTarget;
    int64_t simulatorTimeUs = robotState.timestamp;

    // initialize the internal states of the simulator control law (needed for convergence detection)
    control_law_output_t controlLawOutput =
      simulatorKinematicControlLaw_.computeOutput(simulatorState.pose, motionTarget, 0);
    simulatorKinematicControlLaw_.resetInternalStates(controlLawOutput.angularVelocity,
                                                      -1);   // assume unconverged state at initialization

    // state trajectory to the simulator start state
    std::vector<motion_state_t> simulatedTrajectoryToInitialState;
    simulatedTrajectoryToInitialState.push_back(simulatorState);

#ifdef DEBUG_ROBOT_SIMULATOR_STATE_INITIALIZATION
    std::cout << "DEBUG: RobotSimulator: State initialization: (v, omega, rightwheel speed, left wheel speed)\n";
    std::cout << "    (" << simulatorState.velocity.linear << ' ' << simulatorState.velocity.angular << ' '
              << simulatorState.differentialWheels.rightWheel.speed << ' '
              << simulatorState.differentialWheels.leftWheel.speed << ")\n";
#endif

    // fast-forward state to the beginning of the simulation range
    int64_t timeIncrementUs = utils::sec_to_usec(defaultTimeStep_);   // discrete time increment
    while (simulatorTimeUs < startTimeUs) {
        // compute command
        control_law_output_t controlLawOutput =
          simulatorKinematicControlLaw_.computeOutput(simulatorState.pose, motionTarget, simulatorTimeUs);
        robot::velocity_command_t velocityCommand(controlLawOutput.linearVelocity, controlLawOutput.angularVelocity);
        robot::joystick_command_t joystickCommand =
          simulatorJoystickControl_.computeOutput(simulatorState, controlLawOutput);
        robot::motion_command_t simulatorCommand(robot::AUTONOMOUS_CONTROLLER, velocityCommand, joystickCommand);

        // propagate state and time
        timeIncrementUs = std::min(timeIncrementUs, startTimeUs - simulatorTimeUs);
        simulatorState = simulatorRobotPlantModel_->nextState(simulatorState, simulatorCommand, defaultTimeStep_);
        simulatorTimeUs += timeIncrementUs;

#ifdef DEBUG_ROBOT_SIMULATOR_STATE_INITIALIZATION
        std::cout << "    (" << simulatorState.velocity.linear << ' ' << simulatorState.velocity.angular << ' '
                  << simulatorState.differentialWheels.rightWheel.speed << ' '
                  << simulatorState.differentialWheels.leftWheel.speed << ")\n";
#endif

        // save state trajectory
        simulatedTrajectoryToInitialState.push_back(simulatorState);
    }

    // save data for actual simulation
    isInitialized_ = true;
    initializedConvergenceStartTimeUs_ =
      simulatorKinematicControlLaw_
        .getConvergenceStartTimeUs();   // controller remembers when the robot was within the convergence zone.
    initializedPreviousAngularVelocityCommand_ = simulatorKinematicControlLaw_.getPreviousAngularVelocityCommand();
    initializedSimulatorStartTimeUs_ = startTimeUs;
    initializedSimulatorState_ = simulatorState;

    return simulatedTrajectoryToInitialState;
}


// simulate robot trajectory toward a motion target to the previously set time horizon and time step
void RobotSimulator::estimateRobotTrajectory(const motion_target_t& candidateMotionTarget,
                                             robot_trajectory_info_t& trajectory)
{
    return estimateRobotTrajectory(candidateMotionTarget, defaultTimeStep_, defaultTimeLength_, trajectory);
}


// simulate robot trajectory toward a motion target to the given time horizon and timestep.
void RobotSimulator::estimateRobotTrajectory(const motion_target_t& candidateMotionTarget,
                                             float timeStep,
                                             float timeLength,
                                             robot_trajectory_info_t& trajectory)
{
    assert(isInitialized_);

    return estimateRobotTrajectoryFromIntialState(candidateMotionTarget,
                                                  initializedSimulatorState_,
                                                  initializedSimulatorStartTimeUs_,
                                                  timeStep,
                                                  timeLength,
                                                  trajectory,
                                                  initializedPreviousAngularVelocityCommand_,
                                                  initializedConvergenceStartTimeUs_);
}


// simulate robot trajectory toward a motion target to the given time horizon and timestep.
void RobotSimulator::estimateRobotTrajectoryFromIntialState(const motion_target_t& candidateMotionTarget,
                                                            const motion_state_t& initialState,
                                                            int64_t initialTimeUs,
                                                            float timeStep,
                                                            float timeLength,
                                                            robot_trajectory_info_t& trajectory,
                                                            float prevAngularVelocityCommand,
                                                            int64_t convergenceStartTimeUs)
{
    const float kDefaultDuration = 30.0;   // This should be a parameter...?

#ifdef DEBUG_ROBOT_SIMULATOR
    std::cout << "DEBUG: RobotSimulator: Running the simulator for: (timestep, duration, numSteps) = : (" << timeStep
              << ',' << timeLength << ',' << timeLength / timeStep_ << ")\n";
    if (timeLength < 0.0) {
        std::cout << "                           or if the motion target has been reached." << '\n';
    }
#endif

    // reset controller and flags
    simulatorKinematicControlLaw_.resetInternalStates(prevAngularVelocityCommand, convergenceStartTimeUs);
    bool hasReachedstopCondition = false;
    bool haveReachedTarget = false;

    // initial states and time
    motion_state_t simulatorState = initialState;
    int64_t simulatorTimeUs = initialTimeUs;

    // save to a trajectory
    trajectory.timestamp = simulatorTimeUs;
    trajectory.timestep = timeStep;
    trajectory.motionTarget = candidateMotionTarget;
    trajectory.states.push_back(simulatorState);
    // command is not computed yet

    int64_t timeIncrementUs = utils::sec_to_usec(timeStep);
    // do a default long timeLength is the timeLength is not properly specified. We're gonna exit when the target is
    // reached.
    size_t numStepsLeft = (timeLength < 0.0) ? kDefaultDuration / timeStep : timeLength / timeStep;
    while (!hasReachedstopCondition) {
        // compute command from the current state
        control_law_output_t controlLawOutput =
          simulatorKinematicControlLaw_.computeOutput(simulatorState.pose, candidateMotionTarget, simulatorTimeUs);
        robot::velocity_command_t velocityCommand(controlLawOutput.linearVelocity, controlLawOutput.angularVelocity);
        robot::joystick_command_t joystickCommand =
          simulatorJoystickControl_.computeOutput(simulatorState, controlLawOutput);
        robot::motion_command_t simulatorCommand(robot::AUTONOMOUS_CONTROLLER, velocityCommand, joystickCommand);

        // propagate simulated state and time
        simulatorTimeUs += timeIncrementUs;   // timestamps are in int64_t (microseconds)
        simulatorState = simulatorRobotPlantModel_->nextState(simulatorState,
                                                              simulatorCommand,
                                                              timeStep);   // planner interval is in float (seconds)

#ifdef DEBUG_ROBOT_SIMULATOR
        std::cout << "DEBUG: RobotSimulator: Command: (" << simulatorCommand.velocityCommand.linear << ','
                  << simulatorCommand.velocityCommand.angular << ',' << simulatorCommand.joystickCommand.forward << ','
                  << simulatorCommand.joystickCommand.left << ")\n";
        std::cout << "DEBUG: RobotSimulator: Pose: (" << simulatorState.pose.x << ',' << simulatorState.pose.y << ','
                  << simulatorState.pose.theta << ")\n";
#endif

        // record controller error and command
        trajectory.controlLawCoordinates.push_back(controlLawOutput.coords);
        trajectory.commands.push_back(simulatorCommand);

        // record updated state
        trajectory.states.push_back(simulatorState);

        // NOTE: command from the last state is never computed so the trajectory
        //       will have one less command sample than the state sample.

        // check to see if the robot has reached the target
        numStepsLeft--;
        haveReachedTarget = controlLawOutput.haveReachedTarget;
        hasReachedstopCondition = (timeLength < 0.0) ? (haveReachedTarget || numStepsLeft == 0) : numStepsLeft == 0;
    }

    // save if the trajectory is the one leading to the target
    trajectory.hasArrivedAtTarget = haveReachedTarget;
}


// // methods for estimating kinematic robot path connecting two poses.
// // FIXME: this needs to move out of the robot simulator. This function was to be used for RRT stuff, and to do that
// //        I need collision checking and I don't need to slow down. This will serve as a template, though.
// robot_trajectory_info_t RobotSimulator::estimateKinematicRobotPath(const pose_t&        startPose,
//                                                                    const pose_t&        targetPose,
//                                                                    const approach_direction_t& direction) const
// {
//
//     // initialize the simulatorKinematicContolLaw, a control law target, and initial time.
//     simulatorKinematicControlLaw_.initializeControlLawStates(0.0, -1);
//     motion_target_t controlLawTarget(targetPose, direction, 0.75);
// //    motion_target_t controlLawTarget(targetPose, direction, params_.velocityGainForKinematicPathGeneration);
//     int64_t simulatorTimeUs = 0;
//
//     // intialize trajectory structure
//     robot_trajectory_info_t  trajectory;
//     trajectory.poses.push_back(startPose);
//
//     // temporary containers
//     control_law_output_t       controlLawOutput;
//     simulator_motion_command_t motionCommand;
//     motion_state_t   simulatedMotionState;
//
//     // until-reached-target kinematic simulator
//     int64_t timeIncrementUs = int64_t(timeStep*1000000);
//     while(!trajectory.hasArrivedAtTarget)
//     {
//         // compute command
//         controlLawOutput = simulatorKinematicControlLaw_.computeOutput(simulatedMotionState.pose, controlLawTarget,
//         simulatorTimeUs);
//
//         // constructring motionCommand structure for the dynamic robot model. No need for joystick commands here.
//         motionCommand.linearVelocity  = controlLawOutput.linearVelocity;
//         motionCommand.angularVelocity = controlLawOutput.angularVelocity;
//
//
//         // propagate simulated state and time
//         simulatorTimeUs += timeIncrementUs;
//         simulatedMotionState = simulatorKinematicRobotPlantModel_.nextState(simulatedMotionState, motionCommand,
//         timeStep);
//
//         // record updated state
//         trajectory.poses.push_back(simulatedMotionState.pose);
//         trajectory.velocities.push_back(simulatedMotionState.velocity);
//
//         // check to see if the robot has reached the target
//         trajectory.hasArrivedAtTarget = simulatorKinematicControlLaw_.haveReachedTarget(simulatedMotionState.pose,
//         controlLawTarget.pose, simulatorTimeUs);
//     }
//
//     return trajectory;
// }

}   // namespace mpepc
}   // namespace vulcan
