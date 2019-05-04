/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     differential_torque_drive.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of DifferentialTorqueDrive.
*/

#ifndef DIFFERENTIAL_TORQUE_PLANT_H
#define DIFFERENTIAL_TORQUE_PLANT_H

#include <robot/model/robot_plant_model.h>

namespace vulcan
{

namespace robot
{

const std::string DIFFERENTIAL_TORQUE_PLANT_TYPE("differential_torque_plant");

/**
* DifferentialTorqueDrive is a plant model considering the velocity of each
* wheel individually. The parameter under control is the maximum wheel acceleration.
* Each update considers the necessary wheel velocity and changes them to produce
* the final velocities.
*/
class DifferentialTorquePlant : public RobotPlantModel
{
public:

    /**
    * Constructor for DifferentialTorqueDrive.
    *
    * \param    params          Parameters for the robot plant model.
    */
    DifferentialTorquePlant(const differential_torque_plant_params_t& params);

    // RobotPlantModel interface
    /**
    * nextState calculates the state of the robot at the next time step based on the command
    * and previous state of the robot.
    *
    * \param    state           Complete state of the robot at the beginning of the update cycle
    * \param    command         Command received by the robot (simulator)
    * \param    timestep        Time interval between the current and the next state
    * \return   New state of the robot.
    */
    virtual motion_state_t nextState(const motion_state_t& state, const motion_command_t& command, float timestep) final; /* override */

//     /**
//     * nextState calculates the state of the robot at the next time step based on the next
//     * desired velocity and previous state of the robot.
//     *
//     * \param    desired         Desired velocity of the robot
//     * \param    state           Complete state of the robot on the last update
//     * \param    timestep        Time step over which to evaluate the trajectory
//     * \return   New state of the robot.
//     */
//     virtual motion_state_t nextState(const velocity_t& desired, const motion_state_t& state, float timestep);

private:

    struct wheel_speeds_t
    {
        float left;
        float right;

        wheel_speeds_t(void) {}
        wheel_speeds_t(const velocity_t& velocity, float wheelbase);

        velocity_t toRobot(float wheelbase) const;
    };

    differential_torque_plant_params_t params_;
};

} // robot
} // vulcan

#endif // PLANNER_METRIC_SIMULATOR_DIFFERENTIAL_TORQUE_DRIVE_H
