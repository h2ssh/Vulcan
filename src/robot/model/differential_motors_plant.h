/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** 
* \file   differntial_drive_with_motors.h
* \author Jong Jin Park and Paul Foster 
*
*/

#ifndef DIFFERENTIAL_MOTORS_PLANT_H
#define DIFFERENTIAL_MOTORS_PLANT_H

#include <robot/model/robot_plant_model.h>
#include <robot/model/motor_model.h>

namespace vulcan
{

namespace robot
{

const std::string DIFFERENTIAL_MOTORS_PLANT_TYPE("differential_motors_plant");

/**
* DifferentialMotorsPlant is an instance of RobotPlantModel which encodes
* discete-time dynamics of a differential wheeled vehicle with two independent motors.
* 
* This particular model is a composition of a Curtis controller model and a model for two independent motors.
*/
class DifferentialMotorsPlant : public RobotPlantModel
{
public:

    /**
    * Constructor for DifferentialMotorsPlant.
    *
    * \param    params          Parameters determining the behavior of the robot plant model.
    */
    DifferentialMotorsPlant(const differential_motors_plant_params_t& params);

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

private:
    
    differential_drive_wheels_t nextWheelState(const differential_drive_wheels_t& wheelState, const joystick_command_t& joystickCommand, float timestep);

    differential_motors_plant_params_t params_;

};


} // robot
} // vulcan

#endif // DIFFERENTIAL_MOTORS_PLANT_H
