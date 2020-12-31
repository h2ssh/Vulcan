/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pd_robot_plant.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of PDRobot.
*/

#ifndef PD_ROBOT_PLANT_H
#define PD_ROBOT_PLANT_H

#include "robot/model/robot_plant_model.h"

namespace vulcan
{

namespace robot
{
    
const std::string PD_ROBOT_PLANT_TYPE("pd_robot_plant");

/**
* PDRobot is an instance of RobotPlantModel that considers the dynamics of the system,
* acceleration and jerk, to determine the motion of the robot. The dynamics of the robot
* is approximated as a simple mass-spring-damper (2nd order system), i.e. a PD control.
* 
* The configuration parameters for PDRobot are:
*
*   [PDRobotParameters]
*   p_gain                          = proportional gain for velocity control
*   d_gain                          = differential gain for velocity control
*   use_acceleration_saturation     = flag for whether acceleration should saturate
*   use_jerk_saturation             = flag for whether jerk should saturate
*   linear_acceleration_saturation  = value at which linear acceleration saturates
*   angular_acceleration_saturation = values at which angular acceleration saturates
*/
class PDRobotPlant : public RobotPlantModel
{
public:

    /**
    * Constructor for PDRobot.
    *
    * \param    params          Parameters determining the behavior of the robot plant model
    */
    PDRobotPlant(const pd_robot_plant_params_t& params);

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

    pd_robot_plant_params_t params_;
};


} // robot
} // vulcan

#endif // PD_ROBOT_PLANT_H
