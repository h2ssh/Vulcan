/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     kinematic_robot.h
* \author   Collin Johnson and Jong Jin Park
*
* Declaration of KinematicRobotPlant.
*/

#ifndef KINEMATIC_ROBOT_PLANT_H
#define KINEMATIC_ROBOT_PLANT_H

#include <robot/model/robot_plant_model.h>

namespace vulcan
{

namespace robot
{

const std::string KINEMATIC_ROBOT_PLANT_TYPE("kinematic_robot_plant");

/**
* KinematicRobotPlant is a very simple robot plant model assuming:
*
*   A) The commanded velocity is realized instantly (i.e. infinite acceleration is assumed).
*   B) The robot moves along an constant-velocity arc at each time step.
*/
class KinematicRobotPlant : public RobotPlantModel
{
public:

    /**
    * Constructor for SimpleKinematicController.
    */
    KinematicRobotPlant(void);

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
    virtual motion_state_t nextState(const motion_state_t& state, const motion_command_t& command, const float timestep) final; /* override */
};


} // robot
} // vulcan

#endif // KINEMATIC_ROBOT_PLANT_H
