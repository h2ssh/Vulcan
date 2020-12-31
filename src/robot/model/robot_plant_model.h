/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     robot_plant_model.h
* \author   Jong Jin Park and Collin Johnson
*
* Declaration of the RobotPlantModel interface for the RobotSimulator and the create_robot_plant_model()
* factory for creating new instances of RobotPlantModel.
*/

#ifndef ROBOT_PLANT_MODEL_H
#define ROBOT_PLANT_MODEL_H

#include "core/motion_state.h"
#include "robot/commands.h"
#include "robot/model/params.h"
#include <memory>
#include <string>

namespace vulcan
{

namespace robot
{
    
class RobotPlantModel
{
public:
    
    /**
    * virtual destructor for the RobotPlantModel.
    */
    virtual ~RobotPlantModel(void) {};

    /**
    * nextState calculates the state of the robot at the next time step based on the next
    * desired velocity and previous state of the robot.
    *
    * \param    state           Complete state of the robot at the beginning of the update cycle
    * \param    command         Command received by the robot simulator
    * \param    timestep        Time interval between the current and the next state
    * \return   New state of the robot.
    */
    virtual motion_state_t nextState(const motion_state_t& state, const motion_command_t& command, float timestep) = 0;
    
    bool isUsingJoystick(const plant_model_params_t& params);
    
protected:
    
    /**
    * turnDriveTurnIntegration is a 4-th order fixed-time integration of poses assuming a half_turn-drive-half_turn motion.
    * Rather than doing a direct Euler integral, this method leapfrogs the states based on the average linear and angular velocities.
    * Despite its simplicity, the model does caputure the non-holonmic constraints of the robot (that it cannot move sideways),
    * and we can get satisfactory precison without using external ODE libraries using this very simple, fixed-time integrator.
    * 
    * \param    priorPose               Pose to be integrated from
    * \param    averageVelocity         Average velocity over the discrete time step
    * \param    timestep                Duration of the time step
    * \return   The next pose as the result of the integration
    */
    pose_t turnDriveTurnIntegration(const pose_t& priorPose, const velocity_t& averageVelocity, float timestep);
};


/**
* create_robot_plant_model is a factory for creating new instances of the RobotPlantModel.
* 
* \param    type        Type of plant model for the robot to be created
* \param    params      Parameters for the plant model
* \return   Instance of RobotPlantModel.
*/

std::unique_ptr<RobotPlantModel> create_robot_plant_model(const std::string& type, const plant_model_params_t& params);


} // robot
} // vulcan

#endif // ROBOT_PLANT_MODEL_H