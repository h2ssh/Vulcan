/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     robot_plant_model.cpp
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of the factory for creating RobotPlantModel instances.
 */

#include "robot/model/robot_plant_model.h"
#include "robot/model/differential_motors_plant.h"
#include "robot/model/differential_torque_plant.h"
#include "robot/model/kinematic_robot_plant.h"
#include "robot/model/pd_robot_plant.h"
#include <iostream>

namespace vulcan
{

namespace robot
{

bool RobotPlantModel::isUsingJoystick(const plant_model_params_t& params)
{
    if (params.type == DIFFERENTIAL_TORQUE_PLANT_TYPE) {
        return true;
    } else {
        return false;
    }
}


pose_t RobotPlantModel::turnDriveTurnIntegration(const pose_t& priorPose,
                                                 const velocity_t& averageVelocity,
                                                 const float timestep)
{
    float halfDeltaTheta = averageVelocity.angular * timestep * 0.5;
    float travelDistance = averageVelocity.linear * timestep;

    return pose_t(priorPose.x + travelDistance * cos(priorPose.theta + halfDeltaTheta),
                  priorPose.y + travelDistance * sin(priorPose.theta + halfDeltaTheta),
                  angle_sum(priorPose.theta, halfDeltaTheta * 2.0));
}


std::unique_ptr<RobotPlantModel> create_robot_plant_model(const std::string& type, const plant_model_params_t& params)
{
    if (type == KINEMATIC_ROBOT_PLANT_TYPE) {
        return std::unique_ptr<RobotPlantModel>(new KinematicRobotPlant());
    } else if (type == PD_ROBOT_PLANT_TYPE) {
        return std::unique_ptr<RobotPlantModel>(new PDRobotPlant(params.pdRobotPlantParams));
    } else if (type == DIFFERENTIAL_TORQUE_PLANT_TYPE) {
        return std::unique_ptr<RobotPlantModel>(new DifferentialTorquePlant(params.differentialTorquePlantParams));
    } else if (type == DIFFERENTIAL_MOTORS_PLANT_TYPE) {
        return std::unique_ptr<RobotPlantModel>(new DifferentialMotorsPlant(params.differentialMotorsPlantParams));
    } else {
        std::cerr << "ERROR: Unknown robot plant type: " << type << std::endl;
        assert(false);
    }

    return std::unique_ptr<RobotPlantModel>();
}


}   // namespace robot
}   // namespace vulcan
