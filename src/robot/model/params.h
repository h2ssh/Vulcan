/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Jong Jin Park
* 
* Definition of robot_model_params_t, robot_collision_model_parasm_t, and robot_dynamics_model_params_t.
*/

#ifndef ROBOT_MODEL_PARAMS_H
#define ROBOT_MODEL_PARAMS_H

#include <string>
#include "math/geometry/polygon.h"
#include "math/geometry/rectangle.h"

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace robot
{

const std::string kRobotModelConfigFilename("robot_model.cfg");


/**
* differential_motors_plant_params_t
*/
struct differential_motors_plant_params_t
{
    float wheelbase = 0.603;
    
    float motorMu    = 0.26; // mechanical energy loss due to rolling friction. higher alpha can induce higher oscillation
    float motorBeta  = 5.5;  // motor energy change from velocity lower beta induces slower response and higher overshoot
    float motorGamma = 0.165*motorBeta/motorMu; // motor energy loss due to electric resistance, etc.
    float motorAlpha = 0.026*motorBeta; // mapping from drive command to motor input.
    
    float turnRateInPlace   = 0.45;
    float turnRateBase      = 0.45;
    float turnReductionRate = 0.006;
    
    differential_motors_plant_params_t(void) {};
    differential_motors_plant_params_t(const utils::ConfigFile& config);
};

/**
* differential_torque_plant_params_t
*/
struct differential_torque_plant_params_t
{
    float wheelbase;
    float wheelAccelMax;
	float wheelVelMax;
    
    differential_torque_plant_params_t(void) {};
    differential_torque_plant_params_t(const utils::ConfigFile& config);
};

/**
* pd_robot_plant_params_t
*/
struct pd_robot_plant_params_t
{
    float pGain;
    float dGain;

    bool  useAccelerationSaturation;
    bool  useJerkSaturation;

    float linearAccelSaturation;
    float angularAccelSaturation;

    float linearJerkSaturation;
    float angularJerkSaturation;
    
    pd_robot_plant_params_t(void) {};
    pd_robot_plant_params_t(const utils::ConfigFile& config);
};

/**
* plant_model_params_t
*/
struct plant_model_params_t // parameters for robot plant models
{
    std::string type; // kinematic_robot, pd_robot, differential_torque_drive, and differential_drive_with_motors
    
    // no parameter is required for simple kinemtic robot, as command instantly turns into state
    pd_robot_plant_params_t            pdRobotPlantParams;
    differential_torque_plant_params_t differentialTorquePlantParams;
    differential_motors_plant_params_t differentialMotorsPlantParams;
    
    plant_model_params_t(void) {};
    plant_model_params_t(const utils::ConfigFile& config);
};


/**
* collision_model_params_t
*/
struct collision_model_params_t
{
    std::string            type;
    float                  circleModelRadius;
    math::Rectangle<float> rectangleModel;
    math::Polygon<float>   convexPolygonModel;
    float                  bodyCellSize;
    
    collision_model_params_t(void) {};
    collision_model_params_t(const utils::ConfigFile& config);
};


/**
* robot_model_params_t
*/
struct robot_model_params_t
{
    collision_model_params_t collisionModelParams;
    plant_model_params_t     plantModelParams;
    
    robot_model_params_t(void) {};
    robot_model_params_t(const utils::ConfigFile& config);
};

} // robot
} // vulcan

#endif // ROBOT_MODEL_PARAMS_H