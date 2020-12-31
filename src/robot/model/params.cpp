/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.cpp
* \author   Jong Jin Park
*
* Definition of load functions for the robot_model_params_t params structs.
*/

#include "robot/model/params.h"
#include "utils/config_file.h"
#include "utils/config_file_utils.h"
#include <cassert>

namespace vulcan
{
namespace robot
{
    
const std::string kRobotCollisionModelHeading("RobotCollisionModelParameters");
const std::string kCollisionModelTypeKey     ("collision_model_type");
const std::string kCircleModelRadiusKey      ("circle_model_radius_m");
const std::string kRectangleModelKey         ("rectangle_model");
const std::string kConvexPolygonModelKey     ("convex_polygon_model");
const std::string kBodyCellSizeKey           ("body_cell_size_m");

const std::string kRobotPlantModelHeading("RobotPlantModelParameters");
const std::string kDynamicsModelTypeKey  ("plant_model_type");
    
const std::string kPDRobotHeading  ("PDRobotParameters");
const std::string kPGainKey        ("p_gain");
const std::string kDGainKey        ("d_gain");
const std::string kUseAccelSatKey  ("use_acceleration_saturation");
const std::string kUseJerkSatKey   ("use_jerk_saturation");
const std::string kLinAccSatValKey ("linear_acceleration_saturation");
const std::string kAngAccSatValKey ("angular_acceleration_saturation");
const std::string kLinJerkSatValKey("linear_jerk_saturation");
const std::string kAngJerkSatValKey("angular_jerk_saturation");

const std::string kDiffTorquesHeading("DifferentialTorquesParameters");
const std::string kWheelbase1Key     ("wheel_base");
const std::string kWheelAccelKey     ("wheel_accel_max");
const std::string kWheelVelKey     ("wheel_vel_max");

const std::string kDiffMotorsHeading("DifferentialMotorsParameters");
const std::string kWheelbase2Key    ("wheel_base");
const std::string kMotorMuKey       ("motor_mu");
const std::string kMotorBetaKey     ("motor_beta");
const std::string kMotorGammaKey    ("motor_gamma_coeff");
const std::string kMotorAlphaKey    ("motor_alpha_coeff");
const std::string kTurnInPlaceKey   ("turn_rate_in_place");
const std::string kTurnRateBaseKey  ("turn_rate_base");
const std::string kTurnReductionKey ("turn_rate_reduction");


robot_model_params_t::robot_model_params_t(const utils::ConfigFile& config)
: collisionModelParams(collision_model_params_t(config))
, plantModelParams(plant_model_params_t(config))
{
}


collision_model_params_t::collision_model_params_t(const utils::ConfigFile& config)
: type(config.getValueAsString(kRobotCollisionModelHeading, kCollisionModelTypeKey))
, circleModelRadius(config.getValueAsFloat(kRobotCollisionModelHeading, kCircleModelRadiusKey))
, rectangleModel(utils::create_rectangle_from_string(config.getValueAsString(kRobotCollisionModelHeading, kRectangleModelKey)))
, convexPolygonModel(utils::create_point_vector_from_string(config.getValueAsString(kRobotCollisionModelHeading, kConvexPolygonModelKey)))
, bodyCellSize(config.getValueAsFloat(kRobotCollisionModelHeading, kBodyCellSizeKey))
{
    assert(convexPolygonModel.size() > 3); // need at least 4 vertices to make a proper polygon (the first and the last vertex is always identical)
    assert(bodyCellSize > 0.01f);
}


plant_model_params_t::plant_model_params_t(const utils::ConfigFile& config)
: type(config.getValueAsString(kRobotPlantModelHeading, kDynamicsModelTypeKey))
, pdRobotPlantParams(config)
, differentialTorquePlantParams(config)
, differentialMotorsPlantParams(config)
{
}


pd_robot_plant_params_t::pd_robot_plant_params_t(const utils::ConfigFile& config)
: pGain(config.getValueAsFloat(kPDRobotHeading, kPGainKey))
, dGain(config.getValueAsFloat(kPDRobotHeading, kDGainKey))
, useAccelerationSaturation(config.getValueAsBool(kPDRobotHeading, kUseAccelSatKey))
, useJerkSaturation(config.getValueAsBool(kPDRobotHeading, kUseJerkSatKey))
, linearAccelSaturation(config.getValueAsFloat(kPDRobotHeading, kLinAccSatValKey))
, angularAccelSaturation(config.getValueAsFloat(kPDRobotHeading, kAngAccSatValKey))
, linearJerkSaturation(config.getValueAsFloat(kPDRobotHeading, kLinJerkSatValKey))
, angularJerkSaturation(config.getValueAsFloat(kPDRobotHeading, kAngJerkSatValKey))
{
}


differential_torque_plant_params_t::differential_torque_plant_params_t(const utils::ConfigFile& config)
: wheelbase(config.getValueAsFloat(kDiffTorquesHeading, kWheelbase1Key))
, wheelAccelMax(config.getValueAsFloat(kDiffTorquesHeading, kWheelAccelKey))
, wheelVelMax(config.getValueAsFloat(kDiffTorquesHeading, kWheelVelKey))
{
}


differential_motors_plant_params_t::differential_motors_plant_params_t(const utils::ConfigFile& config)
: wheelbase(config.getValueAsFloat(kDiffMotorsHeading, kWheelbase2Key))
, motorMu(config.getValueAsFloat(kDiffMotorsHeading, kMotorMuKey))
, motorBeta(config.getValueAsFloat(kDiffMotorsHeading, kMotorBetaKey))
, motorGamma(config.getValueAsFloat(kDiffMotorsHeading, kMotorGammaKey))
, motorAlpha(config.getValueAsFloat(kDiffMotorsHeading, kMotorAlphaKey))
, turnRateInPlace(config.getValueAsFloat(kDiffMotorsHeading, kTurnInPlaceKey))
, turnRateBase(config.getValueAsFloat(kDiffMotorsHeading, kTurnRateBaseKey))
, turnReductionRate(config.getValueAsFloat(kDiffMotorsHeading, kTurnReductionKey))
{
    assert(wheelbase         > 0.0f);
    assert(motorMu           > 0.0f);
    assert(motorBeta         > 0.0f);
    assert(motorGamma        > 0.0f);
    assert(motorAlpha        > 0.0f);
    assert(turnRateInPlace   > 0.0f);
    assert(turnRateBase      > 0.0f);
    assert(turnReductionRate < 0.01f);
    
    motorGamma *= motorBeta/motorMu;
    motorAlpha *= motorBeta;
}


} // robot
} // vulcan