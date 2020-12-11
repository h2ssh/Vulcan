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
* Definition of constructor for params struct for the TraejctoryPlanner.
*/

#include <mpepc/trajectory/params.h>
#include <utils/config_file.h>
#include <utils/config_file_utils.h>

namespace vulcan
{
namespace mpepc
{

const std::string kMPEPCOptimizerHeading("MPEPCOptimizerParameters");
const std::string kInitialTargetsKey    ("target_list_file");
const std::string kUseGlobalOpt         ("use_global_optimizer");
const std::string kUseLocalOptKey       ("use_local_optimizer");
const std::string kMinBoundRadiusKey    ("min_radius");
const std::string kMinBoundThetaKey     ("min_theta");
const std::string kMinBoundDeltaKey     ("min_delta");
const std::string kMinBoundK1Key        ("min_k1");
const std::string kMinBoundGainKey      ("min_velocity_gain");
const std::string kMaxBoundRadiusKey    ("max_radius");
const std::string kMaxBoundThetaKey     ("max_theta");
const std::string kMaxBoundDeltaKey     ("max_delta");
const std::string kMaxBoundK1Key        ("max_k1");
const std::string kMaxBoundGainKey      ("max_velocity_gain");
const std::string kTolRadiusKey         ("radius_tolerance");
const std::string kTolThetaKey          ("theta_tolerance");
const std::string kTolDeltaKey          ("delta_tolerance");
const std::string kGainTolKey           ("gain_tolerance");
const std::string kFtolKey              ("function_tolerance");
const std::string kGlobalOptMultKey     ("tolerance_multiplier_for_coarse_optimization");
const std::string kMaxIterGlobalKey     ("max_global_iterations");
const std::string kMaxIterLocalKey      ("max_local_iterations");
const std::string kInitialStepRadKey    ("radius_initial_step");
const std::string kInitialStepThetaKey  ("theta_initial_step");
const std::string kInitialStepDeltaKey  ("delta_initial_step");
const std::string kInitialStepGainKey   ("gain_initial_step");
const std::string kGriadentStepRadKey   ("radius_gradient_step_size");
const std::string kGradientStepThetaKey ("theta_gradient_step_size");
const std::string kGradientStepDeltaKey ("delta_gradient_step_size");
const std::string kGradientStepGainKey  ("gain_gradient_step_size");

const std::string kTrajectoryEvaluatorHeading ("TrajectoryEvaluatorParameters");
const std::string kEvaluatorSampleTimeKey     ("time_between_samples_s");
const std::string kUseFixedObjectRadiusKey    ("dynamic_object_has_fixed_radius");
const std::string kObjectRadiusKey            ("fixed_dynamic_object_radius_m");
const std::string kConsideSingleObjectKey     ("consider_only_the_closest_object");
const std::string kStaticPassingRadiusKey     ("min_passing_radius_static_obstacle_m");
const std::string KDynamicPassingRadiusKey    ("min_passing_radius_dynamic_object_m");
const std::string kStaticUncertaintyStdMinKey ("uncertainty_min_std_static_obstacle");
const std::string kDynamicUncertaintyStdMinKey("uncertainty_min_std_dynamic_object");
const std::string kLinVelUncertaintyStdKey    ("uncertainty_std_weight_linear_vel");
const std::string kAngVelUncertaintyStdKey    ("uncertainty_std_weight_angular_vel");
const std::string kLinVelActionWeightKey      ("action_cost_weight_linear_vel");
const std::string kAngVelActionWeightKey      ("action_cost_weight_angular_vel");
const std::string kNegativeVelCostMultKey     ("negative_velocity_cost_multiplier");
const std::string kLinAccActionWeightKey      ("action_cost_weight_linear_accel");
const std::string kAngAccActionWeightKey      ("action_cost_weight_angular_accel");
const std::string kConsiderCollisionAreaKey   ("should_consider_collision_area");
const std::string kBaseCollisionCostKey       ("base_piecewise_collision_cost");
const std::string kVelocityCollisionWeightKey ("velocity_weight_collision_cost");
const std::string kCollisionAreaWeightKey     ("collision_area_weight");
const std::string kAngularTravelCostWeightKey ("path_cost_angular_travel_weight");


mpepc_optimizer_params_t::mpepc_optimizer_params_t(const utils::ConfigFile& config)
: targetListFile        (config.getValueAsString(kMPEPCOptimizerHeading, kInitialTargetsKey))
, useGlobalOptimizer    (config.getValueAsBool  (kMPEPCOptimizerHeading, kUseGlobalOpt))
, useLocalOptimizer     (config.getValueAsBool  (kMPEPCOptimizerHeading, kUseLocalOptKey))
, minRadius             (config.getValueAsDouble(kMPEPCOptimizerHeading, kMinBoundRadiusKey))
, minTheta              (config.getValueAsDouble(kMPEPCOptimizerHeading, kMinBoundThetaKey))
, minDelta              (config.getValueAsDouble(kMPEPCOptimizerHeading, kMinBoundDeltaKey))
, minK1                 (config.getValueAsDouble(kMPEPCOptimizerHeading, kMinBoundK1Key))
, minVelocityGain       (config.getValueAsDouble(kMPEPCOptimizerHeading, kMinBoundGainKey))
, maxRadius             (config.getValueAsDouble(kMPEPCOptimizerHeading, kMaxBoundRadiusKey))
, maxTheta              (config.getValueAsDouble(kMPEPCOptimizerHeading, kMaxBoundThetaKey))
, maxDelta              (config.getValueAsDouble(kMPEPCOptimizerHeading, kMaxBoundDeltaKey))
, maxK1                 (config.getValueAsDouble(kMPEPCOptimizerHeading, kMaxBoundK1Key))
, maxVelocityGain       (config.getValueAsDouble(kMPEPCOptimizerHeading, kMaxBoundGainKey))
, radiusTolerance       (config.getValueAsDouble(kMPEPCOptimizerHeading, kTolRadiusKey))
, thetaTolerance        (config.getValueAsDouble(kMPEPCOptimizerHeading, kTolThetaKey))
, deltaTolerance        (config.getValueAsDouble(kMPEPCOptimizerHeading, kTolDeltaKey))
, gainTolerance         (config.getValueAsDouble(kMPEPCOptimizerHeading, kGainTolKey))
, functionTolerance     (config.getValueAsDouble(kMPEPCOptimizerHeading, kFtolKey))
, globalOptTolMultiplier(config.getValueAsDouble(kMPEPCOptimizerHeading, kGlobalOptMultKey))
, maxGlobalIterations   (config.getValueAsUInt32(kMPEPCOptimizerHeading, kMaxIterGlobalKey))
, maxLocalIterations    (config.getValueAsUInt32(kMPEPCOptimizerHeading, kMaxIterLocalKey))
, radiusInitialStep     (config.getValueAsDouble(kMPEPCOptimizerHeading, kInitialStepRadKey))
, thetaInitialStep      (config.getValueAsDouble(kMPEPCOptimizerHeading, kInitialStepThetaKey))
, deltaInitialStep      (config.getValueAsDouble(kMPEPCOptimizerHeading, kInitialStepDeltaKey))
, gainInitialStep       (config.getValueAsDouble(kMPEPCOptimizerHeading, kInitialStepGainKey))
, radiusGradientStepSize(config.getValueAsDouble(kMPEPCOptimizerHeading, kGriadentStepRadKey))
, thetaGradientStepSize (config.getValueAsDouble(kMPEPCOptimizerHeading, kGradientStepThetaKey))
, deltaGradientStepSize (config.getValueAsDouble(kMPEPCOptimizerHeading, kGradientStepDeltaKey))
, gainGradientStepSize  (config.getValueAsDouble(kMPEPCOptimizerHeading, kGradientStepGainKey))
{
    assert(maxRadius > minRadius);
    assert(maxTheta > minTheta);
    assert(maxDelta > minDelta);
    assert(maxVelocityGain > minVelocityGain);
    assert(globalOptTolMultiplier > 0.99);
}


trajectory_evaluator_params_t::trajectory_evaluator_params_t(const utils::ConfigFile& config)
: timeBetweenSamples(config.getValueAsFloat(kTrajectoryEvaluatorHeading, kEvaluatorSampleTimeKey))
, useFixedObjectRadius(config.getValueAsBool(kTrajectoryEvaluatorHeading, kUseFixedObjectRadiusKey))
, fixedObjectRadius(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kObjectRadiusKey))
, shouldConsiderOnlyTheClosestObject(config.getValueAsBool(kTrajectoryEvaluatorHeading, kConsideSingleObjectKey))
, minStaticPassingRadius(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kStaticPassingRadiusKey))
, minDynamicPassingRadius(config.getValueAsDouble(kTrajectoryEvaluatorHeading, KDynamicPassingRadiusKey))
, staticUncertaintyStdMin(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kStaticUncertaintyStdMinKey))
, dynamicUncertaintyStdMin(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kDynamicUncertaintyStdMinKey))
, linearVelocityUncertaintyStdWeight(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kLinVelUncertaintyStdKey))
, angularVelocityUncertaintyStdWeight(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kAngVelUncertaintyStdKey))
, linearVelocityActionWeight(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kLinVelActionWeightKey))
, angularVelocityActionWeight(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kAngVelActionWeightKey))
, negativeVelocityActionCostMultiplier(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kNegativeVelCostMultKey))
, linearAccelerationActionWeight(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kLinAccActionWeightKey))
, angularAccelerationActionWeight(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kAngAccActionWeightKey))
, shouldConsiderCollisionArea(config.getValueAsBool(kTrajectoryEvaluatorHeading, kConsiderCollisionAreaKey))
, baseCostOfCollision(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kBaseCollisionCostKey))
, velocityWeightOfCollision(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kVelocityCollisionWeightKey))
, collisionAreaWeight(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kCollisionAreaWeightKey))
, angularTravelWeightForPathCost(config.getValueAsDouble(kTrajectoryEvaluatorHeading, kAngularTravelCostWeightKey))
{
    assert(timeBetweenSamples > 0.0);
    assert(fixedObjectRadius  > 0.0);
    assert(minStaticPassingRadius   >= 0.0);
    assert(minDynamicPassingRadius  >= 0.0);
    assert(staticUncertaintyStdMin  >= 0.0);
    assert(dynamicUncertaintyStdMin >= 0.0);
    assert(linearVelocityUncertaintyStdWeight  >= 0.0);
    assert(angularVelocityUncertaintyStdWeight >= 0.0);
    assert(minAcceptableSurvivability >= -0.01 && minAcceptableSurvivability <= 1.01);
    assert(linearVelocityActionWeight  >= 0.0);
    assert(angularVelocityActionWeight >= 0.0);
    assert(linearAccelerationActionWeight  >= 0.0);
    assert(angularAccelerationActionWeight >= 0.0);
    assert(negativeVelocityActionCostMultiplier >= 1.0);
    assert(baseCostOfCollision > 0.0);
    assert(velocityWeightOfCollision > 0.0);
    assert(collisionAreaWeight >= 0.0);
    assert(angularTravelWeightForPathCost >= 0.0);
}


trajectory_planner_params_t::trajectory_planner_params_t(const utils::ConfigFile& config,
                                                         const utils::ConfigFile& controllerConfig,
                                                         const utils::ConfigFile& robotConfig)
: simulatorParams(controllerConfig, robotConfig)
, evaluatorParams(config)
, robotBodyParams(robotConfig)
, optimizerParams(config)
{
    // nothing special to be done here.
}

} // mpepc
} // vulcan
