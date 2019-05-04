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
* Declaration of params struct for TraejctoryPlanner.
*/

#ifndef MPEPC_TRAJECTORY_PLANNER_PARAMS_H
#define MPEPC_TRAJECTORY_PLANNER_PARAMS_H

#include <mpepc/simulator/params.h>
#include <robot/model/params.h>
#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace mpepc
{

struct mpepc_optimizer_params_t
{
    std::string targetListFile;

    bool useGlobalOptimizer;
    bool useLocalOptimizer;

    bool   useChanceConstraint;
    double minAcceptableChance;

    double minRadius;
    double minTheta;
    double minDelta;
    double minK1;
    double minVelocityGain;

    double maxRadius;
    double maxTheta;
    double maxDelta;
    double maxK1;
    double maxVelocityGain;

    double radiusTolerance;
    double thetaTolerance;
    double deltaTolerance;
    double gainTolerance;

    double functionTolerance;
    double globalOptTolMultiplier;

    std::size_t maxGlobalIterations;
    std::size_t maxLocalIterations;

    double radiusInitialStep;
    double thetaInitialStep;
    double deltaInitialStep;
    double gainInitialStep;

    double radiusGradientStepSize;
    double thetaGradientStepSize;
    double deltaGradientStepSize;
    double gainGradientStepSize;

    mpepc_optimizer_params_t(void) {};
    mpepc_optimizer_params_t(const utils::ConfigFile& config);
};

struct trajectory_evaluator_params_t
{
    // sub-sampling interval
    float timeBetweenSamples;

    // dynamic object options
    bool   useFixedObjectRadius;
    double fixedObjectRadius;

    // consider all objects vs single important object
    bool   shouldConsiderOnlyTheClosestObject;

    // underestimation of distances if needed
    double minStaticPassingRadius;
    double minDynamicPassingRadius;

    // chance constraint
    bool   useChanceConstraint;
    double minAcceptableSurvivability;

    // uncertainty computation
    double staticUncertaintyStdMin;
    double dynamicUncertaintyStdMin;
    double linearVelocityUncertaintyStdWeight;
    double angularVelocityUncertaintyStdWeight;

    // action cost computation
    double linearVelocityActionWeight;
    double angularVelocityActionWeight;
    double negativeVelocityActionCostMultiplier;
    double linearAccelerationActionWeight;
    double angularAccelerationActionWeight;

    // collision cost computation
    bool   shouldConsiderCollisionArea;
    double baseCostOfCollision;
    double velocityWeightOfCollision;
    double collisionAreaWeight;

    // path cost computation
    double angularTravelWeightForPathCost;

    trajectory_evaluator_params_t(void) {};
    trajectory_evaluator_params_t(const utils::ConfigFile& config);
};

struct trajectory_planner_params_t
{
    robot_simulator_params_t        simulatorParams;
    trajectory_evaluator_params_t   evaluatorParams;
    robot::collision_model_params_t robotBodyParams;
    mpepc_optimizer_params_t        optimizerParams;

    trajectory_planner_params_t(void) {};
    trajectory_planner_params_t(const utils::ConfigFile& config,
                                const utils::ConfigFile& controllerConfig,
                                const utils::ConfigFile& robotConfig);
};


} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_TRAJECTORY_PLANNER_PARAMS_H
