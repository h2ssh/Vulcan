/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_estimator_factory.cpp
* \author   Collin Johnson
* 
* Definition of GoalEstimatorFactory.
*/

#include "tracker/goals/goal_estimator_factory.h"
#include "tracker/goals/ballistic_estimator.h"
#include "tracker/goals/fixed_estimator.h"
#include "tracker/goals/gateway_estimator.h"
#include "tracker/object_motion.h"
#include "tracker/tracking_environment.h"

namespace vulcan
{
namespace tracker
{

GoalEstimatorFactory::GoalEstimatorFactory(const gateway_goal_estimator_params_t& gatewayParams)
: gatewayParams_(gatewayParams)
{
}


std::unique_ptr<GoalEstimator> GoalEstimatorFactory::createGoalEstimator(const ObjectMotion& motion, 
                                                                         const tracking_environment_t& environment)
{
    environment_ = &environment;
    motion.accept(*this);
    return std::move(created_);
}


void GoalEstimatorFactory::visitStriding(const StridingMotion& motion)
{
    // If we have a striding motion, then either ballistic or gateway depending on whether or not local topo map
    // is available
    if(environment_->ltm)
    {
        created_ = std::make_unique<GatewayGoalEstimator>(gatewayParams_);
    }
    else 
    {
        created_ = std::make_unique<BallisticGoalEstimator>();
    }
}


void GoalEstimatorFactory::visitSteady(const SteadyMotion& motion)
{
    // If we have a steady motion, then either ballistic or gateway depending on whether or not local topo map
    // is available
    if(environment_->ltm)
    {
        created_ = std::make_unique<GatewayGoalEstimator>(gatewayParams_);
    }
    else 
    {
        created_ = std::make_unique<BallisticGoalEstimator>();
    }
}


void GoalEstimatorFactory::visitFixedEndpoint(const FixedEndpointMotion& motion)
{
    // The fixed endpoint always uses the fixed estimator
    created_ = std::make_unique<FixedGoalEstimator>();
}


void GoalEstimatorFactory::visitStationary(const StationaryMotion& motion)
{
    // Stationary motion is always a ballistic estimator
    created_ = std::make_unique<BallisticGoalEstimator>();
}

} // namespace tracker
} // namespace vulcan
