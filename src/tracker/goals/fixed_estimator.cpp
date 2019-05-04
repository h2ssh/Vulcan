/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     fixed_estimator.cpp
* \author   Collin Johnson
* 
* Definition of FixedGoalEstimator.
*/

#include <tracker/goals/fixed_estimator.h>
#include <tracker/motions/fixed_endpoint.h>
#include <tracker/motions/stationary.h>
#include <utils/stub.h>
#include <cassert>

namespace vulcan
{
namespace tracker
{


ObjectGoalDistribution FixedGoalEstimator::estimateFixedEndpointGoal(const FixedEndpointMotion& motion)
{
    PRINT_STUB("FixedGoalEstimator::estimateFixedEndpointGoal");
    // A stationary object is always stationary
    auto position = motion.position();
    auto velocity = motion.velocity();
    return ObjectGoalDistribution{ObjectGoal{position, std::atan2(velocity.y, velocity.x), std::log(1.0)}};
}


ObjectGoalDistribution FixedGoalEstimator::estimateStationaryGoal(const StationaryMotion& motion) 
{
    // A stationary object is always stationary
    auto position = motion.position();
    auto velocity = motion.velocity();
    return ObjectGoalDistribution{ObjectGoal{position, std::atan2(velocity.y, velocity.x), std::log(1.0)}};
}


ObjectGoalDistribution FixedGoalEstimator::estimateSteadyGoal(const SteadyMotion& motion) 
{
    assert(!"ERROR: GatewayGoalEstimator: Cannot estimate goals for SteadyMotion.\n");
    return ObjectGoalDistribution{};
}


ObjectGoalDistribution FixedGoalEstimator::estimateStridingGoal(const StridingMotion& motion) 
{
    assert(!"ERROR: GatewayGoalEstimator: Cannot estimate goals for StridingMotion.\n");
    return ObjectGoalDistribution{};
}

} // namespace tracker
} // namespace vulcan
