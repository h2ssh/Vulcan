/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     ballistic_estimator.cpp
* \author   Collin Johnson
* 
* Definition of BallisticGoalEstimator.
*/

#include "tracker/goals/ballistic_estimator.h"
#include "tracker/motions/stationary.h"
#include "tracker/motions/steady.h"
#include "tracker/motions/striding.h"
#include "utils/stub.h"
#include <cassert>

namespace vulcan
{
namespace tracker 
{
    

ObjectGoalDistribution BallisticGoalEstimator::estimateFixedEndpointGoal(const FixedEndpointMotion& motion)
{
    assert(!"ERROR: GatewayGoalEstimator: Cannot estimate goals for FixedEndpointMotion.\n");
    return ObjectGoalDistribution{};
}


ObjectGoalDistribution BallisticGoalEstimator::estimateStationaryGoal(const StationaryMotion& motion) 
{
    // A stationary object is always stationary
    auto position = motion.position();
    auto velocity = motion.velocity();
    return ObjectGoalDistribution{ObjectGoal{position, std::atan2(velocity.y, velocity.x), std::log(1.0)}};
}


ObjectGoalDistribution BallisticGoalEstimator::estimateSteadyGoal(const SteadyMotion& motion) 
{
    // For now, use the dumbest possible prediction and just shoot the goal the desired timestep forward in time
    // TODO: Make this smarter so it doesn't put the goal somewhere unreachable
    auto goalPosition = motion.position() + Point<double>(motion.velocity().x * relativeGoalTime_,
                                                                motion.velocity().y * relativeGoalTime_);
    
    return ObjectGoalDistribution{ObjectGoal{goalPosition, 
                                             std::atan2(motion.velocity().y, motion.velocity().x),
                                             std::log(1.0)}};
}


ObjectGoalDistribution BallisticGoalEstimator::estimateStridingGoal(const StridingMotion& motion) 
{
    PRINT_STUB("FixedGoalEstimator::estimateStridingGoal");
    return ObjectGoalDistribution{};
}

} // namespace tracker
} // namespace vulcan
