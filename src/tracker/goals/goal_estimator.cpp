/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_estimator.cpp
* \author   Collin Johnson
*
* Definition of GoalEstimator and goal_estimator_params_t.
*/

#include "tracker/goals/goal_estimator.h"
#include "tracker/motions/fixed_endpoint.h"
#include "tracker/motions/stationary.h"
#include "tracker/motions/steady.h"
#include "tracker/motions/striding.h"

namespace vulcan
{
namespace tracker
{
    
ObjectGoalDistribution GoalEstimator::estimateGoal(const ObjectMotion& motion, 
                                                   double relativeGoalTime,
                                                   const tracking_environment_t& environment,
                                                   goal_estimator_debug_info_t* debug)
{
    environment_ = &environment;
    relativeGoalTime_ = std::max(relativeGoalTime, 1.0);    // always look at least one second out
    debug_ = debug;
    
    motion.accept(*this);   // visitors will set the goals_ field
    
    return goals_;
}


void GoalEstimator::visitFixedEndpoint(const FixedEndpointMotion& motion)
{
    goals_ = estimateFixedEndpointGoal(motion);
}


void GoalEstimator::visitStationary(const StationaryMotion& motion)
{
    goals_ = estimateStationaryGoal(motion);
}


void GoalEstimator::visitSteady(const SteadyMotion& motion)
{
    goals_ = estimateSteadyGoal(motion);
}


void GoalEstimator::visitStriding(const StridingMotion& motion)
{
    goals_ = estimateStridingGoal(motion);
}

} // namespace tracker
} // namespace vulcan
