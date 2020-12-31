/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     fixed_estimator.h
* \author   Collin Johnson
* 
* Declaration of FixedGoalEstimator.
*/

#ifndef TRACKER_GOALS_FIXED_ESTIMATOR_H
#define TRACKER_GOALS_FIXED_ESTIMATOR_H

#include "tracker/goals/goal_estimator.h"

namespace vulcan
{
namespace tracker
{
    
/**
* FixedGoalEstimator estimates the goal position of a FixedEndpoint object some number of seconds into the future. The
* goal estimation considers both a sliding motion and a rotating motion to determine where the moving center of the
* object will be in the future, based on the requested duration into the future of the goal estimate.
* 
* For sliding objects, the future position is based on the estimated length of the object at the desired time. For
* pivoting objects, the future position is based on the expected angle of the object relative to its pivot point.
*/
class FixedGoalEstimator : public GoalEstimator
{
public:
    
private:
    
    /////   GoalEstimator interface   /////
    ObjectGoalDistribution estimateFixedEndpointGoal(const FixedEndpointMotion& motion) override;
    ObjectGoalDistribution estimateStationaryGoal(const StationaryMotion& motion) override;
    ObjectGoalDistribution estimateSteadyGoal(const SteadyMotion& motion) override;
    ObjectGoalDistribution estimateStridingGoal(const StridingMotion& motion) override;
};

} // namespace tracker
} // namespace vulcan

#endif // TRACKER_GOALS_FIXED_ENDPOINT_H
