/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     ballistic_estimator.h
 * \author   Collin Johnson
 *
 * Declaration of BallisticGoalEstimator.
 */

#ifndef TRACKER_GOALS_BALLISTIC_ESTIMATOR_H
#define TRACKER_GOALS_BALLISTIC_ESTIMATOR_H

#include "tracker/goals/goal_estimator.h"

namespace vulcan
{
namespace tracker
{

/**
 * BallisticGoalEstimator
 */
class BallisticGoalEstimator : public GoalEstimator
{
public:
private:
    /////   GoalEstimator interface   /////
    ObjectGoalDistribution estimateFixedEndpointGoal(const FixedEndpointMotion& motion) override;
    ObjectGoalDistribution estimateStationaryGoal(const StationaryMotion& motion) override;
    ObjectGoalDistribution estimateSteadyGoal(const SteadyMotion& motion) override;
    ObjectGoalDistribution estimateStridingGoal(const StridingMotion& motion) override;
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_GOALS_BALLISTIC_ESTIMATOR_H
