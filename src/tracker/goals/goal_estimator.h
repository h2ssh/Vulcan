/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     goal_estimator.h
 * \author   Collin Johnson
 *
 * Declaration of GoalEstimator and goal_estimator_params_t.
 */

#ifndef TRACKER_GOALS_GOAL_ESTIMATOR_H
#define TRACKER_GOALS_GOAL_ESTIMATOR_H

#include "tracker/goal.h"
#include "tracker/motions/visitor.h"
#include <memory>

namespace vulcan
{
namespace tracker
{

class ObjectMotion;
struct tracking_environment_t;
struct goal_estimator_debug_info_t;

/**
 * GoalEstimator is an abstract base class for various types of GoalEstimators.
 */
class GoalEstimator : public ObjectMotionVisitor
{
public:
    /**
     * Destructor for GoalEstimator.
     */
    virtual ~GoalEstimator(void) = default;

    /**
     * estimateGoal estimates a new goal for the object
     *
     * \param    motion              Current estimate of the steady motion of the object
     * \param    relativeGoalTime    Time in the future at which the goal should be estimated relative to the current
     *               time (seconds)
     * \param    environment         Environment in which robot is operating
     * \param    debug               Place to store any generated debug info
     * \pre  relativeGoalTime >= 0
     * \return   Best estimate of the object's goal.
     */
    ObjectGoalDistribution estimateGoal(const ObjectMotion& motion,
                                        double relativeGoalTime,
                                        const tracking_environment_t& environment,
                                        goal_estimator_debug_info_t* debug);

    // ObjectMotionVisitor interface
    void visitFixedEndpoint(const FixedEndpointMotion& motion) override;
    void visitStationary(const StationaryMotion& motion) override;
    void visitSteady(const SteadyMotion& motion) override;
    void visitStriding(const StridingMotion& motion) override;

protected:
    // State variables to maintain per-update
    const tracking_environment_t* environment_ = nullptr;
    double relativeGoalTime_ = 0.0;
    goal_estimator_debug_info_t* debug_ = nullptr;

    // Methods for estimating goals for the different types of motions
    virtual ObjectGoalDistribution estimateFixedEndpointGoal(const FixedEndpointMotion& motion) = 0;
    virtual ObjectGoalDistribution estimateStationaryGoal(const StationaryMotion& motion) = 0;
    virtual ObjectGoalDistribution estimateSteadyGoal(const SteadyMotion& motion) = 0;
    virtual ObjectGoalDistribution estimateStridingGoal(const StridingMotion& motion) = 0;

private:
    ObjectGoalDistribution goals_;
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_GOALS_GOAL_ESTIMATOR_H
