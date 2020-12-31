/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     goal_estimator_factory.h
 * \author   Collin Johnson
 *
 * Declaration of GoalEstimatorFactory.
 */

#ifndef TRACKER_GOALS_GOAL_ESTIMATOR_FACTORY_H
#define TRACKER_GOALS_GOAL_ESTIMATOR_FACTORY_H

#include "tracker/goals/gateway_estimator.h"
#include "tracker/motions/visitor.h"
#include <memory>

namespace vulcan
{
namespace tracker
{

/**
 * GoalEstimatorFactory
 */
class GoalEstimatorFactory : public ObjectMotionVisitor
{
public:
    /**
     * Constructor for GoalEstimatorFactory.
     *
     * \param    gatewayParams       Parameters for the GatewayGoalEstimator
     */
    GoalEstimatorFactory(const gateway_goal_estimator_params_t& gatewayParams);

    /**
     * createGoalEstimator creates an instance of GoalEstimator based on the type of motion and the environment
     * knowledge that the tracker has of the environment.
     *
     * \param    motion          Motion for which goals will be estimated
     * \param    environment     Environment in which the robot is moving
     * \return   An instance of GoalEstimator for the correct type of GoalEstimator given the environment and motion.
     */
    std::unique_ptr<GoalEstimator> createGoalEstimator(const ObjectMotion& motion,
                                                       const tracking_environment_t& environment);

    // ObjectMotionVisitor interface
    void visitStriding(const StridingMotion& motion) override;
    void visitSteady(const SteadyMotion& motion) override;
    void visitFixedEndpoint(const FixedEndpointMotion& motion) override;
    void visitStationary(const StationaryMotion& motion) override;

private:
    gateway_goal_estimator_params_t gatewayParams_;   // parameters to use for gateway goal estimators

    const tracking_environment_t* environment_;   // environment for the motion when being created
    std::unique_ptr<GoalEstimator> created_;      // the estimator created on the last call to createGoalEstimator
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_GOALS_GOAL_ESTIMATOR_FACTORY_H
