/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gateway_estimator.cpp
 * \author   Collin Johnson
 *
 * Definition of GatewayGoalEstimator.
 */

#include "tracker/goals/gateway_estimator.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/local_topo_map.h"
#include "math/boundary.h"
#include "tracker/motions/stationary.h"
#include "tracker/motions/steady.h"
#include "tracker/motions/striding.h"
#include "tracker/tracking_environment.h"
#include "utils/algorithm_ext.h"
#include "utils/config_file.h"
#include "utils/stub.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace tracker
{

/////   gateway_goal_estimator_params_t implementation   /////
const std::string kGatewayGoalEstimatorHeading("GatewayGoalEstimatorParameters");
const std::string kTimeWindowKey("time_window_ms");
const std::string kTimeDecayKey("time_decay_constant");
const std::string kMinGoalSpeedKey("min_goal_speed_mps");
const std::string kAccelDurationKey("accel_duration_ms");

gateway_goal_estimator_params_t::gateway_goal_estimator_params_t(const utils::ConfigFile& config)
: timeWindowUs(config.getValueAsInt32(kGatewayGoalEstimatorHeading, kTimeWindowKey) * 1000)
, timeDecayConstant(config.getValueAsDouble(kGatewayGoalEstimatorHeading, kTimeDecayKey))
, minGoalBehaviorSpeed(config.getValueAsDouble(kGatewayGoalEstimatorHeading, kMinGoalSpeedKey))
, accelDuration(config.getValueAsInt32(kGatewayGoalEstimatorHeading, kAccelDurationKey) / 1000.0)
{
    assert(timeWindowUs > 0);
    assert(timeDecayConstant > 0);

    minGoalBehaviorSpeed = std::max(minGoalBehaviorSpeed, 0.0);
    accelDuration = std::max(accelDuration, 0.0);
}


/////   GatewayGoalEstimator implementation   /////

GatewayGoalEstimator::GatewayGoalEstimator(const gateway_goal_estimator_params_t& params) : params_(params)
{
}


ObjectGoalDistribution GatewayGoalEstimator::estimateFixedEndpointGoal(const FixedEndpointMotion& motion)
{
    assert(!"ERROR: GatewayGoalEstimator: Cannot estimate goals for FixedEndpointMotion.\n");
    return ObjectGoalDistribution{};
}


ObjectGoalDistribution GatewayGoalEstimator::estimateStationaryGoal(const StationaryMotion& motion)
{
    // A stationary object is always stationary
    auto position = motion.position();
    auto velocity = motion.velocity();
    return ObjectGoalDistribution{ObjectGoal{position, std::atan2(velocity.y, velocity.x), 1.0}};
}


ObjectGoalDistribution GatewayGoalEstimator::estimateSteadyGoal(const SteadyMotion& motion)
{
    assert(environment_);
    assert(environment_->ltm);   // only valid to estimate gateways if there's actually an LTM

    auto areas = environment_->ltm->allAreasContaining(motion.position());

    // If there aren't any areas with this object, then there can't be any goals.
    if (areas.empty()) {
        //         std::cout << "WARNING: Object at " << motion.position() << " is not located in any areas!\n";
        return ObjectGoalDistribution{};
    }

    // Assign the new area if the previous area wasn't found.
    auto currentArea = environment_->ltm->areaWithId(currentAreaId_);
    if (!utils::contains(areas, currentArea)) {
        currentArea = areas.front();
        currentAreaId_ = currentArea->id();
        goals_.clear();   // when the area changes, so do all the goals

        distOverTime_.clear();
    }
    assert(currentArea);

    // The current area is now known, so goal estimation can be performed
    auto& gateways = currentArea->gateways();
    for (auto& g : gateways) {
        // If a particular goal isn't found in the current set of gateways, then add it
        goal_t newGoal{g, params_.timeWindowUs};

        if (!utils::contains(goals_, newGoal)) {
            goals_.emplace_back(newGoal);
            distOverTime_.push_back(std::vector<double>());
        }
    }

    // Estimate the likelihood for each gateway -- fast state better reflects actual uncertainty
    auto state = motion.fastMotionState();

    Matrix velCov = state.getCovariance().submat(velXIndex, velXIndex, velYIndex, velYIndex);
    Matrix accelCov = state.getCovariance().submat(accelXIndex, accelXIndex, accelYIndex, accelYIndex);
    Matrix cov = velCov + (accelCov * params_.accelDuration * params_.accelDuration);

    state = motion.slowMotionState();   // slow state is more stable estimate

    double xVel = state[velXIndex] + state[accelXIndex] * params_.accelDuration;
    double yVel = state[velYIndex] + state[accelYIndex] * params_.accelDuration;

    double heading = std::atan2(state[velYIndex] + state[accelYIndex] * params_.accelDuration,
                                state[velXIndex] + state[accelXIndex] * params_.accelDuration);

    auto headingUncertainty = math::heading_uncertainty(xVel, yVel, cov);
    double sigma = std::sqrt(headingUncertainty);

    // There must be some velocity for a goal to exist.
    double speed = std::sqrt((xVel * xVel) + (yVel * yVel));
    bool hasGoalBehavior = speed > params_.minGoalBehaviorSpeed;

    for (auto& g : goals_) {
        likelihood_t meas{motion.timestamp(), 0.0};

        if (hasGoalBehavior) {
            auto range = math::boundary_heading_range(g.gateway.boundary(), motion.position(), heading);
            double prob = math::boundary_heading_probability(range, sigma);
            prob = std::max(prob, 1e-8);   // clamp to at least a small probability to avoid negative infinities
            meas.logLikelihood = std::log(prob);
        } else {
            meas.logLikelihood = std::log(1.0 / goals_.size());   // not moving fast enough to have an opinion
        }

        g.logLikelihoods.push(meas);
        g.logProbability = goalLogProbability(g);
    }

    std::vector<ObjectGoal> goalDist;
    for (auto& g : goals_) {
        goalDist.emplace_back(g.gateway.boundary(), g.gateway.direction(), g.logProbability);
    }

    auto distribution = ObjectGoalDistribution{goalDist};

    //     for(std::size_t n = 0; n < goals_.size(); ++n)
    //     {
    //         distOverTime_[n].push_back(distribution[n].probability());
    //     }
    //
    //
    //     plot << "set style line 1 linecolor rgb '0x87ceeb' linewidth 2\n";
    //     plot << "set style line 2 linecolor rgb '0xff6347' linewidth 2\n";
    //     plot << "set style line 3 linecolor rgb '0x6a5acd' linewidth 2\n";
    //     plot << "set style line 4 linecolor rgb '0xda70d6' linewidth 2\n";
    //     plot << "set style line 5 linecolor rgb '0x32cd32' linewidth 2\n";
    //     plot << "set title 'Intention estimates for an agent moving through an intersection'\n";
    //
    //     plot << "set ylabel 'Probability'\n";
    //     plot << "set yrange [-0.1:1.1]\n";
    //     plot << "set xlabel 'Time Step'\n";
    //
    //     plot << "set for [i=1:" << distOverTime_.size() << "] linetype i\n";
    //
    //     plot << "title(n) = sprintf(\"%d\", n)\n";
    //     plot << "plot for [n=1:" << distOverTime_.size() << "] '-' u :1 w lines ls n title title(n)\n";
    //
    //     for(auto& dist : distOverTime_)
    //     {
    //         plot.send1d(dist);
    //     }

    return distribution;
}


double GatewayGoalEstimator::goalLogProbability(const goal_t& goal) const
{
    double logProbability = goal.logPrior;
    for (auto& meas : goal.logLikelihoods) {
        double weight =
          std::exp(-utils::usec_to_sec(goal.logLikelihoods.timestamp() - meas.timestamp) / params_.timeDecayConstant);
        logProbability += meas.logLikelihood * weight;
    }

    return logProbability;
}


ObjectGoalDistribution GatewayGoalEstimator::estimateStridingGoal(const StridingMotion& motion)
{
    PRINT_STUB("FixedGoalEstimator::estimateStridingGoal");
    return ObjectGoalDistribution{};
}

}   // namespace tracker
}   // namespace vulcan
