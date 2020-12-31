/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gateway_estimator.h
 * \author   Collin Johnson
 *
 * Declaration of GatewayGoalEstimator.
 */

#ifndef TRACKER_GOALS_GATEWAY_ESTIMATOR_H
#define TRACKER_GOALS_GATEWAY_ESTIMATOR_H

#include "hssh/local_topological/gateway.h"
#include "hssh/utils/id.h"
#include "tracker/goals/goal_estimator.h"
#include "utils/fixed_duration_buffer.h"

#include <gnuplot-iostream.h>

namespace vulcan
{
namespace utils
{
class ConfigFile;
}
namespace tracker
{

/**
 * gateway_goal_estimator_params_t defines parameters controlling the goal estimator.
 *
 * The parameters are defined at:
 *
 *   [GatewayGoalEstimatorParameters]
 *   time_window_ms = length of the time window to use for likelihood measurements
 *   time_decay_constant = constant to use for the exponential decay
 *   min_goal_speed_mps  = minimum speed agent is moving for it to be considered moving to a goal
 */
struct gateway_goal_estimator_params_t
{
    int64_t timeWindowUs;          ///< Length of the time window to use for goal estimation (microseconds)
    double timeDecayConstant;      ///< Decay constant for the exponential drop-off for information
    double minGoalBehaviorSpeed;   ///< Minimum speed (m/s) for an agent to show goal-directed behavior, not
                                   ///< just milling around
    double accelDuration;          ///< Amount of time to assume constant acceleration for estimate (seconds)

    gateway_goal_estimator_params_t(const utils::ConfigFile& config);
    gateway_goal_estimator_params_t(void) = default;
};

/**
 * GatewayGoalEstimator creates a distribution across all possible gateways in an area.
 *
 * NOTE: The GatewayGoalEstimator currently relies on the gateways in an area not disappearing. If using a known
 * LocalTopoMap, then this is guaranteed. If using an incremental LocalTopoMap, this isn't guaranteed, so the estimation
 * won't work properly.
 */
class GatewayGoalEstimator : public GoalEstimator
{
public:
    /**
     * Constructor for GatewayGoalEstimator.
     *
     * \param    params          Parameters controlling the behavior of the estimator
     */
    GatewayGoalEstimator(const gateway_goal_estimator_params_t& params);

private:
    struct likelihood_t
    {
        int64_t timestamp;
        double logLikelihood;
    };

    struct goal_t
    {
        hssh::Gateway gateway;
        double logPrior = 0.0;
        double logProbability = 0.0;
        utils::FixedDurationBuffer<likelihood_t> logLikelihoods;

        goal_t(void) = default;
        goal_t(const hssh::Gateway& g, int64_t duration) : gateway(g), logLikelihoods(duration) { }

        bool operator==(const goal_t& rhs) const { return gateway.isSimilarTo(rhs.gateway); }
    };

    gateway_goal_estimator_params_t params_;

    hssh::Id currentAreaId_ = hssh::kInvalidId;
    std::vector<goal_t> goals_;

    //     Gnuplot plot;
    std::vector<std::vector<double>> distOverTime_;

    double goalLogProbability(const goal_t& goal) const;

    /////   GoalEstimator interface   /////
    ObjectGoalDistribution estimateFixedEndpointGoal(const FixedEndpointMotion& motion) override;
    ObjectGoalDistribution estimateStationaryGoal(const StationaryMotion& motion) override;
    ObjectGoalDistribution estimateSteadyGoal(const SteadyMotion& motion) override;
    ObjectGoalDistribution estimateStridingGoal(const StridingMotion& motion) override;
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_GOALS_GATEWAY_ESTIMATOR_H
