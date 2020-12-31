/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     director.h
 * \author   Collin Johnson
 *
 * Declaration of StateEstimatorDirector.
 */

#ifndef ROBOT_STATE_DIRECTOR_H
#define ROBOT_STATE_DIRECTOR_H

#include "robot/state/params.h"
#include "system/director.h"
#include "utils/condition_variable.h"
#include <memory>

namespace vulcan
{
struct odometry_t;
namespace robot
{

class StateEstimator;

/**
 * StateEstimatorDirector organizes computation of data for the state_estimator module.
 * The director creates all requested StateEstimator instances from the module configuration.
 * Then it goes into a cycle of updating at a specified interval.
 */
class StateEstimatorDirector : public system::Director
{
public:
    /**
     * Constructor for StateEstimatorDirector.
     *
     * \param    params          Parameters for the module
     */
    StateEstimatorDirector(const utils::CommandLine& command, const utils::ConfigFile& config);

    /**
     * Destructor for StateEstimatorDirector.
     */
    virtual ~StateEstimatorDirector(void);

    // system::Director interface
    system::TriggerStatus waitForTrigger(void);
    void subscribeToData(system::ModuleCommunicator& communicator);
    system::UpdateStatus runUpdate(system::ModuleCommunicator& communicator);
    void shutdown(system::ModuleCommunicator& communicator);

    // HACK: Trigger on odometry for now -- is there a way of synchronizing the triggers better
    //       amongst the various estimators?
    void handleData(const odometry_t& odometry, const std::string& channel);

private:
    state_estimator_module_params_t params_;
    std::vector<std::unique_ptr<StateEstimator>> estimators_;

    utils::ConditionVariable updateTrigger_;
};

}   // namespace robot
}   // namespace vulcan

#endif   // ROBOT_STATE_DIRECTOR_H
