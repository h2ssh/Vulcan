/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     director.cpp
* \author   Collin Johnson
* 
* Definition of StateEstimatorDirector.
*/

#include "robot/state/director.h"
#include "robot/state/state_estimator.h"
#include <iostream>
#include <unistd.h>

namespace vulcan
{
namespace robot
{
    
StateEstimatorDirector::StateEstimatorDirector(const utils::CommandLine& command, const utils::ConfigFile& config)
: params_(load_state_estimator_module_params(config))
, updateTrigger_(false)
{
    for(const auto& type : params_.monitorTypes)
    {
        std::cout<<"INFO:StateEstimator: Creating monitor "<<type<<'\n';
        estimators_.push_back(create_state_estimator(type, config));
    }
}


StateEstimatorDirector::~StateEstimatorDirector(void)
{
    // For std::unique_ptr
}


system::TriggerStatus StateEstimatorDirector::waitForTrigger(void)
{
    if(updateTrigger_.timedWait(1000))
    {
        return system::TriggerStatus::not_ready;
    }
    
    updateTrigger_.setPredicate(false);
    return system::TriggerStatus::ready;
}


void StateEstimatorDirector::subscribeToData(system::ModuleCommunicator& communicator)
{
    for(auto& estimator : estimators_)
    {
        estimator->initialize(communicator);
    }
    
    // Subscribe after the estimators because the data is distributed in order of subscription,
    // so this way all estimators will get the new odometry before the director wakes up
    // and does some computation
    communicator.subscribeTo<odometry_t>(this);
}


system::UpdateStatus StateEstimatorDirector::runUpdate(system::ModuleCommunicator& communicator)
{
    for(auto& estimator : estimators_)
    {
        estimator->estimate(communicator);
    }

    // The state_estimator is always running
    return system::UpdateStatus::running;
}


void StateEstimatorDirector::shutdown(system::ModuleCommunicator& communicator)
{
    // TODO: Anything in particular needed when shutting down?
}


void StateEstimatorDirector::handleData(const odometry_t& odometry, const std::string& channel)
{
    updateTrigger_.setPredicate(true);
    updateTrigger_.broadcast();
}

} // namespace robot
} // namespace vulcan
