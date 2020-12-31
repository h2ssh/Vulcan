/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     state_estimator.h
* \author   Collin Johnson
* 
* Declaration of the StateEstimator interface and the create_state_estimator factory.
*/

#ifndef ROBOT_STATE_STATE_ESTIMATOR_H
#define ROBOT_STATE_STATE_ESTIMATOR_H

#include "system/module_communicator.h"

namespace vulcan
{
namespace robot
{
    
class StateEstimator;

/**
* create_state_estimator is a factory for creating new instances of StateEstimator.
* 
* \param    type        Type of StateEstimator to be created
* \param    config      Config file containing the parameters for the StateEstimator being created
* \return   Pointer to the newly created instance of StateEstimator.
*/
std::unique_ptr<StateEstimator> create_state_estimator(const std::string& type, const utils::ConfigFile& config);

/**
* StateEstimator is an interface for a class responsible for estimating some state within the environment
* using the robot's sensor data. While the entire mapping process could be argued as being such a thing,
* a StateEstimator should be focused on a single aspect that isn't a subsumed by the general SLAM approach.
* For example, am I on an elevator or am I moving? These are both useful things that could be considered
* as inputs to a SLAM algorithm. They analyze the sensors and do this one specific thing.
* 
* The interface for StateEstimator consists of two methods, one for initializing the subscriptions to the necessary
* data and one for estimating and sending off the results:
* 
*   * void initialize(system::ModuleCommunicator&)
*       - Subscribe to any necessary data for the estimator
* 
*   * void estimate(system::ModuleCommunicator&)
*       - Update the current estiate and send it off to the world
*/
class StateEstimator
{
public:

    virtual ~StateEstimator(void) { }
    
    /**
    * initialize
    */
    virtual void initialize(system::ModuleCommunicator& communicator) = 0;
    
    /**
    * estimate updates the state being monitored with the most recent chunk of data.
    * 
    * \param    data        Latest batch of sensor data that was received
    */
    virtual void estimate(system::ModuleCommunicator& transmitter) = 0;
};

}
}

#endif // ROBOT_STATE_STATE_ESTIMATOR_H
