/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     remote_encoders.h
* \author   Collin Johnson
*
* Declaration of RemoteEncoders.
*/

#ifndef SENSORS_REMOTE_ENCODERS_H
#define SENSORS_REMOTE_ENCODERS_H

#include "sensors/wheel_encoders.h"
#include "utils/condition_variable.h"
#include "utils/mutex.h"

namespace vulcan
{
namespace sensors
{

const std::string REMOTE_ENCODERS_TYPE("remote");

/**
* RemoteEncoders is a class for receiving encoder values from some remote source via the communication system
* rather than directly via a driver. The getTicks method simply blocks until new ticks are received.
*/
class RemoteEncoders : public WheelEncoders
{
public:

    /**
    * Constructor for RemoteEncoders.
    *
    * \param    params          Parameters for the encoder odometry
    */
    RemoteEncoders(const wheel_encoders_params_t& params);

    // OdometryEstimator interface -- use the inherited versions of other methods
    virtual void initialize(system::ModuleCommunicator& communicator);

    void handleData(const encoder_data_t& encoders, const std::string& channel);

    // WheelEncoders interface
    virtual encoder_data_t getEncoders(void);
    virtual void           resetEncoders(void);

private:

    encoder_data_t encoders;

    utils::ConditionVariable dataTrigger;
    utils::Mutex             dataLock;
};

}
}

#endif // SENSORS_REMOTE_ENCODERS_H
