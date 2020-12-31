/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     remote_encoders.cpp
* \author   Collin Johnson
*
* Definition of RemoteEncoders.
*/

#include "sensors/remote_encoders.h"
#include "system/module_communicator.h"
#include <iostream>

namespace vulcan
{
namespace sensors
{

RemoteEncoders::RemoteEncoders(const wheel_encoders_params_t& params)
    : WheelEncoders(params)
    , dataTrigger(false)
{
}


void RemoteEncoders::initialize(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<encoder_data_t>(this);
}


void RemoteEncoders::handleData(const encoder_data_t& encoders, const std::string& channel)
{
    dataLock.lock();
    this->encoders = encoders;
    dataLock.unlock();

    dataTrigger.setPredicate(true);
    dataTrigger.broadcast();
}


encoder_data_t RemoteEncoders::getEncoders(void)
{
    dataTrigger.wait();
    dataLock.lock();

    encoder_data_t updated = encoders;

    dataLock.unlock();
    dataTrigger.setPredicate(false);

    return updated;
}


void RemoteEncoders::resetEncoders(void)
{
    // TODO
}

}
}
