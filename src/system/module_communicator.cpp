/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     module_communicator.cpp
* \author   Collin Johnson
*
* Definition of ModuleCommunicator.
*/

#include "system/module_communicator.h"
#include <poll.h>

namespace vulcan
{
namespace system
{

ModuleCommunicator::ModuleCommunicator(void)
: haveDebugSubscription_(false)
{
}


ModuleCommunicator::ModuleCommunicator(const utils::CommandLine& cmdLine, const utils::ConfigFile& config)
: ModuleCommunicator()
{
    // No specific initialization is needed
}


ModuleCommunicator::ModuleCommunicator(const std::string& systemUrl, const std::string& debugUrl)
: systemConnection_(systemUrl)
, debugConnection_(debugUrl)
{
}


int ModuleCommunicator::processIncoming(int waitMs)
{

    pollfd fds[2];
    int fdIndex = 0;

    fds[fdIndex].fd       = systemConnection_.getFileno();
    fds[fdIndex++].events = POLLIN;

    // Don't call getFileno if we don't need to
    if(haveDebugSubscription_)
    {
        fds[fdIndex].fd       = debugConnection_.getFileno();
        fds[fdIndex++].events = POLLIN;
    }

    int success = 0;

    if(poll(fds, fdIndex, waitMs) > 0)
    {
        if(fds[0].revents & POLLIN)
        {
            success = systemConnection_.handle();
        }

        if(haveDebugSubscription_ && fds[1].revents & POLLIN)
        {
            success = debugConnection_.handle();
        }
    }

    return success;
}

}
}
