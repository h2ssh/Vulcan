/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     motion_command_filter.cpp
 * \author   Jong Jin Park
 *
 * Definition of MotionCommandFilter.
 */

#include "robot/motion_command_filter.h"
#include "utils/timestamp.h"
#include <iostream>

// #define DEBUG_ACTIVE_SOURCE

namespace vulcan
{
namespace robot
{

MotionCommandFilter::MotionCommandFilter(const command_filter_params_t& params)
: activeSource(NO_SOURCE)
, lastValidCommandTime(0)
, sourceTimeoutInterval(params.sourceTimeout * 1000)
, params(params)
{
}


bool MotionCommandFilter::validMotionCommand(const motion_command_t& command)
{
    bool valid = false;

    //    std::cout<<"MOTION COMMAND FILTER: Validation initiated.\n";

    if (hasSourceTimedOut(utils::system_time_us())) {
        //        std::cout<<"MOTION COMMAND FILTER: sourse has timed out.\n";
        deactivateCurrentSource();
    }

    if (isValidCommand(command)) {
        //        std::cout<<"MOTION COMMAND FILTER: Command validated. Activating command source.\n";
        activateSource(command.source, command.timestamp, params.sourceTimeout);
        valid = true;
    }

    return valid;
}


void MotionCommandFilter::activateSource(command_source_t source, int64_t commandTime, int64_t timeout)
{

#ifdef DEBUG_ACTIVE_SOURCE
    if (source != activeSource) {
        std::cout << "DEBUG:MotionCommandFilter:Changed source from " << activeSource << " to " << source << '\n';
    }
#endif

    activeSource = source;
    lastValidCommandTime = commandTime;
    sourceTimeoutInterval = timeout;
}


void MotionCommandFilter::deactivateCurrentSource(void)
{
    activeSource = NO_SOURCE;
}


bool MotionCommandFilter::hasSourceTimedOut(int64_t commandTime) const
{
    return (sourceTimeoutInterval > 0) && ((commandTime - lastValidCommandTime) > sourceTimeoutInterval);
}


bool MotionCommandFilter::isValidCommand(const motion_command_t& command) const
{
    bool valid = false;

    // Joystick commands are valid only if the joystick is not centered
    if (command.source == ONBOARD_JOYSTICK) {
        valid = (command.joystickCommand.forward != 0) || (command.joystickCommand.left != 0);
    } else if (command.source != NO_SOURCE)   // Other commands are only valid if there is no currently active source,
                                              // or the command comes from the active source
    {
        valid = (command.source == activeSource) || (activeSource == NO_SOURCE);
    }

    valid &= (utils::system_time_us() - command.timestamp) < params.sourceTimeout;

    return valid;
}

}   // namespace robot
}   // namespace vulcan
