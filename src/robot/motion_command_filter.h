/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     motion_command_validator.h
* \author   Jong Jin Park
*
* Declaration of MotionCommandFilter.
*/

#ifndef ROBOT_MOTION_COMMAND_FILTER_H
#define ROBOT_MOTION_COMMAND_FILTER_H

#include "robot/commands.h"
#include "robot/params.h"

namespace vulcan
{
namespace robot
{

/**
* MotionCommandFilter is used to determine whether an incoming motion command is valid and can
* be issued to the wheelchair. Each motion command has an associated source. A high-level arbiter
* determines the active motion command based on user and control inputs. The filter then marks
* as valid only those commands coming from the active source.
*
* An active source either latches for all time or has an associated timeout interval. A source times
* out if it doesn't receive a command within the specified timeout interval. Once the active source
* has timed out, then no commands are accepted.
*
* Currently, the timeout value is not being used. It may be added in the future, but its utility needs
* to be thought through more carefully in the context of the full system.
*
* The MotionCommandFilter accepts change_command_source_message_t to determine the currently active
* command. To check if a command is valid, use the validMotionCommand() method.
*
* Currently, the MotionCommandFilter acts as its own arbiter, determing who has command. Eventually,
* this capability will be pulled out and placed in a separate arbiter module. Right now, the control
* is determined by these rules:
*
*   0) Manual control immediately overrides all other controls.
*   1) Manual control expires after a period of time, allowing any control method afterward.
*/
class MotionCommandFilter
{
public:

    /**
    * Constructor for MotionCommandFilter.
    *
    * \param    params          Parameters controlling the filter
    */
    MotionCommandFilter(const command_filter_params_t& params);

    /**
    * validMotionCommand checks to see if a motion command is valid based on the currently active command source.
    *
    * \param    command         Command to be validated
    * \return   True if the command is valid and should be issued to the wheelchair.
    */
    bool validMotionCommand(const motion_command_t& command);

private:

    void activateSource         (command_source_t source, int64_t commandTime, int64_t timeout);
    void deactivateCurrentSource(void);
    bool hasSourceTimedOut      (int64_t commandTime) const;
    bool isValidCommand         (const motion_command_t& command) const;

    command_source_t activeSource;
    int64_t          lastValidCommandTime;
    int64_t          sourceTimeoutInterval;

    command_filter_params_t params;
};

}
}

#endif // ROBOT_MOTION_COMMAND_VALIDATOR_H
