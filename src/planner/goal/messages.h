/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     messages.h
* \author   Collin Johnson
*
* Definition of messages for controlling behavior of the planner:
*
*   - goal_route_command_message_t
*/

#ifndef PLANNER_GOAL_MESSAGES_H
#define PLANNER_GOAL_MESSAGES_H

#include <string>
#include <cstdint>

namespace vulcan
{
namespace planner
{

/**
* route_command_t specifies a set of commands that can be issued to control the behavior
* of the global topo planner regarding its current route.
*/
enum route_command_t
{
    CANCEL_ROUTE,       ///< Immediately halt execution of the current route and stop the robot
    CONFIRM_ROUTE       ///< If a route needed confirmation, then confirm it, thereby allowing navigation to begin
};

/**
* goal_route_command_message_t controls the handling of a route. The route can be
* cancelled, stopping navigation, or confirmed, starting navigation.
*/
struct goal_route_command_message_t
{
    int64_t timestamp;

    uint32_t        planId;
    route_command_t command;
    std::string     source;
};

}
}

#endif // PLANNER_GOAL_MESSAGES_H
