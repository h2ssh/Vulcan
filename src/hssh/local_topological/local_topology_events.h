/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topology_events.h
* \author   Collin Johnson
*
* Declaration of the various topology events that can be raised by the local_topo module.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPOLOGY_EVENTS_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPOLOGY_EVENTS_H

#include <hssh/local_topological/local_place.h>
#include <hssh/local_topological/lambda.h>

namespace vulcan
{
namespace hssh
{

/**
* place_event_type_t defines the possible events that can occur regarding a place.
*/
enum place_event_type_t
{
    PLACE_ENTERED,
    PLACE_EXITED
};

/**
* local_topology_place_event_t defines an event that occurs at some place in the local topology.
*/
struct local_topology_place_event_t
{
    int64_t               timestamp;
    place_event_type_t    type;
    int                   placeId;
    LocalPlaceOld            place;
    local_path_fragment_t entryFragment;    // Valid for PLACE_ENTERED and PLACE_EXITED events
    local_path_fragment_t exitFragment;     // Only valid for PLACE_EXITED events
    Lambda                transform;        // Only valid for PLACE_ENTERED -- measured lambda with uncertainty between previous and current place
};

/**
* path_event_type_t defines the possible path events that can occur.
*/
enum path_event_type_t
{
    PATH_TURNAROUND
};

/**
* local_topology_path_event_t defines an event that occurs while the robot moves along a path.
*/
struct local_topology_path_event_t
{
    int64_t           timestamp;
    path_event_type_t type;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPOLOGY_EVENTS_H
