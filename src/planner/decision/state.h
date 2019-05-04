/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     state.h
* \author   Collin Johnson
*
* Definition of DecisionState.
*/

#ifndef PLANNER_DECISION_STATE_H
#define PLANNER_DECISION_STATE_H

#include <core/pose.h>
#include <hssh/local_topological/local_topology_events.h>
#include <hssh/local_topological/local_path.h>

namespace vulcan
{
namespace planner
{

/**
* DecisionState contains the state needed for the decision planner. Local path and local place events are
* provided to the planner in order to determine the next action to take. At any given time, the planner might receive
* a new place event, path event, or local path. When a new place is encountered, it is assumed the full place is
* visible because this is a basic assumption of places (right now!). The pose is always assumed to be updated, but
* isn't relevant for changing the decision planner target at the current time.
*/
struct DecisionState
{
    bool haveNewPlaceEvent;
    bool haveNewPathEvent;
    bool haveNewLocalPath;

    pose_t                      pose;
    hssh::local_topology_place_event_t placeEvent;
    hssh::local_topology_path_event_t  pathEvent;
    hssh::LocalPath                    path;

    decision_state_t(void)
        : haveNewPlaceEvent(false)
        , haveNewPathEvent(false)
        , haveNewLocalPath(false)
    {
    }
};

}
}

#endif // PLANNER_DECISION_STATE_H
