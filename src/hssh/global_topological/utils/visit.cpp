/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visit.cpp
* \author   Collin Johnson
*
* Definition of TopologicalVisit.
*/

#include <hssh/global_topological/utils/visit.h>
#include <hssh/local_topological/local_topo_map.h>
#include <iostream>
#include <cassert>

#define DEBUG_VISIT

namespace vulcan
{
namespace hssh
{

TopologicalVisit::TopologicalVisit(int depth, const AreaTransitionEvent& entryEvent)
: depth_(depth)
, areaId_(entryEvent.enteredId())
, entryEvent_(entryEvent)
, localArea_(entryEvent_.enteredArea())
, entryPose_(entryEvent.pose())
{
    assert(entryEvent_.enteredArea());

#ifdef DEBUG_VISIT
    std::cout << "DEBUG: New TopologicalVisit: Entered: " << entryEvent.description() << " at " << entryPose_.pose()
        << " with depth " << depth << '\n';
    if(localArea_->type() == AreaType::decision_point)
    {
        auto place = std::static_pointer_cast<LocalPlace>(localArea_);
        std::cout << " Cycle:" << place->star();
    }
    std::cout << '\n';
#endif
}


const LocalArea* TopologicalVisit::localArea(void) const
{
    return localArea_ ? localArea_.get() : nullptr;
}


boost::optional<AreaTransitionEvent> TopologicalVisit::exitEvent(void) const
{
    if(exitEvent_.exitedArea())
    {
        return exitEvent_;
    }
    else
    {
        return boost::none;
    }
}


bool TopologicalVisit::setExitEvent(const AreaTransitionEvent& exitEvent)
{
    assert(exitEvent.exitedArea());

    if(exitEvent.exitedArea()->type() != localArea_->type())
    {
        std::cerr << "ERROR: Invalid event: Expected exit: " << localArea_->type() << " Occurred: "
            << exitEvent.exitedArea()->type() << '\n';

        // Check if the entered area is the same, due to a double-event firing. We can at least update the entered
        // model in this case.
        if(exitEvent.enteredArea()->type() == localArea_->type())
        {
            std::cout << "ERROR: Detected an event double-firing. Using the updated entry information.\n";
            areaId_ = exitEvent.enteredArea()->id();
            entryPose_ = exitEvent.pose();
            localArea_ = exitEvent.enteredArea();
            entryEvent_ = exitEvent;
        }

        return false;
//         assert(exitEvent.exitedArea()->type() == localArea_->type());
    }

    areaId_ = exitEvent.exitedArea()->id();
    exitEvent_ = exitEvent;
    lastPose_ = exitEvent.pose();
    localArea_ = exitEvent.exitedArea();

#ifdef DEBUG_VISIT
    std::cout << "DEBUG: TopologicalVisit: Depth: " << depth_ << " Exited area: " << exitEvent.description();
    if(localArea_->type() == AreaType::decision_point)
    {
        auto place = std::static_pointer_cast<LocalPlace>(localArea_);
        std::cout << " Cycle:" << place->star();
    }
    std::cout << '\n';
#endif

    return true;
}


void TopologicalVisit::addPathEvent(const TurnAroundEvent& pathEvent)
{
    assert(pathEvent.path());
//     assert(pathEvent.path()->id() == areaId_);

//     pathEvents_.push_back(pathEvent);

    // Ensure the events are in increasing order
    std::sort(pathEvents_.begin(), pathEvents_.end(), [](const TurnAroundEvent& lhs, const TurnAroundEvent& rhs) {
        return lhs.sequenceId() < rhs.sequenceId();
    });

#ifdef DEBUG_VISIT
    std::cout << "DEBUG: TopologicalVisit: Depth: " << depth_
        << " Added new path event: " << pathEvent.description() << '\n';
#endif
}


void TopologicalVisit::updatePose(const LocalPose& pose)
{
    // Only change the last pose if the exit event hasn't occurred because the lastPose_ reflects the last
    // pose that was recorded in the area being visited, which is by definition either the most recent in time or
    // else the pose at exit.
    if(!exitEvent_.exitedArea())
    {
        lastPose_ = pose;
    }
}

} // namespace hssh
} // namespace vulcan
