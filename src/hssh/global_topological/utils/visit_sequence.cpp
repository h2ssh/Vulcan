/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visit_sequence.cpp
* \author   Collin Johnson
*
* Definition of TopologicalVisitSequence.
*/

#include "hssh/global_topological/utils/visit_sequence.h"
#include "utils/algorithm_ext.h"
#include <iostream>

// Turn this on to get a warning message if the entry and exit area ids don't match
// #define ASSUME_PERFECT_IDS

namespace vulcan
{
namespace hssh
{

void TopologicalVisitSequence::addEvent(const LocalAreaEvent& event, const LocalTopoMap& eventMap)
{
    currentMap_ = &eventMap;
    event.accept(*this);
}


void TopologicalVisitSequence::addPose(const LocalPose& pose)
{
    lastPose_ = pose;

    if(currentVisit_)
    {
        currentVisit_->updatePose(pose);
    }
}


int TopologicalVisitSequence::eraseVisitsBefore(int depth)
{
    if(sequence_.empty())
    {
        return 0;
    }

    return utils::erase_remove_if(sequence_, [depth](const TopologicalVisit::Ptr& visit) {
        return visit->depth() < depth;
    });
}


int TopologicalVisitSequence::numVisits(void) const
{
    return sequence_.empty() ? 0 : sequence_.size() - 1;    // ignore last event b/c it represents incomplete state
}


TopologicalVisit::Ptr TopologicalVisitSequence::visitAt(int index) const
{
    // If this is a valid index, then just return the associated visit
    if((index >= 0) && (index < numVisits()))
    {
        return sequence_[index];
    }

    // Otherwise indicate an error by passing back a nullptr
    return nullptr;
}


void TopologicalVisitSequence::visitAreaTransition(const AreaTransitionEvent& event)
{
    // For the transition, if there is a current visit, then this must be the exited event
    bool isValid = true;
    if(currentVisit_)
    {
        // If the exited id and the current visit match, then this is the exit transition for the visit.
        // If they don't match, then something is wrong because entered and exited events don't correspond to the
        // correct sequence of areas. Output a message for now.
#ifdef ASSUME_PERFECT_IDS
        if(currentVisit_->areaId() != event.exitedId())
        {
            std::cerr << "WARNING: TopologicalVisitSequence: The currently visited area was exited by the next area transition."
                << " The area sequence output by local_topo_hssh is somehow flawed and incorrect. Current:"
                << currentVisit_->areaId() << " Exited:" << event.exitedId() << " Setting it as the exit anyway...\n";
        }
#endif // ASSUME_PERFECT_IDS

        isValid = currentVisit_->setExitEvent(event);
    }

    // The event might be wrong, since the local_topo_hssh occasionally double-fires events. In that case, just ignore
    // it and wait for valid data to roll through
    if(isValid)
    {
        // Now create a new visit for the area that was just entered.
        sequence_.push_back(std::make_shared<TopologicalVisit>(nextVisitDepth_, event));
        ++nextVisitDepth_;
        currentVisit_ = sequence_.back().get();
    }
    else if(sequence_.size() > 1)
    {
        std::cerr << "ERROR: Error in visit sequence was detected. Attempting to bail out the state of the graph, but "
            " there are no guarantees things will keep working. Fix the place detection!\n";
        sequence_[sequence_.size() - 2]->setExitEvent(event);
    }
}


void TopologicalVisitSequence::visitTurnAround(const TurnAroundEvent& event)
{
    // A turn-around event only applies to the current visit.
    if(currentVisit_)
    {
        // If the turn around doesn't happen in the same area as the current visit, then it needs to be discarded
        // with a warning saying that the local topological event sequence doesn't seem to be consistent
        if(currentVisit_->areaId() != event.areaId())
        {
            std::cerr << "WARNING: TopologicalVisitSequence: Received a TurnAroundEvent for a different area than the current visit:"
                << " Current:" << currentVisit_->areaId() << " Event:" << event.areaId() << '\n';
        }
        else
        {
            currentVisit_->addPathEvent(event);
            std::cout << "INFO: TopologicalVisitSequence: Turned around on path in area " << event.areaId() << '\n';
        }
    }
    // If there isn't a current visit, then indicate something slightly strange has happened in that we turned around
    // while not seeming to be at an area
    else
    {
        std::cerr << "WARNING: TopologicalVisitSequence: Received a TurnAroundEvent while not in an area.\n";
    }
}

} // namespace hssh
} // namespace vulcan
