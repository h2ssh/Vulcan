/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     transition_cycle.cpp
* \author   Collin Johnson
*
* Definition of GlobalTransitionCycle.
*/

#include <hssh/global_topological/transition_cycle.h>
#include <hssh/global_topological/utils/local_to_global.h>
#include <hssh/global_topological/utils/visit.h>
#include <utils/cyclic_iterator.h>
#include <algorithm>
#include <iterator>
#include <cassert>

// #define DEBUG_CONSTRUCT

namespace vulcan
{
namespace hssh
{

bool cycle_rotations_match(const GlobalTransitionCycle& lhs,
                           const GlobalTransitionCycle& rhs,
                           std::size_t rotationOffset);


GlobalTransitionCycle::GlobalTransitionCycle(const GlobalArea& globalArea, const SmallScaleStar& star)
{
    // For each fragment in the small-scale star, create a corresponding GlobalTransition. The order in which the
    // fragments processed is the same order they are stored in the global cycle.
    for(auto& frag : star)
    {
        if(frag.navigable)
        {
            transitions_.emplace_back(next_id(),
                                      globalArea,
                                      GlobalArea(frag.type),
                                      NavigationStatus::navigable,
                                      ExplorationStatus::frontier);
        }
        else
        {
            transitions_.emplace_back(next_id(), globalArea);
        }
    }

#ifdef DEBUG_CONSTRUCT
    std::cout << "DEBUG: GlobalTransitionCycle: Created a new cycle from star:\n" << star << "\n to cycle \n"
        << *this << '\n';
#endif
}


GlobalTransition GlobalTransitionCycle::next(const GlobalTransition& trans) const
{
    auto transIt = std::find(transitions_.begin(), transitions_.end(), trans);

    // If the transition exists, then go to the next transition
    if(transIt != transitions_.end())
    {
        auto nextIt = transIt + 1;

        // If the next is the end, then wrap around to the beginning of the cycle
        if(nextIt == transitions_.end())
        {
            return transitions_.front();
        }
        // Otherwise just return the next.
        else
        {
            return *nextIt;
        }
    }

    // If the transition isn't found, next is an invalid transition.
    return GlobalTransition();
}


GlobalTransition GlobalTransitionCycle::previous(const GlobalTransition& trans) const
{
    auto transIt = std::find(transitions_.begin(), transitions_.end(), trans);

    // If the transition exists, then go to the previous transition
    if(transIt != transitions_.end())
    {
        // If the current is the beginning, then wrap around to the end of the cycle
        if(transIt == transitions_.begin())
        {
            return transitions_.back();
        }
        // Otherwise just use the previous transition
        else
        {
            return *(transIt - 1);
        }
    }

    // If the transition isn't found, previous is an invalid transition.
    return GlobalTransition();
}


GlobalTransition GlobalTransitionCycle::aligned(const GlobalTransition& trans) const
{
    auto transIt = std::find(transitions_.begin(), transitions_.end(), trans);

    // If the transition exists, then jump to the other side of the cycle
    if(transIt != transitions_.end())
    {
        int index = std::distance(transitions_.begin(), transIt);
        index = (index + (size() / 2)) % size();
        return transitions_[index];
    }

    return GlobalTransition();
}


bool GlobalTransitionCycle::contains(const GlobalTransition& transition) const
{
    return transitionIndex(transition) < size();
}


std::size_t GlobalTransitionCycle::transitionIndex(const GlobalTransition& transition) const
{
    return std::distance(transitions_.begin(), std::find(transitions_.begin(), transitions_.end(), transition));
}


bool GlobalTransitionCycle::replaceTransition(const GlobalTransition& trans, const GlobalTransition& newTrans)
{
    auto transIt = std::find(transitions_.begin(), transitions_.end(), trans);

    if(transIt != transitions_.end())
    {
        *transIt = newTrans;
    }

    return transIt != transitions_.end();
}


bool operator==(const GlobalTransitionCycle& lhs, const GlobalTransitionCycle& rhs)
{
    if(lhs.size() != rhs.size())
    {
        return false;
    }

    for(std::size_t n = 0; n < lhs.size(); ++n)
    {
        if(cycle_rotations_match(lhs, rhs, n))
        {
            return true;
        }
    }

    return false;
}


bool operator!=(const GlobalTransitionCycle& lhs, const GlobalTransitionCycle& rhs)
{
    return !(lhs == rhs);
}


std::ostream& operator<<(std::ostream& out, const GlobalTransitionCycle& cycle)
{
    std::copy(cycle.begin(), cycle.end(), std::ostream_iterator<GlobalTransition>(out, " -> "));
    return out;
}


bool are_cycles_compatible(const GlobalTransitionCycle& lhs,
                           const GlobalTransition& lhsEntry,
                           const GlobalTransitionCycle& rhs,
                           const GlobalTransition& rhsEntry)
{
    // Cycles can't be compatible if they aren't individually equal
    if(lhs != rhs)
    {
        return false;
    }

    // Find the starting point of the lhsEntry in its transitions
    auto lhsIt = std::find(lhs.begin(), lhs.end(), lhsEntry);
    // If the entry isn't found, then the cycles must be incompatible
    if(lhsIt == lhs.end())
    {
        return false;
    }
    // Find the starting point of the rhsEntry in its transitions
    auto rhsIt = std::find(rhs.begin(), rhs.end(), rhsEntry);
    // If the entry isn't found, then the cycles must be incompatible
    if(rhsIt == rhs.end())
    {
        return false;
    }

    auto lhsRange = utils::make_cyclic_iterator(lhsIt, lhs.begin(), lhs.end());
    auto rhsRange = utils::make_cyclic_iterator(rhsIt, rhs.begin(), rhs.end());

    return std::equal(lhsRange.first, lhsRange.second, rhsRange.first, rhsRange.second, are_similar_transitions);
}


bool cycle_rotations_match(const GlobalTransitionCycle& lhs,
                           const GlobalTransitionCycle& rhs,
                           std::size_t rotationOffset)
{
    // PRE: lhs.size() == rhs.size()
    assert(lhs.size() == rhs.size());

    auto rhsRange = utils::make_cyclic_iterator(rhs.begin() + rotationOffset, rhs.begin(), rhs.end());
    return std::equal(lhs.begin(), lhs.end(), rhsRange.first, rhsRange.second, are_similar_transitions);
}

} // namespace hssh
} // namespace vulcan
