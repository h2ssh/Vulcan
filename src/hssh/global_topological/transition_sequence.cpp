/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     transition_sequence.cpp
* \author   Collin Johnson
*
* Definition of GlobalTransitionSequence and SequencePosition.
*/

#include <hssh/global_topological/transition_sequence.h>
#include <utils/stub.h>
#include <algorithm>

namespace vulcan
{
namespace hssh
{

//////////////////  SequencePosition definition  /////////////////

inline bool in_range(double start, double end, double value)
{
    return (value >= start) && (value <= end);
}


SequencePosition::SequencePosition(double distFromStart, double length)
: distFromStart_(distFromStart)
, length_(length)
{
}


bool SequencePosition::overlaps(const SequencePosition& rhs) const
{
    double start = distFromStart_ - (length_ / 2);
    double end = distFromStart_ + (length_ / 2);
    double rhsStart = rhs.distFromStart_ - (rhs.length_ / 2);
    double rhsEnd = rhs.distFromStart_ + (rhs.length_ / 2);

    return in_range(start, end, rhsStart)
        || in_range(start, end, rhsEnd)
        || in_range(rhsStart, rhsEnd, start)
        || in_range(rhsStart, rhsEnd, end);
}


//////////////////  GlobalTransitionSequence definition  /////////////////

GlobalTransitionSequence::GlobalTransitionSequence(const GlobalArea& area,
                                                   const std::vector<TransitionAffordance>& transitions,
                                                   const Line<float>& centerLine)
{
    for(auto& t : transitions)
    {
        transitions_.emplace_back(next_id(),
                                  area,
                                  GlobalArea(AreaType::destination),
                                  NavigationStatus::navigable,
                                  ExplorationStatus::frontier);
        double distFromStart = distance_between_points(t.gateway().center(), centerLine.a);
        positions_.emplace_back(distFromStart, t.gateway().length());
    }
}


GlobalTransition GlobalTransitionSequence::next(const GlobalTransition& begin, TopoDirection direction) const
{
    // Plus direction means next has a higher index
    if(direction == TopoDirection::plus)
    {
        auto transIt = std::find(transitions_.begin(), transitions_.end(), begin);
        if((transIt != transitions_.end()) && (transIt + 1 != transitions_.end()))
        {
            return *(transIt + 1);
        }
    }
    // Minus direction means next has a lower index
    else // direction == TopoDirection::minus
    {
        auto transIt = std::find(transitions_.rbegin(), transitions_.rend(), begin);
        if((transIt != transitions_.rend()) && (transIt + 1 != transitions_.rend()))
        {
            return *(transIt + 1);
        }
    }

    // Didn't find anything so return an unknown
    return GlobalTransition();
}


GlobalTransition GlobalTransitionSequence::previous(const GlobalTransition& begin, TopoDirection direction) const
{
    // The previous is just the next in the opposite direction
    return next(begin, opposite_direction(direction));
}


SequencePosition GlobalTransitionSequence::position(const GlobalTransition& transition) const
{
    auto transIt = std::find(transitions_.begin(), transitions_.end(), transition);

    return (transIt != transitions_.end()) ? positions_[std::distance(transitions_.begin(), transIt)]
        : SequencePosition();
}


bool GlobalTransitionSequence::replaceTransition(const GlobalTransition& trans, const GlobalTransition& newTrans)
{
    // Can't use std::replace b/c it doesn't indicate if it actually replaced anything or not
    auto transIt = std::find(transitions_.begin(), transitions_.end(), trans);

    if(transIt != transitions_.end())
    {
        *transIt = newTrans;
    }

    return transIt != transitions_.end();
}


GlobalTransitionSequence GlobalTransitionSequence::merge(const GlobalTransitionSequence& rhs) const
{
    PRINT_PRETTY_STUB();
    return *this;
}


GlobalTransitionSequence GlobalTransitionSequence::reverse(void) const
{
    // Reverse the order of the transitions, all positions, and the center line start/end.
    GlobalTransitionSequence reversedSeq(*this);
    std::reverse(reversedSeq.transitions_.begin(), reversedSeq.transitions_.end());
    std::reverse(reversedSeq.positions_.begin(), reversedSeq.positions_.end());
    std::swap(reversedSeq.centerLine_.a, reversedSeq.centerLine_.b);

    // Subtract dist from the length to make the dist relative to the start of the reversed path
    double pathLength = length(centerLine_);
    for(auto& p : reversedSeq.positions_)
    {
        p = SequencePosition(pathLength - p.distFromStart(), p.length());
    }

    return reversedSeq;
}

} // namespace hssh
} // namespace vulcan
