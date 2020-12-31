/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     global_path_segment.cpp
 * \author   Collin Johnson
 *
 * Definition of GlobalPathSegment.
 */

#include "hssh/global_topological/global_path_segment.h"
#include "hssh/global_topological/global_location.h"
#include "hssh/global_topological/global_path.h"

namespace vulcan
{
namespace hssh
{

bool operator!=(const Lambda& lhs, const Lambda& rhs)
{
    return (lhs.x != rhs.x) || (lhs.y != rhs.y) || (lhs.theta != rhs.theta);
}


GlobalPathSegment::GlobalPathSegment(Id id,
                                     const GlobalTransition& plus,
                                     const GlobalTransition& minus,
                                     const Lambda& lambda,
                                     const GlobalTransitionSequence& left,
                                     const GlobalTransitionSequence& right,
                                     bool isExplored)
: id_(id)
, plusTransition_(plus)
, minusTransition_(minus)
, leftSequence_(left)
, rightSequence_(right)
, initialLambda_(lambda)
, haveExploredLambda_(isExplored)
{
    initialLambda_ = lambda;
    lambdas_.push_back(lambda);
    std::cout << "Path segment: " << id_ << " Set initial lambda: (" << initialLambda_.x << ',' << initialLambda_.y
              << ',' << initialLambda_.theta << ")\n";
}


GlobalPathSegment::GlobalPathSegment(Id id,
                                     const GlobalTransition& plus,
                                     const GlobalTransition& minus,
                                     const GlobalTransitionSequence& left,
                                     const GlobalTransitionSequence& right)
: GlobalPathSegment(id, plus, minus, Lambda(), left, right, false)
{
}

// Operator overloads
bool GlobalPathSegment::operator==(const GlobalPathSegment& rhs)
{
    return (plusTransition_ == rhs.plusTransition_) && (minusTransition_ == rhs.minusTransition_);
}


bool GlobalPathSegment::operator!=(const GlobalPathSegment& rhs)
{
    return !operator==(rhs);
}


GlobalPathSegment GlobalPathSegment::reverse(void) const
{
    // Reversing means the plus and minus ends switch and the left and right transition sequence switch sides and
    // have the order of destinations reversed
    GlobalPathSegment reversed(id_,
                               minusTransition_,
                               plusTransition_,
                               initialLambda_.invert(),
                               rightSequence_.reverse(),
                               leftSequence_.reverse(),
                               haveExploredLambda_);

    // Copy over the other measured lambdas as well
    for (size_t n = 1; n < lambdas_.size(); ++n) {
        reversed.addLambda(lambdas_[n].invert());
    }

    return reversed;
}


bool GlobalPathSegment::replaceTransition(const GlobalTransition& oldTrans, const GlobalTransition& newTrans)
{
    if (oldTrans == minusTransition_) {
        minusTransition_ = newTrans;
        return true;
    } else if (oldTrans == plusTransition_) {
        plusTransition_ = newTrans;
        return true;
    }

    // Try replacing it in one of the sequences, since it wasn't an end transition -- transitions are globally unique
    // so it can't be on the left and right side!
    return leftSequence_.replaceTransition(oldTrans, newTrans) || rightSequence_.replaceTransition(oldTrans, newTrans);
}


GlobalLocation GlobalPathSegment::locationOnSegment(const GlobalTransition& entry) const
{
    auto direction = TopoDirection::null;

    // If the area is entered from the plus direction, heading in minus direction.
    if (entry == plusTransition_) {
        direction = TopoDirection::minus;
    }
    // If entered from minus, then heading to plus
    else if (entry == minusTransition_) {
        direction = TopoDirection::plus;
    }
    // Otherwise, entered from a sequence, so there's no direction


    return GlobalLocation(id_, entry, direction);
}


void GlobalPathSegment::addLambda(const Lambda& lambda)
{
    if (isFrontier()) {
        initialLambda_ = lambda;
        lambdas_[0] = lambda;
        std::cout << "Path segment: " << id_ << " Replaced initial frontier lambda: (" << initialLambda_.x << ','
                  << initialLambda_.y << ',' << initialLambda_.theta << ")\n";
    }

    if (!isFrontier()) {
        if (!haveExploredLambda_) {
            lambdas_[0] = lambda;
            initialLambda_ = lambda;
            haveExploredLambda_ = true;
            std::cout << "Path segment " << id_ << ": Replaced lambda with explored lambda.\n";
        } else if (lambdas_.back() != lambda) {
            lambdas_.push_back(lambda);
        }

        std::cout << "Path segment " << id_ << " lambdas:\n";
        for (auto& l : lambdas_) {
            std::cout << "(" << l.x << ',' << l.y << ',' << l.theta << ")\n";
        }
    }

    //     lambdas_[0] = lambda;
    //     initialLambda_.merge(lambda); lambdas_[0] = initialLambda_;
}


GlobalTransition opposite_end(const GlobalPathSegment& segment, const GlobalTransition& end)
{
    if (segment.plusTransition() == end) {
        return segment.minusTransition();
    } else if (segment.minusTransition() == end) {
        return segment.plusTransition();
    }
    // The given transition wasn't actually an end!
    else {
        return GlobalTransition();
    }
}

}   // namespace hssh
}   // namespace vulcan
