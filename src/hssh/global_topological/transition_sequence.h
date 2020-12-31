/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     transition_sequence.h
 * \author   Collin Johnson
 *
 * Declaration of GlobalTransitionSequence and SequencePosition.
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_SEQUENCE_H
#define HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_SEQUENCE_H

#include "hssh/global_topological/transition.h"
#include "hssh/local_topological/affordances/transition.h"
#include <cereal/access.hpp>
#include <iosfwd>

namespace vulcan
{
namespace hssh
{

/**
 * SequencePosition holds distance of a transition center from the start of the path (minus end). The length of the
 * transition is also provided, thereby defining a range [distFromStart - length/2, distFromStart + length/2] that the
 * transition occupied along the side of the path segment.
 */
class SequencePosition
{
public:
    /**
     * Constructor for SequencePosition.
     *
     * \param    distFromStart           Distance from the start of a path
     * \param    length                  Length of the transition
     */
    SequencePosition(double distFromStart, double length);

    /**
     * Default constructor for SequencePosition.
     */
    SequencePosition(void) = default;

    /**
     * distFromStart retrieves the distance from the start of the path segment (minus end) to the center of the
     * associated transition.
     */
    double distFromStart(void) const { return distFromStart_; }

    /**
     * length retrieves the length of the transition, which corresponds to the length of the gateway that defines the
     * underlying transition affordance.
     */
    double length(void) const { return length_; }

    /**
     * overlaps checks if this position overlaps with another position. Two positions overlap if some part of their
     * ranges overlap.
     *
     *   [distFromStart - length/2, distFromStart + length/2]
     *
     * \param    rhs         Other position to check for overlap
     * \return True if the distance ranges overlap by any amount.
     */
    bool overlaps(const SequencePosition& rhs) const;

private:
    double distFromStart_ = 0.0;
    double length_ = 0.0;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar(distFromStart_, length_);
    }
};


/**
 * GlobalTransitionSequence is the symbolic representation of the sequence of transitions along one side of the path
 * segment. The transition sequence contains zero or more transitions. Each transition is represented by a
 * GlobalTransition and is associated with a SequencePosition.
 *
 * Each transition in the cycle has three properties:
 *
 *   - next      : next transition in the sequence in the specified direction
 *   - previous  : previous transition in the sequence in the specified direction
 *   - position  : position of the transition relative to the minus end
 *
 * The sequence is arranged such that iterating through the transitions moves in the plus direction along the path
 * segment. Thus the first transition is closest to the minus end and the last transition is closest to the plus end.
 */
class GlobalTransitionSequence
{
public:
    using Iter = std::vector<GlobalTransition>::const_iterator;

    /**
     * Default constructor for GlobalTransitionSequence.
     */
    GlobalTransitionSequence(void) = default;

    /**
     * Constructor for GlobalTransitionSequence.
     *
     * \param    area                Area that this sequence is associated with
     * \param    transitions         Local transitions to create global transition for
     * \param    centerLine          Center line of the associated path segment running from minus end to plus end
     */
    GlobalTransitionSequence(const GlobalArea& area,
                             const std::vector<TransitionAffordance>& transitions,
                             const Line<float>& centerLine);

    /**
     * next retrieves the next area along the sequence from begin.
     *
     * \param    begin           Transition from which to get the next in the sequence
     * \param    direction       Direction in which to get the next transition
     * \return   The next transition in the given direction. If there are no more transitions, then an invalid
     *   transition (i.e. default-constructed) is returned.
     */
    GlobalTransition next(const GlobalTransition& begin, TopoDirection direction) const;

    /**
     * previous retrieves the previous area along the sequence from begin.
     *
     * \param    begin           Transition from which to get the previous in the sequence
     * \param    direction       Direction in which to get the previous transition
     * \return   The previous transition in the given direction. If there are no more transitions, then an invalid
     *   transition (i.e. default-constructed) is returned.
     */
    GlobalTransition previous(const GlobalTransition& begin, TopoDirection direction) const;

    /**
     * position retrieves the position of the given transition along the path segment.
     *
     * \param    transition          Transition to get position of
     * \return   Position associated with trans. If trans isn't in the sequence, then an empty position is returned.
     */
    SequencePosition position(const GlobalTransition& transition) const;

    /**
     * replaceTransition replaces a particular transition in the sequence. Replacing a transition is used whenever a
     * loop closure is established or a transition that was known prior to construction of the sequence needs to be
     * incorporated into the sequence.
     *
     * \param    trans           Existing transition in the sequence
     * \param    newTrans        New transition to replace trans with
     * \return   True if trans was found in the sequence and thus newTrans is now in the sequence.
     */
    bool replaceTransition(const GlobalTransition& trans, const GlobalTransition& newTrans);

    /**
     * merge merges a transition sequence with this transition sequence to create a new sequence. The merge process
     * merges any transitions with maximal overlap. It assumes the range is the max of the two.
     *
     * \param    rhs             Other transition to include in the merging
     * \return   A new TransitionSequence containing the merged transitions of this and rhs.
     */
    GlobalTransitionSequence merge(const GlobalTransitionSequence& rhs) const;

    /**
     * reverse reverses the transition sequence to change the order of the transitions. The positions are also switched
     * to assume they start from the other end of the path.
     *
     * \return   A new TransitionSequence with the transitions stored in the opposite order.
     */
    GlobalTransitionSequence reverse(void) const;

    // Iterator support considering each transition separately
    std::size_t size(void) const { return transitions_.size(); }
    Iter begin(void) const { return transitions_.begin(); }
    Iter end(void) const { return transitions_.begin(); }
    GlobalTransition at(int index) const { return transitions_.at(index); }
    GlobalTransition operator[](int index) const { return transitions_[index]; }

private:
    std::vector<GlobalTransition> transitions_;
    std::vector<SequencePosition> positions_;
    Line<float> centerLine_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar(transitions_, positions_, centerLine_);
    }
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_SEQUENCE_H
