/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     transition_cycle.h
 * \author   Collin Johnson
 *
 * Declaration of GlobalTransitionCycle and associated interface functions:
 *   - are_cycles_compatible : determine if two cycles could possibly be the same
 *   - operator<<            : output operator for cycles
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_CYCLE_H
#define HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_CYCLE_H

#include "hssh/global_topological/transition.h"
#include "hssh/local_topological/small_scale_star.h"
#include <cereal/access.hpp>
#include <iosfwd>

namespace vulcan
{
namespace hssh
{

class TopologicalVisit;

/**
 * GlobalTransitionCycle is the symbolic representation of the the topology of a place (decision point and destination)
 * in the global topological map. The transition cycle is a cyclic ordering of the transitions available at a place.
 *
 * Each transition in the cycle has three properties:
 *
 *   - next     : the next transition in the cycle iterating counter-clockwise
 *   - previous : the next transition in the cycle iterating counter-clockwise
 *   - aligned  : the transition aligned with this transition (directly across in cycle)
 *
 */
class GlobalTransitionCycle
{
public:
    using Iter = std::vector<GlobalTransition>::const_iterator;

    /**
     * Default constructor for GlobalTransitionCycle.
     */
    GlobalTransitionCycle(void) = default;

    /**
     * Constructor for GlobalTransitionCycle.
     *
     * \param    globalArea          Global area that was entered
     * \param    star                Small-scale star from which the GlobalTransitionCycle will be constructed
     */
    GlobalTransitionCycle(const GlobalArea& globalArea, const SmallScaleStar& star);

    /**
     * next retrieves the next transition in the cycle iterating counter-clockwise.
     *
     * \param    trans       Transition to get next from
     * \return   Next transition in the cycle, if trans is in the cycle. Otherwise an invalid transition is returned.
     */
    GlobalTransition next(const GlobalTransition& trans) const;

    /**
     * previous retrieves the previous transition in the cycle iterating counter-clockwise.
     *
     * \param    trans       Transition to get previous from
     * \return   Previous transition in the cycle, if trans is in the cycle. Otherwise an invalid transition is
     * returned.
     */
    GlobalTransition previous(const GlobalTransition& trans) const;

    /**
     * aligned retrieves the transition aligned with the provided transition.
     *
     * \param    trans       Transition to get aligned transition from
     * \return   Transition aligned with trans, if trans is in the cycle. Otherwise an invalid transition is returned.
     */
    GlobalTransition aligned(const GlobalTransition& trans) const;


    /**
     * contains checks if this cycle contains the specified transition.
     *
     * \param    transition          Transition to check
     * \return   True if this transition is part of the cycle.
     */
    bool contains(const GlobalTransition& transition) const;

    /**
     * transitionIndex retrieves the index of the specified transition in the cycle. This index corresponds to the
     * local_path_fragment_t index from which the GlobalTransition was created.
     *
     * \param    transition          Transition to get index for
     * \return   Index of this transition in the cycle. this->size() if contains(transition) == false.
     */
    std::size_t transitionIndex(const GlobalTransition& transition) const;

    /**
     * replaceTransition replaces a particular transition in the cycle. Replacing a transition is used whenever a loop
     * closure is established or a transition that was known prior to construction of the cycle needs to be incorporated
     * into the cycle.
     *
     * \param    trans           Existing transition in the cycle
     * \param    newTrans        New transition to replace trans with
     * \return   True if trans was found in the cycle and thus newTrans is now in the cycle.
     */
    bool replaceTransition(const GlobalTransition& trans, const GlobalTransition& newTrans);

    // Iterator support considering each transition separately
    std::size_t size(void) const { return transitions_.size(); }
    Iter begin(void) const { return transitions_.begin(); }
    Iter end(void) const { return transitions_.end(); }
    GlobalTransition at(int index) const { return transitions_.at(index); }
    GlobalTransition operator[](int index) const { return transitions_[index]; }

private:
    std::vector<GlobalTransition> transitions_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar(transitions_);
    }
};

/////////////  Non-member interface for LocalTransitionCycle   //////////////////////////

/**
 * Equality operators for transition cycles.
 *
 * Two cycles are considered the same if there exists some rotation of lhs s.t. the corresponding transitions in the
 * cycles have the same navigable status and lead to the same type of area.
 *
 * \param    lhs             One of the cycles to compare
 * \param    rhs             The other cycle to compare
 * \return   True if some rotation of lhs exists such that all matching transitions are equal between lhs and rhs.
 */
bool operator==(const GlobalTransitionCycle& lhs, const GlobalTransitionCycle& rhs);
bool operator!=(const GlobalTransitionCycle& lhs, const GlobalTransitionCycle& rhs);

// Output operator just outputs each of the internal transitions in order around the cycle.
std::ostream& operator<<(std::ostream& out, const GlobalTransitionCycle& cycle);

/**
 * are_cycles_compatible checks if two transition cycles are compatible, meaning there exists a permutation of the
 * sequence of transitions in lhs such that the robot entering lhs would see the same transition cycle as rhs entry.
 *
 * The compatible star check can be used to determine if the observed transition cycle matches the expected transition
 * cycle for a given map. If they match, then the cycles are compatible. If they don't match, then there's something
 * wrong with the cycles.
 *
 * The compatible check is needed because there are multiple permutation of a transition cycle that are equivalent and
 * can result in a valid match.
 *
 * \param    lhs             One of the cycles to compare
 * \param    lhsEntry        The entry transition for lhs
 * \param    rhs             One of the cycles to compare
 * \param    rhsEntry        The entry transition for rhs
 * \return   True if some rotation of lhs results in a match in the topology of the lhs and rhs cycles.
 */
bool are_cycles_compatible(const GlobalTransitionCycle& lhs,
                           const GlobalTransition& lhsEntry,
                           const GlobalTransitionCycle& rhs,
                           const GlobalTransition& rhsEntry);

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_CYCLE_H
