/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_path_segment.h
* \author   Collin Johnson
*
* Declaration of GlobalPathSegment plus utilities including:
*
*   - opposite_end : retrieve the opposite end of the path segment from the current one
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PATH_SEGMENT_H
#define HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PATH_SEGMENT_H

#include "hssh/global_topological/area.h"
#include "hssh/global_topological/transition.h"
#include "hssh/global_topological/transition_sequence.h"
#include "hssh/local_topological/lambda.h"
#include "hssh/utils/id.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <memory>

namespace vulcan
{
namespace hssh
{

struct GlobalLocation;

/**
* GlobalPathSegment is the atomic unit of a Path. A GlobalPathSegment has a plus and minus transition, corresponding
* to motion in that direction. For example, moving into a Place from the GlobalPathSegment through the plus transition is
* motion in the plus direction. A GlobalPathSegment also contains a lambda value, which is the measured transform
* of the plus place to the minus place in the plus place frame.
*
* Each GlobalPathSegment is assigned a unique id on construction. The unique id can be used to in a std::map to make sure
* the correct path segment is referred to. The id is not used for testing equality. Equality relies on the GlobalTransitions.
* The unique id is only assigned in the non-default constructors.
*/
class GlobalPathSegment
{
public:

    using Ptr = std::shared_ptr<GlobalPathSegment>;

    // Default constructor for GlobalPathSegment
    GlobalPathSegment(void) = default;

    /**
    * Constructor for GlobalPathSegment.
    *
    * This constructor should be used whenever a full path segment is being added to a map, that is a path segment
    * where both end transitions have been explored.
    *
    * \param    id          Unique id for the path segment
    * \param    plus        Transition on the plus side
    * \param    minus       Transition on the minus side
    * \param    lambda      Measured lambda for the segment
    * \param    left        Left transition sequence
    * \param    right       Right transition sequence
    * \param    isExplored  Flag indicating if this path segment should be considered explored
    *                       Most segments are created explored, unless the robot exits from the entry transition
    *                       or starts at the path segment
    */
    GlobalPathSegment(Id id,
                      const GlobalTransition& plus,
                      const GlobalTransition& minus,
                      const Lambda& lambda,
                      const GlobalTransitionSequence& left,
                      const GlobalTransitionSequence& right,
                      bool isExplored);

    /**
    * Constructor for GlobalPathSegment.
    *
    * Creates a path segment where the lambda isn't know yet. This constructor should be used whenever a path segment
    * is entered. The lambda can then be updated once the path segment is exited and a better estimate of lambda is
    * known.
    *
    * \param    id          Unique id for the path segment
    * \param    plus        Transition on the plus side
    * \param    minus       Transition on the minus side
    * \param    left        Left transition sequence
    * \param    right       Right transition sequence
    */
    GlobalPathSegment(Id id,
                      const GlobalTransition& plus,
                      const GlobalTransition& minus,
                      const GlobalTransitionSequence& left,
                      const GlobalTransitionSequence& right);

    // Operator overloads
    bool operator==(const GlobalPathSegment& rhs);
    bool operator!=(const GlobalPathSegment& rhs);

    // Accessors for the state
    Id id(void) const { return id_; }
    GlobalTransition plusTransition(void)  const { return plusTransition_; }
    GlobalTransition minusTransition(void) const { return minusTransition_; }
    GlobalArea plusPlace(void) const { return plusTransition_.otherArea(toArea()); }
    GlobalArea minusPlace(void) const { return minusTransition_.otherArea(toArea()); }
    const GlobalTransitionSequence& leftSequence(void) const { return leftSequence_; }
    const GlobalTransitionSequence& rightSequence(void) const { return rightSequence_; }
    Lambda lambda(void) const { return initialLambda_; }
    const std::vector<Lambda>& getAllLambdas(void) const { return lambdas_; }

    /**
    * toArea retrieves the GlobalArea description of the path segment.
    */
    GlobalArea toArea(void) const { return GlobalArea(id_, AreaType::path_segment); }

    // Query the state of the segment
    /**
    * isFrontier checks to see if this segment is a frontier. A frontier segment has only one valid transition
    * and the other transition is a dummy pointing to an undiscovered place.
    *
    * \return   True if the segment has a frontier.
    */
    bool isFrontier(void) const { return plusTransition_.isFrontier() || minusTransition_.isFrontier(); }

    // Perform simple operations
    /**
    * reverse creates a new GlobalPathSegment that reverses the transitions and inverts the lambda value.
    *
    * \return   GlobalPathSegment that is the reverse/inverse of the current segment.
    */
    GlobalPathSegment reverse(void) const;

    /**
    * replaceTransition replaces a transition stored in the area. The transition is being replaced because some sort
    * of loop closure or area has been added such that the area on the other side has changed in some way.
    *
    * \param    oldTrans            Transition currently contained in the path segment
    * \param    newTrans            New transition to use for the path segment
    * \return   True if oldTrans was found and successfully replaced with newTrans.
    */
    bool replaceTransition(const GlobalTransition& oldTrans, const GlobalTransition& newTrans);

    /**
    * locationOnSegment finds the location on the path segment for a given entry transition. The location depends on
    * exactly what part of the segment was entered, so this method should be used to ensure correct functionality when
    * finding the location along a path segment.
    *
    * \param    entry           Entry transition for the segment
    * \return   A valid GlobalLocation with the correct state for being on a path segment. An invalid location if the
    *   entry transition isn't part of this path segment.
    */
    GlobalLocation locationOnSegment(const GlobalTransition& entry) const;

    /**
    * addLambda adds a new Lambda measurement to the GlobalPathSegment. Each traversal of the segment
    * creates a new Lambda estimate which needs to then be incorporated into the overall place layout.
    * At some point the Lambdas will probably be maintained with a Kalman filter, but for I just
    * create extra edges. It isn't as efficient, but it'll work for now.
    *
    * \param    lambda          Lambda to be added to the segment
    */
    void addLambda(const Lambda& lambda);

private:

    Id id_;
    GlobalTransition plusTransition_;
    GlobalTransition minusTransition_;
    GlobalTransitionSequence leftSequence_;
    GlobalTransitionSequence rightSequence_;
    Lambda initialLambda_;
    bool haveExploredLambda_ = false;
    std::vector<Lambda> lambdas_;

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( id_,
            plusTransition_,
            minusTransition_,
            leftSequence_,
            rightSequence_,
            initialLambda_,
            lambdas_
        );
    }
};

/**
* opposite_end finds the opposite transition from the one provided.
*
* \return   minus if end == plus, plus if end == minus, invalid otherwise
*/
GlobalTransition opposite_end(const GlobalPathSegment& segment, const GlobalTransition& end);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_PATH_SEGMENT_H
