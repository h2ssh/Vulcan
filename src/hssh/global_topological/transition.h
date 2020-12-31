/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     transition.h
* \author   Collin Johnson
*
* Declaration of GlobalTransition along with operator==,!=, and << and support functions:
* 
*   - create_frontier_transition : creates a transition leading to some frontier area
*   - are_similar_transitions : checks if two transitions have the same navigation status and lead to the same type
*       of area 
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_H
#define HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_H

#include "hssh/global_topological/area.h"
#include "hssh/utils/id.h"
#include <cereal/access.hpp>

namespace vulcan
{
namespace hssh
{

/**
* NavigationStatus defines the possible navigation states for a transition.
*/
enum class NavigationStatus
{
    navigable,      ///< The transition can be crossed
    null,           ///< The transition is a place-holder to maintain the cyclic aligned pairs, but cannot be navigated
};

/**
* ExplorationStatus defines the possible exploration states for a transition.
*/
enum class ExplorationStatus
{
    explored,       ///< The transition has been crossed at some point
    frontier,       ///< The transition leads to a frontier area -- either never crossed or not fully explored
};

/**
* GlobalTransition identifies the transition between two areas in the global topological map. Each transition has the
* following properties:
*
*   - a unique id : this id is unique amongst all GlobalTransitions in the map
*   - plus area   : area that exists on the plus side of the transition
*   - minus area  : area that exists on the minus side of the transition
*   - navigable   : flag indicating if the transition is navigable -- for purpose of indicating the end of a path
*   - exploration : enum defining possible exploration states of a transition (frontier, explored)
*
* NOTE: The plus and minus areas are arbitrary. For this implementation, the following convention is used:
*   - a destination is always a plus area, meaning travel to a destination is always in the plus direction
*   - leaving a destination is always in the minus direction
*   - the plus/minus area for a path endpoint-place transition is determined by the GlobalPath
*/
class GlobalTransition
{
public:

    /**
    * Constructor for GlobalTransition.
    *
    * Creates a transition between two areas.
    * 
    * \param    id          Id to assign the transition
    * \param    plusArea    Area on the plus side
    * \param    minusArea   Area on the minus side
    * \param    navigable   Flag indicating if the transition can be navigated
    * \param    explored    Flag indicating if the transition has been crossed at some poitn
    */
    GlobalTransition(Id id,
                     GlobalArea plusArea,
                     GlobalArea minusArea,
                     NavigationStatus navigable,
                     ExplorationStatus explored);
    
    /**
    * Constructor for GlobalTransition.
    * 
    * Creates a null transition for the provided area.
    * 
    * \param    id          Id to assign the transition
    * \param    area        Known area associated with the null transition
    */
    GlobalTransition(Id id, GlobalArea area);
    
    // The default constructor is valid
    GlobalTransition(void) = default;

    /**
    * id retrieves the unique id for the transition.
    */
    Id id(void) const { return id_; }
    
    /**
    * valid checks if a transition is valid. Valid transitions are a valid id and at least one valid area.
    */
    bool valid(void) const;

    /**
    * plusArea retrieves the plus area for the transition.
    */
    GlobalArea plusArea(void) const { return plusArea_; }

    /**
    * minusArea retrieves the minus area for the transition.
    */
    GlobalArea minusArea(void) const { return minusArea_; }
    
    /**
    * otherArea retrieves the area on the other side of the transition from the provided area. If there is no valid
    * area on the other side, then the returned GlobalArea will be marked as invalid.
    * 
    * \param    thisArea        One of the areas bounded by the transition
    * \return   The area on the other side of the transition. If thisArea == plusArea, then minusArea.
    *   If thisArea == minusArea, then plusArea. Otherwise, an invalid GlobalArea (id == kInvalidId)
    */
    GlobalArea otherArea(const GlobalArea& thisArea) const;
    
    /**
    * visitedArea retrieves the visited area for a frontier transition. A frontier transition will have one area
    * with a frontier id. This method will return the other area.
    * 
    * When this method is called on a non-frontier, the plusArea is returned.
    * 
    * \return   The non-frontier area associated with this transition. The plus area if the transition is explored.
    */
    GlobalArea visitedArea(void) const;
    
    /**
    * hasArea checks if one of the areas bounded by this transition is the provided area.
    * 
    * \param    area            Area to check
    * \return   minusArea() == area || plusArea() == area.
    */
    bool hasArea(const GlobalArea& area) const { return (plusArea_ == area) || (minusArea_ == area); }

    /**
    * isNavigable checks if the transition can be navigated.
    */
    bool isNavigable(void) const { return navigable_ == NavigationStatus::navigable; }

    /**
    * isFrontier checks if the transition has yet to be crossed.
    */
    bool isFrontier(void) const { return explored_ == ExplorationStatus::frontier; }

    /**
    * markExplored marks the transition as having been explored. This method should be called once the robot has
    * crossed the gateway associated with the transition.
    */
    void markExplored(void) { explored_ = ExplorationStatus::explored; }
    
private:

    Id id_ = kInvalidId;
    GlobalArea plusArea_;
    GlobalArea minusArea_;
    NavigationStatus navigable_ = NavigationStatus::null;
    ExplorationStatus explored_ = ExplorationStatus::frontier;
    
    // Serialization support
    friend class cereal::access;
    
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( id_,
            plusArea_,
            minusArea_,
            navigable_,
            explored_
        );
    }
};

///////////////   Non-member interface for GlobalTransition   ////////////////////////////
/**
* Equality is if the two areas have the same Id.
*/
bool operator==(const GlobalTransition& lhs, const GlobalTransition& rhs);
bool operator!=(const GlobalTransition& lhs, const GlobalTransition& rhs);

/**
* Output is:  Transition id: Plus:plusArea Minus:minusArea Nav:(true/false)  Exp:(true/false)
*/
std::ostream& operator<<(std::ostream& out, const GlobalTransition& transition);

/**
* are_similar_transitions checks if two transitions are similar. Similar transitions have the same navigation status and
* have the same types of area on each side.
* 
* The similar check doesn't compare the ids of the transitions. This function is meant to be used in determining if
* two transitions could be the same without requiring them to be globally unique.
* 
* \param    lhs         One of the transitions to consider 
* \param    rhs         The other transition to consider 
* \return   True if the type of the areas matches and the navigation status matches.
*/
bool are_similar_transitions(const GlobalTransition& lhs, const GlobalTransition& rhs);


} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_TRANSITION_H
