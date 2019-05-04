/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_location.h
* \author   Collin Johnson
*
* Definition of the GlobalLocation, which includes path_state_t and place_state_t. Taken
* together, these classes define the recent history of the robot's motion through
* the topological map.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_LOCATION_H
#define HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_LOCATION_H

#include <hssh/global_topological/transition.h>
#include <hssh/types.h>
#include <core/pose.h>
#include <iosfwd>

namespace vulcan
{
namespace hssh
{

/**
* GlobalLocation contains the recent state of the robot moving within the topological map. The robot state
* has two components, a path state and a place state. These states are maintained for the most recent
* motion along a path or within a place. Thus, when in a place, the path state won't change until the
* place is exited and the robot is once again moving along a path. Similarly, the state maintained for
* motion in a place doesn't reset until a new place is entered.
*/
struct GlobalLocation
{
    int64_t timestamp = 0;          ///< Time for which this location is current
    
    Id areaId = kInvalidId;             ///< Id of the area the robot is in
    AreaType areaType = AreaType::area; ///< Type of area the robot is currently in
    GlobalTransition entryTransition;   ///< Transition through which the area was entered
    TopoDirection pathDirection = TopoDirection::null;        ///< Direction the robot is heading, if it is on a path

    pose_t localToReferenceTransform;    ///< Transform from the reference frame of the local map to the reference map
                                                ///< for the most recently visited place.
                                                ///< Used at current place for appearance matching
                                                ///< Used at next place for getting correct orientation of Lambda

    /**
    * Constructor for GlobalLocation.
    * 
    * Creates a GlobalLocation for a place.
    * 
    * \param    id          Id of the area
    * \param    type        Type of the area
    * \param    entry       Entry transition into the area
    */
    GlobalLocation(Id id, AreaType type, const GlobalTransition& entry);
    GlobalLocation(GlobalArea area, const GlobalTransition& entry);
    
    /**
    * Constructor for GlobalLocation.
    * 
    * Creates a location for a GlobalPathSegment.
    * 
    * \param    id          Id of the area
    * \param    entry       Entry transition to the area
    * \param    direction   Direction the robot is facing along the path segment 
    */
    GlobalLocation(Id id, const GlobalTransition& entry, TopoDirection direction);
    GlobalLocation(GlobalArea area, const GlobalTransition& entry, TopoDirection direction);
    
    // Default constructor for GlobalLocation
    GlobalLocation(void) = default;
    
    // Serialization support
    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( timestamp,
            areaId,
            areaType,
            entryTransition,
            pathDirection,
            localToReferenceTransform
        );
    }
};


/////   Helper functions   /////

/**
* exited_area finds the area that was exited to arrive at the current location.
* 
* \param    location            Current topological location
* \return   The area that was exited to reach the current location -- might be invalid if at the original area.
*/
GlobalArea exited_area(const GlobalLocation& location);

/**
* current_area creates the current area from the location.
* 
* \param    location            Current topological location
* \return   The area the robot is currently in.
*/
GlobalArea current_area(const GlobalLocation& location);

/////   Operators   /////
std::ostream& operator<<(std::ostream& out, const GlobalLocation& state);

// Equality if at the same area with the same entry transition
bool operator==(const GlobalLocation& lhs, const GlobalLocation& rhs);
bool operator!=(const GlobalLocation& lhs, const GlobalLocation& rhs);

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_GLOBAL_LOCATION_H
