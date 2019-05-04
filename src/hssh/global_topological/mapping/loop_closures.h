/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     loop_closures.h
* \author   Collin Johnson
* 
* Declaration of functions that check if a loop closure can be made between two transitions:
* 
*   - is_possible_loop_closure : perform a quick check to see if the types of areas are consistent
*   - are_compatible_path_segments : check if the path + transition matches are valid for the given path segment 
*   - are_compatible_places : check if the entry + place combinations are compatible with a loop closure 
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LOOP_CLOSURES_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LOOP_CLOSURES_H

namespace vulcan
{
namespace hssh
{
    
class GlobalArea;
class GlobalPlace;
class GlobalPathSegment;
class GlobalTransition;
struct GlobalLocation;

/**
* is_possible_loop_closure checks if it is even possible that a loop closure exists for the frontier transition and 
* the newly entered area. This function is intended as an initial filter before doing the more complicated checks
* for compatible path segments as described below.
* 
* A possible loop closure requires:
* 
*   - The new area to be the same type as the existing area
*   - The area on the other side of the frontier transition to be unknown or of the same type as the previous area
* 
* This function performs the initial type checks. After these checks, you should use the specific functions that check 
* the symbolic match between two areas of the same type. 
* 
* \param    newLocation             New location of the robot to see if it might be the existing area
* \param    frontierTransition      Frontier transition in the existing area 
* \param    existingArea            Existing area against which to check for a valid loop closure
* \return   True if a loop closure might exist.
*/
bool is_possible_loop_closure(const GlobalLocation& newLocation,
                              const GlobalTransition& frontierTransition, 
                              const GlobalArea& existingArea);

/**
* are_compatible_path_segments 
*/
bool are_compatible_path_segments(const GlobalPathSegment& lhs, 
                                  const GlobalTransition& lhsEntry,
                                  const GlobalPathSegment& rhs, 
                                  const GlobalTransition& rhsEntry);

/**
* are_compatible_places 
*/
bool are_compatible_places(const GlobalPlace& lhs, 
                           const GlobalTransition& lhsEntry,
                           const GlobalPlace& rhs, 
                           const GlobalTransition& rhsEntry);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LOOP_CLOSURES_H
