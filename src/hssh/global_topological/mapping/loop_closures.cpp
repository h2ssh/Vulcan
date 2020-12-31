/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     loop_closures.cpp
* \author   Collin Johnson
*
* Definition of functions that check if a loop closure can be made between two transitions:
*
*   - is_possible_loop_closure
*   - are_compatible_path_segments
*   - are_compatible_places
*/

#include "hssh/global_topological/mapping/loop_closures.h"
#include "hssh/global_topological/area.h"
#include "hssh/global_topological/transition.h"
#include "hssh/global_topological/global_location.h"
#include "hssh/global_topological/global_path_segment.h"
#include "hssh/global_topological/global_place.h"
#include "utils/stub.h"
#include <string>

// #define DEBUG_PATH_LOOPS 1
// #define DEBUG_PLACE_LOOPS 0

namespace vulcan
{
namespace hssh
{

std::string cycle_nav_string(const GlobalTransitionCycle& cycle);


bool is_possible_loop_closure(const GlobalLocation& newLocation,
                              const GlobalTransition& frontierTransition,
                              const GlobalArea& existingArea)
{
    // Either a place on the other end with a known type, or we've never even seen it before, so it can match anything
    bool exitedPlace = is_place_type(exited_area(newLocation).type());
    bool frontierIsPlace = is_place_type(frontierTransition.otherArea(existingArea).type());
    bool isPossible = (newLocation.areaType == existingArea.type())
        && ((exitedPlace == frontierIsPlace)
            || (frontierTransition.otherArea(existingArea).type() == AreaType::frontier));

#if DEBUG_PLACE_LOOPS || DEBUG_PATH_LOOPS
    std::cout << "DEBUG: is_possible_loop_closure: New:" << newLocation << " Previous: Trans:" << frontierTransition
        << " Area:" << existingArea << " Possible? " << isPossible << '\n';
#endif

    return isPossible;
}


bool are_compatible_path_segments(const GlobalPathSegment& lhs,
                                  const GlobalTransition& lhsEntry,
                                  const GlobalPathSegment& rhs,
                                  const GlobalTransition& rhsEntry)
{
    return true;
//     GlobalArea lhsEnd;
//     if(lhs.plusTransition() == lhsEntry)
//     {
//         lhsEnd = lhs.minusPlace();
//     }
//     else if(lhs.minusTransition() == lhsEntry)
//     {
//         lhsEnd = lhs.plusPlace();
//     }
//     else
//     {
//         // lhsEnd is an unknown area -- not worrying about transition sequences yet.
//     }
//
//     GlobalArea rhsEnd;
//     if(rhs.plusTransition() == rhsEntry)
//     {
//         rhsEnd = rhs.minusPlace();
//     }
//     else if(rhs.minusTransition() == rhsEntry)
//     {
//         rhsEnd = rhs.plusPlace();
//     }
//     else
//     {
//         // rhsEnd is an unknown area -- not worrying about transition sequences yet.
//     }
//
//     return ((rhsEnd.type() == lhsEntry.otherArea(lhs.toArea()).type()) || (rhsEnd.type() == AreaType::frontier))
//         && ((lhsEnd.type() == rhsEntry.otherArea(rhs.toArea()).type()) || (lhsEnd.type() == AreaType::frontier));
}


bool are_compatible_places(const GlobalPlace& lhs,
                           const GlobalTransition& lhsEntry,
                           const GlobalPlace& rhs,
                           const GlobalTransition& rhsEntry)
{
    bool areCompatible = are_cycles_compatible(lhs.cycle(), lhsEntry, rhs.cycle(), rhsEntry);

#if DEBUG_PLACE_LOOPS
    std::cout << "DEBUG: are_compatible_places: Checking between " << cycle_nav_string(lhs.cycle())
        << " entry:" << lhs.cycle().transitionIndex(lhsEntry) << " and " << cycle_nav_string(rhs.cycle())
         << " entry: " << rhs.cycle().transitionIndex(rhsEntry) << " Compatible? " << areCompatible << '\n';
#endif

    return areCompatible;
}

std::string cycle_nav_string(const GlobalTransitionCycle& cycle)
{
    std::string s;
    for(auto& t : cycle)
    {
        s += (t.isNavigable()) ? "1" : "0";
    }
    return s;
}

} // namespace hssh
} // namespace vulcans
