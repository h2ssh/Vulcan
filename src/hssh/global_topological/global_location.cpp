/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_location.cpp
* \author   Collin Johnson
*
* Definition of constructors and operators for GlobalLocation.
*/

#include <hssh/global_topological/global_location.h>
#include <ostream>

namespace vulcan
{
namespace hssh
{

GlobalLocation::GlobalLocation(Id id, AreaType type, const GlobalTransition& entry)
: areaId(id)
, areaType(type)
, entryTransition(entry)
, pathDirection(TopoDirection::null)
{
}


GlobalLocation::GlobalLocation(GlobalArea area, const GlobalTransition& entry)
: GlobalLocation(area.id(), area.type(), entry)
{
}


GlobalLocation::GlobalLocation(Id id, const GlobalTransition& entry, TopoDirection direction)
: areaId(id)
, areaType(AreaType::path_segment)
, entryTransition(entry)
, pathDirection(direction)
{
}


GlobalLocation::GlobalLocation(GlobalArea area, const GlobalTransition& entry, TopoDirection direction)
: GlobalLocation(area.id(), entry, direction)
{
}


GlobalArea exited_area(const GlobalLocation& location)
{
    return location.entryTransition.otherArea(GlobalArea(location.areaId, location.areaType));
}


GlobalArea current_area(const GlobalLocation& location)
{
    return GlobalArea(location.areaId, location.areaType);
}


std::ostream& operator<<(std::ostream& out, const GlobalLocation& state)
{
    out << state.areaType << " :: " << state.areaId << " via " << state.entryTransition.id();
    if(state.areaType == AreaType::path_segment)
    {
        out << " facing " << state.pathDirection << '\n';
    }
    
    return out;
}


bool operator==(const GlobalLocation& lhs, const GlobalLocation& rhs)
{
    return (lhs.areaId == rhs.areaId) && (lhs.entryTransition == rhs.entryTransition);
}


bool operator!=(const GlobalLocation& lhs, const GlobalLocation& rhs)
{
    return !(lhs == rhs);
}

} // namespace hssh
} // namespace vulcan
