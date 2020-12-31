/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area.cpp
 * \author   Collin Johnson
 *
 * Definition of GlobalArea.
 */

#include "hssh/global_topological/area.h"
#include <ostream>

namespace vulcan
{
namespace hssh
{

GlobalArea::GlobalArea(Id id, AreaType type) : id_(id), type_(type)
{
    assert(type != AreaType::area);
}


GlobalArea::GlobalArea(AreaType type) : GlobalArea(kFrontierId, type)
{
    assert(type != AreaType::area);
}


bool operator==(const GlobalArea& lhs, const GlobalArea& rhs)
{
    return (lhs.id() == rhs.id()) && (lhs.type() == rhs.type());
}


bool operator!=(const GlobalArea& lhs, const GlobalArea& rhs)
{
    return !(lhs == rhs);
}


std::ostream& operator<<(std::ostream& out, const GlobalArea& area)
{
    out << area.id() << "::" << area.type();
    return out;
}

}   // namespace hssh
}   // namespace vulcan
