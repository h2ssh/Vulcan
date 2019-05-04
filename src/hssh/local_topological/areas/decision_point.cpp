/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_point.cpp
* \author   Collin Johnson
*
* Definition of LocalDecisionPoint.
*/

#include <hssh/local_topological/areas/decision_point.h>
#include <hssh/local_topological/area_visitor.h>
#include <sstream>

namespace vulcan 
{
namespace hssh 
{

LocalDecisionPoint::LocalDecisionPoint(const SmallScaleStar&       star,
                                       const LocalPerceptualMap&   map,
                                       int                         id,
                                       const AreaExtent&           extent,
                                       const std::vector<Gateway>& gateways)
: LocalPlace(star, map, id, extent, gateways)
{
}


std::string LocalDecisionPoint::description(void) const
{
    std::ostringstream str;
    str << "decision " << id() << ':' << boundary(math::ReferenceFrame::GLOBAL);
    return str.str();
}


void LocalDecisionPoint::accept(LocalAreaVisitor& visitor) const
{
    visitor.visitDecisionPoint(*this);
}

} // namespace hssh
} // namespace vulcan
