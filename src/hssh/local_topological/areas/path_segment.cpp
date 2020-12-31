/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     path_segment.cpp
 * \author   Collin Johnson
 *
 * Definition of LocalPathSegment.
 */

#include "hssh/local_topological/areas/path_segment.h"
#include "hssh/local_topological/area_visitor.h"
#include <sstream>

namespace vulcan
{
namespace hssh
{

LocalPathSegment::LocalPathSegment(const TransitionAffordance& plus,
                                   const TransitionAffordance& minus,
                                   const std::vector<TransitionAffordance>& leftDestinations,
                                   const std::vector<TransitionAffordance>& rightDestinations,
                                   const Lambda& lambda,
                                   int id,
                                   const AreaExtent& extent,
                                   const std::vector<Gateway>& gateways)
: LocalArea(id, extent, gateways)
, plusTransition_(plus)
, minusTransition_(minus)
, plus_(plus.target().toPoint(), minus.target().toPoint(), TopoDirection::plus)
, minus_(plus.target().toPoint(), minus.target().toPoint(), TopoDirection::minus)
, leftDestinations_(leftDestinations)
, rightDestinations_(rightDestinations)
, lambda_(lambda)
{
}


std::string LocalPathSegment::description(void) const
{
    std::ostringstream str;
    str << "path " << id() << ':' << plus_.target() << "->" << minus_.target();
    return str.str();
}


void LocalPathSegment::visitAffordances(NavigationAffordanceVisitor& visitor) const
{
    for (auto& dest : leftDestinations_) {
        dest.accept(visitor);
    }

    for (auto& dest : rightDestinations_) {
        dest.accept(visitor);
    }

    plus_.accept(visitor);
    minus_.accept(visitor);
}


void LocalPathSegment::accept(LocalAreaVisitor& visitor) const
{
    visitor.visitPathSegment(*this);
}


bool LocalPathSegment::isEndpoint(const LocalArea& adj) const
{
    return adj.hasGateway(plusTransition_.gateway()) || adj.hasGateway(minusTransition_.gateway());
}

}   // namespace hssh
}   // namespace vulcan
