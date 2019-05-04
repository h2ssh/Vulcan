/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_route.cpp
* \author   Collin Johnson
*
* Definition of LocalTopoRoute and LocalTopoRouteVisit.
*/

#include <hssh/local_topological/local_topo_route.h>

namespace vulcan
{
namespace hssh
{

//////////////////// LocalTopoRouteVisit implementation ////////////////////////////
    
LocalTopoRouteVisit::LocalTopoRouteVisit(LocalArea::Ptr area,
                                         Point<float> entry,
                                         Point<float> exit,
                                         double distance,
                                         boost::optional<Gateway> entryGateway,
                                         boost::optional<Gateway> exitGateway)
: area_(area)
, entry_(entry)
, exit_(exit)
, distance_(distance)
, entryGateway_(entryGateway)
, exitGateway_(exitGateway)
{
    assert(area_);
}


//////////////////// LocalTopoRoute implementation ////////////////////////////////

LocalTopoRoute::LocalTopoRoute(void)
: length_(0.0)
{
}


void LocalTopoRoute::addVisit(const LocalTopoRouteVisit& visit)
{
    visits_.push_back(visit);
    length_ += visit.distance();
}


std::ostream& operator<<(std::ostream& out, const LocalTopoRoute& route)
{
    if(route.empty())
    {
        return out;
    }
    
    // Need to give the start point
    out << route.front().entryPoint() << " -> ";
    
    // And then just exit points can be used
    for(auto& visit : route)
    {
        out << visit.exitPoint() << " -> ";
    }
    return out;
}

}
}
