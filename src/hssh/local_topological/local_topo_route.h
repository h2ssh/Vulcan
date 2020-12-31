/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_topo_route.h
* \author   Collin Johnson
*
* Declaration of LocalTopoRoute and LocalTopoRouteVisit, which together define a route from one area to another within
* a LocalTopoMap.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_ROUTE_H
#define HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_ROUTE_H

#include "hssh/local_topological/area.h"

namespace vulcan
{
namespace hssh
{
    
/**
* LocalTopoRouteVisit represents an area visited while traveling along some route through the environment. Each visit
* has an entry point into the area and an exit point from the area as well as a total distance traveled while in the
* area.
*/
class LocalTopoRouteVisit
{
public:
    
    /**
    * Constructor for LocalTopoRouteVisit.
    * 
    * The entry point is either the starting point for the entire route or the center of the entry gateway.
    * The exit point is either the ending point for the entire route or the center of the exit gateway.
    * 
    * \param    area            Area visited
    * \param    entry           Entry point to the area
    * \param    exit            Exit point of the area
    * \param    distance        Distance traveled moving from entry to exit
    */
    LocalTopoRouteVisit(LocalArea::Ptr area,
                        Point<float> entry,
                        Point<float> exit,
                        double distance,
                        boost::optional<Gateway> entryGateway = boost::none,
                        boost::optional<Gateway> exitGateway = boost::none);
    
    /**
    * area retrieves the area visited.
    */
    LocalArea& area(void) const { return *area_; }
    
    /**
    * entryPoint retrieves the entry point for the area visit.
    */
    Point<float> entryPoint(void) const { return entry_; }
    
    /**
    * exitPoint retrieves the exit point for the area visit.
    */
    Point<float> exitPoint(void) const { return exit_; }
    
    /**
    * distance retrieves the distance traveled within the area during the visit.
    */
    double distance(void) const { return distance_; }
    
    /**
    * entryGateway retrieves the entry gateway for the area, if it was entered via gateway.
    */
    boost::optional<Gateway> entryGateway(void) const { return entryGateway_; }
    
    /**
    * exitGateway retrieves the exit gateway for the area, if it was exited via gateway.
    */
    boost::optional<Gateway> exitGateway(void) const { return exitGateway_; }
    
private:
    
    LocalArea::Ptr area_;
    Point<float> entry_;
    Point<float> exit_;
    double distance_;
    boost::optional<Gateway> entryGateway_;
    boost::optional<Gateway> exitGateway_;
};


/**
* LocalTopoRoute describes the route from one area to another within the LocalTopoMap. A route is a sequence of visits
* to areas in the map. Each visit is described by the area and the amount of distance traveled within a particular area.
*/
class LocalTopoRoute
{
public:
    
    using const_iterator = std::vector<LocalTopoRouteVisit>::const_iterator;
    using const_reference = std::vector<LocalTopoRouteVisit>::const_reference;
    
    /**
    * Constructor for LocalTopoRoute.
    * 
    * Constructs an empty route. 
    * 
    * WARNING: Attempting to use an empty route will likely result in catastrophe, so ensure a non-empty route via
    * .empty() before using the route for anything.
    */
    LocalTopoRoute(void);

    /**
    * addVisit adds a new visit to the end of the route. For the visit to make sense, the entry point should be the
    * same as the previous exit point.
    */
    void addVisit(const LocalTopoRouteVisit& visit);
    
    /**
    * startArea retrieves the starting area for the route.
    */
    LocalArea& startArea(void) const { return visits_.front().area(); }
    
    /**
    * goalArea retrieves the goal area for the route.
    */
    LocalArea& endArea(void) const { return visits_.back().area(); }
    
    /**
    * length retrieves the overall length of the route.
    */
    double length(void) const { return length_; }
    
    // Iterator access for the areas visited in the route
    std::size_t size(void) const { return visits_.size(); }
    bool empty(void) const { return visits_.empty(); }
    const_iterator begin(void) const { return visits_.begin(); }
    const_iterator end(void) const { return visits_.end(); }
    const_reference front(void) const { return visits_.front(); }
    const_reference back(void) const { return visits_.back(); }
    
public:

    std::vector<LocalTopoRouteVisit> visits_;
    double length_;
};

// Output operator for displaying a route -- output is the sequence of event points
std::ostream& operator<<(std::ostream& out, const LocalTopoRoute& route);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_LOCAL_TOPO_ROUTE_H
