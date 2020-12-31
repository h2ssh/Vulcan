/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_interface.h
* \author   Collin Johnson
* 
* Declaration of GoalInterface.
*/

#ifndef PLANNER_INTERFACE_GOAL_INTERFACE_H
#define PLANNER_INTERFACE_GOAL_INTERFACE_H

#include "planner/interface/goal.h"
#include "core/pose.h"
#include <iosfwd>
#include <map>

namespace vulcan
{
namespace planner
{

/**
* GoalInterface provides support for known, named locations (poses and areas) in a global map. The interface supports
* creating new named locations via simple methods as well as querying the available goals within the current map.
* 
* Currently, all locations are assumed to be in the same map. Future updates to this interface will support a better
* query structure where locations and maps are tied together, allowing for better traversal of a hierarchical structure
* of space. TODO
*
* Currently, all goals are assumed reachable given the current robot state. However, a topological area can't be reached
* if only a global metric map is available. Eventually, the interface should filter to only show the available goals.
*/
class GoalInterface
{
public:

    using GoalIter = std::vector<Goal>::const_iterator;
    
    /**
    * addGlobalPose adds a new goal to take the robot to a particular global pose in a global metric map.
    * 
    * If a global pose already exists with the given name, then the previous pose will be overwritten.
    * 
    * \param    name            Name of the pose
    * \param    pose            Global pose to add
    */
    void addGlobalPose(const std::string& name, const pose_t& pose);
    
    /**
    * addGlobalArea adds a new goal to take the robot to some global area in a global topo map.
    * 
    * If a global area already exists with the given name, then the previous location will be overwritten.
    * 
    * TODO
    * 
    * \param    name            Name of the area
    * \param    area            Area to add
    */
//     void addGlobalArea(const std::string& name, const hssh::GlobalArea::Id& area);

    /**
    * addLocalArea adds a new goal to take the robot to some local area in a local topo map. This approach is
    * for support for topological navigation until the global topological comes into existence.
    * 
    * \param    name            Name of the area
    * \param    area            Area to add
    */
    void addLocalArea(const std::string& name, hssh::LocalArea::Id area);


    // Iterator support for accessing the goals
    std::size_t size(void) const { return goals_.size(); }
    bool empty(void) const { return goals_.empty(); }
    GoalIter begin(void) const { return goals_.cbegin(); }
    GoalIter end(void) const { return goals_.cend(); }

    // I/O support
    /**
    * load loads a set of Goals from the provided stream.
    *
    * load appends any goals to the existing list of goals.
    *
    * The format is one Goal per line. See Goal for details.
    *
    * \param    in          Input stream from which to load the goals
    * \return   The number of goals loaded.
    */
    std::size_t load(std::istream& in);

    /**
    * save saves the current set of Goals to the provided stream.
    *
    * One Goal is saved per line. See Goal for details.
    *
    * \param    out         Output stream to write goals to
    * \return   True if all goals were successfully written.
    */
    bool save(std::ostream& out);

private:

    std::vector<Goal> goals_;
};

}
}

#endif // PLANNER_INTERFACE_GOAL_INTERFACE_H
