/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     goal.h
 * \author   Collin Johnson
 *
 * Declaration of Goal.
 */

#ifndef PLANNER_INTERFACE_GOAL_H
#define PLANNER_INTERFACE_GOAL_H

#include "core/pose.h"
#include "hssh/local_topological/area.h"
#include <boost/optional.hpp>
#include <iosfwd>

namespace vulcan
{
namespace planner
{

/**
 * Goal represents a named goal location in the robot's global knowledge of the environment. A Goal is defined by at
 * least one of the following:
 *
 *   - A pose in the GlobalMetricMap
 *   - An area id in a LocalTopoMap   (temporary functionality)
 *   - An area id in a GlobalTopoMap
 *
 * Each Goal has a name and at least one of the above. A goal can be modified to include additional descriptors in
 * different map types via the setXXXX functions. These will overwrite any previous data they contain.
 *
 * Goal I/O is supported via stream operators.
 *
 * The format for a saved Goal is:
 *
 *       name type1_desc type1_goal type2_desc type2_goal ....
 *
 * where name is the name of the goal, type1_desc is the description of what type of map the type1_goal defines, and
 * type1_goal contains the actual goal description.
 */
class Goal
{
public:
    /**
     * Default constructor for Goal.
     */
    Goal(void);

    /**
     * Constructor for Goal.
     *
     * Create a Goal associated with a global pose.
     *
     * \param    name            Name of the goal
     * \param    pose            Global pose associated with the goal
     */
    Goal(const std::string& name, const pose_t& pose);

    /**
     * Constructor for Goal.
     *
     * Create a Goal associated with a local topo area.
     *
     * \param    name            Name of the goal
     * \param    area            Id of the associated area
     */
    Goal(const std::string& name, hssh::LocalArea::Id area);

    /**
     * Constructor for Goal.
     *
     * Create a Goal associated with a global topo area.
     *
     * TODO
     *
     * \param    name            Name of the goal
     * \param    area            Id of the associated area
     */
    //     Goal(const std::string& name, hssh::GlobalArea::Id area);

    /**
     * name retrieves the name of the Goal.
     */
    const std::string& name(void) const { return name_; }

    /**
     * globalPose retrieves the global pose associated with the goal, if there is one.
     */
    boost::optional<pose_t> globalPose(void) const { return globalPose_; }

    /**
     * localArea retrieves the local area associated with the goal, if there is one.
     */
    boost::optional<hssh::LocalArea::Id> localArea(void) const { return localArea_; }

    /**
     * globalArea retrieves the global area associated with the goal, if there is one.
     *
     * TODO
     */
    //     boost::optional<hssh::GlobalArea::Id> globalArea(void) const { return globalArea_; }

    /**
     * setGlobalPose assigns a new global pose to the goal.
     */
    void setGlobalPose(const pose_t& pose) { globalPose_ = pose; }

    /**
     * setLocalArea assigns a new local area to the goal.
     */
    void setLocalArea(hssh::LocalArea::Id area) { localArea_ = area; }

    /**
     * setGlobalArea assigns a new global area to the goal.
     *
     * TODO
     */
    //     void setGlobalArea(hssh::GlobalArea::Id area) { globalArea_ = area; }

    // I/O operators
    // A Goal is assumed to take an entire line. It will consume the stream using getline().
    friend std::istream& operator>>(std::istream& in, Goal& g);
    friend std::ostream& operator<<(std::ostream& out, const Goal& g);

private:
    std::string name_;
    boost::optional<pose_t> globalPose_;
    //     boost::optional<hssh::GlobalArea::Id> globalArea_;
    boost::optional<hssh::LocalArea::Id> localArea_;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_INTERFACE_GOAL_H
