/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision_interface.h
 * \author   Collin Johnson
 *
 * Declaration of DecisionInterface.
 */

#ifndef PLANNER_INTERFACE_DECISION_INTEFACE_H
#define PLANNER_INTERFACE_DECISION_INTEFACE_H

#include "hssh/local_topological/area_visitor.h"
#include "hssh/local_topological/location.h"
#include "planner/interface/decision.h"
#include <vector>

namespace vulcan
{
namespace hssh
{
class LocalTopoMap;
}
namespace planner
{

/**
 * DecisionInterface is responsible for determining the possible actions the robot can take given the current local
 * topology. The interface receives the current local topo state:  LocalTopoMap and LocalLocation.
 */
class DecisionInterface : public hssh::LocalAreaVisitor
{
public:
    using DecisionIter = std::vector<Decision>::const_iterator;

    /**
     * determineActions determines the actions available to the robot in its current topological location. These actions
     * are described generically as Decisions. The user should select among these actions.
     *
     * If the LocalLocation hasn't changed, then no new actions will be found.
     *
     * The determined actions can be accessed via the iterators.
     *
     * \param    map         Local topological description of the nearby environment
     * \param    location    Location of the robot within this map
     * \param    pose        Current pose of the robot
     * \return   Number of new actions found for the current location, which will be 0 if the location is unchanged from
     *           the previous update.
     */
    int determineActions(const hssh::LocalTopoMap& map, const hssh::LocalLocation& location, const pose_t& pose);

    // Iterator access for the actions
    std::size_t size(void) const { return decisions_.size(); }
    DecisionIter begin(void) const { return decisions_.begin(); }
    DecisionIter end(void) const { return decisions_.end(); }
    const Decision& operator[](std::size_t index) { return decisions_[index]; }

    // hssh::LocalAreaVisitor interface
    void visitDecisionPoint(const hssh::LocalDecisionPoint& decision) override;
    void visitDestination(const hssh::LocalDestination& destination) override;
    void visitPathSegment(const hssh::LocalPathSegment& path) override;

private:
    std::string moduleName_;
    std::vector<Decision> decisions_;
    const hssh::LocalTopoMap* map_;
    hssh::LocalLocation location_;
    pose_t pose_;
};

}   // namespace planner
}   // namespace vulcan

#endif   // PLANNER_INTERFACE_DECISION_INTEFACE_H
