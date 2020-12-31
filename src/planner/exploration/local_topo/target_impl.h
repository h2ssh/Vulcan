/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     target_impl.h
 * \author   Collin Johnson
 *
 * Declaration of LocalAreaTarget and GatewayTarget subclasses of LocalTopoExplorationTarget.
 */

#ifndef PLANNER_EXPLORATION_LOCAL_TOPO_TARGET_IMPL_H
#define PLANNER_EXPLORATION_LOCAL_TOPO_TARGET_IMPL_H

#include "hssh/local_topological/area.h"
#include "hssh/local_topological/event_visitor.h"
#include "hssh/local_topological/gateway.h"
#include "planner/exploration/local_topo/target.h"

namespace vulcan
{
namespace planner
{

/**
 * LocalAreaTarget is a target that has the robot drive to the center of an area. The target is considered to be
 * visited once the robot has entered the area, as determined by an AreaTransitionEvent.
 *
 * The displayed boundary is the polygon boundary of the extent of the associated LocalArea.
 */
class LocalAreaTarget
: public LocalTopoExplorationTarget
, public hssh::LocalAreaEventVisitor
{
public:
    /**
     * Constructor for LocalAreaTarget.
     *
     * \param    area            Area to be explored
     */
    LocalAreaTarget(const hssh::LocalArea& area);

    /**
     * areaId retrieves the area id of the area associated with this target.
     */
    hssh::LocalArea::Id areaId(void) const { return areaId_; }

    /**
     * areaType retrieves the type of area associated with this target.
     */
    hssh::AreaType areaType(void) const { return areaType_; }

    // LocalTopoExplorationTarget interface
    bool wasVisited(void) const override { return visited_; }
    bool checkVisited(const hssh::LocalAreaEvent& event) override;
    std::shared_ptr<mpepc::NavigationTask> explorationTask(const pose_t& pose) const override;
    math::Polygon<float> boundary(void) const override { return boundary_; };

    // LocalAreaEventVisitor interface
    void visitAreaTransition(const hssh::AreaTransitionEvent& event) override;
    void visitTurnAround(const hssh::TurnAroundEvent& event) override;

private:
    bool visited_;                                         ///< Flag indicating if the target has been visited
    hssh::LocalArea::Id areaId_;                           // Id of the associated area to check when visited
    hssh::AreaType areaType_;                              // Type of the area
    std::shared_ptr<mpepc::NavigationTask> plannerTask_;   // Task created from the LocalArea extent
    math::Polygon<float> boundary_;                        // Boundary of the area extent

    // Serialization support
    LocalAreaTarget(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(visited_, areaId_, areaType_, plannerTask_, boundary_);
    }
};


/**
 * GatewayTarget is a target that has the robot drive across a gateway. The target is considered to be visited when
 * an AreaTransition indicates the transition the associated gateway.
 *
 * The displayed boundary is a rectangle oriented with the gateway boundary.
 */
class GatewayTarget
: public LocalTopoExplorationTarget
, public hssh::LocalAreaEventVisitor
{
public:
    /**
     * Constructor for GatewayTarget.
     *
     * \param    gateway         Gateway that needs to be crossed during exploration
     */
    GatewayTarget(const hssh::Gateway& gateway);

    // LocalTopoExplorationTarget interface
    bool wasVisited(void) const override { return visited_; }
    bool checkVisited(const hssh::LocalAreaEvent& event) override;
    std::shared_ptr<mpepc::NavigationTask> explorationTask(const pose_t& pose) const override;
    math::Polygon<float> boundary(void) const override { return boundary_; }

    // LocalAreaEventVisitor interface
    void visitAreaTransition(const hssh::AreaTransitionEvent& event) override;
    void visitTurnAround(const hssh::TurnAroundEvent& event) override;

private:
    bool visited_;                    ///< Flag indicating if the target has been visited
    hssh::Gateway gateway_;           ///< Gateway to be crossed for this target to be satisfied
    math::Polygon<float> boundary_;   ///< Rectangle approximation of the boundary

    // Serialization support
    GatewayTarget(void) { }

    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar)
    {
        ar(visited_, gateway_, boundary_);
    }
};

}   // namespace planner
}   // namespace vulcan

// Smart pointer serialization support
#include <cereal/archives/binary.hpp>
#include <cereal/types/polymorphic.hpp>

CEREAL_REGISTER_TYPE(vulcan::planner::LocalAreaTarget)
CEREAL_REGISTER_TYPE(vulcan::planner::GatewayTarget)

#endif   // PLANNER_EXPLORATION_LOCAL_TOPO_TARGET_IMPL_H
