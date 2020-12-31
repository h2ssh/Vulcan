/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topo_agent.cpp
* \author   Collin Johnson
*
* Definition find_topo_agents function.
*/

#include "mpepc/social/topo_agent.h"
#include "hssh/local_topological/area.h"
#include "hssh/local_topological/local_topo_map.h"
#include "tracker/dynamic_object_collection.h"
#include "tracker/dynamic_object_visitor.h"
#include "tracker/objects/rigid.h"
#include "core/float_comparison.h"

// #define DEBUG_TOPO_AGENTS

namespace vulcan
{
namespace mpepc
{

class ObjectValidator : public tracker::DynamicObjectVisitor
{
public:

    bool isValid(void) const { return isValid_; }

    // tracker::DynamicObjectVisitor interface
    void visitPerson(const tracker::Person & person) override
    {
        // Ignore
    }

    void visitPivotingObject(const tracker::PivotingObject & door) override
    {
        // Ignore
    }

    void visitRigid(const tracker::RigidObject & object) override
    {
        using namespace vulcan::tracker;
        auto state = object.slowMotionState();
        isValid_ = (std::sqrt(state(velXIndex, velXIndex)) + std::sqrt(state(velYIndex, velYIndex))) < 0.5;
    }

    void visitSlidingObject(const tracker::SlidingObject & door) override
    {
        // Ignore
    }

    void visitUnclassified(const tracker::UnclassifiedObject & object) override
    {
        // Ignore
    }

private:

    bool isValid_ = false;
};

// Converts the ObjectDestination into a Gateway for use in the topological representation of an agent
struct agent_goal_converter_t : public boost::static_visitor<hssh::Gateway>
{
    hssh::LocalAreaPtr area;

    hssh::Gateway operator()(const Point<double>& position)
    {
        assert(area);
        assert(!area->gateways().empty());
        // Use the gateway with the closest center to position
        auto minGwyIt = std::min_element(area->gateways().begin(),
                                         area->gateways().end(),
                                         [position](const auto& lhs, const auto& rhs) {
            return squared_point_distance(position, lhs.center())
                < squared_point_distance(position, rhs.center());
        });

        return *minGwyIt;
    }

    hssh::Gateway operator()(const Line<double>& boundary)
    {
        return operator()(center(boundary));
    }
};


bool is_viable_agent(const tracker::DynamicObject::ConstPtr& object, const hssh::LocalTopoMap& topoMap);
hssh::LocalAreaPtr best_area_with_object(const tracker::DynamicObject::ConstPtr& object,
                                         const hssh::LocalTopoMap& topoMap);


bool operator==(const topo_agent_t& lhs, const topo_agent_t& rhs)
{
    return (absolute_fuzzy_equal(lhs.state.x, rhs.state.x))
        && (absolute_fuzzy_equal(lhs.state.y, rhs.state.y))
        && (lhs.areaId == rhs.areaId)
        && (lhs.gatewayId == rhs.gatewayId);
}


bool operator!=(const topo_agent_t& lhs, const topo_agent_t& rhs)
{
    return !(lhs == rhs);
}


std::vector<topo_agent_t> find_topo_agents(const std::vector<dynamic_object_trajectory_t>& agents,
                                           const hssh::LocalTopoMap& topoMap)
{
    std::vector<topo_agent_t> topoAgents;

    for(auto& obj : agents)
    {
        if(auto topo = convert_to_topo_agent(obj.laserObject, topoMap))
        {
            topoAgents.push_back(*topo);
        }
    }

    return topoAgents;
}


std::vector<topo_agent_t> find_topo_agents(const tracker::DynamicObjectCollection& agents,
                                           const hssh::LocalTopoMap& topoMap)
{
    std::vector<topo_agent_t> topoAgents;

    for(auto& obj : agents)
    {
        if(auto topo = convert_to_topo_agent(obj, topoMap))
        {
            topoAgents.push_back(*topo);
        }
    }

    return topoAgents;
}


boost::optional<topo_agent_t> convert_to_topo_agent(const tracker::DynamicObject::ConstPtr& object,
                                                    const hssh::LocalTopoMap& topoMap)
{
    // Check if the object is in a known area
    auto area = best_area_with_object(object, topoMap);

    // If not in a known area, then the object can't be a topo agent
    // If there aren't any gateways, we also can't represent the topological state
    if(!area || area->gateways().empty())
    {
        return boost::none;
    }

    // Is the object something that could exist?
    ObjectValidator validator;
    object->accept(validator);
    if(!validator.isValid() || !is_viable_agent(object, topoMap))
    {
        return boost::none;
    }

    // Find the gateway that corresponds to the object's goal
    agent_goal_converter_t converter;
    converter.area = area;
    auto goal = object->goals().bestGoal();
    auto gateway = goal.destination().apply_visitor(converter);

    topo_agent_t agent;
    agent.radius = object->radius();
    agent.state = dynamic_object_state_t(object->motionState());
    agent.areaId = area->id();
    agent.gatewayId = gateway.id();

    return agent;
}


bool is_viable_agent(const tracker::DynamicObject::ConstPtr& object, const hssh::LocalTopoMap& topoMap)
{
    // Only worry about objects that are too fast (i.e. sensor noise) or a non-human size
    const double kMinAgentVelocity = 0.0;
    const double kMaxAgentVelocity = 2.0;
    const double kMinRadius = 0.075;
    const double kMaxRadius = 0.3;

    double velocity = std::sqrt(std::pow(object->velocity().x, 2.0) + std::pow(object->velocity().y, 2.0));
    if(velocity < kMinAgentVelocity)
    {
#ifdef DEBUG_TOPO_AGENTS
        std::cout << "Ignored slow object " << object->position() << " Vel: " << velocity
            << " cutoff:" << kMinAgentVelocity << '\n';
#endif
        return false;
    }

    if(velocity > kMaxAgentVelocity)
    {
#ifdef DEBUG_TOPO_AGENTS
        std::cout << "Ignored fast object " << object->position() << " Vel: " << velocity
            << " cutoff:" << kMaxAgentVelocity << '\n';
#endif
        return false;
    }

    if(object->radius() < kMinRadius)
    {
#ifdef DEBUG_TOPO_AGENTS
        std::cout << "Ignored small object " << object->position() << " Rad: " << object->radius()
            << " cutoff:" << kMinRadius << '\n';
#endif
        return false;
    }

    if(object->radius() > kMaxRadius)
    {
#ifdef DEBUG_TOPO_AGENTS
        std::cout << "Ignored large object " << object->position() << " Rad: " << object->radius()
            << " cutoff:" << kMaxRadius << '\n';
#endif
        return false;
    }

    auto agentCell = utils::global_point_to_grid_cell(object->position(), topoMap.voronoiSkeleton());
    if(topoMap.voronoiSkeleton().getMetricDistance(agentCell.x, agentCell.y) + object->radius() < 0.3)
    {
#ifdef DEBUG_TOPO_AGENTS
        std::cout << "Ignored object with cell too close to the wall " << object->position()
        << " Dist: " << topoMap.voronoiSkeleton().getMetricDistance(agentCell.x, agentCell.y) << '\n';
#endif
        return false;
    }

    return true;
}


hssh::LocalAreaPtr best_area_with_object(const tracker::DynamicObject::ConstPtr& object,
                                         const hssh::LocalTopoMap& topoMap)
{
    auto areas = topoMap.allAreasContaining(object->position());

    if(areas.empty())
    {
        return nullptr;
    }
    else if(areas.size() == 1)
    {
        return areas.front();
    }

    // need to find the best of the areas it still contains.
    // Take the area for which the object is closest to the center
    auto areaIt = std::min_element(areas.begin(), areas.end(), [&object](const auto& lhs, const auto& rhs) {
        return distance_between_points(object->position(), lhs->center().toPoint())
            < distance_between_points(object->position(), rhs->center().toPoint());
    });

    return *areaIt;
}

} // namespace mpepc
} // namespace vulcan
