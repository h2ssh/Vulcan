/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     learned_norm_cost.cpp
* \author   Collin Johnson
*
* Definition of a cost function for costs associated with learned social norms in the environment.
*/

#include "mpepc/cost/social_cost.h"
#include "mpepc/cost/cost_map.h"
#include "mpepc/social/social_norm_utils.h"
#include "mpepc/types.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/local_topo_route.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "utils/algorithm_ext.h"
#include <boost/range/iterator_range.hpp>

using namespace vulcan::hssh;

namespace vulcan
{
namespace mpepc
{

const double kSocialCostModifier = 1;

struct learned_cost_state_t
{
    float robotRadius;

    Point<float> entry;
    Point<float> exit;

    cell_t startCell;
    cell_t endCell;

    CellSet markedCells;

    const TopoSituationResponse* response;
    const LocalArea* area;

    const learned_norm_cost_params_t* params;
    const planning_environment_t* env;
    const VoronoiSkeletonGrid* skeleton;
    CostMap* costs;
};

namespace
{
void filter_unimportant_agents(position_t robot, std::vector<topo_agent_t>& otherAgents);
topo_agent_t create_agent_for_visit(const hssh::LocalTopoRouteVisit& visit, const motion_state_t& state);
const TopoSituationResponse* find_situation_response(const learned_norm_cost_params_t& params,
                                                     const PlaceSituation& situation);
const TopoSituationResponse* find_situation_response(const learned_norm_cost_params_t& params,
                                                     const PathSituation& situation);
void skeleton_learned_cost(learned_cost_state_t& state);
float normalized_cell_distance(const learned_cost_state_t& state,
                               cell_t gridCell,
                               int skelIdx,
                               const CellVector& skeleton);
}


void learned_norm_cost(const hssh::LocalTopoRoute& route,
                       const learned_norm_cost_params_t& params,
                       const planning_environment_t& env,
                       const std::vector<dynamic_object_trajectory_t>& objects,
                       CostMap& costs,
                       learned_norm_info_t& norms)
{
    // No learned norm cost if no topological map information
    if(!env.ltm)
    {
        return;
    }

    norms.agents = find_topo_agents(objects, *env.ltm);
    filter_unimportant_agents(env.robotState.pose.toPoint(), norms.agents);

    learned_cost_state_t state;
    state.robotRadius = env.robotRadius;
    state.params = &params;
    state.costs = &costs;
    state.env = &env;
    state.skeleton = &env.ltm->voronoiSkeleton();

    for(auto& visit : route)
    {
        // Construct the robot agent for the particular visit
        topo_agent_t robotAgent = create_agent_for_visit(visit, env.robotState);

        state.area = &visit.area();
        state.entry = visit.entryPoint();
        state.exit = visit.exitPoint();

        state.startCell = utils::global_point_to_grid_cell(state.entry, *state.skeleton);
        state.endCell = utils::global_point_to_grid_cell(state.exit, *state.skeleton);

        state.markedCells.clear();

        if(visit.area().type() == AreaType::path_segment)
        {
            PathSituation situation(robotAgent, norms.agents, params.numLateralBins, *env.ltm);
            state.response = find_situation_response(params, situation);
            skeleton_learned_cost(state);
            norms.pathSituations.push_back(situation);
        }
        else
        {
            PlaceSituation situation(robotAgent, norms.agents, *env.ltm);
            state.response = find_situation_response(params, situation);
            skeleton_learned_cost(state);
            norms.placeSituations.push_back(situation);
        }
    }
}

namespace
{

void filter_unimportant_agents(position_t robot, std::vector<topo_agent_t>& otherAgents)
{
    // Only care about other agents within a certain distance, so the agent has had some time to react to
    // their presence
    const double kMaxImportantDist = 10.0;

    utils::erase_remove_if(otherAgents, [&robot, kMaxImportantDist](auto& other) {
        return distance_between_points(robot, Point<float>(other.state.x, other.state.y))
            > kMaxImportantDist;
    });
}

topo_agent_t create_agent_for_visit(const hssh::LocalTopoRouteVisit& visit, const motion_state_t& state)
{
    topo_agent_t agent;
    agent.state.x = visit.entryPoint().x;
    agent.state.y = visit.entryPoint().y;

    // Assume constant robot velocity in the direction of the exit
    double headingToExit = angle_to_point(visit.entryPoint(), visit.exitPoint());
    agent.state.xVel = std::cos(headingToExit) * state.velocity.linear;
    agent.state.yVel = std::sin(headingToExit) * state.velocity.linear;

    agent.areaId = visit.area().id();

    // If there's an exit gateway, just use that
    if(auto exitGateway = visit.exitGateway())
    {
        agent.gatewayId = exitGateway->id();
    }
    // if there's not an exit, then just use the gateway furthest from the entry
    else if(auto entryGateway = visit.entryGateway())
    {
        auto gateways = visit.area().gateways();
        auto maxIt = std::max_element(gateways.begin(), gateways.end(), [&entryGateway](auto& lhs, auto& rhs) {
            return squared_point_distance(lhs.center(), entryGateway->center())
                < squared_point_distance(rhs.center(), entryGateway->center());
        });

        agent.gatewayId = maxIt->id();
    }
    // Otherwise, there's no known gateway exit
    else
    {
        agent.gatewayId = -1;
    }

    return agent;
}


const TopoSituationResponse* find_situation_response(const learned_norm_cost_params_t& params,
                                                     const PlaceSituation& situation)
{
    for(auto& response : params.responses)
    {
        if(response.isResponseForSituation(situation))
        {
            return &response;
        }
    }

    return &params.defaultResponsePlace;
}


const TopoSituationResponse* find_situation_response(const learned_norm_cost_params_t& params,
                                                     const PathSituation& situation)
{
    for(auto& response : params.responses)
    {
        if(response.isResponseForSituation(situation))
        {
            return &response;
        }
    }

    return &params.defaultResponsePath;
}


void skeleton_learned_cost(learned_cost_state_t& state)
{
    // The Voronoi skeleton is used to approximate the center of the path
    auto cellsAlongRoute = skeleton_cells_along_route(state.startCell, state.endCell, *state.skeleton);

    int nearestIdx = 0;

    for(auto globalCell : state.area->extent())
    {
        auto gridCell = utils::global_point_to_grid_cell_round(globalCell, *state.skeleton);

        // If costs are assigned, don't add more cost
        if(state.markedCells.find(gridCell) != state.markedCells.end())
        {
            continue;
        }

        state.markedCells.insert(gridCell);

        nearestIdx = nearest_skeleton_index(gridCell, cellsAlongRoute, nearestIdx);
        float normalizedDist = 0.0f;

        // If there are cells along the route, then area-type costs have valid values
        if(!cellsAlongRoute.empty())
        {
            normalizedDist = normalized_cell_distance(state, gridCell, nearestIdx, cellsAlongRoute);
        }

        gridCell = utils::global_point_to_grid_cell_round(globalCell, *state.costs);

        int32_t cost = std::max(0, state.costs->getValueNoCheck(gridCell.x, gridCell.y) - kNonRouteCost);
        cost += kStraightCellDist * kSocialCostModifier * state.response->distanceCost(normalizedDist);
        state.costs->setValue(gridCell.x, gridCell.y, cost);
    }

    std::cout << "Situation response:" << *state.response << " dist:";
    for(auto& dist : state.response->distribution())
    {
        printf(" %.3f", dist);
    }
    std::cout << '\n';
}


float normalized_cell_distance(const learned_cost_state_t& state,
                               cell_t gridCell,
                               int skelIdx,
                               const CellVector& skeleton)
{
    cell_t skelCell = skeleton[skelIdx];

    // The path is narrowed by the robot radius because that is when it is adjacent to the wall, not when the center
    // is adjacent, which it couldn't be
    double pathRadius = state.skeleton->getMetricDistance(skelCell.x, skelCell.y) - state.robotRadius;
    // Can't use skeleton directly because we only care about distance to relevant portion of skeleton, which doesn't
    // include all nooks and crannies along hallways. If there's a small alcove or a branch leading into an office,
    // then it might be wrong because the distance measured isn't to the same wall as the center of the skeleton
    double distToSkel = distance_between_points(gridCell, skelCell) * state.skeleton->metersPerCell();
    double lateralDistance = pathRadius - distToSkel;

    if(is_right_of_skeleton(gridCell, skelIdx, skeleton))
    {
        // Right of the path it's the full distance minus the distance remaining to the nearest wall
        lateralDistance = pathRadius + distToSkel;
    }

    if(pathRadius > 0.0)
    {
        lateralDistance = std::max(lateralDistance, 0.0);
        return std::min(lateralDistance / (2.0 * pathRadius), 1.0);
    }

    return -1.0;
}

} // anonymous namespace

} // namespace mpepc
} // namespace vulcan
