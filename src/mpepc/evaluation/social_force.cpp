/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     social_force.cpp
* \author   Collin Johnson
*
* Definition of interaction_social_forces and trajectory_social_forces.
*/

#include <mpepc/evaluation/social_force.h>
#include <mpepc/evaluation/interaction.h>
#include <core/line.h>
#include <boost/optional.hpp>
#include <tuple>

// #define DEBUG_PASSING

namespace vulcan
{
namespace mpepc
{

struct social_forces_state_t
{
    double radius;
    position_t position;
    Vector heading;
    Vector velocity;

    social_forces_state_t(void)
    : radius(1.0)
    , heading(2)
    , velocity(2)
    {
    }
};

// get<0> = blame, get<1> = force
std::tuple<double, double> forces_from_agent(const topo_agent_t& agent,
                                             const social_forces_state_t& state,
                                             const social_forces_params_t& params);
boost::optional<passing_object_t> is_passing_object(const topo_agent_t& agent,
                                                    const math::Rectangle<float>& robotBoundary,
                                                    double speed,
                                                    double heading);
void assign_ids_to_passing_objects(std::vector<social_forces_t>& forces);


social_forces_t interaction_social_forces(const interaction_t& interaction, const social_forces_params_t& params)
{
    social_forces_t forces;
    forces.timestamp = interaction.timestamp;
    forces.pose = interaction.pose;
    forces.isInteracting = !interaction.agents.empty();

    social_forces_state_t state;
    state.position = interaction.pose.toPoint();
    state.heading[0] = std::cos(forces.pose.theta);
    state.heading[1] = std::sin(forces.pose.theta);

    state.velocity = state.heading * interaction.velocity.linear;

    forces.blame = 0.0;
    forces.force = 0.0;

    double agentBlame = 0.0;
    double agentForce = 0.0;

    auto robotBoundary = params.robotBoundary;
    robotBoundary.rotate(interaction.pose.theta);
    robotBoundary.translate(interaction.pose.x, interaction.pose.y);

    for(auto& agent : interaction.agents)
    {
        std::tie(agentBlame, agentForce) = forces_from_agent(agent, state, params);

        if(agentBlame > forces.blame)
        {
            forces.blame = agentBlame;
            forces.blameObj = position_t(agent.state.x, agent.state.y);
        }

        if(agentForce > forces.force)
        {
            forces.force = agentForce;
            forces.forceObj = position_t(agent.state.x, agent.state.y);
        }

        if(agent.areaId == interaction.areaId)
        {
            if(auto passingObj = is_passing_object(agent,
                                                   robotBoundary,
                                                   interaction.velocity.linear,
                                                   interaction.pose.theta))
            {
                forces.passingObj.push_back(*passingObj);
            }
        }
    }

    return forces;
}


std::vector<social_forces_t> trajectory_social_forces(const std::vector<interaction_t>& interactions,
                                                      const social_forces_params_t& params)
{
    std::vector<social_forces_t> forces;

    std::transform(interactions.begin(), interactions.end(), std::back_inserter(forces), [&](const auto& i) {
        return interaction_social_forces(i, params);
    });

    assign_ids_to_passing_objects(forces);

    return forces;
}


std::tuple<double, double> forces_from_agent(const topo_agent_t& agent,
                                             const social_forces_state_t& state,
                                             const social_forces_params_t& params)
{
    Vector agentDir(2);
    agentDir[0] = agent.state.x - state.position.x;
    agentDir[1] = agent.state.y - state.position.y;

    double distToAgent = distance_between_points(agent.state.x,
                                                       agent.state.y,
                                                       state.position.x,
                                                       state.position.y);
    distToAgent -= agent.radius + params.robotRadius;
    distToAgent = std::max(0.0, distToAgent);

    double cosPhi = -arma::as_scalar(arma::trans(agentDir) * state.heading);
    double anisotropicScale = params.lambda_p + ((1.0 - params.lambda_p) * (0.5 * (1.0 + cosPhi)));
    double socialForce = params.a_p * std::exp(-distToAgent / params.b_p) * anisotropicScale;

    Point<double> futurePosition(state.position.x + (state.velocity[0] * params.tau),
                                       state.position.y + (state.velocity[1] * params.tau));
    Line<double> robotTraj(state.position, futurePosition);
    Point<double> agentPosition(agent.state.x, agent.state.y);
    Point<double> projectedAgentPosition = closest_point_on_line_segment(agentPosition, robotTraj);
    distToAgent = distance_between_points(agentPosition, projectedAgentPosition);

    double blame = 1.0 / (1.0 + std::exp(distToAgent));

    return std::make_tuple(blame, socialForce);
}


boost::optional<passing_object_t> is_passing_object(const topo_agent_t& agent,
                                                    const math::Rectangle<float>& robotBoundary,
                                                    const double speed,
                                                    const double heading)
{
    // If not moving, not actually passing anything!
    if(speed < 0.25)
    {
        return boost::none;
    }

    auto agentPosition = Point<float>(agent.state.x, agent.state.y);
    auto nearestPoint = robotBoundary.closestPointOnBoundary(agentPosition);
    double distToAgent = distance_between_points(agentPosition, nearestPoint.first) - agent.radius;

    double agentSpeed = std::sqrt((agent.state.xVel * agent.state.xVel) + (agent.state.yVel * agent.state.yVel));
    double agentHeading = std::atan2(agent.state.yVel, agent.state.xVel);
    bool isOncoming = angle_diff_abs(agentHeading, heading) > M_PI_2;
    double relativeSpeed = isOncoming ? speed + agentSpeed : speed - agentSpeed;

    // If inside the robot, it must be passing VERY close!
    if((distToAgent < 0.0) || robotBoundary.contains(agentPosition))
    {
#ifdef DEBUG_PASSING
        std::cout << "Hitting something: " << robotBoundary << " agent: " << agentPosition << " radius: "
            << agent.radius << " dist: " << distToAgent << '\n';
#endif // DEBUG_PASSING
        return passing_object_t(agentPosition, 0.0, relativeSpeed, math::RectSide::inside);
    }

    // If the closest point is an endpoint, then not moving along one of the sides
    if((nearestPoint.first == robotBoundary.bottomLeft) || (nearestPoint.first == robotBoundary.bottomRight)
        || (nearestPoint.first == robotBoundary.topRight) || (nearestPoint.first == robotBoundary.topLeft))
    {
        return boost::none;
    }

    if((nearestPoint.second == math::RectSide::left) || (nearestPoint.second == math::RectSide::right))
    {
#ifdef DEBUG_PASSING
        std::cout << "Passing something: " << robotBoundary << " agent: " << agentPosition << " radius: "
            << agent.radius << " dist: " << distToAgent << '\n';
#endif // DEBUG_PASSING

        // Store which side of the agent we passed on using the RectSide. If moving in same direction, then
        // the pass is opposite the robot side. If opposite directions, the agent left is our left too.
        // If we moving the same direction, but our relative speed is higher, then the agent side we pass on is
        // the opposite side of the closest point to the robot boundary
        if(!isOncoming && (relativeSpeed > 0))
        {
            nearestPoint.second = (nearestPoint.second == math::RectSide::left) ? math::RectSide::right
                : math::RectSide::left;
        }

        return passing_object_t(agentPosition, distToAgent, relativeSpeed, nearestPoint.second);
    }

    return boost::none;
}


void assign_ids_to_passing_objects(std::vector<social_forces_t>& forces)
{
    int nextId = 1;

    // Go through each set of forces. If consecutive forces have passing objects, then they need the same id.
    for(std::size_t n = 2; n < forces.size(); ++n)
    {
        // If last was empty and now we have some passing, then we've moved on to the next id
        if(forces[n - 2].passingObj.empty() && forces[n - 1].passingObj.empty() && !forces[n].passingObj.empty())
        {
            ++nextId;
        }

        for(auto& obj : forces[n].passingObj)
        {
            obj.id = nextId;
        }
    }

    std::cout << "INFO: social_forces: Found " << nextId << " passing events.\n";
}

} // namespace mpepc
} // namespace vulcan
