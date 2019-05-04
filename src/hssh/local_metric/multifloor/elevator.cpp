/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     elevator.cpp
* \author   Collin Johnson
*
* Definition of Elevator.
*/

#include <hssh/local_metric/multifloor/elevator.h>
#include <hssh/local_metric/multifloor/floor.h>
#include <math/geometry/shape_fitting.h>
#include <robot/elevator.h>
#include <iostream>

#define DEBUG_NEW_FLOOR

namespace vulcan
{
namespace hssh
{

Elevator::Elevator(int                                                   id,
                   int                                                   currentFloor,
                   const std::map<int, math::Rectangle<float>>&          boundaries,
                   const std::map<int, std::vector<floor_transition_t>>& transitions)
    : id(id)
    , boundaries(boundaries)
    , transitions(transitions)
    , currentFloor(currentFloor)
{
}


Elevator::Elevator(int id, const pose_t& pose, int floorId, const LocalPerceptualMap& lpm)
    : id(id)
    , currentFloor(floorId)
{
    calculateBoundary(pose, floorId, lpm);

    transitions.insert(std::make_pair(floorId, std::vector<floor_transition_t>()));
}


math::Rectangle<float> Elevator::getBoundary(int floorId) const
{
    auto boundaryIt = boundaries.find(floorId);

    if(boundaryIt != boundaries.end())
    {
        return boundaryIt->second;
    }

    return math::Rectangle<float>();
}


bool Elevator::isRobotOnElevator(const pose_t& pose, int floorId) const
{
    auto boundaryIt = boundaries.find(floorId);

    if(boundaryIt != boundaries.end())
    {
        return boundaryIt->second.contains(pose.toPoint());
    }

    return false;
}


void Elevator::addFloor(const robot::elevator_t& elevator, const pose_t& pose, int floorId, const LocalPerceptualMap& lpm)
{
    calculateBoundary(pose, floorId, lpm);
    addFloorTransition(currentFloor, floorId, elevator.distance);
    inferAdditionalTransitions(floorId, elevator.distance);
}


int Elevator::findNewFloor(const robot::elevator_t& elevator, double minDistanceBetweenFloors) const
{
    // Go through all known floor transitions. If a transition has been made from the current floor,
    // then see if the elevator moved about the same distance. If so, then we're on that floor again.

    auto transIt = transitions.find(currentFloor);

    if(transIt == transitions.end())
    {
        return -1;
    }

    for(const auto& transition : transIt->second)
    {
        bool doFloorsMatch = std::abs(transition.height - elevator.distance) < minDistanceBetweenFloors;

#ifdef DEBUG_NEW_FLOOR
        std::cout<<"DEBUG:Elevator:Transition:"<<transition.height<<" Measured:"<<elevator.distance<<" Match? "<<doFloorsMatch<<'\n';
#endif

        if(doFloorsMatch)
        {
            return transition.endFloor;
        }
    }

    return -1;
}


void Elevator::calculateBoundary(const pose_t& pose, int floorId, const LocalPerceptualMap& lpm)
{
    std::vector<Point<float>> endpoints;

    Point<int> start = utils::global_point_to_grid_cell(pose.toPoint(), lpm);

    for(double angle = 0; angle < 2.0*M_PI; angle += M_PI*5.0/180.0)
    {
        float deltaX = cos(angle);
        float deltaY = sin(angle);

        float xPosition = start.x;
        float yPosition = start.y;

        Point<int> rayCell(start.x, start.y);

        while(lpm.isCellInGrid(rayCell) && (lpm.getCellTypeNoCheck(rayCell) & kFreeOccGridCell))
        {
            xPosition += deltaX;
            yPosition += deltaY;

            rayCell.x = xPosition;
            rayCell.y = yPosition;
        }

        endpoints.push_back(utils::grid_point_to_global_point(rayCell, lpm));
    }

    boundaries.insert(std::make_pair(floorId, math::axis_aligned_bounding_rectangle<float>(endpoints.begin(),
                                                                                           endpoints.end())));

#ifdef DEBUG_NEW_FLOOR
    std::cout<<"Boundary for Floor "<<floorId<<':'<<boundaries[floorId]<<" in Elevator"<<id<<'\n';
#endif
}


void Elevator::addFloorTransition(int floorA, int floorB, double height)
{
    transitions[floorA].push_back({floorA, floorB, height});
    
    auto transIt = transitions.find(floorB);
    if(transIt != transitions.end())
    {
        transIt->second.push_back({floorB, floorA, -height});
    }
    else
    {
        decltype(transIt->second) newFloorTransitions;
        newFloorTransitions.push_back({floorB, floorA, -height});
        transitions.insert(std::make_pair(floorB, newFloorTransitions));
    }
}


void Elevator::inferAdditionalTransitions(int floorId, double height)
{
    // All floors on the elevator MUST be connected, so whenever visiting a new floor, establish
    // any necessary connections
    
    auto transIt = transitions.find(currentFloor);
    
    for(const auto& transition : transIt->second)
    {
        if(!doesTransitionExist(transition.endFloor, floorId))
        {
            addFloorTransition(transition.endFloor, floorId, height-transition.height);
        }
    }
}


bool Elevator::doesTransitionExist(int floorA, int floorB) const
{
    // A floor transition to itself exists by definition and doesn't need to be added to the map
    if(floorA == floorB)
    {
        return true;
    }
    
    auto transIt = transitions.find(floorA);
    
    for(const auto& transition : transIt->second)
    {
        if(transition.endFloor == floorB)
        {
            return true;
        }
    }
    
    return false;
}

} // namespace hssh
} // namespace vulcan
