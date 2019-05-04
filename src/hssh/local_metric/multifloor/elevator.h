/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     elevator.h
* \author   Collin Johnson
*
* Declaration of Elevator.
*/

#ifndef HSSH_LOCAL_METRIC_MULTIFLOOR_ELEVATOR_H
#define HSSH_LOCAL_METRIC_MULTIFLOOR_ELEVATOR_H

#include <math/geometry/rectangle.h>
#include <map>
#include <memory>
#include <cstdint>

namespace vulcan
{
struct pose_t;
namespace robot { struct elevator_t; }
namespace hssh
{

class Floor;
class LocalPerceptualMap;

struct floor_transition_t
{
    int    startFloor;
    int    endFloor;
    double height;
};

/**
* Elevator
*/
class Elevator
{
public:
    
    /**
    * Constructor for Elevator.
    * 
    * Builds an Elevator from an existing model.
    * 
    * \param    id              Id of the elevator
    * \param    currentFloor    Id of the current floor
    * \param    boundaries      Boundaries of the elevator on each floor
    * \param    transitions     Known floor transitions
    */
    Elevator(int                                                   id,
             int                                                   currentFloor,
             const std::map<int, math::Rectangle<float>>&          boundaries,
             const std::map<int, std::vector<floor_transition_t>>& transitions);
    

    /**
    * Constructor for Elevator.
    *
    * \param    id              Id of the elevator
    * \param    pose            Pose of robot on the floor
    * \param    floorId         Floor on which the elevator will be located
    * \param    lpm             Current LPM around the robot
    */
    Elevator(int id, const pose_t& pose, int floorId, const LocalPerceptualMap& lpm);

    int  getId(void) const { return id; }
    void setCurrentFloor(int floorId) { currentFloor = floorId; }
    int  getCurrentFloor(void) const  { return currentFloor; }
    
    math::Rectangle<float>                         getBoundary(int floorId) const;
    std::map<int, math::Rectangle<float>>          getBoundaries(void)  const { return boundaries; }
    std::map<int, std::vector<floor_transition_t>> getTransitions(void) const { return transitions; }

    void addFloor(const robot::elevator_t& elevator, const pose_t& pose, int floorId, const LocalPerceptualMap& lpm);
    bool isRobotOnElevator(const pose_t& pose, int floorId) const;
    int  findNewFloor(const robot::elevator_t& elevator, double minDistanceBetweenFloors) const;

private:
    
    void calculateBoundary(const pose_t& pose, int floorId, const LocalPerceptualMap& lpm);
    
    void addFloorTransition        (int floorA, int floorB, double height);
    void inferAdditionalTransitions(int floorId, double height);
    bool doesTransitionExist       (int floorA, int floorB) const;

    int id;

    std::map<int, math::Rectangle<float>>          boundaries;
    std::map<int, std::vector<floor_transition_t>> transitions;

    int currentFloor;
};

}
}

#endif // HSSH_LOCAL_METRIC_MULTIFLOOR_ELEVATOR_H
