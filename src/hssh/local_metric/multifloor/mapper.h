/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     mapper.h
* \author   Collin Johnson
*
* Declaration of MultiFloorMapper.
*/

#ifndef HSSH_LOCAL_METRIC_MULTIFLOOR_MAPPER_H
#define HSSH_LOCAL_METRIC_MULTIFLOOR_MAPPER_H

#include <hssh/local_metric/multifloor/params.h>
#include <robot/elevator.h>
#include <core/pose.h>
#include <map>
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

class MultiFloorMap;
class Mapper;
class Localizer;
class MetricRelocalizer;
class Floor;
class Elevator;

struct multi_floor_input_t
{
    robot::elevator_t elevator;
    pose_t     pose;

    Mapper&            mapper;
    MetricRelocalizer& relocalizer;
};

struct multi_floor_output_t
{

};

/**
* MultiFloorMapper is a simple extension of the local metric mapping that processes elevator detection events to
* determine where the robot is in a building with multiple floors.
*/
class MultiFloorMapper
{
public:

    /**
    * Constructor for MultiFloorMapper.
    *
    * \param    params          Parameters for the mapper
    */
    MultiFloorMapper(const multi_floor_mapper_params_t& params);
    
    /**
    * setMap sets the current map in which the robot is moving. The map contains the current floor,
    * current elevator, and all discovered Floors and Elevators.
    */
    void setMap(const MultiFloorMap& map);
    
    /**
    * getMap retrieves the current multifloor map that has been constructed.
    */
    MultiFloorMap getMap(void) const;

    /**
    * processElevator processes a new robot::elevator_t input. If the robot has just started moving on an elevator,
    * the appropriate elevator will be set or created. If the elevator has stopped, the robot will be told to relocalize
    * on a previous floor if one has been saved. Otherwise, a new floor and LPM will be created for the robot.
    *
    * processElevator can have side-effects, as noted in the class description.
    *
    * \param[in]    input           Input for the mapper
    * \param[out]   output          Output generated mostly for debugging purposes
    * \return   True if any output has been generated, i.e. the elevator state has triggered some sort of mapping change.
    */
    bool processElevator(const multi_floor_input_t& input, multi_floor_output_t& output);

private:

    void handleEnteringElevator(const multi_floor_input_t& input);
    void handleExitingElevator (const multi_floor_input_t& input);

    std::shared_ptr<Floor>    createFloor   (const multi_floor_input_t& input);
    std::shared_ptr<Elevator> createElevator(const multi_floor_input_t& input);

    void relocalizeOnCurrentFloor(const multi_floor_input_t& input);
    
    void saveCurrentMap(void);

    bool amOnElevator;

    std::map<int, std::shared_ptr<Floor>>    floors;
    std::vector<std::shared_ptr<Elevator>> elevators;

    std::shared_ptr<Floor>    currentFloor;
    std::shared_ptr<Elevator> currentElevator;

    int nextFloorId;
    int nextElevatorId;

    multi_floor_mapper_params_t params;
};

}
}

#endif // HSSH_LOCAL_METRIC_MULTIFLOOR_MAPPER_H
