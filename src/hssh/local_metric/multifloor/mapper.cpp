/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     mapper.cpp
* \author   Collin Johnson
*
* Definition of MultiFloorMapper.
*/

#include "hssh/local_metric/multifloor/mapper.h"
#include "hssh/local_metric/multifloor/floor.h"
#include "hssh/local_metric/multifloor/elevator.h"
#include "hssh/metrical/mapping/mapper.h"
#include "hssh/metrical/relocalization/metric_relocalizer.h"
#include "hssh/metrical/relocalization/filter_initializer_impl.h"
#include "hssh/local_metric/multi_floor_map.h"
#include "hssh/local_metric/multifloor/multi_floor_map_io.h"
#include "hssh/metrical/data.h"
#include <iostream>

#define DEBUG

namespace vulcan
{
namespace hssh
{

MultiFloorMapper::MultiFloorMapper(const multi_floor_mapper_params_t& params)
    : amOnElevator(false)
    , nextFloorId(0)
    , nextElevatorId(0)
    , params(params)
{
    MultiFloorMap loaded = MultiFloorMapIO::load("multifloor.map");

    if(!loaded.getFloors().empty())
    {
        for(auto floor : loaded.getFloors())
        {
            floors.insert(std::make_pair(floor->getId(), floor));

            if(floor->getId() == loaded.getCurrentFloor())
            {
                currentFloor = floor;
            }
        }

        elevators = loaded.getElevators();

        for(auto elevator : elevators)
        {
            if(elevator->getId() == loaded.getCurrentElevator())
            {
                currentElevator = elevator;
                break;
            }
        }
    }
}


void MultiFloorMapper::setMap(const MultiFloorMap& map)
{
    currentFloor.reset();
    currentElevator.reset();
    floors.clear();
    elevators.clear();
    
    for(const auto& floor : map.getFloors())
    {
        floors.insert(std::make_pair(floor->getId(), floor));
        
        if(floor->getId() == map.getCurrentFloor())
        {
            currentFloor = floor;
        }
    }
    
    elevators = map.getElevators();
    
    for(const auto& elevator : elevators)
    {
        if(elevator->getId() == map.getCurrentElevator())
        {
            currentElevator = elevator;
            break;
        }
    }
}


MultiFloorMap MultiFloorMapper::getMap(void) const
{
    if(currentElevator)
    {
        return MultiFloorMap(currentFloor->getId(), currentElevator->getId(), floors, elevators);
    }
    else
    {
        return MultiFloorMap(currentFloor->getId(), NOT_ON_ELEVATOR_ID, floors, elevators);
    }
}


bool MultiFloorMapper::processElevator(const multi_floor_input_t& input, multi_floor_output_t& output)
{
    if(!amOnElevator && (input.elevator.state != robot::ELEVATOR_STOPPED))
    {
        handleEnteringElevator(input);
        saveCurrentMap();
    }
    else if(amOnElevator && input.elevator.state == robot::ELEVATOR_STOPPED)
    {
        handleExitingElevator(input);
        saveCurrentMap();
    }

    return false;
}


void MultiFloorMapper::handleEnteringElevator(const multi_floor_input_t& input)
{
    amOnElevator = true;

    // In the case where this is the first encountered elevator, create a floor for that elevator
    if(!currentFloor)
    {
        currentFloor = createFloor(input);
    }
    else // if coming back on the elevator, update the LPM with what was found on the previous trip
    {
        currentFloor->updateLPM(input.mapper.getLPM());
    }

    currentElevator = currentFloor->isOnKnownElevator(input.pose);

    // If there is no currentElevator, then this is the first trip on it from the current floor
    if(!currentElevator)
    {
        currentElevator = createElevator(input);
        currentFloor->addElevator(currentElevator);
    }

#ifdef DEBUG
    std::cout<<"DEBUG:MultiFloorMapper: Entered! Floor:"<<currentFloor->getId()<<" Elevator:"<<currentElevator->getId()
             <<" Boundary:"<<currentElevator->getBoundary(currentFloor->getId())<<'\n';
#endif
}


void MultiFloorMapper::handleExitingElevator(const multi_floor_input_t& input)
{
    // When the elevator stops, reset map so something new will be created on the next update

    int newFloorId = currentElevator->findNewFloor(input.elevator, params.minDistanceBetweenFloors);

    if(newFloorId != -1)
    {
        currentFloor = floors[newFloorId];
        relocalizeOnCurrentFloor(input);
    }
    else
    {
        currentElevator->addFloor(input.elevator, input.pose, nextFloorId, input.mapper.getLPM());

        input.mapper.resetMap(input.pose);
        currentFloor = createFloor(input);
        currentFloor->addElevator(currentElevator);
    }

    currentElevator->setCurrentFloor(currentFloor->getId());

    amOnElevator = false;

#ifdef DEBUG
    std::cout<<"DEBUG:MultiFloorMapper: Exited! New floor:"<<newFloorId<<'\n';
#endif
}


std::shared_ptr<Floor> MultiFloorMapper::createFloor(const multi_floor_input_t& input)
{
    // Create the floor, push it on to the collection of floors
    std::shared_ptr<Floor> newFloor(new Floor(nextFloorId++, input.mapper.getLPM(), params.floorFilename));

    floors.insert(std::make_pair(newFloor->getId(), newFloor));

    return newFloor;
}


std::shared_ptr<Elevator> MultiFloorMapper::createElevator(const multi_floor_input_t& input)
{
    std::shared_ptr<Elevator> newElevator(new Elevator(nextElevatorId++, input.pose, currentFloor->getId(), input.mapper.getLPM()));

    elevators.push_back(newElevator);

    return newElevator;
}


void MultiFloorMapper::relocalizeOnCurrentFloor(const multi_floor_input_t& input)
{
//     relocalization_request_message_t message;
//     message.timestamp          = input.pose.timestamp;
//     message.map                = currentFloor->getLPM();
//     message.initializationMode = INITIALIZE_REGION;
//     message.mapAction          = ACTION_SWITCH_MAPS;
// 
//     message.initialRegion = currentElevator->getBoundary(currentFloor->getId());
// 
//     input.relocalizer.processRequest(message);
    
    std::shared_ptr<OccupancyGrid> map(new LocalPerceptualMap(currentFloor->getLPM()));
    metric_slam_data_t data;
    RegionFilterInitializer initializer(currentElevator->getBoundary(currentFloor->getId()), 3000, 10);
    input.relocalizer.startRelocalization(data, *map, initializer);
}


void MultiFloorMapper::saveCurrentMap(void)
{    
    MultiFloorMapIO::save(MultiFloorMap(currentFloor->getId(), currentElevator->getId(), floors, elevators), "multifloor.map");
}

} // namespace hssh
} // namespace vulcan
