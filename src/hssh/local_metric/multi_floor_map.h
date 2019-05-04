/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     multi_floor_map.h
* \author   Collin Johnson
*
* Declaration of MultiFloorMap.
*/

#ifndef HSSH_LOCAL_METRIC_MULTI_FLOOR_MAP_H
#define HSSH_LOCAL_METRIC_MULTI_FLOOR_MAP_H

#include <hssh/local_metric/multifloor/elevator.h>
#include <hssh/local_metric/multifloor/floor.h>
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{
    
const int NOT_ON_ELEVATOR_ID = -1;

/**
* MultiFloorMap
*/
class MultiFloorMap
{
public:

    /**
    * Default constructor for MultiFloorMap.
    */
    MultiFloorMap(void)
        : currentFloor(-1)
        , currentElevator(NOT_ON_ELEVATOR_ID)
    { }

    /**
    * Constructor for MultiFloorMap.
    *
    * \param    currentFloor        Id of the current floor
    * \param    currentElevator     Id of the current elevator (-1 if none)
    * \param    floors              Floors in the map
    * \param    elevators           Elevators in the map
    */
    MultiFloorMap(int                                           currentFloor,
                  int                                           currentElevator,
                  const std::vector<std::shared_ptr<Floor>>&    floors,
                  const std::vector<std::shared_ptr<Elevator>>& elevators)
        : currentFloor(currentFloor)
        , currentElevator(currentElevator)
        , floors(floors)
        , elevators(elevators)
    {
    }
    
    /**
     * Constructor for MultiFloorMap.
     *
     * \param    currentFloor        Id of the current floor
     * \param    currentElevator     Id of the current elevator (-1 if none)
     * \param    floors              Floors in the map
     * \param    elevators           Elevators in the map
     */
    MultiFloorMap(int                                           currentFloor,
                  int                                           currentElevator,
                  const std::map<int, std::shared_ptr<Floor>>&  floors,
                  const std::vector<std::shared_ptr<Elevator>>& elevators)
        : currentFloor(currentFloor)
        , currentElevator(currentElevator)
        , elevators(elevators)
    {
        for(auto& floor : floors)
        {
            this->floors.push_back(floor.second);
        }
    }

    /**
    * getCurrentFloor retrieves the id of the current floor.
    */
    int getCurrentFloor(void) const { return currentFloor; }

    /**
    * getCurrentElevator retrieves the id of the current elevator.
    */
    int getCurrentElevator(void) const { return currentElevator; }

    /**
    * getFloors retrieves all the floors.
    */
    const std::vector<std::shared_ptr<Floor>>& getFloors(void) const { return floors; }

    /**
    * getElevators retrieves all the elevators.
    */
    const std::vector<std::shared_ptr<Elevator>>& getElevators(void) const { return elevators; }

private:

    int                                    currentFloor;
    int                                    currentElevator;
    std::vector<std::shared_ptr<Floor>>    floors;
    std::vector<std::shared_ptr<Elevator>> elevators;
};

}
}

#endif // HSSH_LOCAL_METRIC_MULTI_FLOOR_MAP_H
