/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     floor.h
* \author   Collin Johnson
*
* Declaration of Floor.
*/

#ifndef HSSH_LOCAL_METRIC_MULTIFLOOR_FLOOR_H
#define HSSH_LOCAL_METRIC_MULTIFLOOR_FLOOR_H

#include "hssh/local_metric/lpm.h"
#include <memory>
#include <string>
#include <vector>
#include <cstdint>

namespace vulcan
{
struct pose_t;
namespace hssh
{

class Elevator;

/**
* Floor represents a floor of a building. A floor consists of an LPM, either saved in memory or on disk, an id,
* and a set of Elevators. The Elevators connect this Floor to other Floors.
*/
class Floor
{
public:

    /**
    * Constructor for Floor.
    *
    * \param    id              Id of the floor
    * \param    lpmFile         File in which to save/load the associated LPM
    */
    Floor(int32_t id, const std::string& lpmFile);

    /**
    * Constructor for Floor.
    *
    * \param    id              Id of the floor
    * \param    lpm             LPM for the floor
    * \param    lpmFile         File in which to save/load the associated LPM
    */
    Floor(int32_t id, const LocalPerceptualMap& lpm, const std::string& lpmFile);

    /**
    * Destructor for Floor.
    */
    ~Floor(void);

    /**
    * isOnKnownElevator checks to see if the robot has entered a known elevator.
    *
    * \param    pose            Pose of robot on the elevator
    * \return   The elevator the robot is on. Otherwise, a null elevator pointer.
    */
    std::shared_ptr<Elevator> isOnKnownElevator(const pose_t& pose) const;

    /**
    * addElevator adds a new Elevator to the Floor.
    *
    * \param    elevator        Elevator to be added
    */
    void addElevator(const std::shared_ptr<Elevator>& elevator);

    /**
    * updateLPM updates the LPM associated with the floor.
    *
    * \param    lpm             New LPM model for the floor
    */
    void updateLPM(const LocalPerceptualMap& lpm);

    // Accessors

    int32_t                   getId(void)      const { return id; }
    const LocalPerceptualMap& getLPM(void)     const { return lpm; }
    std::string               getLPMName(void) const { return lpmFile; }
    
    std::vector<std::shared_ptr<Elevator>> getElevators(void) const { return elevators; }

private:

    int32_t            id;
    LocalPerceptualMap lpm;
    std::string        lpmFile;

    std::vector<std::shared_ptr<Elevator>> elevators;
};

}
}

#endif // HSSH_LOCAL_METRIC_MULTIFLOOR_FLOOR_H
