/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     floor.cpp
* \author   Collin Johnson
*
* Definition of Floor.
*/

#include <hssh/local_metric/multifloor/floor.h>
#include <hssh/local_metric/multifloor/elevator.h>
#include <sstream>

namespace vulcan
{
namespace hssh
{

Floor::Floor(int32_t id, const std::string& lpmFile)
    : id(id)
    , lpmFile(lpmFile)
{
    if(lpmFile.find(".lpm") == std::string::npos)
    {
        std::ostringstream str;
        str << lpmFile << id << ".lpm";
        this->lpmFile = str.str();
    }
}


Floor::Floor(int32_t id, const LocalPerceptualMap& lpm, const std::string& lpmFile)
    : id(id)
    , lpm(lpm)
    , lpmFile(lpmFile)
{
    if(lpmFile.find(".lpm") == std::string::npos)
    {
        std::ostringstream str;
        str << lpmFile << id << ".lpm";
        this->lpmFile = str.str();
    }
}


Floor::~Floor(void)
{
    // TODO: Write the LPM to a file?
}


std::shared_ptr<Elevator> Floor::isOnKnownElevator(const pose_t& pose) const
{
    for(const auto& elevator : elevators)
    {
        if(elevator->isRobotOnElevator(pose, id))
        {
            return elevator;
        }
    }

    return std::shared_ptr<Elevator>();
}


void Floor::addElevator(const std::shared_ptr<Elevator>& elevator)
{
    elevators.push_back(elevator);
}


void Floor::updateLPM(const LocalPerceptualMap& lpm)
{
    this->lpm = lpm;
}

}
}
