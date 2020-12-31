/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_interface.cpp
* \author   Collin Johnson
* 
* Definition of GoalInterface.
*/

#include "planner/interface/goal_interface.h"
#include <iostream>
#include <iterator>

namespace vulcan
{
namespace planner
{

void GoalInterface::addGlobalPose(const std::string& name, const pose_t& pose)
{
    auto goalIt = std::find_if(goals_.begin(), goals_.end(), [&name](const Goal& g) { return g.name() == name; });

    if(goalIt != goals_.end())
    {
        goalIt->setGlobalPose(pose);
    }
    else
    {
        goals_.emplace_back(name, pose);
    }
}


void GoalInterface::addLocalArea(const std::string& name, hssh::LocalArea::Id area)
{
    auto goalIt = std::find_if(goals_.begin(), goals_.end(), [&name](const Goal& g) { return g.name() == name; });

    if(goalIt != goals_.end())
    {
        goalIt->setLocalArea(area);
    }
    else
    {
        goals_.emplace_back(name, area);
    }
}


std::size_t GoalInterface::load(std::istream& in)
{
    // Load the Goals
    std::copy(std::istream_iterator<Goal>(in), std::istream_iterator<Goal>(), std::back_inserter(goals_));
    return goals_.size();
}


bool GoalInterface::save(std::ostream& out)
{
    std::copy(goals_.begin(), goals_.end(), std::ostream_iterator<Goal>(out, ""));
    return out.good();
}

}
}
