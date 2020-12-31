/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     target_set.cpp
* \author   Collin Johnson and Jong Jin Park
* 
* Implementation of MetricTargetSet.
*/

#include "mpepc/metric_planner/script/target_set.h"
#include <algorithm>
#include <istream>
#include <iostream>

namespace vulcan
{

namespace mpepc
{
    
std::istream& operator>>(std::istream& in,  named_pose_t&       target);
std::ostream& operator<<(std::ostream& out, const named_pose_t& target);

std::string replace_character(std::string str, char from, char to);


MetricTargetSet::MetricTargetSet(std::istream& in)
{
    named_pose_t target;
    
    while(in.good())
    {
        in >> target;
        addTarget(target);
    }
}


MetricTargetSet::MetricTargetSet(const std::vector<named_pose_t>& targets)
{
    for(auto& target : targets)
    {
        addTarget(target);
    }
}


void MetricTargetSet::addTarget(const named_pose_t& target)
{
    auto sameIt = std::find(targets.begin(), targets.end(), target);
    
    if(sameIt != targets.end())
    {
        *sameIt = target;
    }
    else
    {
        targets.push_back(target);
    }
}


bool MetricTargetSet::removeTarget(const named_pose_t& target)
{
    auto sameIt = std::find(targets.begin(), targets.end(), target);
    
    if(sameIt != targets.end())
    {
        targets.erase(sameIt);
        return true;
    }
    
    return false;
}


bool MetricTargetSet::saveToFile(std::ostream& out) const
{
    if(out.good())
    {
        for(auto& target : targets)
        {
            out << target;
        }
    }
    
    return out.good();
}


std::istream& operator>>(std::istream& in, named_pose_t& target)
{
    in >> target.name >> target.pose.x >> target.pose.y >> target.pose.theta;
    
    target.name = replace_character(target.name, '_', ' ');
    
    return in;
}


std::ostream& operator<<(std::ostream& out, const named_pose_t& target)
{
    out << replace_character(target.name, ' ', '_') << ' '
        << target.pose.x << ' '
        << target.pose.y << ' '
        << target.pose.theta <<'\n';
    return out;
}


std::string replace_character(std::string str, char from, char to)
{
    for(auto& c : str)
    {
        if(c == from)
        {
            c = to;
        }
    }
    
    return str;
}
    
} // namespace mpepc
} // namespace vulcan
