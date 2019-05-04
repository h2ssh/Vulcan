/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal.cpp
* \author   Collin Johnson
* 
* Definition of Goal.
*/

#include <planner/interface/goal.h>

namespace vulcan
{
namespace planner
{
    
const std::string kGlobalMetricType("gmm");
const std::string kGlobalTopoType("gtm");
const std::string kLocalTopoType("ltm");

pose_t load_pose(std::istream& in);
hssh::LocalArea::Id load_local_area(std::istream& in);
// hssh::GlobalArea::Id load_global_area(std::istream& in);

Goal::Goal(void)
: name_("DEFAULT CONSTRUCTOR")
{
}

    
Goal::Goal(const std::string& name, const pose_t& pose)
: name_(name)
, globalPose_(pose)
{
}

    
Goal::Goal(const std::string& name, hssh::LocalArea::Id area)
: name_(name)
, localArea_(area)
{
}


std::istream& operator>>(std::istream& in, Goal& g)
{
    std::string goalLine;
    std::getline(in, goalLine);
    
    std::istringstream inStr(goalLine);
    inStr >> g.name_;
    
    for(auto& c : g.name_)
    {
        if(c == '_')
        {
            c = ' ';
        }
    }
    
    std::string typeDesc;
    while(true)
    {
        inStr >> typeDesc;
        
        if(inStr.eof())
        {
            break;
        }
        
        if(typeDesc == kGlobalMetricType)
        {
            g.setGlobalPose(load_pose(inStr));
        }
        else if(typeDesc == kGlobalTopoType)
        {
            // TODO
        }
        else if(typeDesc == kLocalTopoType)
        {
            g.setLocalArea(load_local_area(inStr));
        }
    }
    
    return in;
}


std::ostream& operator<<(std::ostream& out, const Goal& g)
{
    std::string goalName = g.name();
    for(auto& c : goalName)
    {
        if(isspace(c))
        {
            c = '_';
        }
    }
    
    out << goalName << ' ';
    
    if(g.globalPose())
    {
        out << kGlobalMetricType << ' ' << g.globalPose()->x << ' ' << g.globalPose()->y << ' ' 
            << g.globalPose()->theta << ' ';
    }
    
    if(g.localArea())
    {
        out << kLocalTopoType << ' ' << g.localArea().get() << ' ';
    }
    
    out << '\n';
    return out;
}


pose_t load_pose(std::istream& in)
{
    pose_t pose;
    in >> pose.x >> pose.y >> pose.theta;
    return pose;
}


hssh::LocalArea::Id load_local_area(std::istream& in)
{
    hssh::LocalArea::Id id;
    in >> id;
    return id;
}

} // namespace planner
} // namespace vulcan
