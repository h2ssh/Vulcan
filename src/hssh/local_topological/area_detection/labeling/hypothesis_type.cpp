/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_type.cpp
* \author   Collin Johnson
*
* Definition of HypothesisType.
*/

#include <hssh/local_topological/area_detection/labeling/hypothesis_type.h>
#include <iostream>
#include <string>

namespace vulcan
{
namespace hssh
{

const std::string kNoneName("none");
const std::string kPlaceName("place");
const std::string kDecisionName("decision");
const std::string kDestName("dest");
const std::string kPathEndName("path_end");
const std::string kPathDestName("path_dest");
const std::string kPathName("path");
const std::string kAreaName("area");



std::ostream& operator<<(std::ostream& out, HypothesisType type)
{
    switch(type)
    {
    case HypothesisType::kNone:
        out << kNoneName;
        break;
    case HypothesisType::kPlace:
        out << kPlaceName;
        break;
    case HypothesisType::kDecision:
        out << kDecisionName;
        break;
    case HypothesisType::kDest:
        out << kDestName;
        break;
    case HypothesisType::kPathEndpoint:
        out << kPathEndName;
        break;
    case HypothesisType::kPathDest:
        out << kPathDestName;
        break;
    case HypothesisType::kPath:
        out << kPathName;
        break;
    case HypothesisType::kArea:
        out << kAreaName;
        break;
    }

    return out;
}


std::istream& operator>>(std::istream& in, HypothesisType& type)
{
    std::string name;
    in >> name;
    if(name == kPlaceName)
    {
        type = HypothesisType::kPlace;
    }
    else if(name == kDecisionName)
    {
        type = HypothesisType::kDecision;
    }
    else if(name == kDestName)
    {
        type = HypothesisType::kDest;
    }
    else if(name == kPathEndName)
    {
        type = HypothesisType::kPathEndpoint;
    }
    else if(name == kPathDestName)
    {
        type = HypothesisType::kPathDest;
    }
    else if(name == kPathName)
    {
        type = HypothesisType::kPath;
    }
    else if(name == kAreaName)
    {
        type = HypothesisType::kArea;
    }
    else //if(name == kNoneName)
    {
        type = HypothesisType::kNone;
    }

    return in;
}

} // namespace hssh
} // namespace vulcan
