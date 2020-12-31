/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     path.cpp
* \author   Collin Johnson
*
* Definition of GlobalPath.
*/

#include "hssh/global_topological/global_path.h"
#include "utils/stub.h"
#include <iostream>
#include <cassert>

#define DEBUG_ADD_PLACE

namespace vulcan
{
namespace hssh
{

// Helper functions
GlobalPath::GlobalPath(Id id)
: id_(id)
{
}


GlobalPath::GlobalPath(Id id, const std::vector<GlobalPathSegment>& segments)
: id_(id)
, segments_(segments)
{
    PRINT_PRETTY_STUB();
    std::cout << "INFO: Need to implement construction of GlobalPath from path segments_.\n";
}


std::ostream& operator<<(std::ostream& out, const GlobalPath& path)
{
    out << "Path " << path.id_ << ':';
    for(Id id : path.places())
    {
        out << id << ' ';
    }

    return out;
}


std::vector<Id> GlobalPath::places(void) const
{
    std::vector<Id> places;

    for(size_t n = 0; n < segments_.size(); ++n)
    {
        places.push_back(segments_[n].plusTransition().otherArea(segments_[n].toArea()).id());

        // For the final place, need to also add the minus side -- might be frontier or the end place on the path
        if(n == segments_.size()-1)
        {
            places.push_back(segments_[n].minusTransition().otherArea(segments_[n].toArea()).id());
        }
    }

    // This code is here to ensure an infinite loop does not get introduced into a path when loops are created in the path segments
    int lastPlace  = -1;
    int placeCount = 0;
    for(size_t n = 0; n < places.size(); ++n)
    {
        if(places[n] == lastPlace)
        {
            ++placeCount;
        }
        else
        {
            placeCount = 0;
        }

        lastPlace = places[n];

        assert(placeCount < 3);
    }

    return places;
}


int GlobalPath::getSegmentIndex(const GlobalPathSegment& segment) const
{
    for(size_t n = 0; n < segments_.size(); ++n)
    {
        if(segments_[n].id() == segment.id())
        {
            return n;
        }
    }

    return -1;
}


bool GlobalPath::hasSegmentWithId(Id id) const
{
    return getSegmentWithId(id).id() == id;
}


GlobalPathSegment GlobalPath::getSegmentWithId(Id id) const
{
    for(auto& s : segments_)
    {
        if(s.id() == id)
        {
            return s;
        }
    }

    return GlobalPathSegment();
}

} // namespace hssh
} // namespace vulcan
