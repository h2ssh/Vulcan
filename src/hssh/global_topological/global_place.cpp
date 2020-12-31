/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_place.cpp
* \author   Collin Johnson
*
* Definition of GlobalPlace.
*/

#include "hssh/global_topological/global_place.h"
#include <algorithm>

namespace vulcan
{
namespace hssh
{

GlobalPlace::GlobalPlace(Id id,
                         AreaType type,
                         Id metricId,
                         const GlobalTransitionCycle& cycle)
: id_(id)
, type_(type)
, cycle_(cycle)
{
    metricPlaceIds_.push_back(metricId);
}


bool GlobalPlace::replaceTransition(const GlobalTransition& oldTrans, const GlobalTransition& newTrans)
{
    return cycle_.replaceTransition(oldTrans, newTrans);
}


bool GlobalPlace::changeMetricId(Id oldId, Id newId)
{
    auto idIt = std::find(metricPlaceIds_.begin(), metricPlaceIds_.end(), oldId);

    if(idIt != metricPlaceIds_.end())
    {
        *idIt = newId;
    }

    return idIt != metricPlaceIds_.end();
}

} // namespace hssh
} // namespace vulcan
