/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     chi.cpp
 * \author   Collin Johnson
 *
 * Definition of Chi.
 */

#include "hssh/global_topological/chi.h"
#include "hssh/global_topological/global_place.h"
#include "hssh/global_topological/topological_map.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

Chi::Chi(const TopologicalMap& map)
{
    for (auto& p : map.places()) {
        poses_.insert(std::make_pair(p.first, map.referenceFrame(p.second->id())));
    }
}


Chi::Chi(const std::unordered_map<Id, pose_distribution_t>& poses, double logLikelihood)
: poses_(poses)
, logLikelihood_(logLikelihood)
{
}


void Chi::setPlacePose(Id placeId, const pose_distribution_t& pose)
{
    poses_[placeId] = pose;
}


void Chi::setLogLikelihood(double logLikelihood)
{
    assert(logLikelihood <= 0.0);
    logLikelihood_ = logLikelihood;
}


pose_distribution_t Chi::getPlacePose(Id placeId) const
{
    auto poseIt = poses_.find(placeId);

    if (poseIt != poses_.end()) {
        return poseIt->second;
    }

    return pose_distribution_t();
}


Lambda Chi::getLambda(Id fromPlace, Id toPlace) const
{
    if ((poses_.find(fromPlace) != poses_.end()) && (poses_.find(toPlace) != poses_.end())) {
        return Lambda(poses_.find(toPlace)->second, poses_.find(fromPlace)->second);
    }

    return Lambda();
}

}   // namespace hssh
}   // namespace vulcan
