/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     nearest_neighbor_association.cpp
 * \author   Collin Johnson
 *
 * Definition of NearestNeighborAssociation and nearest_neighbor_params_t.
 */

#include "tracker/tracking/nearest_neighbor_association.h"
#include "tracker/laser_object.h"
#include "utils/config_file.h"

// #define DEBUG_ASSOCIATION

namespace vulcan
{
namespace tracker
{

//////////////// nearest_neighbor_params_t //////////////////////////////

const std::string kNearestNeighborHeading("NearestNeighborAssociationParameters");
const std::string kMaxDistanceKey("max_boundary_distance_m");
const std::string kUsePredictedKey("use_predicted_boundary");


nearest_neighbor_params_t::nearest_neighbor_params_t(const utils::ConfigFile& config)
: maxBoundaryDistance(config.getValueAsDouble(kNearestNeighborHeading, kMaxDistanceKey))
, usePredictedBoundary(config.getValueAsBool(kNearestNeighborHeading, kUsePredictedKey))
{
    assert(maxBoundaryDistance > 0.0);
}

//////////////// NearestNeighborAssociation //////////////////////////////

NearestNeighborAssociation::NearestNeighborAssociation(const nearest_neighbor_params_t& params) : params_(params)
{
}


object_association_t NearestNeighborAssociation::associateLaserWithTracked(const LaserObject& laser,
                                                                           const TrackingObjectCollection& objects)
{
    // Find the object that is closest
    std::size_t minIndex = 0;
    double minMinDist = 1000000.0;
    double minAvgDist = 1000000.0;

    for (std::size_t n = 0; n < objects.size(); ++n) {
        float dist = 0.0f;
        float avgDist = 0.0f;

        std::tie(dist, avgDist) = objects[n]->distanceTo(laser);
        if ((dist < minMinDist) || ((dist == minMinDist) && (avgDist < minAvgDist))) {
            minMinDist = dist;
            minAvgDist = avgDist;
            minIndex = n;
        }
    }

#ifdef DEBUG_ASSOCIATION
    std::cout << "DEBUG:TrackingObjectSet: Min dist:" << minMinDist << '\n';
#endif

    if ((minMinDist < params_.maxBoundaryDistance)
        || objects[minIndex]->boundary().circleApproximation().contains(laser.center())
        || laser.circleApproximation().contains(objects[minIndex]->boundary().position())) {
        return object_association_t(minIndex, minMinDist);
    } else {
        return object_association_t(-1);
    }
}


object_association_t NearestNeighborAssociation::associateObjectWithTracked(const TrackingObject& object,
                                                                            const TrackingObjectCollection& objects)
{
    // Return first object found whose boundary is valid -- not necessarily the best choice, but works for now
    auto objIt =
      std::find_if(objects.begin(), objects.end(), [&object, this](const std::shared_ptr<TrackingObject>& o) {
          return std::get<1>(object.distanceTo(*o)) < params_.maxBoundaryDistance;
      });

    if (objIt != objects.end()) {
        return object_association_t(std::distance(objects.begin(), objIt), std::get<1>(object.distanceTo(**objIt)));
    } else {
        return object_association_t(-1);
    }
}


std::unique_ptr<DataAssociationStrategy> NearestNeighborAssociation::clone(void)
{
    // A cloned copy only needs to contain the same parameters to be functionally equivalent
    return std::unique_ptr<DataAssociationStrategy>(new NearestNeighborAssociation(params_));
}

}   // namespace tracker
}   // namespace vulcan
