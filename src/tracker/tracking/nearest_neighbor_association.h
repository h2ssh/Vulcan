/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     nearest_neighbor_association.h
 * \author   Collin Johnson
 *
 * Declaration of NearestNeighborAssociation implementation of DataAssociationStrategy interface and an accompanying
 * params struct, nearest_neighbor_params_t.
 */

#ifndef TRACKER_TRACKING_NEAREST_NEIGHBOR_ASSOCIATION_H
#define TRACKER_TRACKING_NEAREST_NEIGHBOR_ASSOCIATION_H

#include "tracker/tracking/data_association.h"

namespace vulcan
{
namespace utils
{
class ConfigFile;
}
namespace tracker
{

const std::string kNearestNeighborAssociationType("nearest-neighbor");

/**
 * nearest_neighbor_params_t defines parameters that control the behavior of the nearest-neighbors data association. The
 * parameters are:
 *
 *   [NearestNeighborAssociationParameters]
 *   max_boundary_distance_m = maximum distance between LaserObject and TrackingObject for an association (meters)
 *   use_predicted_boundary = flag indicating if the boundary predicted into the future should be used
 *
 * Requirements:
 *
 *   - max_boundary_distance > 0.0
 */
struct nearest_neighbor_params_t
{
    double maxBoundaryDistance;   ///< Maximum distance between LaserObject and TrackingObject for an association
    bool usePredictedBoundary;    ///< Flag indicating if the boundary predicted into the future should be used

    nearest_neighbor_params_t(void) = default;
    nearest_neighbor_params_t(const utils::ConfigFile& config);
};


/**
 * NearestNeighborAssociation uses a classic neareset neighbor approach to data association. The data association finds
 * the object in the current collection of objects whose boundary falls closest to the laser points associated with the
 * laser object being matched.
 *
 * Parameters control the following behavior of the nearest neighbor association:
 *
 *   1) The maximum distance from an object to its boundary for it to be considered a match or not.
 *   2) Whether or not first estimate the new boundary of the objects, given their previous states before doing the
 *      data association, i.e. do we do association with where we think the object now is or where we last saw it?
 *
 */
class NearestNeighborAssociation : public DataAssociationStrategy
{
public:
    /**
     * Constructor for NearestNeighborAssociation.
     *
     * \param    params          Parameters for doing nearest-neighbor data association
     */
    NearestNeighborAssociation(const nearest_neighbor_params_t& params);

    // DataAssociationStrategy interface
    object_association_t associateLaserWithTracked(const LaserObject& laser,
                                                   const TrackingObjectCollection& objects) override;
    object_association_t associateObjectWithTracked(const TrackingObject& object,
                                                    const TrackingObjectCollection& objects) override;
    std::unique_ptr<DataAssociationStrategy> clone(void) override;

private:
    nearest_neighbor_params_t params_;
};

}   // namespace tracker
}   // namespace vulcan

#endif   // TRACKER_TRACKING_NEAREST_NEIGHBOR_ASSOCIATION_H
