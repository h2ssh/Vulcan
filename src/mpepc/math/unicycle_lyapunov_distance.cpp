/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     unicycle_lyapunov_distance.cpp
 * \author   Jong Jin Park
 *
 * Definition of UnicycleLyapunovDistance.
 */

#include "mpepc/math/unicycle_lyapunov_distance.h"
#include "core/pose.h"

namespace vulcan
{
namespace mpepc
{

UnicycleLyapunovDistance::UnicycleLyapunovDistance(const pose_t& targetPose,
                                                   const unicycle_lyapunov_distance_params_t& params)
: chart_(targetPose, params.smallRadius)
, params_(params)
{
}


double UnicycleLyapunovDistance::stabilizingDeltaStar(const Point<float> point) const
{
    reduced_egocentric_polar_coords_t coords = chart_.point2rp(point);

    return stabilizing_delta_star(coords.r, coords.phi, params_.kPhi, params_.vectorFieldType, params_.smallRadius);
}


double UnicycleLyapunovDistance::stabilizingHeading(const Point<float> point) const
{
    reduced_egocentric_polar_coords_t coords = chart_.point2rp(point);
    double deltaStar =
      stabilizing_delta_star(coords.r, coords.phi, params_.kPhi, params_.vectorFieldType, params_.smallRadius);

    return deltaStar + coords.lineOfSightAngle;
}


double UnicycleLyapunovDistance::distanceFromPoint(const Point<float>& point) const
{
    reduced_egocentric_polar_coords_t coords = chart_.point2rp(point);

    return distance_on_manifold(coords.r, coords.phi, params_.kPhi);
}


double UnicycleLyapunovDistance::distanceFromPose(const pose_t& pose) const
{
    egocentric_polar_coords_t coords = chart_.pose2rpd(pose);

    return unicycle_nonholonomic_distance(coords.r,
                                          coords.phi,
                                          coords.delta,
                                          params_.kPhi,
                                          params_.kDelta,
                                          params_.vectorFieldType,
                                          params_.smallRadius);
}


}   // namespace mpepc
}   // namespace vulcan
