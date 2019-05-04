/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose.cpp
* \author   Collin Johnnson
*
* Definition of helper functions for pose_t.
*/

#include <core/pose.h>
#include <core/pose_distribution.h>
#include <core/angle_functions.h>
#include <core/float_comparison.h>
#include <cassert>

namespace vulcan
{

pose_t::pose_t(const pose_distribution_t& distribution)
: timestamp(distribution.timestamp)
, x(distribution.x)
, y(distribution.y)
, theta(distribution.theta)
{
}


pose_t pose_t::transformToNewFrame(const pose_t& newFrame) const
{
    Point<float> newPosition = transform(toPoint(), -newFrame.x, -newFrame.y, -newFrame.theta);

    return pose_t(newPosition.x, newPosition.y, angle_diff(theta, newFrame.theta));
}


pose_t pose_t::compound(const pose_t& origin) const
{
    Point<float> newPosition = homogeneous_transform(toPoint(), origin.x, origin.y, origin.theta);
    return pose_t(timestamp, newPosition.x, newPosition.y, angle_sum(theta, origin.theta));
}


bool operator==(const pose_t& lhs, const pose_t& rhs)
{
    return absolute_fuzzy_equal(lhs.x, rhs.x) &&
           absolute_fuzzy_equal(lhs.y, rhs.y) &&
           absolute_fuzzy_equal(lhs.theta, rhs.theta);
}


bool operator!=(const pose_t& lhs, const pose_t& rhs)
{
    return !(lhs == rhs);
}


std::ostream& operator<<(std::ostream& out, const pose_t& pose)
{
    out<<'('<<pose.x<<','<<pose.y<<','<<pose.theta<<')';
    return out;
}


std::ostream& operator<<(std::ostream& out, const pose_6dof_t& pose)
{
    out<<'('<<pose.x<<','<<pose.y<<','<<pose.z<<','<<pose.phi<<','<<pose.rho<<','<<pose.theta<<')';
    return out;
}


pose_t interpolate_pose(const pose_t& priorPose, const pose_t& currentPose, int64_t desiredPoseTime)
{
    assert((priorPose.timestamp <= desiredPoseTime) && (desiredPoseTime <= currentPose.timestamp));

    if(priorPose.timestamp == currentPose.timestamp)
    {
        return priorPose;
    }

    double scale = static_cast<double>(desiredPoseTime - priorPose.timestamp) / (currentPose.timestamp - priorPose.timestamp);

    pose_t interpolatedPose;

    interpolatedPose.timestamp = desiredPoseTime;
    interpolatedPose.x         = priorPose.x + (currentPose.x-priorPose.x)*scale;
    interpolatedPose.y         = priorPose.y + (currentPose.y-priorPose.y)*scale;
    interpolatedPose.theta     = wrap_to_pi(priorPose.theta + angle_diff(currentPose.theta, priorPose.theta)*scale);

    return interpolatedPose;
}

} // namespace vulcan
