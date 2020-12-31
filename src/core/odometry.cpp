/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     odometry.cpp
* \author   Collin Johnson
*
* Definition of odometry support functions:
*
*   - interpolate_odometry
*/

#include "core/odometry.h"
#include "core/angle_functions.h"
#include <cassert>

namespace vulcan
{

odometry_t interpolate_odometry(const odometry_t& before, const odometry_t& after, int64_t time)
{
    assert(before.timestamp <= time);
    assert(after.timestamp >= time);

    if(before.timestamp == time)
    {
        return before;
    }
    else if(after.timestamp == time)
    {
        return after;
    }

    double splitRatio = (time - before.timestamp) / static_cast<double>(after.timestamp - before.timestamp);

    // Can't use the translation and rotation values because there's no guarantee the two odometry measurements
    // were taken consecutively, so the absolute data needs to be used instead
    double trans = std::sqrt(std::pow(after.x-before.x, 2) + std::pow(after.y-before.y,2)) * splitRatio;
    double rot   = angle_diff(after.theta, before.theta) * splitRatio;

    odometry_t interpolatedOdom;
    interpolatedOdom.timestamp   = time;
    interpolatedOdom.x           = before.x + trans*std::cos(before.theta + rot/2.0);
    interpolatedOdom.y           = before.y + trans*std::sin(before.theta + rot/2.0);
    interpolatedOdom.theta       = angle_sum(before.theta, rot);
    interpolatedOdom.translation = trans;
    interpolatedOdom.rotation    = rot;

    return interpolatedOdom;
}

} // namespace vulcan
