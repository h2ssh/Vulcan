/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file
 * \author   Collin Johnson
 *
 * Definition of IMU support functions:
 *
 *   - interpolate_imu
 */

#include "core/imu_data.h"
#include "core/angle_functions.h"
#include "math/interpolation.h"
#include <cassert>

namespace vulcan
{

std::pair<imu_data_t, imu_data_t> interpolate_imu_data(const imu_data_t& before, const imu_data_t& after, int64_t time)
{
    assert(before.timestamp <= time);
    assert(after.timestamp >= time);
    assert(before.timestamp < after.timestamp);

    int64_t timeDeltaUs = time - before.timestamp;
    double interpolationRatio = static_cast<double>(timeDeltaUs) / (after.timestamp - before.timestamp);

    imu_data_t interpolatedIMU;
    interpolatedIMU.timestamp = time;
    interpolatedIMU.sequenceNumber = after.sequenceNumber;
    interpolatedIMU.timeDelta = timeDeltaUs;
    interpolatedIMU.gravityMagnitude = after.gravityMagnitude;

    for (int n = 0; n < 3; ++n) {
        interpolatedIMU.acceleration[n] =
          math::linear_interpolation(before.acceleration[n], after.acceleration[n], interpolationRatio);
        interpolatedIMU.rotationalVelocity[n] =
          math::linear_interpolation(before.rotationalVelocity[n], after.rotationalVelocity[n], interpolationRatio);
        interpolatedIMU.orientation[n] =
          wrap_to_pi(math::linear_interpolation(before.orientation[n], after.orientation[n], interpolationRatio));
    }

    imu_data_t newAfterIMU = after;
    ++newAfterIMU.sequenceNumber;
    newAfterIMU.timeDelta = after.timeDelta - interpolatedIMU.timeDelta;

    return std::make_pair(interpolatedIMU, newAfterIMU);
}

}   // namespace vulcan
