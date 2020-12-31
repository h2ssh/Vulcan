/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     metrics.cpp
 * \author   Collin Johnson
 *
 * Definition of functions that compute the following metrics for an MPEPCLog:
 *
 */

#include "mpepc/evaluation/metrics.h"
#include "utils/plot2d.h"
#include "utils/timestamp.h"
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace mpepc
{

void plot_comfort(MPEPCLog::imu_iterator begin, MPEPCLog::imu_iterator end)
{
}


void plot_acceleration(MPEPCLog::imu_iterator begin, MPEPCLog::imu_iterator end)
{
    std::cout << "Plotting " << std::distance(begin, end) << " acceleration values.\n";

    utils::Plot2D plot("MPEPC Acceleration", "Time (sec)", "Accel (m/s^2)");

    for (auto& imu : boost::make_iterator_range(begin, end)) {
        plot.addData(utils::usec_to_sec(imu.timestamp), imu.acceleration[IMU_X_INDEX], 0);
        plot.addData(utils::usec_to_sec(imu.timestamp), imu.acceleration[IMU_Y_INDEX], 1);
    }

    plot.setTypeName(0, "Forward");
    plot.setTypeName(1, "Lateral");

    plot.plot(utils::PlotStyle::lines);
}


void plot_safety(MPEPCLog::motion_state_iterator beginMotion,
                 MPEPCLog::motion_state_iterator endMotion,
                 MPEPCLog::debug_info_iterator beginInfo,
                 MPEPCLog::debug_info_iterator endInfo)
{
    enum
    {
        linear_idx,
        angular_idx,
        static_dist_idx,
        dynamic_dist_idx,
    };

    utils::Plot2D plot("MPEPC Safety", "Time (sec)", "");

    for (auto& state : boost::make_iterator_range(beginMotion, endMotion)) {
        plot.addData(utils::usec_to_sec(state.timestamp), std::abs(state.velocity.linear), linear_idx);
        plot.addData(utils::usec_to_sec(state.timestamp), std::abs(state.velocity.angular), angular_idx);
    }

    for (auto& info : boost::make_iterator_range(beginInfo, endInfo)) {
        plot.addData(utils::usec_to_sec(info.planReleaseTimeUs), info.clearanceToStaticObs, static_dist_idx);
        plot.addData(utils::usec_to_sec(info.planReleaseTimeUs), info.clearanceToDynObs, dynamic_dist_idx);
    }

    plot.setTypeName(linear_idx, "Linear velocity (m/s)");
    plot.setTypeName(angular_idx, "Angular velocity (rad/s)");
    plot.setTypeName(static_dist_idx, "Distance to static obstacle (m)");
    plot.setTypeName(dynamic_dist_idx, "Distance to dynamic obstacle (m)");

    plot.plot(utils::PlotStyle::lines);
}


void plot_stability()
{
}


void plot_collision()
{
}


void plot_timing()
{
}

}   // namespace mpepc
}   // namespace vulcan
