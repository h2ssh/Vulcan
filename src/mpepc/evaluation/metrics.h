/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     metrics.h
* \author   Collin Johnson
* 
* Declaration of functions that compute the following metrics for an MPEPCLog:
* 
*/

#ifndef MPEPC_EVALUATION_METRICS_H
#define MPEPC_EVALUATION_METRICS_H

#include "mpepc/evaluation/mpepc_log.h"

namespace vulcan
{
namespace mpepc
{

/**
* plot_comfort
*/
void plot_comfort(MPEPCLog::imu_iterator begin, MPEPCLog::imu_iterator end);

/**
* plot_acceleration generates time plot of the raw forward and lateral accelerations. A separate line is drawn for
* each of these values. The x-axis is time. The y-axis is raw acceleration as measured by the IMU.
* 
* The result of this function is the creation of a Gnuplot that will appear and can then be saved later if desired.
* 
* \param    begin           Start of the data to plot
* \param    end             End of the data to plot
*/
void plot_acceleration(MPEPCLog::imu_iterator begin, MPEPCLog::imu_iterator end);

/**
* plot_safety shows linear and angular velocity and distance to nearest static and dynamic obstacle.
*
* A separate line is drawn for each of the four values.
* 
* The result of this function is the creation of a Gnuplot that will appear and can then be saved later if desired.
* 
* \param    beginMotion     Start of the motion data to plot
* \param    endMotion       End of the motion data to plot
* \param    beginInfo       Start of the planner info
* \param    endInfo         End of the planner info
*/
void plot_safety(MPEPCLog::motion_state_iterator beginMotion, 
                 MPEPCLog::motion_state_iterator endMotion,
                 MPEPCLog::debug_info_iterator beginInfo,
                 MPEPCLog::debug_info_iterator endInfo);

/**
* plot_stability
*/
void plot_stability();

/**
* plot_collision
*/
void plot_collision();

/**
* plot_timing
*/
void plot_timing();

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_EVALUATION_METRICS_H
