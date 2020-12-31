/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** \file   simulator_utils.h
 * \author Zongtai Luo
 * Simulator_utils contains some useful functions for genrating and sending sensor data.
 */
#ifndef ENVIRONMENT_SIMULATOR_SIMULATOR_UTILS_H
#define ENVIRONMENT_SIMULATOR_SIMULATOR_UTILS_H

#include "core/laser_scan.h"
#include "core/motion_state.h"
#include "core/point.h"
#include "core/pose.h"
#include "hssh/local_metric/lpm.h"
#include "sensors/wheel_encoders_params.h"
#include "simulator/simulator_params.h"
#include "system/module_communicator.h"
#include "utils/ray_tracing.h"

namespace vulcan
{

namespace sim
{

// simulator_set_up is a struct store commandline parameters for the simulator
struct simulator_set_up_t
{
    std::string mode;
    int case_no;
};


simulator_set_up_t init_simulator_params(int argc, char** argv);


// Map_init is for sending map to the debug_ui
void Map_init(std::string& Simplemap, hssh::LocalPerceptualMap& mylpm, int32_t Id = 1);


int find_range(std::vector<Point<double>>& endpointsgrid,
               std::vector<Point<double>>& endpointsglobal,
               Point<double> startgrid,
               Point<double> startglobal,
               utils::ray_trace_range_t& fake_ray_shot,
               polar_laser_scan_t& fake_polar_laser_scan,
               hssh::LocalPerceptualMap& lpm);

// Send the laser scan at the current pose
std::vector<Point<double>> flaser_scan_producer(pose_t& vulcan_one_state,
                                                hssh::LocalPerceptualMap mylpm,
                                                int64_t timestamp,
                                                system::ModuleCommunicator& connector,
                                                float maxRange_ = 40,
                                                bool send_to_robot = true,
                                                double angle_range = 2 * M_PI,
                                                float angularResolution = 2 * M_PI / 720,
                                                double start_offset = 0.0);

// Send fake encoder signal to robot
void fencoder_producer(int64_t& leftTicksTotal,
                       int64_t& rightTicksTotal,
                       motion_state_t vulcan_state,
                       motion_state_t vulcan_former_state,
                       sensors::wheel_encoders_params_t& wheel_encoder_params,
                       int64_t timestamp,
                       system::ModuleCommunicator& connector);

// Odometry updated according to the difference between two states
void fodometry_producer(motion_state_t& vulcan_state,
                        motion_state_t& vulcan_former_state,
                        int64_t timestamp,
                        system::ModuleCommunicator& connector);

}   // namespace sim
}   // namespace vulcan

#endif