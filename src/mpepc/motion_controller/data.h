/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     data.h
 * \author   Collin Johnson
 *
 * Declaration of motion_controller_data_t that holds all state info for
 * the MotionControllers.
 */

#ifndef MPEPC_MOTOIN_CONTROLLER_DATA_H
#define MPEPC_MOTOIN_CONTROLLER_DATA_H

#include "core/motion_state.h"
#include <cstdint>

namespace vulcan
{

namespace mpepc
{

/**
 * motion_controller_data_t contains the robot state information needed for the
 * MotionController::updateCommand method.
 *
 */
struct motion_controller_data_t
{
    motion_state_t state;    // Current pose and velocity of the robot
    int64_t currentTimeUs;   // Current time
    float timestep;          // Period, in seconds, of the update loop for the controller
};

}   // namespace mpepc
}   // namespace vulcan

#endif   // MPEPC_MOTOIN_CONTROLLER_DATA_H
