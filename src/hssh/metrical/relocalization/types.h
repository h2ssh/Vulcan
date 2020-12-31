/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     types.h
* \author   Collin Johnson
* 
* Declaration of the enum types and messages used for inputs and results in the
* relocalization system. Defined are:
* 
* Enums:
*   - relocalization_initialized_mode_t
*   - relocalization_map_action_t
*   - relocalization_result_t
*   - relocalization_progress_t
* 
* Messages:
*   - relocalization_request_message_t
*   - completed_relocalization_message_t
*/

#ifndef HSSH_UTILS_METRICAL_RELOCALIZATION_TYPES_H
#define HSSH_UTILS_METRICAL_RELOCALIZATION_TYPES_H

#include "core/pose.h"

namespace vulcan
{
namespace hssh
{

/**
* RelocalizationStatus defines the intermediate states through which a relocalization task can go
* before being completed.
*/
enum RelocalizationStatus
{
    NoTask,         ///< A relocalization task has yet to be started
    InPrograss,     ///< More data are needed before the relocalization can be certain
    Success,        ///< The task is complete and the pose has been found
    Failure         ///< The map is wrong or there isn't enough data available to figure out where the robot is
};


/**
* relocalization_progress_t
*/
struct relocalization_progress_t
{
    RelocalizationStatus status;                
    pose_t        relocalizedPose;
};

}
}


#endif // HSSH_UTILS_METRICAL_RELOCALIZATION_TYPES_H
