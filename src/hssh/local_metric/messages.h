/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     messages.h
 * \author   Collin Johnson
 *
 * Declaration of the messages for interacting with the local_metric_hssh and some
 * associated types supporting those messages.
 *
 *   - local_metric_mode_message_t
 *       + local_metric_mode_t
 *   - relocalization_request_message_t
 *       + relocalization_initialization_mode_t
 *       + relocalization_map_action_t
 *   - completed_relocalization_message_t
 *       + relocalization_result_t
 *
 * All of these messages are encapsulated in the local_metric_hssh_message_t to make it
 * easy for adding new messages and having them delivered to other modules, rather than
 * blitzing them with a huge number of individual message inputs to process.
 */

#ifndef HSSH_LOCAL_METRIC_MESSAGES_H
#define HSSH_LOCAL_METRIC_MESSAGES_H

#include "hssh/local_metric/lpm.h"
#include "hssh/metrical/relocalization/filter_initializer.h"
#include "system/message_traits.h"
#include <cereal/types/polymorphic.hpp>
#include <cstdint>

namespace vulcan
{
namespace hssh
{

// NOTE: Rather than my usual split of enums then structs, I'm clumping them by message and associated
//       enums to keep everything more local for easier reading

/////////// local_metric_mode_message_t //////////////////
enum class LocalMetricMode
{
    kLocalizationOnly,
    kFullSlam,
    kHighResolutionLPM
};

struct local_metric_mode_message_t
{
    int64_t timestamp;
    LocalMetricMode mode;
};

/////////// local_metric_glass_eval_message_t //////////////////
enum class GlassEvalCmd : uint8_t
{
    kNOP = 0,
    kSaveMap,
    kLoadMap,
    kSavePoses,
    kSaveScans

};

struct local_metric_glass_eval_message_t
{
    int64_t timestamp;
    GlassEvalCmd cmd;
    std::string filename;
};

/////////// Relocalization messages  //////////////////
struct local_metric_relocalization_request_message_t
{
    int64_t timestamp;
    LocalPerceptualMap map;

    std::shared_ptr<FilterInitializer> initializer;
};

///////////////////// Serialization support ////////////////////////////
template <class Archive>
void serialize(Archive& ar, local_metric_glass_eval_message_t& msg)
{
    ar& msg.timestamp;
    ar& msg.cmd;
    ar& msg.filename;
}

///////////////////// Serialization support ////////////////////////////
template <class Archive>
void serialize(Archive& ar, local_metric_mode_message_t& msg)
{
    ar& msg.timestamp;
    ar& msg.mode;
}

///////////////////// Serialization support ////////////////////////////
template <class Archive>
void serialize(Archive& ar, local_metric_relocalization_request_message_t& msg)
{
    ar& msg.timestamp;
    ar& msg.map;
    ar& msg.initializer;
}

}   // namespace hssh
}   // namespace vulcan

// DEFINE_SYSTEM_MESSAGE(polar_laser_scan_t, ("LAZER"))
DEFINE_SYSTEM_MESSAGE(hssh::local_metric_glass_eval_message_t, ("HSSH_LOCAL_METRIC_GLASS_EVAL_MSG"))
DEFINE_SYSTEM_MESSAGE(hssh::local_metric_mode_message_t, ("HSSH_LOCAL_METRIC_MODE_MSG"))
DEFINE_SYSTEM_MESSAGE(hssh::local_metric_relocalization_request_message_t,
                      ("HSSH_LOCAL_METRIC_RELOCALIZATION_REQUEST_MSG"))

#endif   // HSSH_LOCAL_METRIC_MESSAGES_H
