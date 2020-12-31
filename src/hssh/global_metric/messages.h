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
 * Definition of messages for controlling global_metric_hssh module:
 *
 *   - global_metric_relocalization_request_message_t
 */

#ifndef HSSH_GLOBAL_METRIC_MESSAGES_H
#define HSSH_GLOBAL_METRIC_MESSAGES_H

#include "hssh/global_metric/map.h"
#include "hssh/metrical/relocalization/filter_initializer.h"
#include "system/message_traits.h"
#include <cereal/types/polymorphic.hpp>

namespace vulcan
{
namespace hssh
{

/**
 * global_metric_relocalization_request_message_t requests the global_metric_hssh relocalize in a new map.
 *
 * A request consists of the map in which to localize in and the method by which to initialize the
 * relocalization particle filter.
 */
struct global_metric_relocalization_request_message_t
{
    GlobalMetricMap map;
    std::shared_ptr<FilterInitializer> initializer;
};

template <class Archive>
void serialize(Archive& ar, global_metric_relocalization_request_message_t& msg)
{
    ar(msg.map, msg.initializer);
}

}   // namespace hssh
}   // namespace vulcan

DEFINE_SYSTEM_MESSAGE(hssh::global_metric_relocalization_request_message_t,
                      ("HSSH_GLOBAL_METRIC_RELOCALIZATION_REQUEST"))

#endif   // HSSH_GLOBAL_METRIC_MESSAGES_H
