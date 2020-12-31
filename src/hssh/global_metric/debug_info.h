/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     debug_info.h
* \author   Collin Johnson
* 
* Definition of debug info structs for transmission to the DebugUI.
*/

#ifndef HSSH_GLOBAL_METRIC_DEBUG_INFO_H
#define HSSH_GLOBAL_METRIC_DEBUG_INFO_H

#include "hssh/metrical/localization/debug_info.h"
#include "hssh/metrical/relocalization/debug_info.h"
#include "system/message_traits.h"

namespace vulcan
{
namespace hssh
{

/**
* global_metric_relocalization_info_t contains information from running the MetricRelocalizer.
*/    
struct global_metric_relocalization_info_t
{
    metric_relocalization_debug_info_t info;
};

/**
* global_metric_localization_info_t contains information from running the particle filter.
*/
struct global_metric_localization_info_t
{
    particle_filter_debug_info_t info;
};

// Serialization support
template <class Archive>
void serialize(Archive& ar, global_metric_relocalization_info_t& info)
{
    ar (info.info);
}

template <class Archive>
void serialize(Archive& ar, global_metric_localization_info_t& info)
{
    ar (info.info);
}
    
}
}

DEFINE_DEBUG_MESSAGE(hssh::global_metric_relocalization_info_t, ("DEBUG_HSSH_GLOBAL_METRIC_RELOCALIZATION_INFO"))
DEFINE_DEBUG_MESSAGE(hssh::global_metric_localization_info_t,   ("DEBUG_HSSH_GLOBAL_METRIC_LOCALIZATION_INFO"))

#endif // HSSH_GLOBAL_METRIC_DEBUG_INFO_H
