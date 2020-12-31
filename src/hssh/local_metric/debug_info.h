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
 * Definition of local_metric_localization_debug_info_t.
 */

#ifndef HSSH_LOCAL_METRIC_DEBUG_INFO_H
#define HSSH_LOCAL_METRIC_DEBUG_INFO_H

#include "hssh/metrical/localization/debug_info.h"
#include "hssh/metrical/relocalization/debug_info.h"
#include "system/message_traits.h"

namespace vulcan
{
namespace hssh
{

struct local_metric_localization_debug_info_t
{
    particle_filter_debug_info_t particleFilterInfo;
};

struct local_metric_relocalization_debug_info_t
{
    metric_relocalization_debug_info_t info;
};


template <class Archive>
void serialize(Archive& ar, local_metric_localization_debug_info_t& info)
{
    ar(info.particleFilterInfo);
}

template <class Archive>
void serialize(Archive& ar, local_metric_relocalization_debug_info_t& info)
{
    ar(info.info);
}

}   // namespace hssh
}   // namespace vulcan

DEFINE_DEBUG_MESSAGE(hssh::local_metric_localization_debug_info_t, ("DEBUG_HSSH_LOCAL_METRIC_LOCALIZATION_INFO"))
DEFINE_DEBUG_MESSAGE(hssh::local_metric_relocalization_debug_info_t, ("DEBUG_HSSH_LOCAL_METRIC_RELOCALIZATION_INFO"))

#endif   // HSSH_LOCAL_METRIC_DEBUG_INFO_H
