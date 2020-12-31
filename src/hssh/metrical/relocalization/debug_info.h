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
* Definition of metric_relocalization_debug_info_t.
*/

#ifndef HSSH_UTILS_METRICAL_RELOCALIZATION_DEBUG_INFO_H
#define HSSH_UTILS_METRICAL_RELOCALIZATION_DEBUG_INFO_H

#include "core/pose_distribution.h"
#include "hssh/metrical/localization/debug_info.h"

namespace vulcan
{
namespace hssh
{
    
struct metric_relocalization_debug_info_t
{
    pose_distribution_t   pose;                      ///< Pose in the relocalization map
    std::vector<particle_t>      initialParticles;          ///< Initial set of particles when relocalizing
    particle_filter_debug_info_t particleFilterInfo;        ///< Current state of the relocalization process
};

template <class Archive>
void serialize(Archive& ar, metric_relocalization_debug_info_t& info)
{
    ar( info.pose,
        info.initialParticles,
        info.particleFilterInfo);
}

}
}

#endif // HSSH_UTILS_METRICAL_RELOCALIZATION_DEBUG_INFO_H
