/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     navigation_data.h
 * \author   Collin Johnson
 *
 * Declaration of NavigationData.
 */

#ifndef UI_NAVIGATION_NAVIGATION_DATA_H
#define UI_NAVIGATION_NAVIGATION_DATA_H

#include "core/pose.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/location.h"

namespace vulcan
{
namespace ui
{

/**
 * NavigationData contains data relevant to Goal and Decision navigation.
 */
struct NavigationData
{
    pose_t pose;
    const hssh::LocalPerceptualMap* metricMap = nullptr;
    const hssh::LocalLocation* location = nullptr;
    const hssh::LocalTopoMap* topoMap = nullptr;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_NAVIGATION_NAVIGATION_DATA_H
