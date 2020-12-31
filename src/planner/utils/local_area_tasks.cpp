/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_area_tasks.cpp
 * \author   Collin Johnson
 *
 * Definition of utility functions for dealing with LocalAreas for planning:
 *
 *   - create_navigation_task_for_local_area
 */

#include "planner/utils/local_area_tasks.h"
#include "hssh/local_topological/area.h"
#include "mpepc/metric_planner/task/navigation.h"

namespace vulcan
{
namespace planner
{

std::unique_ptr<mpepc::NavigationTask> create_navigation_task_for_local_area(const hssh::LocalArea& targetArea)
{
    // When moving to a local area, we can move anywhere inside the area and still make it. The preference should
    // be to move to the center, but that's not always a viable position. All freespace is possible for the robot,
    // so it should try to get anywhere.
    std::vector<position_t> viablePositions;
    viablePositions.reserve(targetArea.extent().size());
    std::copy(targetArea.extent().begin(), targetArea.extent().end(), std::back_inserter(viablePositions));

    // Sort in ascending distance from the center of the area, thus preferring to be as close as possible to
    // the middle
    std::sort(viablePositions.begin(), viablePositions.end(), [&targetArea](position_t lhs, position_t rhs) {
        return distance_between_points(lhs, targetArea.center().toPoint())
          < distance_between_points(rhs, targetArea.center().toPoint());
    });

    return std::unique_ptr<mpepc::NavigationTask>(new mpepc::NavigationTask(viablePositions));
}

}   // namespace planner
}   // namespace vulcan
