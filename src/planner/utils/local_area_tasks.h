/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     local_area_tasks.h
* \author   Collin Johnson
* 
* Declaration of utility functions for dealing with LocalAreas for planning:
* 
*   - create_navigation_task_for_local_area : create a task to drive to a provided LocalArea
*/

#ifndef PLANNER_UTILS_LOCAL_AREA_TASKS_H
#define PLANNER_UTILS_LOCAL_AREA_TASKS_H

#include <memory>

namespace vulcan
{
namespace hssh { class LocalArea; }
namespace mpepc { class NavigationTask; }
namespace planner
{

/**
* create_navigation_task_for_local_area creates a MetricPlannerTask to be executed by the metric_planner for driving to
* the specified LocalArea.
* 
* The task will create a NavigationTask where the potential targets are cells in the extent of the area. The target
* will have the robot move to the position closest to the center of the area.
* 
* \param    targetArea          LocalArea target for the robot
* \return   An instance of mpepc::NavigationTask to issue to the metric_planner
*/
std::unique_ptr<mpepc::NavigationTask> create_navigation_task_for_local_area(const hssh::LocalArea& targetArea);


}
}

#endif // PLANNER_UTILS_LOCAL_AREA_TASKS_H
