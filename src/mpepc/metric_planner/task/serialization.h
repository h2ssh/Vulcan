/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     serialization.h
 * \author   Collin Johnson
 *
 * A convenience header for all tasks that might be arriving, but aren't otherwise explicitly included in the
 * metric_planner. Just include this header to automatically enable receiving all possible MetricPlannerTask subclasses.
 */

#ifndef MPEPC_METRIC_PLANNER_TASKS_SERIALIZATION_H
#define MPEPC_METRIC_PLANNER_TASKS_SERIALIZATION_H

#include "mpepc/metric_planner/task/navigation.h"
#include "mpepc/metric_planner/task/rotate.h"
#include "mpepc/metric_planner/task/wait.h"

#endif   // MPEPC_METRIC_PLANNER_TASKS_SERIALIZATION_H
