/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Jong Jin Park
*
* Definition of the params structs for all classes in the metric planner module.
*/

#ifndef MPEPC_METRIC_PLANNER_PARAMS_H
#define MPEPC_METRIC_PLANNER_PARAMS_H

#include <mpepc/grid/params.h>
#include <mpepc/simulator/params.h>
#include <mpepc/metric_planner/task/params.h>
#include <mpepc/trajectory/params.h>

#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace mpepc
{

struct metric_planner_params_t
{
    // MetricPlannerDirector
    int64_t plannerUpdatePeriodUs;
    float   trajectoryTimeLength;
    float   simulatorTimeStep;
    bool    shouldDebugGrids;
    bool    shouldDebugDynamicObjects;

    float progressWarningTimeWindow;
    float progressWarningThreshold;

    double maxSafePositionStd;
    double maxSafeHeadingStd;

    bool   useChanceConstraint = 0;
    double minAcceptableChance;

    int64_t gridRebuildIntervalUs;
    obstacle_distance_grid_builder_params_t obstacleDistanceGridBuilderParams;
    dynamic_object_filter_params_t          dynamicObjectFilterParams;
    dynamic_object_simulator_params_t       dynamicObjectSimulatorParams;

    metric_planner_task_params_t   taskParams;
    task_manifold_builder_params_t taskManifoldBuilderParams;

    trajectory_planner_params_t trajectoryPlannerParams;

    metric_planner_params_t(void) {};
    metric_planner_params_t(const utils::ConfigFile& config, const utils::ConfigFile& controllerConfig, const utils::ConfigFile& robotConfig);
};

/**
* load_metric_planner_params loads the paramete4rs for the metric planner module
* from the provided configuration file.
*
* \param    config          ConfigFile containing the values parsed from the module .cfg file
* \return   Parameters for the current instance of the metric planner module.
*/
metric_planner_params_t load_metric_planner_params(const utils::ConfigFile& config);

} // mpepc
} // vulcan

#endif // MPEPC_METRIC_PLANNER_PARAMS_H
