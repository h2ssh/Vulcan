/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     planner_params.cpp
* \author   Jong Jin Park
*
* Definition of various conversion functions for loading parameters.
*/

#include "mpepc/metric_planner/params.h"
#include "utils/config_file.h"
#include "utils/config_file_utils.h"
#include <cassert>

namespace vulcan
{
namespace mpepc
{

const std::string kMetricPlannerHeading    ("MetricPlannerParameters");
const std::string kPlannerUpdatePeriodKey  ("planner_update_period_ms");
const std::string kTrajectoryTimeLengthKey ("planning_horizon_s");
const std::string kSimulatorTimeStepKey    ("simulator_timestep_s");
const std::string kShouldDebugGridsKey     ("should_debug_grids");
const std::string kShouldDebugObjectsKey   ("should_debug_dynamic_objects");
const std::string kProgressWarningTimeKey  ("progress_warning_time_window_s");
const std::string kProgressWarningThreshKey("progress_warning_threshold");
const std::string kUseChanceConstraintKey  ("use_chance_constraint");
const std::string kMinAcceptableChanceKey  ("min_acceptable_survivability");
const std::string kGridRebuildIntervalKey  ("grid_rebuild_interval_ms");
const std::string kControllerConfigFileKey ("controller_config_file");
const std::string kRobotModelConfigFileKey ("robot_model_config_file");
const std::string kMaxPositionStdKey       ("max_safe_position_std_dev");
const std::string kMaxHeadingStdKey        ("max_safe_heading_std_dev");


metric_planner_params_t::metric_planner_params_t(const utils::ConfigFile& config, const utils::ConfigFile& controllerConfig, const utils::ConfigFile& robotConfig)
: plannerUpdatePeriodUs     (config.getValueAsUInt32(kMetricPlannerHeading, kPlannerUpdatePeriodKey) * 1000ll)
, trajectoryTimeLength      (config.getValueAsFloat (kMetricPlannerHeading, kTrajectoryTimeLengthKey))
, simulatorTimeStep         (config.getValueAsFloat (kMetricPlannerHeading, kSimulatorTimeStepKey))
, shouldDebugGrids          (config.getValueAsBool  (kMetricPlannerHeading, kShouldDebugGridsKey))
, shouldDebugDynamicObjects (config.getValueAsBool  (kMetricPlannerHeading, kShouldDebugObjectsKey))
, progressWarningTimeWindow (config.getValueAsFloat (kMetricPlannerHeading, kProgressWarningTimeKey))
, progressWarningThreshold  (config.getValueAsFloat (kMetricPlannerHeading, kProgressWarningThreshKey))
, maxSafePositionStd        (config.getValueAsDouble(kMetricPlannerHeading, kMaxPositionStdKey))
, maxSafeHeadingStd         (config.getValueAsDouble(kMetricPlannerHeading, kMaxHeadingStdKey))
, useChanceConstraint       (config.getValueAsBool  (kMetricPlannerHeading, kUseChanceConstraintKey))
, minAcceptableChance       (config.getValueAsDouble(kMetricPlannerHeading, kMinAcceptableChanceKey))
, gridRebuildIntervalUs     (config.getValueAsUInt32(kMetricPlannerHeading, kGridRebuildIntervalKey) * 1000ll)
, obstacleDistanceGridBuilderParams(config)
, dynamicObjectFilterParams(config)
, dynamicObjectSimulatorParams(config)
, taskParams(config)
, taskManifoldBuilderParams(config)
, trajectoryPlannerParams(config, controllerConfig, robotConfig)
{
    assert(plannerUpdatePeriodUs > 0);
    assert(trajectoryTimeLength * 1000 > plannerUpdatePeriodUs / 1000);
    assert(simulatorTimeStep > 0);
    assert(maxSafePositionStd > 0.0);
    assert(maxSafeHeadingStd > 0.0);

    trajectoryPlannerParams.optimizerParams.useChanceConstraint = useChanceConstraint;
    trajectoryPlannerParams.optimizerParams.minAcceptableChance = minAcceptableChance;

    trajectoryPlannerParams.evaluatorParams.useChanceConstraint        = useChanceConstraint;
    trajectoryPlannerParams.evaluatorParams.minAcceptableSurvivability = minAcceptableChance;
}


metric_planner_params_t load_metric_planner_params(const utils::ConfigFile& config)
{
    utils::ConfigFile controllerConfig(config.getValueAsString(kMetricPlannerHeading, kControllerConfigFileKey));
    utils::ConfigFile robotConfig(config.getValueAsString(kMetricPlannerHeading, kRobotModelConfigFileKey));

    metric_planner_params_t params(config, controllerConfig, robotConfig);

    params.trajectoryPlannerParams.optimizerParams.useChanceConstraint = params.useChanceConstraint;
    params.trajectoryPlannerParams.optimizerParams.minAcceptableChance = params.minAcceptableChance;

    params.trajectoryPlannerParams.evaluatorParams.useChanceConstraint        = params.useChanceConstraint;
    params.trajectoryPlannerParams.evaluatorParams.minAcceptableSurvivability = params.minAcceptableChance;

    return params;
}

} // namespace planner
} // namespace vulcan
