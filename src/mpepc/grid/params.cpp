/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.cpp
* \author   Jong Jin Park
*
* Definition of the obstacle_distance_grid_builder_params_t and navigation_grid_builder_params_t.
*/

#include <mpepc/grid/params.h>
#include <utils/config_file.h>
#include <utils/config_file_utils.h>
#include <cassert>

namespace vulcan
{

namespace mpepc
{

const std::string kObstDistGridHeading("ObstacleDistanceGridBuilderParameters");
const std::string kUnobsAreObstKey    ("is_unobserved_obstacle");
const std::string kShouldGrowObstKey  ("should_grow_obstacle");
const std::string kGrowthRadiusKey    ("grow_obstacle_radius_m");
const std::string kMaxObstDistKey     ("max_obstacle_distance_m");

const std::string kNavGridHeading             ("NavigationGridBuilderParameters");
const std::string kStaticObstacleKey          ("should_propagate_obstacle_cost");
const std::string kRobotShortRadiusKey        ("robot_short_radius_m");
const std::string kRobotLongRadiusKey         ("robot_long_radius_m");
const std::string kStaticObstacleCostRadiusKey("propagate_cost_radius_m");
const std::string kStaticObstacleCostKey      ("propagated_obstacle_cost");
const std::string kQuasiStaticObjKey          ("should_add_quasi_static_objects");
const std::string kQuasiStaticObjTimeKey      ("quasi_static_prediction_time_s");
const std::string kQuasiStaticObjLengthKey    ("quasi_static_object_length_s");
const std::string kQuasiStaticObjCostRadiusKey("quasi_static_object_cost_radius_m");
const std::string kQuasiStaticObjCostPeakKey  ("quasi_static_object_cost_peak");
const std::string kQuasiStaticObjCostSlopeKey ("quasi_static_object_cost_slope");


obstacle_distance_grid_builder_params_t::obstacle_distance_grid_builder_params_t(const utils::ConfigFile& config)
: isUnobservedObstacle(config.getValueAsBool  (kObstDistGridHeading, kUnobsAreObstKey))
, maxObstacleDistance (config.getValueAsUInt16(kObstDistGridHeading, kMaxObstDistKey))
, shouldGrowObstacle  (config.getValueAsBool  (kObstDistGridHeading, kShouldGrowObstKey))
, growObstacleRadius  (config.getValueAsFloat (kObstDistGridHeading, kGrowthRadiusKey))
{
    assert(maxObstacleDistance >= 3.0f);
    assert(growObstacleRadius  >= 0.0f);
}

navigation_grid_builder_params_t::navigation_grid_builder_params_t(const utils::ConfigFile& config)
: shouldPropagateObstacleCost(config.getValueAsBool (kNavGridHeading, kStaticObstacleKey))
, robotShortRadius           (config.getValueAsFloat(kNavGridHeading, kRobotShortRadiusKey))
, robotLongRadius            (config.getValueAsFloat(kNavGridHeading, kRobotLongRadiusKey))
, staticObstacleCostRadius   (config.getValueAsFloat(kNavGridHeading, kStaticObstacleCostRadiusKey))
, staticObstacleCost         (config.getValueAsFloat(kNavGridHeading, kStaticObstacleCostKey))
, shouldAddQuasiStaticObjects(config.getValueAsBool (kNavGridHeading, kQuasiStaticObjKey))
, quasiStaticTiming          (config.getValueAsFloat(kNavGridHeading, kQuasiStaticObjTimeKey))
, quasiStaticLength          (config.getValueAsFloat(kNavGridHeading, kQuasiStaticObjLengthKey))
, quasiStaticCostRadius      (config.getValueAsFloat(kNavGridHeading, kQuasiStaticObjCostRadiusKey))
, quasiStaticCostPeak        (config.getValueAsFloat(kNavGridHeading, kQuasiStaticObjCostPeakKey))
, quasiStaticCostSlope       (config.getValueAsFloat(kNavGridHeading, kQuasiStaticObjCostSlopeKey))
{
    assert(staticObstacleCostRadius >= 0.0f);
    assert(robotShortRadius > 0.0f);
    assert(robotLongRadius >= robotShortRadius);
    assert(quasiStaticTiming >= 0.0f);
    assert(quasiStaticLength >= 0.0f);
    assert(quasiStaticCostRadius >= 0.0f);
}

} // namespace mpepc
} // namespace vulcan
