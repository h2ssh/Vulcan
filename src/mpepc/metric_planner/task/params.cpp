/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.cpp
* \author   Jong Jin Park and Collin Johnson
*
* Definition of load functions for the params structs for the various task classes.
*/

#include <mpepc/metric_planner/task/params.h>
#include <utils/config_file.h>
#include <utils/config_file_utils.h>
#include <utils/serialized_file_io.h>
#include <cassert>

namespace vulcan
{
namespace mpepc
{

const std::string kNavigationTaskParamsHeading("NavigationTaskParameters");
const std::string kMinAllowedDistFromWalls    ("minimum_allowed_distance_from_walls_m");
const std::string kConvergenceRadiusKey       ("convergence_radius_m");
const std::string kConvergenceAngleKey        ("convergence_angle_rad");
const std::string kShouldUseRRTKey            ("should_use_rrt");
const std::string kGoalNeighborhoodRadiusKey  ("goal_neighborhood_radius");
const std::string kShouldUseDwellTimeKey      ("should_use_dwell_time");
const std::string kDwellTimeWeightKey         ("dwell_time_weight");
const std::string kShouldUseFlowHeuristicKey  ("should_use_orientation_heuristic");
const std::string kAlignToGoalRadiusKey       ("align_to_goal_orientation_radius_m");
const std::string kFlowHeuristicWeightKey     ("orientation_heuristic_weight");
const std::string kShouldUseNHDistNearGoalKey ("should_use_nonholonomic_distance_near_goal");
const std::string kNonholonomicDistKPhiKey    ("nonholonomic_distance_kPhi");
const std::string kNonholonomicDistKDeltaKey  ("nonholonomic_distance_kDelta");
const std::string kNonholonomicHeadingTypeKey ("nonholonomic_distance_reference_heading_type");
const std::string kLearnedNormFileKey         ("learned_norm_filename");


navigation_task_params_t::navigation_task_params_t(const utils::ConfigFile config)
: minimumSafeDistanceFromWalls     (config.getValueAsFloat (kNavigationTaskParamsHeading, kMinAllowedDistFromWalls))
, convergenceRadius                (config.getValueAsFloat (kNavigationTaskParamsHeading, kConvergenceRadiusKey))
, convergenceAngle                 (config.getValueAsFloat (kNavigationTaskParamsHeading, kConvergenceAngleKey))
, shouldUseRRT                     (config.getValueAsBool  (kNavigationTaskParamsHeading, kShouldUseRRTKey))
, goalNeighborhoodRadius           (config.getValueAsDouble(kNavigationTaskParamsHeading, kGoalNeighborhoodRadiusKey))
, shouldUseDwellTime               (config.getValueAsBool  (kNavigationTaskParamsHeading, kShouldUseDwellTimeKey))
, dwellTimeWeight                  (config.getValueAsDouble(kNavigationTaskParamsHeading, kDwellTimeWeightKey))
, shouldUseOrientationHeuristic    (config.getValueAsBool  (kNavigationTaskParamsHeading, kShouldUseFlowHeuristicKey))
, useGoalOrientationRadius         (config.getValueAsDouble(kNavigationTaskParamsHeading, kAlignToGoalRadiusKey))
, orientationHeuristicWeight       (config.getValueAsDouble(kNavigationTaskParamsHeading, kFlowHeuristicWeightKey))
, shouldUseNonholonomicDistNearGoal(config.getValueAsBool  (kNavigationTaskParamsHeading, kShouldUseNHDistNearGoalKey))
, kPhi                             (config.getValueAsDouble(kNavigationTaskParamsHeading, kNonholonomicDistKPhiKey))
, kDelta                           (config.getValueAsDouble(kNavigationTaskParamsHeading, kNonholonomicDistKDeltaKey))
{
    std::string referenceHeadingType = config.getValueAsString(kNavigationTaskParamsHeading, kNonholonomicHeadingTypeKey);
    if(referenceHeadingType == "smooth")
    {
        shouldUseSmoothControlHeading = true;
    }
    else if(referenceHeadingType == "gradient")
    {
        shouldUseSmoothControlHeading = false;
    }
    else
    {
        std::cout<<"Error: TaskParams: Unkown reference heading type.\n";
        assert(false);
    }
}

pacing_task_params_t::pacing_task_params_t(const utils::ConfigFile config)
: havePreferredOrientation(false)
, preferredDistance(2.0)
, preferredOrientation(M_PI/2)
{
}

playground_task_params_t::playground_task_params_t(const utils::ConfigFile config)
{
}

metric_planner_task_params_t::metric_planner_task_params_t(const utils::ConfigFile& config)
: navigationTaskParams(config)
, pacingTaskParams(config)
, playgroundTaskParams(config)
{

}


navigation_task_manifold_params_t::navigation_task_manifold_params_t(const utils::ConfigFile& config)
: shouldBuildRRT(false)
, shouldDebugGrid(true)
, shouldDebugRRT(false)
, navGridBuilderParams(config)
// , rrtBuilderParams(config)
{
    obstacleCostParams.robotRadius = navGridBuilderParams.robotShortRadius;
    obstacleCostParams.obstacleCostRadius = navGridBuilderParams.staticObstacleCostRadius;
    obstacleCostParams.obstacleCost = navGridBuilderParams.staticObstacleCost;

    quasiStaticParams.costRadius = navGridBuilderParams.quasiStaticCostRadius;
    quasiStaticParams.costPeak = navGridBuilderParams.quasiStaticCostPeak;
    quasiStaticParams.costSlope = navGridBuilderParams.quasiStaticCostSlope;

    useLearnedNorm = utils::load_serializable_from_file(config.getValueAsString(kNavigationTaskParamsHeading,
                                                                                kLearnedNormFileKey),
                                                        learnedNormParams);

    std::cout << "Learned norms:\n"
        << "Generic path:  " << learnedNormParams.defaultResponsePath << '\n'
        << "Generic place: " << learnedNormParams.defaultResponsePlace << '\n';
    std::copy(learnedNormParams.responses.begin(),
              learnedNormParams.responses.end(),
              std::ostream_iterator<TopoSituationResponse>(std::cout, "\n"));


}

task_manifold_builder_params_t::task_manifold_builder_params_t(const utils::ConfigFile& config)
: navigationTaskManifoldParams(config)
// , pacingTaskManifoldParams(config)
// , playgroundTaskManifoldParams(config)
{

}


} // namespace planner
} // namespace vulcan
