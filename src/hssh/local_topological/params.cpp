/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.cpp
* \author   Collin Johnson
*
* Definition of load_local_topology_params and associated ConfigFile processing functions.
*/

#include "utils/config_file.h"
#include "utils/config_file_utils.h"
#include "hssh/local_topological/params.h"
#include <cassert>

namespace vulcan
{
namespace hssh
{

const std::string kModuleHeading("ModuleParameters");
const std::string kSmallScaleKey("small_scale_space_boundary_type");

const std::string AREA_DETECTOR_HEADING    ("AreaDetectorParameters");
const std::string SKELETON_BUILDER_TYPE_KEY("skeleton_builder_type");
const std::string kShouldBuildSkeletonKey  ("should_build_skeleton");
const std::string kShouldComputeIsovists   ("should_compute_isovists");
const std::string kShouldFindGatewaysKey   ("should_find_gateways");
const std::string kMaxIsovistRangeKey      ("max_isovist_range_m");
const std::string kNumIsovistRaysKey       ("num_isovist_rays");
const std::string kShouldSaveHistogramKey  ("should_save_isovist_histogram");
const std::string kHistogramMapNameKey     ("histogram_map_name");

const std::string PRUNER_HEADING       ("SkeletonPrunerParameters");
const std::string REDUCER_KEY          ("reducer_type");
const std::string MIN_EXIT_DISTANCE_KEY("min_exit_point_distance");

const std::string ENDPOINT_VALIDATOR_HEADING("EndpointValidatorParameters");
const std::string MAX_ENDPOINT_DIST_KEY     ("max_endpoint_dist_from_frontier_edge_m");

const std::string ISOVIST_GATEWAYS_HEADING("IsovistVoronoiGatewayGeneratorParameters");
const std::string CLOSE_ANGLE_KEY         ("close_angle_threshold_degrees");
const std::string GATEWAY_STRAIGHTNESS_KEY("min_gateway_straightness_degrees");
const std::string ENDPOINT_DIST_KEY       ("endpoint_distance_threshold_m");
const std::string SOURCE_BASED_KEY        ("use_source_based_gateways");
const std::string SKELETON_BASED_KEY      ("use_skeleton_based_gateways");

const std::string ORIENTATION_GATEWAYS_HEADING("IsovistOrientationGatewayGeneratorParameters");
const std::string NUM_ABOVE_MEAN_KEY      ("num_above_mean");
const std::string SAVE_VISIBILITY_DATA_KEY("save_gradient_data");

const std::string CLASSIFIER_BASED_GATEWAYS_HEADING("ClassifierBasedGeneratorParameters");
const std::string GATEWAY_FEAT_RADIUS_KEY ("gateway_feature_radius");

const std::string GATEWAY_LOCATOR_HEADING("GatewayLocatorParameters");
const std::string GENERATOR_TYPE_KEY("gateway_generator_type");

const std::string SKELETON_BUILDER_HEADING("SkeletonBuilderParameters");
const std::string COASTAL_DISTANCE_KEY    ("coastal_distance");
const std::string ISLAND_AREA_KEY         ("max_island_area_in_cells");
const std::string MIN_DEAD_END_ANGLE_KEY  ("min_dead_end_angle_separation_degrees");

const std::string STAR_BUILDER_HEADING ("SmallScaleStarBuilderParameters");
const std::string STAR_BUILDER_TYPE_KEY("star_builder_type");

const std::string BEAM_BUILDER_HEADING("BeamStarBuilderParameters");
const std::string ANGLE_DEVIATION_KEY ("gateway_angle_deviation_threshold");

const std::string SIMILARITY_BUILDER_HEADING("PathSimilarityStarBuilderParameters");
const std::string AMBIGUOUS_SCORE_KEY       ("ambiguous_score_percent");
const std::string MIN_SIMILARITY_KEY        ("minimum_path_similarity");

const std::string kHypothesisEvaluatorHeading("AreaHypothesisEvaluatorParameters");
const std::string kBoundaryFilenameKey("boundary_model_filename");

const std::string AREA_TRACKER_HEADING("AreaTrackerParameters");
const std::string BOUNDARY_OVERLAP_KEY("place_boundary_overlap");
const std::string EXPLORED_PERCENT_KEY("place_explored_percent");
const std::string MIN_UPDATE_KEY      ("min_updates_for_confirmation");
const std::string MAX_MISSES_KEY      ("max_misses_for_removal");

const std::string kAreaClassifierHeading("AreaClassifierParameters");
const std::string kClassifierTypeKey    ("classifier_type");
const std::string kStoredMapKey         ("stored_map_file");
const std::string kAppropriateStrainKey ("appropriateness_strain_weight");
const std::string kConstraintStrainKey  ("constraint_strain_weight");
const std::string kClassifierFileKey    ("classifier_file");

const std::string EVENT_DETECTOR_HEADING("EventDetectorParameters");
const std::string DETECTOR_TYPES_KEY    ("detector_types");
const std::string POSE_TRACE_DIST_KEY   ("distance_between_trace_poses_m");

const std::string PATH_DIRECTION_HEADING("PathDirectionDetectorParameters");
const std::string HYST_RADIUS_KEY       ("hysteresis_radius");

const std::string kTransitionDetectorHeading("AreaTransitionDetectorParameters");
const std::string kMinTransitionDistKey("min_dist_between_transition_of_same_gateway_m");
const std::string kIgnoreTimestamps("ignore_map_timestamp");


skeleton_builder_params_t                  load_skeleton_builder_params    (const utils::ConfigFile& config);
skeleton_pruner_params_t                   load_pruner_params              (const utils::ConfigFile& config);
gateway_generator_params_t                 load_gateway_generator_params(const utils::ConfigFile& config);
isovist_voronoi_gateway_generator_params_t load_isovist_voronoi_gateways_params(const utils::ConfigFile& config);
isovist_orientation_gateway_generator_params_t load_isovist_orientation_gateways_params(const utils::ConfigFile& config);
classifier_based_generator_params_t        load_classifier_based_gateways_params(const utils::ConfigFile& config);
endpoint_validator_params_t                load_validator_params           (const utils::ConfigFile& config);
gateway_locator_params_t                   load_gateway_locator_params     (const utils::ConfigFile& config);
area_classifier_params_t                   load_area_classifier_params     (const utils::ConfigFile& config);
small_scale_star_builder_params_t          load_star_builder_params        (const utils::ConfigFile& config);
beam_star_builder_params_t                 load_beam_builder_params        (const utils::ConfigFile& config);
path_similarity_star_builder_params_t      load_similarity_builder_params  (const utils::ConfigFile& config);
hypothesis_evaluator_params_t              load_hypothesis_evaluator_params(const utils::ConfigFile& config);
area_tracker_params_t                      load_area_tracker_params        (const utils::ConfigFile& config);
area_transition_detector_params_t          load_area_transition_params     (const utils::ConfigFile& config);
path_direction_detector_params_t           load_path_direction_params      (const utils::ConfigFile& config);


area_detector_params_t::area_detector_params_t(const utils::ConfigFile& config)
: skeletonBuilderType(config.getValueAsString(AREA_DETECTOR_HEADING, SKELETON_BUILDER_TYPE_KEY))
, shouldBuildSkeleton(config.getValueAsBool(AREA_DETECTOR_HEADING, kShouldBuildSkeletonKey))
, shouldComputeIsovists(config.getValueAsBool(AREA_DETECTOR_HEADING, kShouldComputeIsovists))
, shouldFindGateways(config.getValueAsBool(AREA_DETECTOR_HEADING, kShouldFindGatewaysKey))
, maxIsovistRange(config.getValueAsFloat(AREA_DETECTOR_HEADING, kMaxIsovistRangeKey))
, numIsovistRays(config.getValueAsInt32(AREA_DETECTOR_HEADING, kNumIsovistRaysKey))
, shouldSaveIsovistHistograms(config.getValueAsBool(AREA_DETECTOR_HEADING, kShouldSaveHistogramKey))
, histogramMapName(config.getValueAsString(AREA_DETECTOR_HEADING, kHistogramMapNameKey))
, skeletonParams(load_skeleton_builder_params(config))
, prunerParams(load_pruner_params(config))
, locatorParams(load_gateway_locator_params(config))
, classifierParams(load_area_classifier_params(config))
{
    assert(maxIsovistRange > 0.0);
    assert(numIsovistRays > 1);
}


event_detector_params_t::event_detector_params_t(const utils::ConfigFile& config)
: detectorTypes(utils::split_into_strings(config.getValueAsString(EVENT_DETECTOR_HEADING, DETECTOR_TYPES_KEY), ','))
, transitionParams(load_area_transition_params(config))
, pathDirectionParams(load_path_direction_params(config))
{
}


local_topology_params_t::local_topology_params_t(const utils::ConfigFile& config)
: areaParams(config)
, eventParams(config)
, smallScaleSpaceType(config.getValueAsString(kModuleHeading, kSmallScaleKey))
{
    assert(!smallScaleSpaceType.empty());
}


skeleton_builder_params_t load_skeleton_builder_params(const utils::ConfigFile& config)
{
    skeleton_builder_params_t params;

    params.coastalDistance      = config.getValueAsFloat(SKELETON_BUILDER_HEADING, COASTAL_DISTANCE_KEY);
    params.maxIslandAreaInCells = config.getValueAsInt32(SKELETON_BUILDER_HEADING, ISLAND_AREA_KEY);
    params.minDeadEndAngleSeparation = config.getValueAsFloat(SKELETON_BUILDER_HEADING, MIN_DEAD_END_ANGLE_KEY) * M_PI / 180.0f;

    return params;
}


skeleton_pruner_params_t load_pruner_params(const utils::ConfigFile& config)
{
    skeleton_pruner_params_t params;

    params.reducerType = config.getValueAsString(PRUNER_HEADING, REDUCER_KEY);
    params.minExitPointDistance = config.getValueAsFloat(PRUNER_HEADING, MIN_EXIT_DISTANCE_KEY);

    return params;
}


gateway_generator_params_t load_gateway_generator_params(const utils::ConfigFile& config)
{
    gateway_generator_params_t params;

    params.voronoiParams = load_isovist_voronoi_gateways_params(config);
    params.orientationParams = load_isovist_orientation_gateways_params(config);
    params.classifierParams = load_classifier_based_gateways_params(config);

    return params;
}


isovist_voronoi_gateway_generator_params_t load_isovist_voronoi_gateways_params(const utils::ConfigFile& config)
{
    isovist_voronoi_gateway_generator_params_t params;

    params.closeAngleThresold = config.getValueAsDouble(ISOVIST_GATEWAYS_HEADING, CLOSE_ANGLE_KEY) * M_PI / 180.0;
    params.minGatewayStraightness = config.getValueAsDouble(ISOVIST_GATEWAYS_HEADING, GATEWAY_STRAIGHTNESS_KEY)
        * M_PI / 180.0;
    params.endpointDistanceThreshold = config.getValueAsDouble(ISOVIST_GATEWAYS_HEADING, ENDPOINT_DIST_KEY);
    params.useSkeletonBasedGateways = config.getValueAsBool(ISOVIST_GATEWAYS_HEADING, SKELETON_BASED_KEY);
    params.useSourceBasedGateways = config.getValueAsBool(ISOVIST_GATEWAYS_HEADING, SOURCE_BASED_KEY);

    params.numAboveMean = config.getValueAsInt32(ISOVIST_GATEWAYS_HEADING, NUM_ABOVE_MEAN_KEY);
    params.saveGradientData = config.getValueAsBool(ISOVIST_GATEWAYS_HEADING, SAVE_VISIBILITY_DATA_KEY);

    assert(params.numAboveMean > 0);
    assert(params.closeAngleThresold >= 0.0f);
    assert(params.minGatewayStraightness > M_PI_2);
    assert(params.endpointDistanceThreshold >= 0.0f);
    assert(params.useSkeletonBasedGateways || params.useSourceBasedGateways);

    return params;
}


isovist_orientation_gateway_generator_params_t load_isovist_orientation_gateways_params(const utils::ConfigFile& config)
{
    isovist_orientation_gateway_generator_params_t params;

    params.numAboveMean    = config.getValueAsInt32(ORIENTATION_GATEWAYS_HEADING, NUM_ABOVE_MEAN_KEY);
    params.saveGradientData = config.getValueAsBool(ORIENTATION_GATEWAYS_HEADING, SAVE_VISIBILITY_DATA_KEY);

    assert(params.numAboveMean > 0);

    return params;
}


classifier_based_generator_params_t load_classifier_based_gateways_params(const utils::ConfigFile& config)
{
    classifier_based_generator_params_t params;

    params.featureRadius = config.getValueAsInt32(CLASSIFIER_BASED_GATEWAYS_HEADING, GATEWAY_FEAT_RADIUS_KEY);
    params.classifierFilename = config.getValueAsString(CLASSIFIER_BASED_GATEWAYS_HEADING, kClassifierFileKey);

    assert(params.featureRadius >= 0);
    assert(!params.classifierFilename.empty());

    return params;
}


gateway_locator_params_t load_gateway_locator_params(const utils::ConfigFile& config)
{
    gateway_locator_params_t params;

    params.validatorParams = load_validator_params(config);
    params.generatorParams = load_gateway_generator_params(config);
    params.gatewayGeneratorType = config.getValueAsString(GATEWAY_LOCATOR_HEADING, GENERATOR_TYPE_KEY);

    return params;
}


endpoint_validator_params_t load_validator_params(const utils::ConfigFile& config)
{
    endpoint_validator_params_t params;

    params.maxEndpointDistFromEdge = config.getValueAsFloat(ENDPOINT_VALIDATOR_HEADING, MAX_ENDPOINT_DIST_KEY);

    return params;
}


area_classifier_params_t load_area_classifier_params(const utils::ConfigFile& config)
{
    area_classifier_params_t params;
    params.classifierType = config.getValueAsString(kAreaClassifierHeading, kClassifierTypeKey);

    params.storedMapFile = config.getValueAsString(kAreaClassifierHeading, kStoredMapKey);

    params.starBuilderParams = load_star_builder_params(config);
    params.evaluatorParams   = load_hypothesis_evaluator_params(config);
    params.areaTrackerParams = load_area_tracker_params(config);

    params.appropriatenessStrainWeight = config.getValueAsDouble(kAreaClassifierHeading, kAppropriateStrainKey);
    params.constraintStrainWeight = config.getValueAsDouble(kAreaClassifierHeading, kConstraintStrainKey);

    params.classifierFilename = config.getValueAsString(kAreaClassifierHeading, kClassifierFileKey);

    // No negative weights are allowed and at least one weight must be greater than 0 so the search can progress
    assert(params.appropriatenessStrainWeight >= 0.0);
    assert(params.constraintStrainWeight >= 0.0);
    assert(params.appropriatenessStrainWeight + params.constraintStrainWeight > 0.0);

    return params;
}


small_scale_star_builder_params_t load_star_builder_params(const utils::ConfigFile& config)
{
    small_scale_star_builder_params_t params;

    params.starBuilderType = config.getValueAsString(STAR_BUILDER_HEADING, STAR_BUILDER_TYPE_KEY);

    params.beamBuilderParams       = load_beam_builder_params(config);
    params.similarityBuilderParams = load_similarity_builder_params(config);

    return params;
}


beam_star_builder_params_t load_beam_builder_params(const utils::ConfigFile& config)
{
    beam_star_builder_params_t params;

    // config file is in degrees (much easier to specify), but want radians to actually use
    params.angleDeviationThreshold = config.getValueAsFloat(BEAM_BUILDER_HEADING, ANGLE_DEVIATION_KEY) * M_PI/180.0f;

    return params;
}


path_similarity_star_builder_params_t load_similarity_builder_params(const utils::ConfigFile& config)
{
    path_similarity_star_builder_params_t params;

    params.ambiguousScorePercent = config.getValueAsFloat(SIMILARITY_BUILDER_HEADING, AMBIGUOUS_SCORE_KEY);
    params.minimumPathSimilarity = config.getValueAsFloat(SIMILARITY_BUILDER_HEADING, MIN_SIMILARITY_KEY);

    return params;
}


hypothesis_evaluator_params_t load_hypothesis_evaluator_params(const utils::ConfigFile& config)
{
    hypothesis_evaluator_params_t params;

    params.boundaryModelFilename = config.getValueAsString(kHypothesisEvaluatorHeading, kBoundaryFilenameKey);
//     assert(!params.boundaryModelFilename.empty());

    return params;
}


area_tracker_params_t load_area_tracker_params(const utils::ConfigFile& config)
{
    area_tracker_params_t params;

    params.placeBoundaryOverlap      = config.getValueAsFloat(AREA_TRACKER_HEADING, BOUNDARY_OVERLAP_KEY);
    params.stableExploredPercent     = config.getValueAsFloat(AREA_TRACKER_HEADING, EXPLORED_PERCENT_KEY);
    params.minUpdatesForConfirmation = config.getValueAsInt32(AREA_TRACKER_HEADING, MIN_UPDATE_KEY);
    params.maxMissesForRemoval       = config.getValueAsInt32(AREA_TRACKER_HEADING, MAX_MISSES_KEY);

    return params;
}


area_transition_detector_params_t load_area_transition_params(const utils::ConfigFile& config)
{
    area_transition_detector_params_t params;
    params.minDistBetweenSameGatewayTransitions = config.getValueAsDouble(kTransitionDetectorHeading,
                                                                          kMinTransitionDistKey);
    params.ignoreStaleMapTimestamps = config.getValueAsBool(kTransitionDetectorHeading, kIgnoreTimestamps);
    return params;
}


path_direction_detector_params_t load_path_direction_params(const utils::ConfigFile& config)
{
    path_direction_detector_params_t params;
    params.hysteresisRadius = config.getValueAsFloat(PATH_DIRECTION_HEADING, HYST_RADIUS_KEY);
    return params;
}

} // namespace hssh
} // namespace vulcan
