/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Collin Johnson
*
* Declaration of params structs for local_topo and load_local_toplogy_params() for
* processing a ConfigFile.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_PARAMS_H
#define HSSH_LOCAL_TOPOLOGICAL_PARAMS_H

#include <string>
#include <vector>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace hssh
{

struct skeleton_builder_params_t
{
    float coastalDistance;
    int   maxIslandAreaInCells;
    float minDeadEndAngleSeparation;
};

struct skeleton_pruner_params_t
{
    std::string reducerType;
    float       minExitPointDistance;
};

struct endpoint_validator_params_t
{
    float maxEndpointDistFromEdge;
};

struct isovist_voronoi_gateway_generator_params_t
{
    double closeAngleThresold;
    double minGatewayStraightness;
    double endpointDistanceThreshold;
    bool useSkeletonBasedGateways;
    bool useSourceBasedGateways;

    int numAboveMean;
    bool saveGradientData;
};

struct isovist_orientation_gateway_generator_params_t
{
    int numAboveMean;
    bool saveGradientData;
};

struct classifier_based_generator_params_t
{
    int featureRadius;
    std::string classifierFilename;
};

struct gateway_generator_params_t
{
    isovist_voronoi_gateway_generator_params_t voronoiParams;
    isovist_orientation_gateway_generator_params_t orientationParams;
    classifier_based_generator_params_t classifierParams;
};

struct gateway_locator_params_t
{
    std::string gatewayGeneratorType;

    endpoint_validator_params_t validatorParams;
    gateway_generator_params_t generatorParams;
};

struct beam_star_builder_params_t
{
    float angleDeviationThreshold;
};

struct path_similarity_star_builder_params_t
{
    float ambiguousScorePercent;
    float minimumPathSimilarity;
};

struct small_scale_star_builder_params_t
{
    std::string starBuilderType;

    beam_star_builder_params_t            beamBuilderParams;
    path_similarity_star_builder_params_t similarityBuilderParams;
};

struct hypothesis_evaluator_params_t
{
    std::string boundaryModelFilename;
};

struct area_tracker_params_t
{
    float placeBoundaryOverlap;
    float stableExploredPercent;
    int   minUpdatesForConfirmation;
    int   maxMissesForRemoval;

    float placeBoundaryRadius;
};

/**
* MCMCSamplingParams controls the internal behavior of the sampling algorithm. See the params for a description.
*/
struct MCMCSamplingParams
{
    double failingConstraintLogProb = -3.0;     ///< Probability cost of a failing constraint
    double repeatConfigDecreaseLogProb = -3.0;  ///< How much probability decrease for repeated states

    int maxIterations = 25;            ///< Maximum number of iterations to try to find a solution
    int samplesPerIteration = 25;       ///< Number of change samples to draw per iteration
};

struct area_classifier_params_t
{
    std::string classifierType;

    // Params for StoredMapClassifier
    std::string storedMapFile;

    // Params for AffordanceLabelingClassifier
    small_scale_star_builder_params_t starBuilderParams;
    hypothesis_evaluator_params_t     evaluatorParams;
    area_tracker_params_t             areaTrackerParams;

    double appropriatenessStrainWeight;
    double constraintStrainWeight;

    MCMCSamplingParams mcmcParams;

    std::string classifierFilename;
};

struct area_detector_params_t
{
    std::string skeletonBuilderType;
    bool        shouldBuildSkeleton;
    bool        shouldComputeIsovists;
    bool        shouldFindGateways;

    float maxIsovistRange;
    int numIsovistRays;

    bool shouldSaveIsovistHistograms;
    std::string histogramMapName;

    skeleton_builder_params_t skeletonParams;
    skeleton_pruner_params_t  prunerParams;
    gateway_locator_params_t  locatorParams;
    area_classifier_params_t  classifierParams;

    area_detector_params_t(const utils::ConfigFile& config);
};

struct area_transition_detector_params_t
{
    double minDistBetweenSameGatewayTransitions;
    bool ignoreStaleMapTimestamps;      // flag for use with generating events from a ground-truth map to ignore the
                                        // timestamp check necessary during incremental mapping
};

struct path_direction_detector_params_t
{
    float hysteresisRadius;
};

struct event_detector_params_t
{
    std::vector<std::string> detectorTypes;
    float                    distanceBetweenTracePoses;

    area_transition_detector_params_t transitionParams;
    path_direction_detector_params_t  pathDirectionParams;

    event_detector_params_t(const utils::ConfigFile& config);
};

struct local_topology_params_t
{
    area_detector_params_t  areaParams;
    event_detector_params_t eventParams;

    std::string smallScaleSpaceType;    // what type of SmallScaleSpaceBoundary should be created?

    local_topology_params_t(const utils::ConfigFile& config);
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_PARAMS_H
