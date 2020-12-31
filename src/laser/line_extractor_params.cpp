/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "utils/config_file.h"
#include "laser/line_extractor_params.h"


namespace vulcan
{
namespace laser
{

const std::string LINE_EXTRACTOR_HEADING("LaserLineExtractorParameters");
const std::string ALGORITHM_KEY("extraction_algorithm");

// Clump the parameters for the various line extraction algorithms together because a decent number are
// shared between the different algorithms
const std::string SPLIT_AND_MERGE_HEADING("SplitAndMergeParameters");
const std::string QUICK_SPLIT_HEADING("QuickSplitParameters");
const std::string INCREMENTAL_HEADING("IncrementalParameters");
const std::string ANGLE_SEGMENTATION_HEADING("AngleSegmentation");

const std::string CLUSTER_DIST_KEY("cluster_distance");
const std::string MAX_LINE_DIST_KEY("max_distance_from_line");
const std::string SLOPE_TOL_KEY("slope_tolerance");
const std::string DIST_TOL_KEY("distance_tolerance");
const std::string MIN_POINTS_KEY("min_points");
const std::string INITIAL_LENGTH_KEY("initial_length");
const std::string INCREMENT_KEY("increment");
const std::string CORR_THRESH_KEY("correlation_threshold");
const std::string ANGLE_THRESH_KEY("angle_threshold_degrees");
const std::string WINDOW_SIZE_KEY("window_size");
const std::string MAX_POINT_DIST_KEY("max_point_distance");


split_and_merge_params_t    load_split_and_merge_params(const vulcan::utils::ConfigFile& config);
quick_split_params_t        load_quick_split_params(const vulcan::utils::ConfigFile& config);
incremental_params_t        load_incremental_params(const vulcan::utils::ConfigFile& config);
angle_segmentation_params_t load_angle_segmentation_params(const vulcan::utils::ConfigFile& config);


line_extractor_params_t load_line_extractor_params(const vulcan::utils::ConfigFile& config)
{
    line_extractor_params_t params;

    params.extractorParams = load_laser_line_extractor_params(config);

    return params;
}


laser_line_extractor_params_t load_laser_line_extractor_params(const vulcan::utils::ConfigFile& config)
{
    laser_line_extractor_params_t params;

    params.extractionAlgorithm = config.getValueAsString(LINE_EXTRACTOR_HEADING, ALGORITHM_KEY);

    params.mergeParams       = load_split_and_merge_params(config);
    params.quickParams       = load_quick_split_params(config);
    params.incrementalParams = load_incremental_params(config);
    params.angleParams       = load_angle_segmentation_params(config);

    return params;
}


split_and_merge_params_t load_split_and_merge_params(const vulcan::utils::ConfigFile& config)
{
    split_and_merge_params_t params;

    params.clusterDistance     = config.getValueAsFloat(SPLIT_AND_MERGE_HEADING, CLUSTER_DIST_KEY);
    params.maxDistanceFromLine = config.getValueAsFloat(SPLIT_AND_MERGE_HEADING, MAX_LINE_DIST_KEY);
    params.slopeTolerance      = config.getValueAsFloat(SPLIT_AND_MERGE_HEADING, SLOPE_TOL_KEY);
    params.distanceTolerance   = config.getValueAsFloat(SPLIT_AND_MERGE_HEADING, DIST_TOL_KEY);
    params.minPoints           = config.getValueAsUInt16(SPLIT_AND_MERGE_HEADING, MIN_POINTS_KEY);

    return params;
}


quick_split_params_t load_quick_split_params(const vulcan::utils::ConfigFile& config)
{
    quick_split_params_t params;

    params.clusterDistance     = config.getValueAsFloat(QUICK_SPLIT_HEADING, CLUSTER_DIST_KEY);
    params.maxDistanceFromLine = config.getValueAsFloat(QUICK_SPLIT_HEADING, MAX_LINE_DIST_KEY);
    params.minPoints           = config.getValueAsUInt16(QUICK_SPLIT_HEADING, MIN_POINTS_KEY);

    return params;
}


incremental_params_t load_incremental_params(const vulcan::utils::ConfigFile& config)
{
    incremental_params_t params;

    params.initialLength        = config.getValueAsUInt16(INCREMENTAL_HEADING, INITIAL_LENGTH_KEY);
    params.increment            = config.getValueAsUInt16(INCREMENTAL_HEADING, INCREMENT_KEY);
    params.correlationThreshold = config.getValueAsFloat(INCREMENTAL_HEADING, CORR_THRESH_KEY);
    params.minPoints            = config.getValueAsUInt16(INCREMENTAL_HEADING, MIN_POINTS_KEY);

    return params;
}


angle_segmentation_params_t load_angle_segmentation_params(const vulcan::utils::ConfigFile& config)
{
    angle_segmentation_params_t params;

    params.thresholdAngleDegrees = config.getValueAsFloat(ANGLE_SEGMENTATION_HEADING, ANGLE_THRESH_KEY);
    params.windowSize            = config.getValueAsUInt16(ANGLE_SEGMENTATION_HEADING, WINDOW_SIZE_KEY);
    params.maxPointDistance      = config.getValueAsFloat(ANGLE_SEGMENTATION_HEADING, MAX_POINT_DIST_KEY);
    params.minPoints             = config.getValueAsUInt16(ANGLE_SEGMENTATION_HEADING, MIN_POINTS_KEY);

    return params;
}

} // namespace laser
} // namespace vulcan
