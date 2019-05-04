/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_LASER_LINE_EXTRACTOR_LINE_EXTRACTOR_PARAMS_H
#define SENSORS_LASER_LINE_EXTRACTOR_LINE_EXTRACTOR_PARAMS_H

#include <stdint.h>
#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace laser
{

struct split_and_merge_params_t
{
    float    clusterDistance;
    float    maxDistanceFromLine;
    float    slopeTolerance;
    float    distanceTolerance;
    uint16_t minPoints;
};

struct quick_split_params_t
{
    float    clusterDistance;
    float    maxDistanceFromLine;
    uint16_t minPoints;
};

struct incremental_params_t
{
    uint16_t initialLength;
    uint16_t increment;
    float    correlationThreshold;
    uint16_t minPoints;
};

struct angle_segmentation_params_t
{
    float    thresholdAngleDegrees;
    uint16_t windowSize;
    float    maxPointDistance;
    uint16_t minPoints;
};

struct laser_line_extractor_params_t
{
    std::string extractionAlgorithm;

    split_and_merge_params_t    mergeParams;
    quick_split_params_t        quickParams;
    incremental_params_t        incrementalParams;
    angle_segmentation_params_t angleParams;
};

struct line_extractor_params_t
{
    laser_line_extractor_params_t extractorParams;
};

line_extractor_params_t       load_line_extractor_params      (const utils::ConfigFile& config);
laser_line_extractor_params_t load_laser_line_extractor_params(const utils::ConfigFile& config);

}
}

#endif // SENSORS_LASER_LINE_EXTRACTOR_LINE_EXTRACTOR_PARAMS_H
