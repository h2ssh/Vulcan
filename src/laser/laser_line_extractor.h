/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     laser_line_extractor.h
 * \author   Collin Johnson
 *
 * Declaration of LaserLineExtractor.
 */

#ifndef SENSORS_LASER_LINE_EXTRACTOR_LASER_LINE_EXTRACTOR_H
#define SENSORS_LASER_LINE_EXTRACTOR_LASER_LINE_EXTRACTOR_H

#include "core/laser_scan.h"
#include "laser/line_extractor_params.h"

namespace vulcan
{
namespace sensors
{
struct laser_scan_t;
}

namespace laser
{

struct laser_scan_lines_t;

/**
 * LaserLineExtractor takes a laser scan and extracts all distinguishable lines from the scan,
 * producing a laser_scan_lines_t structure. Multiple line extraction algorithms are available:
 *
 *   split_and_merge    : recursively split lines around point furthest from line, after all lines generated, merge
 * close lines back together quick_split        : essentially split_and_merge without the merge step incremental : start
 * with line of a minimum length, keep increasing length until line model is bad, move on to next line
 *   angle_segmentation : sliding window along scan, once angle between points in window > threshold, make a split
 */
class LaserLineExtractor
{
public:
    /**
     * Constructor for LaserLineExtractor.
     */
    LaserLineExtractor(const laser_line_extractor_params_t& params);

    /**
     * extractLines extracts lines from the provided scan and produces a laser_scan_lines_t structure.
     * The scan field of the laser_scan_lines_t structure will not be modified.
     */
    void extractLines(const polar_laser_scan_t& scan, laser_scan_lines_t& lines);

    /**
     * setExtractionAlgorithm sets the extraction algorithm to be used for generating lines.
     */
    void setExtractionAlgorithm(const std::string& algorithm);

private:
    enum extraction_algorithm_t
    {
        SPLIT_AND_MERGE,
        QUICK_SPLIT,
        INCREMENTAL,
        ANGLE_SEGMENTATION
    };

    void extractorFromString(const std::string& extractionString);   // extractor is specified as a string in the params

    extraction_algorithm_t extractor;

    cartesian_laser_scan_t cartesian;

    laser_line_extractor_params_t params;
};

}   // namespace laser
}   // namespace vulcan

#endif   // SENSORS_LASER_LINE_EXTRACTOR_LASER_LINE_EXTRACTOR_H
