/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     laser_line_extractor.cpp
 * \author   Collin Johnson
 *
 * Definition of LaserLineExtractor.
 */

#include "laser/laser_line_extractor.h"
#include "core/laser_scan.h"
#include "laser/laser_scan_lines.h"
#include "laser/line_extraction.h"
#include "laser/line_extractor_params.h"
#include <iostream>
#include <string>


namespace vulcan
{
namespace laser
{

const std::string SPLIT_AND_MERGE_NAME("split_and_merge");
const std::string QUICK_SPLIT_NAME("quick_split");
const std::string INCREMENTAL_NAME("incremental");
const std::string ANGLE_SEGMENTATION_NAME("angle_segmentation");


LaserLineExtractor::LaserLineExtractor(const laser_line_extractor_params_t& params) : params(params)
{
    extractorFromString(params.extractionAlgorithm);
}


void LaserLineExtractor::extractLines(const polar_laser_scan_t& scan, laser_scan_lines_t& lines)
{
    extracted_lines_t extracted;

    polar_scan_to_cartesian_scan_in_robot_frame(scan, cartesian, true);

    switch (extractor) {
    case SPLIT_AND_MERGE:
        extracted = split_and_merge(scan, cartesian, params.mergeParams);
        break;

    case QUICK_SPLIT:
        extracted = quick_split(scan, cartesian, params.quickParams);
        break;

    case INCREMENTAL:
        extracted = incremental(scan, cartesian, params.incrementalParams);
        break;

    case ANGLE_SEGMENTATION:
        extracted = angle_segmentation(scan, cartesian, params.angleParams);
        break;
    }

    lines.lines = extracted.lines;
    lines.scanPointToLineIndices = extracted.indices;
}


void LaserLineExtractor::setExtractionAlgorithm(const std::string& algorithm)
{
    extractorFromString(algorithm);
}


void LaserLineExtractor::extractorFromString(const std::string& extractionString)
{
    if (extractionString == SPLIT_AND_MERGE_NAME) {
        extractor = SPLIT_AND_MERGE;
    } else if (extractionString == QUICK_SPLIT_NAME) {
        extractor = QUICK_SPLIT;
    } else if (extractionString == INCREMENTAL_NAME) {
        extractor = INCREMENTAL;
    } else if (extractionString == ANGLE_SEGMENTATION_NAME) {
        extractor = ANGLE_SEGMENTATION;
    } else {
        std::cout << "ERROR! LaserLineExtractor: Invalid line extraction algorithm: " << extractionString
                  << ". Using default:" << QUICK_SPLIT_NAME << '\n';

        extractor = QUICK_SPLIT;
    }
}

}   // namespace laser
}   // namespace vulcan
