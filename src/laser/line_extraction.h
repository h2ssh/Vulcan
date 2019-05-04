/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef LASER_LINE_EXTRACTION_H
#define LASER_LINE_EXTRACTION_H

#include <stdint.h>
#include <vector>
#include <core/line.h>

/**
* \file     line_extraction.h
*   line_extraction.h contains a series of functions that can be used for extracting lines from a set of laser data.
*/

namespace vulcan
{
struct cartesian_laser_scan_t;
struct polar_laser_scan_t;

namespace laser
{

struct split_and_merge_params_t;
struct quick_split_params_t;
struct incremental_params_t;
struct angle_segmentation_params_t;

/*
* NOTE: Of the below algorithms, only quick_split is functioning properly. The others all have
*       issues that make them useless. quick_split works well and fast though, which is why it
*       ends up being used. You'll have to do more work if it isn't providing satisfactory results.
*/


struct extracted_lines_t
{
    extracted_lines_t(size_t numPoints = 0) :
                indices(numPoints)
    {
    }

    std::vector<Line<float>> lines;
    std::vector<int16_t>           indices;
};


/**
* split_and_merge applies the split-and-merge algorithm to a set of points to extract the best-fit lines
* from the data. Split-and-merge performs very fast and is thus a good choice for accurate line extraction.
*
* The output lines are in Cartesian coordinates in the same reference frame as the provided lidar data.
*
* \param    data            Lidar data from which to extract the lines
* \param    params          Paramters for the line extraction
* \param    lines           Lines found in the data (output)
* \param    clusters        Clusters found in the data (output)
*/
extracted_lines_t split_and_merge(const polar_laser_scan_t& scan, const cartesian_laser_scan_t& cartesian, const split_and_merge_params_t& params);

/**
* quick_split uses a faster version of the line split that only calculates line parameters after the lines have been extracted.
* quick_split is very similar to split-and-merge. The key difference is that a regression is not calculated for each set of points,
* but rather the line is assumed to run from the first to last point in the set, thus removing a large chunk of the calculation time.
*
* \param    data                LidarData containing the lines to be extracted
* \param    clusterDist         Distance threshold for the initial clustering
* \param    threshold           Distance threshold for a point from the line
* \param    tolerance           Variation in line slope allowed for lines to be considered one line
* \param    lines               Lines found by split-and-merge
*/
extracted_lines_t quick_split(const polar_laser_scan_t& scan, const cartesian_laser_scan_t& cartesian, const quick_split_params_t& params);

/**
* incremental applies the incremental algorithm to a set of points to extract the best-fit lines from
* the data. The incremental algorithm produces accurate results, but is not quite as fast as the
* split_and_merge algorithm.
*
* \param    coords          Coordinates from which the lines will be extracted
* \param    threshold       Minimum correlation for a line to be accepted as the real deal
* \param    minPoints       Minimum number of points for a line to be considered valid
* \param    lines           Lines found in the data (output)
*/
extracted_lines_t incremental(const polar_laser_scan_t& scan, const cartesian_laser_scan_t& cartesian, const incremental_params_t& params);

/**
* angle_segmentation breaks the scan up into regions based on the angles formed between the consecutive points.
*
* The approach used is to consider three consecutive data points. The points will be considered part of the same segment
* if the angle that they form is within some tolerance of a perfectly straight line.
*
* If a point in the window is beyond the maximum distance, then a split will be placed at that position.
*
* \param    coords              Coordinates to be segmented
* \param    angle               Angle threshold to consider
* \param    splitPoints         Indices at which the splits exist in the provided data (output)
* \param    maxDist             Maximum distance for a point before it is considered to be erroneous (meters)
*/
extracted_lines_t angle_segmentation(const polar_laser_scan_t& scan, const cartesian_laser_scan_t& cartesian, const angle_segmentation_params_t& params);

/** filter_lines filters the lines based on length. */
void filter_lines(extracted_lines_t& lines, double minLength);


}
}

#endif // LASER_LINE_EXTRACTION_H
