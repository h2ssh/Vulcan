/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "laser/line_extraction.h"
#include "core/angle_functions.h"
#include "core/laser_scan.h"
#include "core/point.h"
#include "laser/line_extractor_params.h"
#include "math/regression.h"
#include "utils/minmax.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

//#define DEBUG_SPLIT
// #define DEBUG_QUICK
#define DEBUG_INCR
//#define DEBUG_FILTER
// #define DEBUG_CLUSTER
// #define DEBUG_TLS
// #define DEBUG_SLS
//#define DEBUG_MERGE
//#define DEBUG_BREAK


using vulcan::Line;
using vulcan::Point;
using vulcan::laser::extracted_lines_t;


/** Struct to represent lines and their set for the extraction algorithms. */
struct PointSet
{
    /** Constructor for PointSet. */
    PointSet(std::pair<size_t, size_t> points) : set(points) { }

    /** Constructor for PointSet. */
    PointSet(std::pair<size_t, size_t> points, const Line<float>& line) : set(points), line(line) { }

    std::pair<size_t, size_t> set;   ///< The set of points represented by the PointSet
    Line<float> line;                ///< Line fit to this set of points
    int clusterNum;                  ///< Initial cluster in which the set was created
};


/** Helper to establish an initial set of clusters for the various line extraction algorithms. */
void initial_clustering(const std::vector<Point<float>>& points, double dist, std::vector<PointSet>& sets);

/** Helper to perform radial clustering using polar lidar data. */
void initial_clustering_polar(const vulcan::polar_laser_scan_t& scan, double distDiff, std::vector<PointSet>& sets);
void split_set(PointSet& set, const std::vector<Point<float>>& points, double distThresh, std::vector<PointSet>& split);
int max_two_above(const std::vector<Point<float>>& points,
                  int begin,
                  int end,
                  const Line<float>& line,
                  double threshold);
void merge_sets(const std::vector<PointSet>& sets,
                const std::vector<Point<float>>& coords,
                double slopeThresh,
                double distThresh,
                std::vector<PointSet>& merged);
void filter_sets(const std::vector<PointSet>& sets,
                 const std::vector<Point<float>>& coords,
                 uint16_t minPoints,
                 double minLength,
                 std::vector<Line<float>>& lines);

/** lines_from_segments creates Lines from segments in raw lidar data. */
extracted_lines_t
  lines_from_segments(const std::vector<Point<float>>& points, const std::vector<int>& segments, int minPoints);

/** Helper to convert a set of PointSets into a set of lines. */
void build_lines(std::vector<PointSet>& sets,
                 const std::vector<Point<float>>& coords,
                 uint16_t minPoints,
                 vulcan::laser::extracted_lines_t& extracted);

/** Helper to convert a PointSet into a line. */
Line<float> build_line(const PointSet& set, const std::vector<Point<float>>& coords);

/** Helper for converting to radians. */
inline double to_radians(double degs)
{
    return degs * M_PI / 180.0;
}


// Split-and-merge implementation
extracted_lines_t vulcan::laser::split_and_merge(const polar_laser_scan_t& scan,
                                                 const cartesian_laser_scan_t& cartesian,
                                                 const split_and_merge_params_t& params)
{
    std::vector<Point<float>> coords(cartesian.numPoints);
    std::copy(cartesian.scanPoints.begin(), cartesian.scanPoints.end(), coords.begin());

    std::vector<PointSet> pointSets;   // for the initial clustering, then can be reused after splitting because the
                                       // splits are the new official spots

    initial_clustering_polar(scan, params.clusterDistance, pointSets);

    std::vector<PointSet> splitSets;

    for (int x = 0, end = pointSets.size(); x < end; ++x) {
        split_set(pointSets[x], coords, params.maxDistanceFromLine, splitSets);
    }

    pointSets.clear();
    merge_sets(splitSets, coords, params.slopeTolerance, params.distanceTolerance, pointSets);

    extracted_lines_t extracted(coords.size());
    build_lines(pointSets, coords, params.minPoints, extracted);

    return extracted;
}


void initial_clustering_polar(const vulcan::polar_laser_scan_t& scan, double distDiff, std::vector<PointSet>& sets)
{
    int curCluster = 0;

    PointSet curSet(std::make_pair(0, 0));

    for (int x = 1; x < scan.numRanges; ++x) {
        if (fabs(static_cast<float>(scan.ranges[x - 1] - scan.ranges[x])) > distDiff) {
            curSet.set.second = x;
            curSet.clusterNum = curCluster++;
            sets.push_back(curSet);

            curSet.set.first = x;
            curSet.set.second = x;
        }
    }

    // Slap in the final set
    curSet.set.second = scan.numRanges;
    curSet.clusterNum = curCluster;
    sets.push_back(curSet);
}


void split_set(PointSet& set, const std::vector<Point<float>>& points, double distThresh, std::vector<PointSet>& split)
{
    /*
     * The split should proceed recursively as follows:
     *
     * 0) Build line from the set.
     * 1) Find the split pos (if there is one)
     * 2) Recursively call split_set on each new set. The first called should be the set with the lower start index to
     * ensure that the ordering of the sets remains monotonic 3) Base case: set can't be split, so add it and return.
     */

    set.line = build_line(set, points);

    int splitPos = max_two_above(points, set.set.first, set.set.second, set.line, distThresh);

    if (splitPos > 0) {
        PointSet newSet(std::make_pair(set.set.first, splitPos));
        split_set(newSet, points, distThresh, split);

        newSet = PointSet(std::make_pair(splitPos, set.set.second));
        split_set(newSet, points, distThresh, split);
    } else {
        split.push_back(set);
    }
}


int max_two_above(const std::vector<Point<float>>& points,
                  int begin,
                  int end,
                  const Line<float>& line,
                  double threshold)
{
    /*
     * max_two_above finds a place where two consecutive values sit outside the maximum distance from line
     * threshold. The two values there are the maximum of all such pairs is returned.
     *
     * For this algorithm, picture a window sliding across the points. The window that contains the two highest
     * points wins the day.
     */

    double maxDist = 0;
    double curDist = 0;
    double nextDist = 0;
    int maxPos = -1;

    for (int x = begin; x < end - 1; ++x) {
        nextDist = vulcan::distance_to_line(points[x + 1], line);

        // Now if nextDist > threshold and cur and next have the same sign -- in this case, there's a new max value in
        // town
        if ((fabs(curDist) > threshold) && (fabs(nextDist) > threshold) && (nextDist * curDist > 0)
            && (curDist > maxDist)) {
            maxDist = curDist;
            maxPos = x;
        }

        curDist = nextDist;
    }

    return maxPos;
}


void merge_sets(const std::vector<PointSet>& sets,
                const std::vector<Point<float>>& coords,
                double slopeThresh,
                double distThresh,
                std::vector<PointSet>& merged)
{
    /*
     * To merge want to see the following are true:
     *
     * 0) Slope of the line is very similar
     * 1) Distance between the lines is close -- for this, take point on one line and find projection to other line,
     * then look at the distance of projection vector
     *
     * Continue merging the same line until no longer can be merged -- duh! The lines here are in monotonic ascending
     * order as received by the laser scanner which makes the merging process simpler.
     */

    if (sets.empty()) {
        return;
    }

    PointSet curSet(sets[0].set);
    size_t mergeStart = sets[0].set.first;   // start of region to merge

    for (int x = 1, end = sets.size(); x < end; ++x) {
        // Check if either endpoint is close enough to be thorough
        if (fabs(vulcan::slope(sets[x - 1].line) - vulcan::slope(sets[x].line)) > slopeThresh
            || (fabs(vulcan::distance_to_line(sets[x].line.a, sets[x - 1].line) > distThresh)
                && fabs(vulcan::distance_to_line(sets[x].line.b, sets[x - 1].line) > distThresh))) {
            curSet.set.first = mergeStart;
            curSet.set.second = sets[x - 1].set.second;
            // Need to create a new line for the freshly merged region
            curSet.line = build_line(curSet, coords);

            merged.push_back(curSet);

            // The start of the next merged region is the start of the next set that wasn't merged
            mergeStart = sets[x].set.first;
        }
        // Otherwise, the segment is merged with the next, so ignore it and carry on
    }

    // Need to create the final cluster because it fit in with those before
    curSet.set.first = mergeStart;
    curSet.set.second = sets[sets.size() - 1].set.second;

    merged.push_back(curSet);
}


// Quick split implementation
extracted_lines_t vulcan::laser::quick_split(const polar_laser_scan_t& scan,
                                             const cartesian_laser_scan_t& cartesian,
                                             const quick_split_params_t& params)
{
    /*
     * The steps for the quick-split are:
     *
     * 1) Convert to Cartesian coordinates.
     * 2) The first line set goes from the start to end.
     * 3) Find point with greatest distance from the line and split there.
     * 4) Continue splitting until no points can be split greater than the threshold.
     * 5) After all lines are split, merge them based on the threshold and tolerance values.
     * 6) Build better models of the line via total_least_squares for all the lines remaining.
     */

    std::vector<PointSet> points;

    const std::vector<Point<float>>& coords = cartesian.scanPoints;

    initial_clustering_polar(scan, params.clusterDistance, points);

#ifdef DEBUG_QUICK
    std::cout << "quick_split: Number of initial clusters: " << points.size() << '\n';
#endif

    int splitPos = -1;   // Position of the split for the current line

    for (size_t n = 0; n < points.size(); ++n) {
        // Grab the next set
        PointSet& curSet = points[n];

        // If the set is less than the minimum number of points, don't investigate, as it is already going
        // to be filtered out, so the effort will be wasted
        if (curSet.set.second - curSet.set.first < params.minPoints) {
            continue;
        }

        // Set the line for the set of points
        curSet.line.a = coords[curSet.set.first];
        curSet.line.b = coords[curSet.set.second - 1];
        splitPos = max_two_above(coords, curSet.set.first, curSet.set.second, curSet.line, params.maxDistanceFromLine);

#ifdef DEBUG_QUICK
        std::cout << "quick_split: Current line: " << curSet.line << '\n';
#endif

        // Split point was found
        if (splitPos > 0) {
            std::pair<size_t, size_t> firstSet(curSet.set.first, splitPos + 1);
            std::pair<size_t, size_t> secondSet(splitPos, curSet.set.second);

            points.push_back(PointSet(firstSet));
            points.push_back(PointSet(secondSet));
            points.erase(points.begin() + n);

            --n;   // decrease n because the point set under consideration has been removed, so the next
                   // set to consider has migrated toward the front of the vector

#ifdef DEBUG_QUICK
            std::cout << "quick_split: Split position: " << splitPos << '\n';
#endif
        }
    }

#ifdef DEBUG_QUICK
    std::cout << "quick_split: Lines:\n";
    for (std::vector<PointSet>::const_iterator lineIt = points.begin(), endIt = points.end(); lineIt != endIt;
         ++lineIt) {
        std::cout << "Scan points: " << lineIt->set.first << " -> " << lineIt->set.second << " Line: " << lineIt->line
                  << '\n';
    }
#endif

    extracted_lines_t extracted(coords.size());
    build_lines(points, cartesian.scanPoints, params.minPoints, extracted);

    return extracted;
}


// Incremental extraction implementation
extracted_lines_t vulcan::laser::incremental(const polar_laser_scan_t& scan,
                                             const cartesian_laser_scan_t& cartesian,
                                             const incremental_params_t& params)
{
    /*
     * The incremental approach to line extraction processes lines in an ever-growing configuration.
     *
     * 1) Construct a line from the first two points.
     * 2) Add new 5 points to the line model.
     * 3) Recomputer line parameters.
     * 4) If some line condition is satisfied, go to 2.
     * 5) Otherwise, return the newly added points and save the previous line.
     * 6) Use the next two points, and go to 1.
     */

    const std::vector<Point<float>>& coords = cartesian.scanPoints;

    std::vector<PointSet> pointSets;

    double r = 0;         // Correlation value for the set of points
    size_t tempEnd = 0;   // Temporary end to the current set

    PointSet currentSet(std::make_pair(0, params.initialLength));

    // Loop until all the point are accounted for
    while (currentSet.set.second < coords.size()) {
        // Expand the current set
        tempEnd = min(currentSet.set.second + params.increment, coords.size());

        while (tempEnd > currentSet.set.second) {
            // Correlation defines whether the line is valid or not, if points are well-correlated, then all is well
            // with the fit
            r = math::correlation(coords.begin() + currentSet.set.first, coords.begin() + tempEnd);

#ifdef DEBUG_INCR
            std::cout << "incremental: Correlation among the points: " << r
                      << " Number of points: " << (tempEnd - currentSet.set.first) << '\n';
#endif

            // Line is valid, so keep expanding it
            if (fabs(r) > params.correlationThreshold) {
                break;
            }

            --tempEnd;
        }

        // If tempEnd < INCREMENT, then reached the end of the line, so create a new line and carry on, otherwise, loop
        // again
        if (tempEnd < currentSet.set.second + params.increment) {
            currentSet.set.second = tempEnd;

            pointSets.push_back(currentSet);

            currentSet.set.first = currentSet.set.second;
            currentSet.set.second = currentSet.set.second + params.initialLength;

#ifdef DEBUG_INCR
            std::cout << "incremental: Added line:" << currentSet.set.first << "->" << currentSet.set.second
                      << " r:" << r << '\n';
#endif
        } else   // Otherwise, shift the end point and keep going
        {
            currentSet.set.second = tempEnd;
        }
    }

    extracted_lines_t extracted(coords.size());
    build_lines(pointSets, coords, params.minPoints, extracted);

    return extracted;
}


// Angle segmentation implementation
extracted_lines_t vulcan::laser::angle_segmentation(const polar_laser_scan_t& scan,
                                                    const cartesian_laser_scan_t& cartesian,
                                                    const angle_segmentation_params_t& params)
{
    /*
     * The strategy here is to look at the angle between vectors in the coords. If the angle is less than the
     * provided threshold angle, then a split point will be added. However, it is likely that for any given corner,
     * there will be multiple locations that could be validly split, so the location with the greatest angle will be
     * used so that the decision will hopefully be the best possible location. To do this, track when a violation has
     * occurred, save that point, then keep going, once the angles start decreasing, then you have moved beyond the
     * split point, so
     */

    const std::vector<Point<float>>& coords = cartesian.scanPoints;

    // Need to check the distance from PI / 2 for the angles as the angle found is [0, PI], but I want only the curvy
    // spots
    double threshAng = std::abs(angle_diff(PI_F, params.thresholdAngleDegrees));
    double curAngle = 0;
    double nextAngle = 0;
    double minAngle = M_PI;
    float curDist = 0;
    bool needSplit = false;

    std::vector<int> splitPoints;

    for (size_t x = params.windowSize, end = coords.size() - params.windowSize; x < end; ++x) {
        curAngle = angle_between_points(coords[x - params.windowSize], coords[x + params.windowSize], coords[x]);
        nextAngle = angle_between_points(coords[x - params.windowSize], coords[x + params.windowSize - 1], coords[x]);

        // If the current is less than max distance and the next point is greater than max dist, then a break should
        // happen Or if the current is greater than max distance and the next is less than max dist, then a break should
        // also happen
        curDist = distance_between_points(coords[x], coords[x + 1]);

        if (curDist > params.maxPointDistance) {
            splitPoints.push_back(x);
            minAngle = M_PI;
            needSplit = false;
        } else if (((curAngle < threshAng) && (nextAngle < threshAng))
                   || needSplit)   // need to make a split here -- a split is made by adding the current position to the
                                   // list of segments
        {
            if (curAngle > minAngle)   // need to split at the previous position
            {
                splitPoints.push_back(x - 1);
                minAngle = M_PI;
                needSplit = false;

                // Increment x beyond the window size from the previous split because otherwise you'll get some double
                // splits to happen where the max angle was already encountered, but you haven't finished splitting the
                // rest of the data
                x += params.windowSize - 1;
            } else {
                minAngle = curAngle;
                needSplit = true;
            }
        }
    }

    splitPoints.push_back(coords.size() - 1);

    return lines_from_segments(coords, splitPoints, params.minPoints);
}


extracted_lines_t
  lines_from_segments(const std::vector<Point<float>>& points, const std::vector<int>& segments, int minPoints)
{
    // lines_from_segments creates Lines from segments in raw lidar data. The provided segments come from the divisions
    // created by angle_segmentation().
    extracted_lines_t extracted(points.size());

    int last = 0;

    for (size_t x = 0; x < segments.size(); ++x) {
        if (segments[x] - last > minPoints) {
            extracted.lines.push_back(
              vulcan::math::total_least_squares(points.begin() + last, points.begin() + segments[x] + 1));

            std::fill(extracted.indices.begin() + last,
                      extracted.indices.begin() + segments[x] + 1,
                      extracted.lines.size() - 1);
        } else {
            std::fill(extracted.indices.begin() + last, extracted.indices.begin() + segments[x] + 1, -1);
        }

        last = segments[x];
    }

    return extracted;
}


/**
 * filter_lines filters out lines that don't meet some minimum length criteria.
 *
 * \param    lines               Lines to be filtered
 * \param    minLength           Minimum length of a valid line
 */
void vulcan::laser::filter_lines(extracted_lines_t& lines, double minLength)
{
    // Filter based on line length, real quick
    //     for(size_t x = 0; x < lines.size();)
    //     {
    //         #ifdef DEBUG_FILTER
    //         std::cout<<"filter_lines: Line: "<<lines[x].line<<" Length: "<<sqrt(pow(lines[x].line.a.x -
    //         lines[x].line.b.x, 2) + pow(lines[x].line.a.y - lines[x].line.b.y, 2))<<" Num points:
    //         "<<lines[x].numPoints
    //                 <<" Density: "<<(sqrt(pow(lines[x].line.a.x - lines[x].line.b.x, 2) + pow(lines[x].line.a.y -
    //                 lines[x].line.b.y, 2)) / lines[x].numPoints)<<'\n';
    //         #endif
    //
    //         if(sqrt(pow(lines[x].line.a.x - lines[x].line.b.x, 2) + pow(lines[x].line.a.y - lines[x].line.b.y, 2)) <
    //         minLength)
    //         {
    //             lines.erase(lines.begin() + x);
    //         }
    //         else
    //         {
    //             ++x;
    //         }
    //     }
}


/**
 * initial_clustering finds an initial set of clusters for the provided points. The clustering is done
 * conservatively so as to not break any potential lines apart.
 *
 * \param    points              Points to be clustered
 * \param    distance            Maximum distance between two points for them to be placed in the same cluster
 * \param    sets                PointSets containing the initial clusters (output)
 */
void initial_clustering(const std::vector<Point<float>>& points, double distance, std::vector<PointSet>& sets)
{
    /*
     * For the clustering, start with the first point in the points vector as comprising the entire
     * set. Then scan through the remaining points one at a time and add them to the current cluster
     * if they are within the threshold range.
     */

    PointSet curSet(std::make_pair(0, 0));
    int clusterNum = 0;

    for (int x = 1, end = points.size(); x < end; ++x) {
        // If the new point is too far from the current point, then end the set and start a new one
        if (distance < (sqrt(pow(points[curSet.set.second].x - points[x].x, 2)
                             + pow(points[curSet.set.second].y - points[x].y, 2))
                        / (sqrt(points[x].x * points[x].x + points[x].y * points[x].y)))) {
            curSet.set.second = x;
            curSet.clusterNum = clusterNum++;
            sets.push_back(curSet);

            curSet.set.first = x;
            curSet.set.second = x;
        }
    }

    // Add the final set to the list of sets
    curSet.set.second = points.size();
    curSet.clusterNum = clusterNum;
    sets.push_back(curSet);
}


/**
 * filter_sets goes through the sets to see which contain valid data and which should be axed. A valid set is both long
 * enough and contains enough points.
 *
 * \param    sets                Sets to be filtered
 * \param    coords              Coordinates from which the sets were culled
 * \param    minPoints           Min number of points for a valid line
 * \param    minLength           Min length for a valid line
 * \param    lines               Lines that were extracted from the sets (output)
 * \param    clusters            Clusters pulled from the sets (output)
 */
void filter_sets(const std::vector<PointSet>& sets,
                 const std::vector<Point<float>>& coords,
                 size_t minPoints,
                 double minLength,
                 std::vector<Line<float>>& lines)
{
    // To filter the sets, go through and see that they have enough points and are longer enough -- if not, then make
    // them into point clusters

    for (int x = 0, end = sets.size(); x < end; ++x) {
        if ((sets[x].set.second - sets[x].set.first) >= minPoints || vulcan::length(sets[x].line) >= minLength) {
            lines.push_back(sets[x].line);   // otherwise, this line is good to go, so let 'er rip!
        }
    }
}


/**
 * build_lines builds a series of lines from the parameters of the provided PointSets.
 *
 * \param    sets                Point sets describing the lines
 * \param    coords              Coordinates of the point that will comprise the lines
 * \param    lines               Lines extracted from the sets (output)
 */
void build_lines(std::vector<PointSet>& sets,
                 const std::vector<Point<float>>& coords,
                 uint16_t minPoints,
                 vulcan::laser::extracted_lines_t& extracted)
{
    int lineIndex = 0;

    for (std::vector<PointSet>::const_iterator setIt = sets.begin(), endIt = sets.end(); setIt != endIt; ++setIt) {
        if (setIt->set.second - setIt->set.first > minPoints) {
            extracted.lines.push_back(
              vulcan::math::total_least_squares(coords.begin() + setIt->set.first, coords.begin() + setIt->set.second));
            //             extracted.lines.push_back(Line<float>(coords[setIt->set.first],
            //             coords[setIt->set.second-1]));

            std::fill(extracted.indices.begin() + setIt->set.first,
                      extracted.indices.begin() + setIt->set.second,
                      lineIndex++);
        } else {
            std::fill(extracted.indices.begin() + setIt->set.first, extracted.indices.begin() + setIt->set.second, -1);
        }
    }
}


/**
 * build_line converts a PointSet into a line.
 *
 * \param    set                 Set to get line for
 * \param    coords              Coords from which the set was pulled
 * \param    line                Line representing the set (output)
 */
Line<float> build_line(const PointSet& set, const std::vector<Point<float>>& coords)
{
    return vulcan::math::total_least_squares(coords.begin() + set.set.first, coords.begin() + set.set.second);
}
