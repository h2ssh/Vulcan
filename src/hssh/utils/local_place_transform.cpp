/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     local_place_transform.cpp
 * \author   Collin Johnson
 *
 * Definition of calculate_transform_to_reference_place.
 */

#include "hssh/utils/local_place_transform.h"
#include "core/conversions.h"
#include "hssh/local_topological/areas/place.h"
#include <iostream>

// #define DEBUG_TRANSFORM
// #define DEBUG_ITERATIONS

namespace vulcan
{
namespace hssh
{

const int NUM_SCAN_RAYS = 180;

struct place_scan_t
{
    // Scan units are cells, not meters, because goal is to find where center of current best matches reference
    //     Point<double> scan[NUM_SCAN_RAYS];
    std::vector<Point<float>> scan;
    bool valid[NUM_SCAN_RAYS];
};

void generate_scan(const LocalPerceptualMap& map, place_scan_t& scan);
pose_t calculate_initial_guess(const local_path_fragment_t& current, const local_path_fragment_t& reference);
pose_t run_scan_matcher(const place_scan_t& currentScan, const LocalPlace& reference, const pose_t& initialGuess);
pose_t find_best_transform(const place_scan_t& scan, const pose_t& initialGuess, const LocalPerceptualMap& map);
void rotate_scan(const place_scan_t& original, float angle, place_scan_t& rotated);
uint32_t score_scan(const place_scan_t& scan, double deltaX, double deltaY, const LocalPerceptualMap& lpm);

inline bool is_cell_in_map(const Point<uint16_t>& cell, const LocalPerceptualMap& lpm)
{
    return (cell.x < lpm.getWidthInCells()) && (cell.y < lpm.getHeightInCells());
}


pose_t calculate_transform_to_reference_place(const LocalPlace& currentPlace,
                                              const LocalPlace& referencePlace,
                                              const local_path_fragment_t& currentFragment,
                                              const local_path_fragment_t& referenceFragment)
{
    place_scan_t currentScan;

    generate_scan(currentPlace.map(), currentScan);

    pose_t initialGuess = calculate_initial_guess(currentFragment, referenceFragment);
    pose_t matcher;   //      = run_scan_matcher(currentScan, referencePlace, initialGuess);
    pose_t best = find_best_transform(currentScan, initialGuess, referencePlace.map());

#ifdef DEBUG_TRANSFORM
    std::cout << "DEBUG:local_place_transform:Current:" << currentFragment.gateway.boundary
              << " Reference:" << referenceFragment.gateway.boundary << " Matcher: " << matcher
              << " Initial:" << initialGuess << " Best:" << best << '\n';
#endif

    return best;
}


void generate_scan(const LocalPerceptualMap& map, place_scan_t& scan)
{
    float increment = 2.0 * M_PI / NUM_SCAN_RAYS;
    float angle = 0.0f;

    Point<double> start(map.getWidthInCells() / 2.0, map.getHeightInCells() / 2.0);

    for (int n = 0; n < NUM_SCAN_RAYS; ++n) {
        angle = increment * n;

        double deltaX = cos(angle);
        double deltaY = sin(angle);

        double xPosition = start.x;
        double yPosition = start.y;

        Point<uint16_t> rayCell(start.x, start.y);

        while (is_cell_in_map(rayCell, map) && (map.getCellType(rayCell) & ~kOccupiedOccGridCell)) {
            xPosition += deltaX;
            yPosition += deltaY;

            rayCell.x = xPosition;
            rayCell.y = yPosition;
        }

        scan.valid[n] = is_cell_in_map(rayCell, map);
        //         scan.scan[n].x = xPosition;
        //         scan.scan[n].y = yPosition;
        scan.scan.push_back(Point<float>(xPosition * map.metersPerCell(), yPosition * map.metersPerCell()));
    }
}


pose_t calculate_initial_guess(const local_path_fragment_t& current, const local_path_fragment_t& reference)
{
    pose_t guess;

    guess.x = reference.gateway.center().x - current.gateway.center().x;
    guess.y = reference.gateway.center().y - current.gateway.center().y;
    //     guess.theta = angle_diff(atan2(reference.gateway.getCenter().y, reference.gateway.getCenter().x),
    //                                    atan2(current.gateway.getCenter().y, current.gateway.getCenter().x));

    //     guess.theta = angle_diff(reference.gateway.direction, current.gateway.direction);
    guess.theta = 0.0f;
    return guess;
}


pose_t run_scan_matcher(const place_scan_t& currentScan, const LocalPlace& reference, const pose_t& initialGuess)
{
    place_scan_t referenceScan;

    generate_scan(reference.map(), referenceScan);

    return pose_t();
}


pose_t find_best_transform(const place_scan_t& scan, const pose_t& initialGuess, const LocalPerceptualMap& map)
{
    const int MAX_ITERATIONS = 100;

    place_scan_t rotated;
    rotated.scan.resize(scan.scan.size());
    rotate_scan(scan, initialGuess.theta, rotated);
    uint32_t score = score_scan(rotated, 0, 0, map);

    float refineInitialStepsize[3] = {2.0f * map.metersPerCell(), 2.0f * map.metersPerCell(), M_PI / 360.0f};
    float refineMinimumStepsize[3] = {0.0005f, 0.0005f, M_PI / 3600.0f};
    float refineShrinkRatio = 0.7;

    float stepsize[3];

    memcpy(stepsize, refineInitialStepsize, 3 * sizeof(float));

    float newxyts[6][3];

    pose_t refinedTransform = initialGuess;

#ifdef DEBUG_ITERATIONS
    std::cout << "DEBUG:find_best_transform:Initial:" << initialGuess << "=" << score << '\n';
#endif

    for (int iterations = 0; iterations < MAX_ITERATIONS; ++iterations) {
        newxyts[0][0] = refinedTransform.x + stepsize[0];
        newxyts[0][1] = refinedTransform.y;
        newxyts[0][2] = refinedTransform.theta;

        newxyts[1][0] = refinedTransform.x - stepsize[0];
        newxyts[1][1] = refinedTransform.y;
        newxyts[1][2] = refinedTransform.theta;

        newxyts[2][0] = refinedTransform.x;
        newxyts[2][1] = refinedTransform.y + stepsize[1];
        newxyts[2][2] = refinedTransform.theta;

        newxyts[3][0] = refinedTransform.x;
        newxyts[3][1] = refinedTransform.y - stepsize[1];
        newxyts[3][2] = refinedTransform.theta;

        newxyts[4][0] = refinedTransform.x;
        newxyts[4][1] = refinedTransform.y;
        newxyts[4][2] = refinedTransform.theta + stepsize[2];

        newxyts[5][0] = refinedTransform.x;
        newxyts[5][1] = refinedTransform.y;
        newxyts[5][2] = refinedTransform.theta - stepsize[2];

        // move in the best direction. (Roughly a local gradient search.)
        bool stepped = false;

        rotate_scan(scan, refinedTransform.theta, rotated);
        for (int i = 0; i < 4; i++) {
            uint32_t newscore = score_scan(rotated, newxyts[i][0], newxyts[i][1], map);

            if (newscore > score) {
                stepped = true;
                score = newscore;
                refinedTransform.x = newxyts[i][0];
                refinedTransform.y = newxyts[i][1];
                refinedTransform.theta = newxyts[i][2];
            }
        }

        for (int i = 4; i < 6; i++) {
            rotate_scan(scan, newxyts[i][2], rotated);

            uint32_t newscore = score_scan(rotated, refinedTransform.x, refinedTransform.y, map);

            if (newscore > score) {
                stepped = true;
                score = newscore;
                refinedTransform.x = newxyts[i][0];
                refinedTransform.y = newxyts[i][1];
                refinedTransform.theta = newxyts[i][2];
            }
        }

#ifdef DEBUG_ITERATIONS
        std::cout << "Iteration:" << iterations << " Guess:" << refinedTransform << "=" << score << '\n';
#endif

        // That step was good. Keep going at the same step size.
        if (stepped) {
            continue;
        }

        // We've rejected that step. Reduce our step size.
        bool searchMore = false;
        for (int i = 0; i < 3; i++) {
            if (stepsize[i] > refineMinimumStepsize[i]) {
                searchMore = true;
                stepsize[i] = std::max(refineMinimumStepsize[i], stepsize[i] * refineShrinkRatio);
            }
        }

        if (!searchMore) {
            break;
        }
    }

    return refinedTransform;
}


void rotate_scan(const place_scan_t& original, float angle, place_scan_t& rotated)
{
    const float cosAngle = cos(angle);
    const float sinAngle = sin(angle);

    for (size_t n = 0; n < original.scan.size(); ++n) {
        rotated.scan[n].x = original.scan[n].x * cosAngle - original.scan[n].y * sinAngle;
        rotated.scan[n].y = original.scan[n].x * sinAngle + original.scan[n].y * cosAngle;
        rotated.valid[n] = original.valid[n];
    }
}


uint32_t score_scan(const place_scan_t& scan, double deltaX, double deltaY, const LocalPerceptualMap& lpm)
{
    uint32_t score = 0;
    Point<uint16_t> cell;

    for (size_t n = 0; n < scan.scan.size(); ++n) {
        cell.x = (scan.scan[n].x + deltaX) * lpm.cellsPerMeter();
        cell.y = (scan.scan[n].y + deltaY) * lpm.cellsPerMeter();

        score += lpm.getCost(cell);
    }

    return score;
}

}   // namespace hssh
}   // namespace vulcan
