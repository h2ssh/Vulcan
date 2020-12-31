/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "core/pose.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_topological/area_detection/voronoi/brushfire_skeleton_builder.h"
#include "hssh/local_topological/params.h"
#include <iostream>


using vulcan::hssh::BrushfireSkeletonBuilder;
using vulcan::hssh::LocalPerceptualMap;
using vulcan::hssh::VoronoiSkeletonGrid;


const uint16_t SMALL_GRID_SIZE = 250;
const uint16_t LARGE_GRID_SIZE = 750;

const float COASTAL_DISTANCE = 10.0f;
const float GRID_CELL_SCALE = 0.05f;


bool test_blank_map(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place);
bool test_corridor(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place);
bool test_large_corridor(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place);
bool test_generic_corridor(BrushfireSkeletonBuilder& brushfire,
                           VoronoiSkeletonGrid& place,
                           uint16_t size,
                           uint16_t start,
                           uint16_t width,
                           uint16_t actualNumSkeletonCells,
                           int skeletonX[2]);
bool test_square_smaller_than_coastal(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place);
bool test_square_larger_than_coastal(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place);
LocalPerceptualMap build_square(uint16_t width, uint16_t height);
bool test_plus(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place);
bool test_big_plus(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place);

/**
 * brushfire_skeleton_builder_test runs the BrushfireSkeletonBuilder on a few fixed maps
 * to see that the algorithm works and that it produces reasonable results.
 */
int main(int argc, char** argv)
{
    vulcan::hssh::skeleton_builder_params_t params;
    params.coastalDistance = COASTAL_DISTANCE;

    VoronoiSkeletonGrid place;

    BrushfireSkeletonBuilder brushfire(params);

    int totalTests = 0;
    int testsPassed = 0;

    if (test_blank_map(brushfire, place)) {
        ++testsPassed;
    }

    ++totalTests;

    if (test_corridor(brushfire, place)) {
        ++testsPassed;
    }

    ++totalTests;

    if (test_large_corridor(brushfire, place)) {
        ++testsPassed;
    }

    ++totalTests;

    if (test_square_smaller_than_coastal(brushfire, place)) {
        ++testsPassed;
    }

    ++totalTests;

    if (test_square_larger_than_coastal(brushfire, place)) {
        ++testsPassed;
    }

    ++totalTests;

    if (test_plus(brushfire, place)) {
        ++testsPassed;
    }

    ++totalTests;

    if (test_big_plus(brushfire, place)) {
        ++testsPassed;
    }

    ++totalTests;

    if (testsPassed == totalTests) {
        std::cout << "SUCCESS: All tests passed'\n";
    } else {
        std::cout << "FAILURE: Passed " << testsPassed << '/' << totalTests << " tests\n";
    }

    return testsPassed == totalTests;
}


bool test_blank_map(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place)
{
    /*
     * For the blank map test, there should be no anchors and no skeleton cells.
     */

    bool success = true;

    LocalPerceptualMap grid(SMALL_GRID_SIZE, SMALL_GRID_SIZE, GRID_CELL_SCALE, vulcan::Point<float>(0, 0), 200, 10);

    //     brushfire.buildSkeleton(grid, vulcan::pose_t(0, 0, 0), place);

    int numAnchorPoints = 0;

    if (numAnchorPoints > 0) {
        std::cout << "ERROR: test_blank_map: Should be no anchor points, but there were " << numAnchorPoints
                  << " anchor points\n";

        success = false;
    }

    int skeletonCellCount = 0;

    for (int x = SMALL_GRID_SIZE; --x >= 0;) {
        for (int y = SMALL_GRID_SIZE; --y >= 0;) {
            if (place.getClassification(x, y) & vulcan::hssh::SKELETON_CELL_SKELETON) {
                ++skeletonCellCount;
            }
        }
    }

    if (skeletonCellCount > 0) {
        std::cout << "ERROR: test_blank_map: Should be no skeleton cells, but there were " << skeletonCellCount
                  << " skeleton cells\n";

        success = false;
    }

    return success;
}


bool test_corridor(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place)
{
    /*
     * For a corridor, the skeleton should run right down the middle. There shouldn't be any anchor points as
     * the brushfire will collide in the center.
     */
    const int CORRIDOR_WIDTH = 101;   // odd to ensure a single skeleton running down the middle
    const int CORRIDOR_START = 50;

    const int SKELETON_X_CELL = CORRIDOR_START + CORRIDOR_WIDTH / 2 + 1;

    int skeletonX[2] = {SKELETON_X_CELL, SKELETON_X_CELL};

    std::cout << "INFO: test_small_corridor: Skeleton cells should be at x=" << skeletonX[0] << '\n';

    return test_generic_corridor(brushfire,
                                 place,
                                 SMALL_GRID_SIZE,
                                 CORRIDOR_START,
                                 CORRIDOR_WIDTH,
                                 SMALL_GRID_SIZE,
                                 skeletonX);
}


bool test_large_corridor(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place)
{
    /*
     * For a large corridor, the skeleton should run on two channels at the coastal distance away from the walls.
     */

    int coastalDistanceInCells = static_cast<int>(COASTAL_DISTANCE / GRID_CELL_SCALE);

    int corridorWidth = 3 * coastalDistanceInCells;   // odd to ensure a single skeleton running down the middle
    int corridorStart = 50;
    int corridorEnd = corridorStart + corridorWidth + 1;

    int skeletonX[2] = {corridorStart + (coastalDistanceInCells - 1), corridorEnd - (coastalDistanceInCells - 1)};

    std::cout << "INFO: test_large_corridor: Skeleton cells should be at x=" << skeletonX[0] << " or x=" << skeletonX[1]
              << '\n';

    return test_generic_corridor(brushfire,
                                 place,
                                 LARGE_GRID_SIZE,
                                 corridorStart,
                                 corridorWidth,
                                 LARGE_GRID_SIZE * 2,
                                 skeletonX);
}


bool test_generic_corridor(BrushfireSkeletonBuilder& brushfire,
                           VoronoiSkeletonGrid& place,
                           uint16_t size,
                           uint16_t start,
                           uint16_t width,
                           uint16_t actualNumSkeletonCells,
                           int skeletonX[2])
{
    /*
     * For a corridor, the skeleton should run right down the middle. There shouldn't be any anchor points as
     * the brushfire will collide in the center.
     */

    bool success = true;

    LocalPerceptualMap grid(size, size, GRID_CELL_SCALE, vulcan::Point<float>(0, 0), 200, 0);

    int x = start;

    for (int y = size; --y >= 0;) {
        grid.setCostNoCheck(vulcan::Point<uint16_t>(x, y), 255);
    }

    ++x;

    for (int i = width; --i >= 0; ++x) {
        for (int y = size; --y >= 0;) {
            grid.setCostNoCheck(vulcan::Point<uint16_t>(x, y), 0);
        }
    }

    x = start + width + 1;

    for (int y = size; --y >= 0;) {
        grid.setCostNoCheck(vulcan::Point<uint16_t>(x, y), 255);
    }

    //     brushfire.buildSkeleton(grid, vulcan::pose_t(0, 0, 0), place);

    int numAnchorPoints = 0;

    if (numAnchorPoints > 0) {
        std::cout << "ERROR: test_corridor: Should be no anchor points, but there were " << numAnchorPoints
                  << " anchor points\n";

        success = false;
    }

    int badCellCount = 0;
    int skeletonCellCount = 0;

    for (int x = size; --x >= 0;) {
        for (int y = size; --y >= 0;) {
            if (place.getClassification(x, y) & vulcan::hssh::SKELETON_CELL_SKELETON) {
                if (x == skeletonX[0] || x == skeletonX[1]) {
                    ++skeletonCellCount;
                } else {
                    ++badCellCount;
                }
            }
        }
    }

    if (badCellCount > 0) {
        std::cout << "ERROR: test_corridor: There were " << badCellCount
                  << " skeleton cells not at the center point of the corridor\n";

        success = false;
    }

    if (skeletonCellCount != actualNumSkeletonCells) {
        std::cout << "ERROR: test_corridor: There were " << skeletonCellCount
                  << " skeleton cells at the center point and there should be " << actualNumSkeletonCells << '\n';

        success = false;
    }

    return success;
}


bool test_square_smaller_than_coastal(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place)
{
    LocalPerceptualMap grid = build_square(SMALL_GRID_SIZE, SMALL_GRID_SIZE);

    return true;
}


bool test_square_larger_than_coastal(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place)
{
    LocalPerceptualMap grid = build_square(LARGE_GRID_SIZE, LARGE_GRID_SIZE);

    return true;
}


LocalPerceptualMap build_square(uint16_t width, uint16_t height)
{
    // The square is just going to border the grid because it makes calculation of where the skeleton should lie much
    // easier
    LocalPerceptualMap grid(width, height, GRID_CELL_SCALE, vulcan::Point<float>(0, 0), 200, 0);

    vulcan::Point<uint16_t> cell(0, 0);

    for (; cell.x < width; ++cell.x) {
        for (cell.y = 0; cell.y < height; ++cell.y) {
            if (cell.x == 0 || cell.y == 0 || cell.x == width - 1 || cell.y == height - 1) {
                grid.setCostNoCheck(cell, 255);
            } else {
                grid.setCostNoCheck(cell, 0);
            }
        }
    }

    return grid;
}


bool test_plus(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place)
{
    LocalPerceptualMap grid(SMALL_GRID_SIZE, SMALL_GRID_SIZE, GRID_CELL_SCALE, vulcan::Point<float>(0, 0), 200, 0);

    return true;
}


bool test_big_plus(BrushfireSkeletonBuilder& brushfire, VoronoiSkeletonGrid& place)
{
    LocalPerceptualMap grid(LARGE_GRID_SIZE, LARGE_GRID_SIZE, GRID_CELL_SCALE, vulcan::Point<float>(0, 0), 200, 0);

    return true;
}
