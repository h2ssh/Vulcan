/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hough_transform_test.cpp
* \author   Collin Johnson
*
* Implementation of some simple unit tests for the HoughTransform class.
*/

#include "utils/hough_transform.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <iostream>
#include <vector>

using namespace vulcan;
using namespace vulcan::utils;

using Grid = CellGrid<int>;
const int kGridSize = 100;
const float kAngleRes = 0.5f;
const float kRadiusRes = 1.0f;


bool cell_func(int x, int y, const Grid& grid)
{
    return grid(x, y) > 0;
}

// Start at the origin and add the given line
void add_line(int dx, int dy, Grid& grid)
{
    dx = std::abs(dx);
    dy = std::abs(dy);
    
    for(std::size_t x = 0, y = 0; (y < grid.getHeightInCells()) && (x < grid.getWidthInCells()); x += dx, y += dy)
    {
        grid(x, y) = 1;
    }
}

void print_hough(const HoughTransform& hough)
{
//     const auto& accum = hough.accumulator();
//     
//     std::cout << "Accumulator: \n";
//     for(std::size_t y = 0; y < accum.getHeightInCells(); ++y)
//     {
//         for(std::size_t x = 0; x < accum.getWidthInCells(); ++x)
//         {
//             std::cout << accum(x, y) << ' ';
//         }
//         std::cout << '\n';
//     }
}


TEST(HoughTransformTest, CanFindHorizontalLine)
{
    Grid g;
    g.setGridSizeInCells(kGridSize, kGridSize);
    g.reset(0);
    add_line(1, 0, g);
    
    HoughTransform transform(kAngleRes, kRadiusRes);
    transform.compute(g, cell_func);
    print_hough(transform);
    
    auto best = transform.bestLine();
    // A horizontal line has theta = pi/2
    EXPECT_LT(std::abs(std::abs(best.theta) - M_PI_2), 0.02);
}


TEST(HoughTransformTest, CanFindVerticalLine)
{
    Grid g;
    g.setGridSizeInCells(kGridSize, kGridSize);
    g.reset(0);
    add_line(0, 1, g);
    
    HoughTransform transform(kAngleRes, kRadiusRes);
    transform.compute(g, cell_func);
    print_hough(transform);
    
    auto best = transform.bestLine();
    
    // A vertical line with have theta = 0
    EXPECT_LT(std::abs(best.theta), 0.02);
}


TEST(HoughTransformTest, CanFindDiagonalLine)
{
    Grid g;
    g.setGridSizeInCells(kGridSize, kGridSize);
    g.reset(0);
    add_line(1, 1, g);
    
    HoughTransform transform(kAngleRes, kRadiusRes);
    transform.compute(g, cell_func);
    print_hough(transform);
    
    auto best = transform.bestLine();
    
    // A diagonal line could be +-pi/4
    EXPECT_LT(std::abs(std::abs(wrap_hough_angle(best.theta)) - M_PI_4), 0.02);
}
