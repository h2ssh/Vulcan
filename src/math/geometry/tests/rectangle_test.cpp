/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     rectangle_test.cpp
* \author   Collin Johnson
*
* Unit tests for ensuring proper functionality of the Rectangle shape.
*/

#include <math/geometry/rectangle.h>
#include <gtest/gtest.h>

using namespace vulcan;
using namespace vulcan::math;


TEST(DistanceToPointTest, InsidePointIsZeroDist)
{
    Rectangle<int> intRect(Point<int>(0, 0), Point<int>(2, 2));

    EXPECT_DOUBLE_EQ(0.0, intRect.distanceToPoint(Point<int>(1, 1)));
    EXPECT_DOUBLE_EQ(0.0, intRect.distanceToPoint(Point<int>(0, 0)));
    EXPECT_DOUBLE_EQ(0.0, intRect.distanceToPoint(Point<int>(0, 2)));
    EXPECT_DOUBLE_EQ(0.0, intRect.distanceToPoint(Point<int>(2, 0)));
    EXPECT_DOUBLE_EQ(0.0, intRect.distanceToPoint(Point<int>(2, 2)));
    EXPECT_DOUBLE_EQ(0.0, intRect.distanceToPoint(Point<int>(2, 1)));

    Rectangle<float> floatRect(Point<float>(0, 0), Point<float>(2, 2));

    EXPECT_DOUBLE_EQ(0.0, floatRect.distanceToPoint(Point<float>(1, 1)));
    EXPECT_DOUBLE_EQ(0.0, floatRect.distanceToPoint(Point<float>(0, 0)));
    EXPECT_DOUBLE_EQ(0.0, floatRect.distanceToPoint(Point<float>(0, 2)));
    EXPECT_DOUBLE_EQ(0.0, floatRect.distanceToPoint(Point<float>(2, 0)));
    EXPECT_DOUBLE_EQ(0.0, floatRect.distanceToPoint(Point<float>(2, 2)));
    EXPECT_DOUBLE_EQ(0.0, floatRect.distanceToPoint(Point<float>(2, 1)));

    Rectangle<double> doubleRect(Point<double>(0, 0), Point<double>(2, 2));

    EXPECT_DOUBLE_EQ(0.0, doubleRect.distanceToPoint(Point<double>(1, 1)));
    EXPECT_DOUBLE_EQ(0.0, doubleRect.distanceToPoint(Point<double>(0, 0)));
    EXPECT_DOUBLE_EQ(0.0, doubleRect.distanceToPoint(Point<double>(0, 2)));
    EXPECT_DOUBLE_EQ(0.0, doubleRect.distanceToPoint(Point<double>(2, 0)));
    EXPECT_DOUBLE_EQ(0.0, doubleRect.distanceToPoint(Point<double>(2, 2)));
    EXPECT_DOUBLE_EQ(0.0, doubleRect.distanceToPoint(Point<double>(2, 1)));

    Rectangle<double> angleRect(Point<double>(0, 2), Point<double>(2, 0), Point<double>(0, -2), Point<double>(-2, 0));

    EXPECT_DOUBLE_EQ(0.0, angleRect.distanceToPoint(Point<double>(1, 1)));
    EXPECT_DOUBLE_EQ(0.0, angleRect.distanceToPoint(Point<double>(0, 0)));
    EXPECT_DOUBLE_EQ(0.0, angleRect.distanceToPoint(Point<double>(0, -2)));
    EXPECT_DOUBLE_EQ(0.0, angleRect.distanceToPoint(Point<double>(-2, 0)));
    EXPECT_DOUBLE_EQ(0.0, angleRect.distanceToPoint(Point<double>(0, 2)));
    EXPECT_DOUBLE_EQ(0.0, angleRect.distanceToPoint(Point<double>(2, 0)));
}


TEST(DistanceToPointTest, OutsidePointIsNonZeroDist)
{
    Rectangle<int> intRect(Point<int>(0, 0), Point<int>(2, 2));

    EXPECT_GT(intRect.distanceToPoint(Point<int>(5, 0)), 0.0);
    EXPECT_GT(intRect.distanceToPoint(Point<int>(-5, 0)), 0.0);
    EXPECT_GT(intRect.distanceToPoint(Point<int>(0, 5)), 0.0);
    EXPECT_GT(intRect.distanceToPoint(Point<int>(0, -5)), 0.0);
    EXPECT_GT(intRect.distanceToPoint(Point<int>(2, 3)), 0.0);
    EXPECT_GT(intRect.distanceToPoint(Point<int>(-2, 2)), 0.0);
    EXPECT_GT(intRect.distanceToPoint(Point<int>(-2, -2)), 0.0);
    EXPECT_GT(intRect.distanceToPoint(Point<int>(2, -2)), 0.0);

    Rectangle<float> floatRect(Point<float>(0, 0), Point<float>(2, 2));

    EXPECT_GT(floatRect.distanceToPoint(Point<float>(5, 0)), 0.0);
    EXPECT_GT(floatRect.distanceToPoint(Point<float>(-5, 0)), 0.0);
    EXPECT_GT(floatRect.distanceToPoint(Point<float>(0, 5)), 0.0);
    EXPECT_GT(floatRect.distanceToPoint(Point<float>(0, -5)), 0.0);
    EXPECT_GT(floatRect.distanceToPoint(Point<float>(2, 3)), 0.0);
    EXPECT_GT(floatRect.distanceToPoint(Point<float>(-2, 2)), 0.0);
    EXPECT_GT(floatRect.distanceToPoint(Point<float>(-2, -2)), 0.0);
    EXPECT_GT(floatRect.distanceToPoint(Point<float>(2, -2)), 0.0);

    Rectangle<double> doubleRect(Point<double>(0, 0), Point<double>(2, 2));

    EXPECT_GT(doubleRect.distanceToPoint(Point<double>(5, 0)), 0.0);
    EXPECT_GT(doubleRect.distanceToPoint(Point<double>(-5, 0)), 0.0);
    EXPECT_GT(doubleRect.distanceToPoint(Point<double>(0, 5)), 0.0);
    EXPECT_GT(doubleRect.distanceToPoint(Point<double>(0, -5)), 0.0);
    EXPECT_GT(doubleRect.distanceToPoint(Point<double>(2, 3)), 0.0);
    EXPECT_GT(doubleRect.distanceToPoint(Point<double>(-2, 2)), 0.0);
    EXPECT_GT(doubleRect.distanceToPoint(Point<double>(-2, -2)), 0.0);
    EXPECT_GT(doubleRect.distanceToPoint(Point<double>(2, -2)), 0.0);

    Rectangle<double> angleRect(Point<double>(0, 2), Point<double>(2, 0), Point<double>(0, -2), Point<double>(-2, 0));

    EXPECT_GT(angleRect.distanceToPoint(Point<double>(5, 0)), 0.0);
    EXPECT_GT(angleRect.distanceToPoint(Point<double>(-5, 0)), 0.0);
    EXPECT_GT(angleRect.distanceToPoint(Point<double>(0, 5)), 0.0);
    EXPECT_GT(angleRect.distanceToPoint(Point<double>(0, -5)), 0.0);
    EXPECT_GT(angleRect.distanceToPoint(Point<double>(2, 2)), 0.0);
    EXPECT_GT(angleRect.distanceToPoint(Point<double>(-2, 2)), 0.0);
    EXPECT_GT(angleRect.distanceToPoint(Point<double>(-2, -2)), 0.0);
    EXPECT_GT(angleRect.distanceToPoint(Point<double>(2, -2)), 0.0);
}
