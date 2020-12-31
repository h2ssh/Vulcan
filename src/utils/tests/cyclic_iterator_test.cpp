/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     cyclic_iterator_test.cpp
 * \author   Collin Johnson
 *
 * cyclic_iterator_test is a test program that tests the functionality of cyclic_iterator.
 */

#include "utils/cyclic_iterator.h"
#include <algorithm>
#include <gtest/gtest.h>
#include <numeric>
#include <vector>

using namespace vulcan;
using namespace vulcan::utils;

std::vector<int> create_test_vector(void)
{
    std::vector<int> v(10);
    std::iota(v.begin(), v.end(), 0);
    return v;
}


TEST(CyclicIteratorTest, CanIterateNormalRange)
{
    auto values = create_test_vector();
    auto cyclicRange = make_cyclic_iterator(values.begin(), values.begin(), values.end());

    EXPECT_NE(cyclicRange.first, cyclicRange.second);

    auto cycIt = cyclicRange.first;
    std::size_t n = 0;
    for (; (n < values.size()) && (cycIt != cyclicRange.second); ++n, ++cycIt) {
        EXPECT_EQ(*cycIt, values[n]);
    }
    EXPECT_EQ(values.size(), n);
    EXPECT_EQ(cyclicRange.second, cycIt);
}


TEST(CyclicIteratorTest, IteratorWrapsAround)
{
    std::size_t kOffset = 5;

    auto values = create_test_vector();
    auto cyclicRange = make_cyclic_iterator(values.begin() + kOffset, values.begin(), values.end());

    auto cycIt = cyclicRange.first;
    std::size_t n = 0;
    for (; (n < values.size()) && (cycIt != cyclicRange.second); ++n, ++cycIt) {
        EXPECT_EQ(*cycIt, values[(n + kOffset) % values.size()]);
    }
    EXPECT_EQ(values.size(), n);
    EXPECT_EQ(cyclicRange.second, cycIt);
}


TEST(CyclicIteratorTest, IteratorWrapsMultipleTimes)
{
    std::size_t kOffset = 5;
    int kNumCycles = 3;

    auto values = create_test_vector();
    auto cyclicRange = make_cyclic_iterator(values.begin() + kOffset, values.begin(), values.end(), 3);

    auto cycIt = cyclicRange.first;
    std::size_t n = 0;
    for (; (cycIt != cyclicRange.second); ++n, ++cycIt) {
        EXPECT_EQ(*cycIt, values[(n + kOffset) % values.size()]);
    }
    EXPECT_EQ(values.size() * kNumCycles, n);
    EXPECT_EQ(cyclicRange.second, cycIt);
}
