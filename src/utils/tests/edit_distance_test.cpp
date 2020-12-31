/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     edit_distance_test.cpp
 * \author   Collin Johnson
 *
 * edit_distance_test implements simple unit tests to check the sanity of the edit distance implementation.
 */

#include "utils/edit_distance.h"
#include <gtest/gtest.h>
#include <iostream>


using namespace vulcan::utils;


TEST(EditDistanceTest, EmptyStringsHaveZeroDist)
{
    std::string x;
    std::string y;

    EXPECT_EQ(0.0, edit_distance(x, y));
}


TEST(EditDistanceTest, EmptyToStringIsInsertionCost)
{
    std::string x;
    std::string y = "a";

    edit_distance_weights_t weights;
    weights.insertion = 5.0;

    EXPECT_EQ(weights.insertion, edit_distance(x, y, weights, true));
}


TEST(EditDistanceTest, StringToEmptyIsDeletionCost)
{
    std::string x = "a";
    std::string y;

    edit_distance_weights_t weights;
    weights.deletion = 5.0;

    EXPECT_EQ(weights.deletion, edit_distance(x, y, weights, true));
}


TEST(EditDistanceTest, OnlyInsertionsIsValid)
{
    std::string x = "a";
    std::string y = "abcdefg";

    edit_distance_weights_t weights;
    weights.insertion = 5.0;

    EXPECT_EQ(weights.insertion * (y.length() - x.length()), edit_distance(x, y, weights, true));
}


TEST(EditDistanceTest, OnlyDeletionsIsValid)
{
    std::string x = "abcdefg";
    std::string y = "a";

    edit_distance_weights_t weights;
    weights.deletion = 5.0;

    EXPECT_EQ(weights.deletion * (x.length() - y.length()), edit_distance(x, y, weights, true));
}


TEST(EditDistanceTest, EEtoCSIsCorrect)
{
    std::string x = "electrical engineering";
    std::string y = "computer science";

    edit_distance_weights_t weights;
    weights.deletion = 2.0;
    weights.insertion = 3.0;
    weights.substitution = 4.0;

    EXPECT_EQ(54.0, edit_distance(x, y, weights, true));
}
