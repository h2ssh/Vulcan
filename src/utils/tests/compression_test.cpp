/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     compression_test.cpp
* \author   Collin Johnson
*
* Unit tests for ensuring the compression and decompression work correctly.
*/

#include "utils/compression.h"
#include <gtest/gtest.h>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/vector.hpp>
#include <random>
#include <typeindex>
#include <typeinfo>
#include <tuple>
#include <unordered_map>
#include <vector>


using namespace vulcan;


using TypeVector = boost::fusion::vector<int8_t, uint8_t, int16_t, uint16_t, int32_t, uint32_t, int64_t, uint64_t>;

std::string type_name(std::type_index index);

template <typename T>
std::vector<T> generate_random_sequence(std::size_t length, double sameValueRatio);

struct RoundtripChecker
{
    double sameValueRatio;
    std::size_t sequenceLength = 10000;

    template <typename T>
    void operator()(T& t) const
    {
        auto initialSequence = generate_random_sequence<T>(sequenceLength, sameValueRatio);
        std::vector<T> compressedSequence;
        utils::compress(initialSequence.begin(), initialSequence.end(), std::back_inserter(compressedSequence));

        std::vector<T> decompressedSequence;
        utils::decompress(compressedSequence.begin(),
                          compressedSequence.end(),
                          std::back_inserter(decompressedSequence));

        EXPECT_EQ(initialSequence.size(), decompressedSequence.size())
            << "Ratio:" << sameValueRatio << " Type:" << type_name(std::type_index(typeid(T)));
        EXPECT_EQ(initialSequence, decompressedSequence)
            << "Ratio:" << sameValueRatio << " Type:" << type_name(std::type_index(typeid(T)));
    }
};


TEST(RoundtripTest, UniformSequenceIsSame)
{
    TypeVector typesToCheck;

    RoundtripChecker checker;
    checker.sameValueRatio = 1.0;
    boost::fusion::for_each(typesToCheck, checker);
}


TEST(RoundtripTest, RandomSequenceIsSame)
{
    TypeVector typesToCheck;
    RoundtripChecker checker;

    checker.sameValueRatio = 0.5;
    boost::fusion::for_each(typesToCheck, checker);

    checker.sameValueRatio = 0.25;
    boost::fusion::for_each(typesToCheck, checker);

    checker.sameValueRatio = 0.0;
    boost::fusion::for_each(typesToCheck, checker);
}


struct CompressionChecker
{
    double sameValueRatio;
    bool expectRatioAboveOne;
    std::size_t sequenceLength = 10000;

    template <typename T>
    void operator()(T& t) const
    {
        auto initialSequence = generate_random_sequence<T>(sequenceLength, sameValueRatio);
        std::vector<T> compressedSequence;
        utils::compress(initialSequence.begin(), initialSequence.end(), std::back_inserter(compressedSequence));

        double compressionRatio = sequenceLength / static_cast<double>(compressedSequence.size());

        std::cout << "Compression ratio for " << type_name(std::type_index(typeid(T))) << ':'
            << compressionRatio << '\n';
        EXPECT_EQ(expectRatioAboveOne, compressionRatio > 1.0)
            << "Ratio:" << sameValueRatio << " Type:" << type_name(std::type_index(typeid(T)));
    }
};


TEST(CompressionTest, UniformSequenceIsSmaller)
{
    TypeVector typesToCheck;

    // A smaller sequence means a compression ratio > 1.0
    CompressionChecker checker;
    checker.sameValueRatio = 1.0;
    checker.expectRatioAboveOne = true;
    boost::fusion::for_each(typesToCheck, checker);
}


TEST(CompressionTest, NeverSameSequenceIsSmaller)
{
    TypeVector typesToCheck;

    // If two values are never the same, the length of the sequence will increase due to overhead with the header
    CompressionChecker checker;
    checker.sameValueRatio = 0.0;
    checker.expectRatioAboveOne = false;
    boost::fusion::for_each(typesToCheck, checker);
}


template <typename T>
std::vector<T> generate_random_sequence(std::size_t length, double sameValueRatio)
{
    std::vector<T> sequence;

    std::random_device rd;
    std::uniform_real_distribution<> dist(0.0, 1.0);
    T nextVal = 0;
    double diffValRatio = 1.0 - sameValueRatio;

    for(std::size_t n = 0; n < length; ++n)
    {
        // The range of the distribution is [0, 1). Thus, if diffVal == 0, this condition never fires and a uniform
        // sequence is created, and if diffVal == 1, it always fires and a completely random sequence is created
        if(dist(rd) < diffValRatio)
        {
            // If the sampled number is less than the ratio of different to same values, then iterate the next value
            // being added to the distribution
            ++nextVal;
        }

        sequence.push_back(nextVal);
    }

    return sequence;
}


std::string type_name(std::type_index index)
{
    static std::unordered_map<std::type_index, std::string> typeNames;
    if(typeNames.empty())
    {
        typeNames[std::type_index(typeid(int8_t))] = "int8_t";
        typeNames[std::type_index(typeid(uint8_t))] = "uint8_t";
        typeNames[std::type_index(typeid(int16_t))] = "int16_t";
        typeNames[std::type_index(typeid(uint16_t))] = "uint16_t";
        typeNames[std::type_index(typeid(int32_t))] = "int32_t";
        typeNames[std::type_index(typeid(uint32_t))] = "uint32_t";
        typeNames[std::type_index(typeid(int64_t))] = "int64_t";
        typeNames[std::type_index(typeid(uint64_t))] = "uint64_t";
    }

    return typeNames[index];
}
