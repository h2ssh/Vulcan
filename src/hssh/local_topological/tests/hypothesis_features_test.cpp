/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_features_test.cpp
* \author   Collin Johnson
*
* Unit tests for ensuring proper functionality of HypothesisFeatures.
*/

#include "hssh/local_topological/area_detection/labeling/hypothesis_features.h"
#include "utils/float_io.h"
#include <gtest/gtest.h>
#include <sstream>
#include <utility>

using namespace vulcan;
using namespace vulcan::hssh;


std::pair<std::string, Vector> random_string_and_vector(void);


TEST(HypothesisFeaturesIOTest, CanLoadFromStream)
{
    // Test that a stream containing a saved HypothesisFeatures is successfully loaded
    auto generated = random_string_and_vector();
    std::istringstream in(generated.first);
    
    HypothesisFeatures features;
    in >> features;
    
    for(std::size_t n = 0; n < features.numFeatures(); ++n)
    {
        EXPECT_DOUBLE_EQ(generated.second[n], features.featureAt(n)) << "Features didn't match at:" << n;
    }
}


TEST(HypothesisFeaturesIOTest, CanLoadMultipleFromStream)
{
    // Test that a stream containing multiple saved HypothesisFeatures successfully loads all of them
    
    const int kNumFeatures = 5;
    std::ostringstream combinedStream;
    std::vector<Vector> vectors;
    
    // Create the vectors to load as features
    for(int n = 0; n < kNumFeatures; ++n)
    {
        auto generated = random_string_and_vector();
        combinedStream << generated.first << '\n';
        vectors.push_back(generated.second);
    }
    
    std::istringstream in(combinedStream.str());
    // Go through and load HypothesisFeatures one at a time and compare them 
    for(int n = 0; n < kNumFeatures; ++n)
    {
        HypothesisFeatures features;
        in >> features;
        for(std::size_t m = 0; m < features.numFeatures(); ++m)
        {
            EXPECT_DOUBLE_EQ(vectors[n][m], features.featureAt(m)) << "Features didn't match at:(" 
                << n <<',' << m << ')';
        }
    }
}


TEST(HypothesisFeaturesIOTest, CanMakeSingleRoundtrip)
{
    // Test that saving and loading from a stream yields the same results
    auto generated = random_string_and_vector();
    std::istringstream in(generated.first);
    
    HypothesisFeatures features;
    in >> features;
    
    std::ostringstream outStream;
    outStream << features;
    
    std::istringstream inStream(outStream.str());
    HypothesisFeatures loaded;
    inStream >> loaded;
    
    for(std::size_t n = 0; n < loaded.numFeatures(); ++n)
    {
        EXPECT_DOUBLE_EQ(features.featureAt(n), loaded.featureAt(n)) << "Features didn't match at:" << n;
    }
}


std::pair<std::string, Vector> random_string_and_vector(void)
{
    // Generate a random feature vector and save it in a string
    HypothesisFeatures f;
    Vector features(f.numFeatures());
    
    features.randu();
    features[0] = 0;
    
    std::ostringstream str;
    str << f.numFeatures() << ' ';
    for(auto f : features)
    {        
        utils::save_floating_point(str, f);
    }
    // Need one more value for the exploration amount
    str << 0.5 << ' ';
    return std::make_pair(str.str(), features);
}
