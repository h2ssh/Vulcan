/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     labeled_area_data_test.cpp
* \author   Collin Johnson
*
* Unit tests for ensuring proper functionality of LabeledAreaData.
*/

#include <hssh/local_topological/training/labeled_area_data.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <sstream>
#include <cstdlib>

using namespace vulcan;
using namespace vulcan::hssh;


HypothesisFeatures create_random_features(void);
LabeledFeatures    create_random_labeled_features(void);
LabeledAreaData    create_random_labeled_data(void);


TEST(LabeledAreaDataIOTest, CanRoundtripData)
{
    auto outData = create_random_labeled_data();
    
    std::ostringstream out;
    out << outData;
    
    std::istringstream in(out.str());
    LabeledAreaData inData;
    in >> inData;
    
    EXPECT_TRUE(outData == inData);
    EXPECT_TRUE(inData == outData);
}


TEST(LabeledAreaDataAddExampleTest, ExampleIsAdded)
{
    const std::string kMapName = "test_map";
    std::vector<LabeledFeatures> examples;
    examples.push_back(create_random_labeled_features());
    
    LabeledAreaData data;
    data.addExamples(kMapName, examples.begin(), examples.end());
    
    EXPECT_TRUE(std::find(data.begin(), data.end(), examples.front()) != data.end());
}


TEST(LabeledAreaDataAddExampleTest, SequentialAddingIsCorrect)
{
    const std::string kMapNameA = "test_map_a";
    const std::string kMapNameB = "test_map_b";
    
    std::vector<LabeledFeatures> examplesA;
    examplesA.push_back(create_random_labeled_features());
    examplesA.push_back(create_random_labeled_features());
    
    std::vector<LabeledFeatures> examplesB;
    examplesB.push_back(create_random_labeled_features());
    examplesB.push_back(create_random_labeled_features());
    
    LabeledAreaData data;
    data.addExamples(kMapNameA, examplesA.begin(), examplesA.end());
    data.addExamples(kMapNameB, examplesB.begin(), examplesB.end());
    
    
    auto mapAData = data.findMapExamples(kMapNameA);
    for(auto& e : examplesA)
    {
        EXPECT_TRUE(std::find(mapAData.begin(), mapAData.end(), e) != mapAData.end());
    }
    
    auto mapBData = data.findMapExamples(kMapNameB);
    for(auto& e : examplesB)
    {
        EXPECT_TRUE(std::find(mapBData.begin(), mapBData.end(), e) != mapBData.end());
    }
}


TEST(LabeledAreaDataAddExampleTest, InterleavedAddingIsCorrect)
{
    const std::string kMapNameA = "test_map_a";
    const std::string kMapNameB = "test_map_b";
    
    std::vector<LabeledFeatures> examplesA;
    examplesA.push_back(create_random_labeled_features());
    examplesA.push_back(create_random_labeled_features());
    
    std::vector<LabeledFeatures> examplesB;
    examplesB.push_back(create_random_labeled_features());
    examplesB.push_back(create_random_labeled_features());
    
    LabeledAreaData data;
    data.addExamples(kMapNameA, examplesA.begin(), examplesA.begin() + 1);
    data.addExamples(kMapNameB, examplesB.begin(), examplesB.begin() + 1);
    data.addExamples(kMapNameA, examplesA.begin()+1, examplesA.end());
    data.addExamples(kMapNameB, examplesB.begin()+1, examplesB.end());
    
    auto mapAData = data.findMapExamples(kMapNameA);
    for(auto& e : examplesA)
    {
        EXPECT_TRUE(std::find(mapAData.begin(), mapAData.end(), e) != mapAData.end());
    }
    
    auto mapBData = data.findMapExamples(kMapNameB);
    for(auto& e : examplesB)
    {
        EXPECT_TRUE(std::find(mapBData.begin(), mapBData.end(), e) != mapBData.end());
    }
}


TEST(LabeledAreaDataRemovedMapTest, ExamplesAreRemoved)
{
    const std::string kMapNameA = "test_map_a";
    const std::string kMapNameB = "test_map_b";

    std::vector<LabeledFeatures> examplesA;
    examplesA.push_back(create_random_labeled_features());
    examplesA.push_back(create_random_labeled_features());

    std::vector<LabeledFeatures> examplesB;
    examplesB.push_back(create_random_labeled_features());
    examplesB.push_back(create_random_labeled_features());

    LabeledAreaData data;
    data.addExamples(kMapNameA, examplesA.begin(), examplesA.end());
    data.addExamples(kMapNameB, examplesB.begin(), examplesB.end());

    // Check that all are correctly removed
    int numRemoved = data.removeMapExamples(kMapNameA);
    EXPECT_EQ(2, numRemoved);

    // There should be no remaining examples associated with the removed map name
    auto mapAData = data.findMapExamples(kMapNameA);
    EXPECT_TRUE(mapAData.empty());

    // Ensure the other data are unchanged
    auto mapBData = data.findMapExamples(kMapNameB);
    for(auto& e : examplesB)
    {
        EXPECT_TRUE(std::find(mapBData.begin(), mapBData.end(), e) != mapBData.end());
    }
}


HypothesisFeatures create_random_features(void)
{
    HypothesisFeatures f;
    Vector features(f.numFeatures());
    features.randu();
    features[0] = 1.0;
    
    return HypothesisFeatures(features, 0.5, 0.99);
}


LabeledFeatures create_random_labeled_features(void)
{
    LabeledFeatures features;
    features.features = create_random_features();
    
    int type = drand48() * 2.99999;
    switch(type)
    {
    case 0:
        features.type = HypothesisType::kPath;
        break;
        
    case 1:
        features.type = HypothesisType::kDest;
        break;
        
    case 2:
    default:
        features.type = HypothesisType::kDecision;
        break;
    }
    
    return features;
}


LabeledAreaData create_random_labeled_data(void)
{
    LabeledAreaData data;
    std::vector<LabeledFeatures> examples(10);
    for(auto& e : examples)
    {
        e = create_random_labeled_features();
    }
    
    data.addExamples("random data", examples.begin(), examples.end());
    data.addExamples("other data", examples.begin(), examples.end());
    return data;
}
