/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_classifier_test.cpp
* \author   Collin Johnson
*
* Definition of HypothesisClassifierTest.
*/

#include <hssh/local_topological/training/hypothesis_classifier_test.h>
#include <hssh/local_topological/training/labeled_area_data.h>
#include <utils/stub.h>
#include <boost/range/iterator_range.hpp>

namespace vulcan
{
namespace hssh
{

MapTestResults generate_results_for_map(const HypothesisClassifier& classifier,
                                        const std::string&              mapName,
                                        LabeledAreaData::FeatureIter    begin,
                                        LabeledAreaData::FeatureIter    end);


HypothesisClassifierTest::HypothesisClassifierTest(const std::string& classifierType,
                                                   const LabeledAreaData& trainingData,
                                                   const LabeledAreaData& testData)
: classifierType_(classifierType)
, trainingData_(trainingData)
, testData_(testData)
{
}


HypothesisClassifierTest::~HypothesisClassifierTest(void)
{
    // For std::unique_ptr
}


void HypothesisClassifierTest::run(void)
{
    classifier_ = HypothesisClassifier::LearnClassifier(trainingData_);

    testResults_ = calculateResults(testData_);
    trainingResults_ = calculateResults(trainingData_);
    overallTest_ = accumulateResults(testResults_);
    overallTraining_ = accumulateResults(trainingResults_);
}


std::vector<MapTestResults> HypothesisClassifierTest::calculateResults(const LabeledAreaData& data)
{
    std::vector<MapTestResults> dataResults;

    for(auto& map : boost::make_iterator_range(data.beginMaps(), data.endMaps()))
    {
        dataResults.push_back(generate_results_for_map(*classifier_,
                                                       map,
                                                       data.beginMapExamples(map),
                                                       data.endMapExamples(map)));
    }

    return dataResults;
}


MapTestResults HypothesisClassifierTest::accumulateResults(const std::vector<MapTestResults>& results)
{
    MapTestResults overall;

    for(auto& result : results)
    {
        overall.numTests += result.numTests;
        overall.numCorrectTests += result.numCorrectTests;
        overall.numDecisionAsPath += result.numDecisionAsPath;
        overall.numDecisionAsDest += result.numDecisionAsDest;
        overall.numDestAsPath += result.numDestAsPath;
        overall.numDestAsDecision += result.numDestAsDecision;
        overall.numPathAsDecision += result.numPathAsDecision;
        overall.numPathAsDest += result.numPathAsDest;
    }

    return overall;
}


MapTestResults generate_results_for_map(const HypothesisClassifier& classifier,
                                        const std::string&              mapName,
                                        LabeledAreaData::FeatureIter    begin,
                                        LabeledAreaData::FeatureIter    end)
{
    MapTestResults results;
    results.mapName  = mapName;
    results.numTests = std::distance(begin, end);

    for(auto& example : boost::make_iterator_range(begin, end))
    {
        results.tests.push_back(std::make_pair(example.type, classifier.classify(example.features)));
    }

    for(auto& test : results.tests)
    {
        if(test.first == test.second)
        {
            ++results.numCorrectTests;
        }
        else if((test.first == HypothesisType::kDecision) && (test.second == HypothesisType::kPath))
        {
            ++results.numDecisionAsPath;
        }
        else if((test.first == HypothesisType::kDecision) && (test.second == HypothesisType::kDest))
        {
            ++results.numDecisionAsDest;
        }
        else if((test.first == HypothesisType::kDest) && (test.second == HypothesisType::kPath))
        {
            ++results.numDestAsPath;
        }
        else if((test.first == HypothesisType::kDest) && (test.second == HypothesisType::kDecision))
        {
            ++results.numDestAsDecision;
        }
        else if((test.first == HypothesisType::kPath) && (test.second == HypothesisType::kDecision))
        {
            ++results.numPathAsDecision;
        }
        else if((test.first == HypothesisType::kPath) && (test.second == HypothesisType::kDest))
        {
            ++results.numPathAsDest;
        }
    }

    return results;
}

} // namespace hssh
} // namespace vulcan
