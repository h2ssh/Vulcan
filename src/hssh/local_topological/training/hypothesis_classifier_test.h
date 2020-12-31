/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_classifier_test.h
* \author   Collin Johnson
* 
* Declaration of HypothesisClassifierTest.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_CLASSIFIER_TEST_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_CLASSIFIER_TEST_H

#include "hssh/local_topological/area_detection/labeling/hypothesis_classifier.h"
#include "hssh/local_topological/training/labeled_area_data.h"
#include <string>
#include <vector>

namespace vulcan
{
namespace hssh
{

/**
* MapTestResults stores the results of tests run on the specified files.
*/
struct MapTestResults
{
    std::string mapName;
    std::vector<std::pair<HypothesisType, HypothesisType>> tests;
    // .first = expected, .second = actual

    // Overall counts
    int numTests = 0;
    int numCorrectTests = 0;

    // Individual types of errors
    int numDecisionAsPath = 0;
    int numDecisionAsDest = 0;
    int numDestAsPath = 0;
    int numDestAsDecision = 0;
    int numPathAsDecision = 0;
    int numPathAsDest = 0;
};

/**
* HypothesisClassifierTest trains a HypothesisClassifier and then checks it against test data. The results of the test
* along with the learned classifier are available via a number of accessor methods.
* 
* Additionally, the results can be saved via the saveResults method to save the results of the classification tests to
* one file per test map. The format of the results files are as follows:
* 
*     TODO
*/
class HypothesisClassifierTest
{
public:
    
    using ResultsIter = std::vector<MapTestResults>::const_iterator;
    
    /**
    * Constructor for HypothesisClassifierTest.
    * 
    * \param    classifierType      Type of classifier to train and calculate results for
    * \param    trainingData        Training data to use to learn the classifier
    * \param    testData            Test data on which to see how well the classifier works
    */
    HypothesisClassifierTest(const std::string& classifierType,
                             const LabeledAreaData& trainingData, 
                             const LabeledAreaData& testData);
    
    /**
    * Destructor for HypothesisClassifierTest.
    */
    ~HypothesisClassifierTest(void);

    /**
    * run runs the classification training algorithm to create a classifier and then tests the training and
    * test data.
    */
    void run(void);
    
    /**
    * saveClassifier saves the learned classifier used for these tests to the specified file.
    * 
    * \param    basename        Base name of the various classifier files that will be saved
    */
    bool saveClassifier(const std::string& basename) const { return classifier_->save(basename); }
    
    /**
    * overallTestResults returns the accumulated results across all provided test maps.
    */
    MapTestResults overallTestResults(void) const { return overallTest_; }
    
    /**
    * overallTrainingResults returns the accumulated results across all provided training maps.
    */
    MapTestResults overallTrainingResults(void) const { return overallTraining_; }
    
    // Iterator support over all results from the test set to provide more detailed map-by-map analysis
    std::size_t sizeTest(void) const { return testResults_.size(); }
    ResultsIter beginTest(void) const { return testResults_.begin(); }
    ResultsIter endTest(void) const { return testResults_.end(); }
    
    // Iterator support overall results from the training set to provide more detailed map-by-map analysis
    std::size_t sizeTraining(void) const { return trainingResults_.size(); }
    ResultsIter beginTraining(void) const { return trainingResults_.begin(); }
    ResultsIter endTraining(void) const { return trainingResults_.end(); }

private:
    
    MapTestResults overallTest_;
    MapTestResults overallTraining_;
    
    std::vector<MapTestResults> testResults_;
    std::vector<MapTestResults> trainingResults_;
    std::unique_ptr<HypothesisClassifier> classifier_;
    std::string classifierType_;
    LabeledAreaData trainingData_;
    LabeledAreaData testData_;

    std::vector<MapTestResults> calculateResults(const LabeledAreaData& data);
    MapTestResults accumulateResults(const std::vector<MapTestResults>& results);
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_CLASSIFIER_TEST_H
