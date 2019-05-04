/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway_classifier_test.h
* \author   Collin Johnson
* 
* Declaration of GatewayClassifierTest.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_TRAINING_GATEWAY_CLASSIFIER_TEST_H
#define HSSH_LOCAL_TOPOLOGICAL_TRAINING_GATEWAY_CLASSIFIER_TEST_H

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace vulcan
{
namespace hssh 
{

class Gateway;
class GatewayClassifier;
class GatewayLocator;
class LabeledGatewayData;
class VoronoiEdges;
class VoronoiIsovistField;
class VoronoiSkeletonGrid;
struct local_topology_params_t;

// Helper types for finding all types of gateways
using SkeletonMap = std::map<std::string, std::shared_ptr<VoronoiSkeletonGrid>>;
using IsovistMap = std::map<std::string, std::shared_ptr<VoronoiIsovistField>>;

/**
* GatewayMapResults stores the results of finding gateways in a particular map for single type of generator.
*/
struct GatewayMapResults
{
    std::string mapName;
    
    int numActualGateways = 0;
    int numTruePositives = 0;
    int numFalsePositives = 0;
    int numFalseNegatives = 0;
};

/**
* GeneratorResults stores the results of finding gateways in all maps for a single gateway generator.
*/
struct GeneratorResults 
{
    std::string generatorName;
    std::vector<GatewayMapResults> trainingResults;
    std::vector<GatewayMapResults> testResults;
    std::shared_ptr<GatewayClassifier> classifier;       // classifier learned, if any
    
    GatewayMapResults overallTest;
    GatewayMapResults overallTraining;
};

/**
* GatewayClassifierTest runs the following test for all types of gateway generators:
* 
*   - Generate all gateways via the GatewayLocator
*   - Compute the precision and recall for each generator w/default settings
*   - Compute and display a PR-curve for the classifier-based locator using both SVM and LR
*/
class GatewayClassifierTest
{
public:
    
    using ResultsIter = std::vector<GeneratorResults>::const_iterator;
    
    /**
    * Constructor for GatewayClassifierTest.
    * 
    * \param    params              Parameters for running the test
    * \param    trainingData        Training data to use
    * \param    testData            Test data for comparison
    * \param    skeletons           Skeletons for the maps in the data
    * \param    isovists            Isovists for the maps in the data
    */
    GatewayClassifierTest(const local_topology_params_t& params,
                          const LabeledGatewayData& trainingData, 
                          const LabeledGatewayData& testData,
                          const SkeletonMap& skeletons,
                          const IsovistMap& isovists);
    
    /**
    * Destructor for GatewayClassifierTest.
    */
    ~GatewayClassifierTest(void);
    
    /**
    * saveClassifier saves the classifier created during the test.
    * 
    * \param    basename        Basename of the files that will be created by the classifier
    * \return   True if the classifier is successfully saved to disk.
    */
    bool saveClassifier(const std::string& basename) const;
    
    // Iterate over the results for the different generators
    std::size_t size(void) const { return results_.size(); }
    ResultsIter begin(void) const { return results_.begin(); }
    ResultsIter end(void) const { return results_.end(); }

private:
    
    std::vector<GeneratorResults> results_;
    std::map<std::string, std::unique_ptr<GatewayLocator>> generators_;
    std::shared_ptr<GatewayClassifier> classifier_;
    
    void learnClassifier(const LabeledGatewayData& trainingData, 
                         const SkeletonMap& skeletons,
                         const IsovistMap& isovists,
                         int radius);
    
    GeneratorResults generateResults(const std::string& generatorName,
                                     GatewayLocator& generator,
                                     const LabeledGatewayData& trainingData,
                                     const LabeledGatewayData& testData,
                                     const SkeletonMap& skeletons,
                                     const IsovistMap& isovists);
    std::vector<GatewayMapResults> testGenerator(GatewayLocator& locator,
                                                 const LabeledGatewayData& data,
                                                 const SkeletonMap& skeletons,
                                                 const IsovistMap& isovists);
    GatewayMapResults calculateResults(const std::string& mapName,
                                       const std::vector<Gateway>& gateways,
                                       const VoronoiEdges& edges,
                                       const LabeledGatewayData& truth);
    GatewayMapResults calculateOverallResults(const std::vector<GatewayMapResults>& results);
    void generatePRCurve(const LabeledGatewayData& trainingData, const LabeledGatewayData& testData);
};

} // namespace hssh 
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_TRAINING_GATEWAY_CLASSIFIER_TEST_H
