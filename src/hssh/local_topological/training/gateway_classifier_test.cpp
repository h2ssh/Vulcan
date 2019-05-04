/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway_classifier_test.cpp
* \author   Collin Johnson
* 
* Definition of GatewayClassifierTest.
*/

#include <hssh/local_topological/training/gateway_classifier_test.h>
#include <hssh/local_topological/training/gateway_errors.h>
#include <hssh/local_topological/training/labeled_gateway_data.h>
#include <hssh/local_topological/area_detection/gateways/classifier_based_generator.h>
#include <hssh/local_topological/area_detection/gateways/isovist_voronoi_gateway_generator.h>
#include <hssh/local_topological/area_detection/gateways/isovist_orientation_gateway_generator.h>
#include <hssh/local_topological/area_detection/gateways/gateway_classifier.h>
#include <hssh/local_topological/area_detection/gateways/gateway_locator.h>
#include <utils/histogram.h>
#include <gnuplot-iostream.h>
#include <boost/range/iterator_range.hpp>
#include <tuple>

namespace vulcan 
{
namespace hssh
{

const std::string kTestGwyClassifierTempFile("tmp_gwy_classifier_test");

constexpr int kThreshIdx = 0;
constexpr int kErrorIdx = 1;
constexpr int kFalseNegIdx = 2;
using ThresholdResult = std::tuple<double, double, int>;

constexpr int kRecallIdx = 0;
constexpr int kPrecisionIdx = 1;
using pr_pair_t = std::tuple<double, double>;

std::vector<pr_pair_t> pr_with_threshold(const LabeledGatewayData& data, 
                                         const GatewayClassifier& classifier,
                                         GatewayClassifier::ClassifierType type);
    

GatewayClassifierTest::GatewayClassifierTest(const local_topology_params_t& params,
                                             const LabeledGatewayData& trainingData, 
                                             const LabeledGatewayData& testData,
                                             const SkeletonMap& skeletons,
                                             const IsovistMap& isovists)
{
    int featureRadius = params.areaParams.locatorParams.generatorParams.classifierParams.featureRadius;
    learnClassifier(trainingData, skeletons, isovists, featureRadius);
    
    generators_[kClassifierBasedGeneratorType] = std::make_unique<GatewayLocator>(
        std::make_unique<ClassifierBasedGenerator>(featureRadius, kTestGwyClassifierTempFile)
    );
    
    generators_[kIsovistOrientationGatewayGeneratorType] = std::make_unique<GatewayLocator>(create_gateway_generator(
        kIsovistOrientationGatewayGeneratorType,
        "test",
        params.areaParams.locatorParams.generatorParams
    ));
    
    generators_[kIsovistVoronoiGatewayGeneratorType] = std::make_unique<GatewayLocator>(create_gateway_generator(
        kIsovistVoronoiGatewayGeneratorType,
        "test",
        params.areaParams.locatorParams.generatorParams
    ));
    
    for(auto& g : generators_)
    {
        results_.emplace_back(generateResults(g.first, *g.second, trainingData, testData, skeletons, isovists));
        
        // If we used the classifier gateways, then store the classifier with the results to allow for easier
        // interaction with the UI.
        if(g.first == kClassifierBasedGeneratorType)
        {
            results_.back().classifier = classifier_;
        }
    }
    
    generatePRCurve(trainingData, testData);
}


GatewayClassifierTest::~GatewayClassifierTest(void)
{
    // For std::unique_ptr
}


bool GatewayClassifierTest::saveClassifier(const std::string& basename) const
{
    return classifier_->save(basename);
}


void GatewayClassifierTest::learnClassifier(const LabeledGatewayData& data, 
                                            const SkeletonMap& skeletons,
                                            const IsovistMap& isovists,
                                            int radius)
{
    std::vector<ThresholdResult> results;
    
    classifier_ = GatewayClassifier::LearnClassifier(data);
    classifier_->save(kTestGwyClassifierTempFile);
    
    // The classifier is learned, so now need to find the best threshold for this classifier
    // This is likely to be SLOW
    for(double minProb = 0.0; minProb <= 1.0; minProb += 0.05)
    {
        classifier_->setThreshold(minProb);
        classifier_->save(kTestGwyClassifierTempFile);
        GatewayLocator locator(std::make_unique<ClassifierBasedGenerator>(radius, kTestGwyClassifierTempFile));
        
        double totalTpError = 0.0;
        double totalFpError = 0.0;
        int totalFn = 0;
        
        for(auto& mapName : boost::make_iterator_range(data.beginMaps(), data.endMaps()))
        {
            LabeledGatewayData mapData = data.findMapExamples(mapName);
            VoronoiEdges edges(*skeletons.at(mapName), SKELETON_CELL_REDUCED_SKELETON);
            locator.locateGateways(*skeletons.at(mapName), *isovists.at(mapName));
            
            CellVector gatewayCells;
            for(auto& gw : locator.getGateways())
            {
                gatewayCells.push_back(gw.skeletonCell());
            }
            
            GatewayErrors errors(gatewayCells, mapData, edges);
            totalTpError += errors.truePosDist();
            totalFpError += errors.falsePosDist();
            totalFn += errors.numFalseNeg();
        }
        
        results.emplace_back(minProb, totalTpError + totalFpError, totalFn);
    }
    
    // Find the threshold that best balances the FP and TP error while having no FN (if possible)
    std::cout << "Classifier results:\nThresh\tError\t#fn\n";
    for(auto& res : results)
    {
        std::cout << std::get<kThreshIdx>(res) << '\t' 
            << std::get<kErrorIdx>(res) << '\t'
            << std::get<kFalseNegIdx>(res) << '\n';
    }
    
    classifier_->setThreshold(0.53);
    classifier_->save(kTestGwyClassifierTempFile);
}


GeneratorResults GatewayClassifierTest::generateResults(
    const std::string& generatorName,
    GatewayLocator& generator,
    const LabeledGatewayData& trainingData,
    const LabeledGatewayData& testData,
    const SkeletonMap& skeletons,
    const IsovistMap& isovists
)
{
    GeneratorResults genResults;
    genResults.generatorName = generatorName;
    genResults.trainingResults = testGenerator(generator, trainingData, skeletons, isovists);
    genResults.testResults = testGenerator(generator, testData, skeletons, isovists);
    genResults.overallTest = calculateOverallResults(genResults.testResults);
    genResults.overallTraining = calculateOverallResults(genResults.trainingResults);
    return genResults;
}


std::vector<GatewayMapResults> GatewayClassifierTest::testGenerator(GatewayLocator& locator,
                                                                    const LabeledGatewayData& data,
                                                                    const SkeletonMap& skeletons,
                                                                    const IsovistMap& isovists)
{
    std::vector<GatewayMapResults> results;
    
    for(auto& mapName : boost::make_iterator_range(data.beginMaps(), data.endMaps()))
    {
        const auto& skeleton = skeletons.find(mapName)->second;
        const auto& field = isovists.find(mapName)->second;
        
        locator.clearGateways();
        locator.locateGateways(*skeleton, *field);
        
        VoronoiEdges edges(*skeleton, SKELETON_CELL_REDUCED_SKELETON);
        results.push_back(calculateResults(mapName, 
                                           locator.getGateways(), 
                                           edges,
                                           data));
    }
    
    return results;
}


GatewayMapResults GatewayClassifierTest::calculateResults(const std::string& mapName,
                                                          const std::vector<Gateway>& gateways,
                                                          const VoronoiEdges& edges,
                                                          const LabeledGatewayData& truth)
{
    GatewayMapResults results;
    results.mapName = mapName;
    
    LabeledGatewayData mapTruth = truth.findMapExamples(mapName);
    
    CellVector gatewayCells;
    for(auto& gwy : gateways)
    {
        gatewayCells.push_back(gwy.skeletonCell());
    }
    GatewayErrors errors(gatewayCells, mapTruth, edges);
    
    results.numTruePositives = errors.numTruePos();
    results.numFalsePositives = errors.numFalsePos();
    results.numFalseNegatives = errors.numFalseNeg();
    results.numActualGateways = results.numTruePositives + results.numFalseNegatives;
    
    return results;
}


GatewayMapResults GatewayClassifierTest::calculateOverallResults(const std::vector<GatewayMapResults>& results)
{
    GatewayMapResults overall;
    overall.mapName = "Overall";
    
    for(auto& r : results)
    {
        overall.numActualGateways += r.numActualGateways;
        overall.numTruePositives += r.numTruePositives;
        overall.numFalsePositives += r.numFalsePositives;
        overall.numFalseNegatives += r.numFalseNegatives;
    }
    
    return overall;
}


void GatewayClassifierTest::generatePRCurve(const LabeledGatewayData& trainingData, 
                                            const LabeledGatewayData& testData)
{
//     std::vector<pr_pair_t> trainingSvm = pr_with_threshold(trainingData, *classifier_, GatewayClassifier::svm);
    std::vector<pr_pair_t> trainingLr = pr_with_threshold(trainingData, *classifier_, GatewayClassifier::adaboost);
//     std::vector<pr_pair_t> testSvm = pr_with_threshold(testData, *classifier_, GatewayClassifier::svm);
    std::vector<pr_pair_t> testLr = pr_with_threshold(testData, *classifier_, GatewayClassifier::adaboost);
    
    static Gnuplot plot;
    
    plot << "set yrange [0:1.01]\n";
    plot << "set xrange [0:1]\n";
    plot << "set title 'P-R Curve for gateway probability'\n";
    plot << "set xlabel 'Recall and (1 - threshold)'\n";
    plot << "set ylabel 'Precision'\n";
//     plot << "plot '-' with lines title 'Training SVM',";
    plot << "plot '-' with lines title 'Training LR',";
//     plot << "'-' with lines title 'Test SVM',";
    plot << "'-' with lines title 'Test LR'\n";
//     plot << "plot '-' with lines title 'Dist Weights', '-' with lines title 'Line Weights'\n";
//     plot.send1d(trainingSvm);
    plot.send1d(trainingLr);
//     plot.send1d(testSvm);
    plot.send1d(testLr);
}

std::vector<pr_pair_t> pr_with_threshold(const LabeledGatewayData& data, 
                                         const GatewayClassifier& classifier,
                                         GatewayClassifier::ClassifierType type)
{
    const double kStepsize = 0.01;
    const int kNumSteps = (1.0 / kStepsize) + 1;
    
    constexpr int kTp = 0;
    constexpr int kFp = 1;
    constexpr int kFn = 2;
    
    using Results = std::tuple<int, int, int>;
    std::vector<Results> results(kNumSteps, std::make_tuple(0, 0, 0));
    
    utils::Histogram probHist(20);
    
    for(auto& example : data)
    {
        auto result = classifier.classifyGateway(example.features, type);
        probHist.addValue(result.probability);
        
        for(int n = 0; n < kNumSteps; ++n)
        {
            const double threshold = 1.0 - (n * kStepsize);
            if((result.probability >= threshold) && example.isGateway)
            {
                ++std::get<kTp>(results[n]);
            }
            else if((result.probability >= threshold) && !example.isGateway)
            {
                ++std::get<kFp>(results[n]);
            }
            else if(result.probability < threshold && example.isGateway)
            {
                ++std::get<kFn>(results[n]);
            }
        }
    }
    
//     std::cout << probHist << '\n';
    
    std::vector<pr_pair_t> prResults(kNumSteps);
    for(int n = 0; n < kNumSteps; ++n)
    {
        int totalTrue = std::get<kTp>(results[n]) + std::get<kFn>(results[n]);
        std::get<kRecallIdx>(prResults[n]) = (totalTrue > 0) ? 
            static_cast<double>(std::get<kTp>(results[n])) / totalTrue : 0.0;
        
        int totalPositives = std::get<kTp>(results[n]) + std::get<kFp>(results[n]);
        std::get<kPrecisionIdx>(prResults[n]) = (totalPositives > 0) ? 
            static_cast<double>(std::get<kTp>(results[n])) / totalPositives : 0.0;
    }
    
    return prResults;
}
    
} // namespace hssh
} // namespace vulcan
