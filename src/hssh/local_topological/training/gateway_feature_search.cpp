/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway_feature_search.cpp
* \author   Collin Johnson
*
* gateway_feature_search is a program that performs a search amongst possible gateway feature sets to find the best
* possible set of features for the provided training data. The program takes in a files in the format defined in
* area_labels.h.
* 
* gateway_feature_search is run using the following command-line:
*
*   ./gateway_feature_search --labels 'maps.lbl' --config-file 'local_topo_hssh.cfg'
*/

#include "hssh/local_topological/training/area_labels.h"
#include "hssh/local_topological/training/gateway_classifier_test.h"
#include "hssh/local_topological/training/gateway_errors.h"
#include "hssh/local_topological/training/local_topo_area_editor.h"
#include "hssh/local_topological/area_detection/gateways/classifier_based_generator.h"
#include "hssh/local_topological/area_detection/gateways/feature_extraction.h"
#include "hssh/local_topological/area_detection/gateways/gateway_classifier.h"
#include "hssh/local_topological/area_detection/gateways/gateway_locator.h"
#include "hssh/local_topological/params.h"
#include "utils/command_line.h"
#include "utils/config_file.h"

using namespace vulcan;
using namespace vulcan::hssh;

////////// Command line arguments ////////////
const std::string kLabelsArg("labels");
const std::string kOutputDirArg("output-dir");

//////// File name pieces ////////
const std::string kIncrementalExt("incr");
const std::string kFullExt("full");

using ScalarVec = std::vector<utils::Isovist::Scalar>;

constexpr int kThreshIdx = 0;
constexpr int kErrorIdx = 1;
constexpr int kFalseNegIdx = 2;
constexpr int kTruePosIdx = 3;
constexpr int kFalsePosIdx = 4;
using ThresholdResult = std::tuple<double, double, int, int, int>;

// One possible set of features to use
struct FeatureSet
{
    int id;
    int radius;
    ScalarVec scalars;
    
    LabeledGatewayData data;
};

std::vector<FeatureSet> create_feature_sets(const LabelsFile::BuildingLabels& labels);
void compute_feature_set_data(FeatureSet& features, const LabelsFile::BuildingLabels& labels);
std::vector<ScalarVec> generate_feature_combinations(void);
void gen_features_helper(ScalarVec::const_iterator begin, 
                         ScalarVec::const_iterator end,
                         ScalarVec featuresSoFar,
                         std::vector<ScalarVec>& allFeatures);
std::vector<ThresholdResult> learn_classifier_for_feature_set(const FeatureSet& features, 
                                                              const LabelsFile::BuildingLabels& labels);


int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file for calculating gateway features", true, "local_topo_hssh.cfg"});
    arguments.push_back({kLabelsArg, "File containing the map names to be used for classification", false, ""});
    
    utils::CommandLine cmdLine(argc, argv, arguments);
    if(!cmdLine.verify())
    {
        return -1;
    }
    
    utils::ConfigFile config(cmdLine.configName());
    LabelsFile labels = load_labels_file(cmdLine.argumentValue(kLabelsArg), local_topology_params_t(config));
    
    std::cout << "INFO: Loaded data for " << labels.fullMaps.size() << " full buildings.\n";
    
    auto featureSets = create_feature_sets(labels.fullMaps);
    std::vector<std::vector<ThresholdResult>> allResults;
    
    for(auto& set : featureSets)
    {
        compute_feature_set_data(set, labels.fullMaps);
        allResults.push_back(learn_classifier_for_feature_set(set, labels.fullMaps));
        set.data.clear();   // clear the data to avoid memory use explosion
    }
    
    auto bestIt = std::min_element(allResults.begin(), allResults.end(), [](const auto& lhs, const auto& rhs) {
        return std::get<kErrorIdx>(lhs.front()) < std::get<kErrorIdx>(rhs.front());
    });
    
    if(bestIt != allResults.end())
    {
        int bestIdx = std::distance(allResults.begin(), bestIt);
        std::cout << "============== Best feature set by Error ==================\nRadius: " 
            << featureSets[bestIdx].radius << " Scalars:";
        for(auto& scalar : featureSets[bestIdx].scalars)
        {
            std::cout << ' ' << utils::Isovist::scalarName(scalar);
        }
        std::cout << '\n';
    }
    
    bestIt = std::min_element(allResults.begin(), allResults.end(), [](const auto& lhs, const auto& rhs) {
        return std::get<kFalseNegIdx>(lhs.front()) < std::get<kFalseNegIdx>(rhs.front());
    });
    
    if(bestIt != allResults.end())
    {
        int bestIdx = std::distance(allResults.begin(), bestIt);
        std::cout << "============== Best feature set by False Negatives ==================\nRadius: " 
            << featureSets[bestIdx].radius << " Scalars:";
        for(auto& scalar : featureSets[bestIdx].scalars)
        {
            std::cout << ' ' << utils::Isovist::scalarName(scalar);
        }
        std::cout << '\n';
    }
    
    bestIt = std::min_element(allResults.begin(), allResults.end(), [](const auto& lhs, const auto& rhs) {
        return (std::get<kTruePosIdx>(lhs.front()) + std::get<kFalsePosIdx>(lhs.front())) 
            < (std::get<kTruePosIdx>(rhs.front()) + std::get<kFalsePosIdx>(rhs.front()));
    });
    
    if(bestIt != allResults.end())
    {
        int bestIdx = std::distance(allResults.begin(), bestIt);
        std::cout << "============== Best feature set by Positives ==================\nRadius: " 
            << featureSets[bestIdx].radius << " Scalars:";
        for(auto& scalar : featureSets[bestIdx].scalars)
        {
            std::cout << ' ' << utils::Isovist::scalarName(scalar);
        }
        std::cout << '\n';
    }
    
    return 0;
}


std::vector<FeatureSet> create_feature_sets(const LabelsFile::BuildingLabels& labels)
{
    std::vector<FeatureSet> problems;
    
    auto scalarCombos = generate_feature_combinations();
    
    int id = 0;
    for(int radius = 0; radius <= 4; ++radius)
    {
        for(const ScalarVec& scalars : scalarCombos)
        {
            FeatureSet features;
            features.id = id++;
            features.radius = radius;
            features.scalars = scalars;
            problems.emplace_back(std::move(features));
        }
    }
    
    return problems;
}


void compute_feature_set_data(FeatureSet& features, const LabelsFile::BuildingLabels& labels)
{
    // For each building name, create a new ClassificationProblem. The problem for each building will consist of
    // all other buildings being training data and this building being test data
    for(auto& building : labels)
    {
        // Go through all the building data
        for(const MapLabels& map : building.second)
        {
            std::vector<LabeledGatewayFeatures> labeledFeatures;
            auto skeletonFeat = extract_gateway_features(*map.edges, *map.isovists, features.scalars, features.radius);
            
            for(const LabeledGatewayFeatures& cell : map.gatewayData)
            {
                if(skeletonFeat.find(cell.cell) != skeletonFeat.end())
                {
                    LabeledGatewayFeatures cellFeatures = cell;
                    cellFeatures.features = skeletonFeat[cell.cell];
                    labeledFeatures.push_back(cellFeatures);
                }
            }
            
            features.data.addExamples(map.name.toMapName(), labeledFeatures.begin(), labeledFeatures.end());
        }
    }
}


std::vector<ScalarVec> generate_feature_combinations(void)
{
//     const std::vector<utils::Isovist::Scalar> kScalars = { 
//         utils::Isovist::kShapeEccentricity,
//         utils::Isovist::kWeightedOrientation,
//         utils::Isovist::kOrientation,
// //         utils::Isovist::kDavg,
//         utils::Isovist::kShapeDistAvg,
//         utils::Isovist::kDskewness,
// //         utils::Isovist::kDstd,
// //         utils::Isovist::kDistRelationAvg,
//         utils::Isovist::kDistRelationStd,
// //         utils::Isovist::kRayCompactness,
//         utils::Isovist::kArea,
//         utils::Isovist::kPerimeter
//     };
    
    std::vector<utils::Isovist::Scalar> kScalars = { 
        utils::Isovist::kArea,
        utils::Isovist::kPerimeter,
        utils::Isovist::kCircularity,
        utils::Isovist::kOrientation,
        utils::Isovist::kWeightedOrientation,
        utils::Isovist::kShapeEccentricity,
        utils::Isovist::kShapeCompactness,
        utils::Isovist::kShapeWaviness,
        utils::Isovist::kMinDistOrientation,
        utils::Isovist::kMaxThroughDist,                // max dist when considering line of rays in both dirs
        utils::Isovist::kMaxThroughDistOrientation,     // orientation of max through dist
        utils::Isovist::kDmax,
        utils::Isovist::kDmin,
        utils::Isovist::kDavg,
        utils::Isovist::kDstd,
        utils::Isovist::kDVariation,
        utils::Isovist::kDskewness,
        utils::Isovist::kShapeDistAvg,
        utils::Isovist::kShapeDistStd,
        utils::Isovist::kShapeDistVariation,
        utils::Isovist::kDeltaAvg,
        utils::Isovist::kDeltaStd,
        utils::Isovist::kDeltaVariation,
        utils::Isovist::kDistRelationAvg,
        utils::Isovist::kDistRelationStd,
        utils::Isovist::kRayCompactness
    };
    
    std::vector<ScalarVec> combinations;
    combinations.push_back(kScalars);
//     gen_features_helper(kScalars.begin(), kScalars.end(), ScalarVec(), combinations);
    return combinations;
}


void gen_features_helper(ScalarVec::const_iterator begin, 
                         ScalarVec::const_iterator end,
                         ScalarVec featuresSoFar,
                         std::vector<ScalarVec>& allFeatures)
{
    // Recursively generate all possible feature vectors
    // Base case -- just add the feature vector
    if(begin == end)
    {
        // If there are some features, then add this set of scalars to the possible combinations
        if(!featuresSoFar.empty())
        {
            allFeatures.push_back(featuresSoFar);
        }
        return;
    }
    
    // Create both possible choices -- add this scalar or don't add it.
    ScalarVec nextFeatures = featuresSoFar;
    gen_features_helper(begin + 1, end, nextFeatures, allFeatures);
    
    nextFeatures.push_back(*begin);
    gen_features_helper(begin + 1, end, nextFeatures, allFeatures);
}


std::vector<ThresholdResult> learn_classifier_for_feature_set(const FeatureSet& features, 
                                                              const LabelsFile::BuildingLabels& labels)
{
    const std::string kClassifierTempFile("tmp_gwy_classifier");
    
    std::vector<ThresholdResult> results;
    
    set_default_gateway_features(features.scalars);
    auto classifier = GatewayClassifier::LearnClassifier(features.data);
    classifier->save(kClassifierTempFile);
    
    CellVector gatewayCells;
    
    std::map<std::string, SkeletonFeatures> mapGwyFeatures;

    for(auto& mapName : boost::make_iterator_range(features.data.beginMaps(), features.data.endMaps()))
    {
        const MapLabels* storedInfo = nullptr;
        
        for(auto& building : labels)
        {
            // Go through all the building data
            for(const MapLabels& map : building.second)
            {
                if(map.name.toMapName() == mapName)
                {
                    assert(storedInfo == nullptr);
                    storedInfo = &map;
                    break;
                }
            }
        }
        
        assert(storedInfo);
        mapGwyFeatures[mapName] =extract_gateway_features_default(*storedInfo->edges,
                                                                  *storedInfo->isovists,
                                                                  features.radius);
    }
    
    // The classifier is learned, so now need to find the best threshold for this classifier
    for(double minProb = 0.5; minProb <= 1.0; minProb += 0.02)
    {
        classifier->setThreshold(minProb);
        classifier->save(kClassifierTempFile);
//         GatewayLocator locator(std::make_unique<ClassifierBasedGenerator>(features.radius, kClassifierTempFile));
        
        double totalTpError = 0.0;
        double totalFpError = 0.0;
        int totalTp = 0;
        int totalFp = 0;
        int totalFn = 0;
        
        for(auto& mapName : boost::make_iterator_range(features.data.beginMaps(), features.data.endMaps()))
        {
            const MapLabels* storedInfo = nullptr;
            
            for(auto& building : labels)
            {
                // Go through all the building data
                for(const MapLabels& map : building.second)
                {
                    if(map.name.toMapName() == mapName)
                    {
                        storedInfo = &map;
                        break;
                    }
                }
            }
            
            LabeledGatewayData mapData = features.data.findMapExamples(mapName);
//             locator.locateGateways(*storedInfo->skeleton, *storedInfo->isovists);
            
            gatewayCells.clear();
            const auto& gwyFeatures = mapGwyFeatures[mapName];
            for(auto& isovist : *storedInfo->isovists)
            {
                if(gwyFeatures.find(isovist.cell()) != gwyFeatures.end())
                {
                    auto classification = classifier->classifyGateway(gwyFeatures.at(isovist.cell()), 
                                                                      GatewayClassifier::adaboost);
                    if(classification.probability > minProb)
                    {
                        gatewayCells.push_back(isovist.cell());
                    }
                }
            }
            
            GatewayErrors errors(gatewayCells, mapData, *storedInfo->edges);
            totalTpError += errors.truePosDist();
            totalFpError += errors.falsePosDist();
            totalFn += errors.numFalseNeg();
            totalTp += errors.numTruePos();
            totalFp += errors.numFalsePos();
        }
        
        results.emplace_back(minProb, totalTpError + totalFpError, totalFn, totalTp, totalFp);
    }
    
    static std::ofstream out("gateway_feature_search_out.txt");
    static int id = 0;
    
    // Find the threshold that best balances the FP and TP error while having no FN (if possible)
    out << "\n\nFeature set " << id++ << ": ";
    for(auto& scalar : features.scalars)
    {
        out << utils::Isovist::scalarName(scalar) << ' ';
    }
    out << "Radius: " << features.radius << "\nThresh\tError\t\t#fn\t#tp\t#fp\tp\tr\n";
    for(auto& res : results)
    {
        double precision = std::get<kTruePosIdx>(res) / static_cast<double>(std::get<kTruePosIdx>(res) 
            + std::get<kFalsePosIdx>(res));
        double recall = std::get<kTruePosIdx>(res) / static_cast<double>(std::get<kTruePosIdx>(res) 
            + std::get<kFalseNegIdx>(res));
        
        out << std::get<kThreshIdx>(res) << '\t' 
            << std::get<kErrorIdx>(res) << "\t\t"
            << std::get<kFalseNegIdx>(res) << '\t'
            << std::get<kTruePosIdx>(res) << '\t'
            << std::get<kFalsePosIdx>(res) << '\t'
            << precision << '\t'
            << recall << '\t' << std::endl;
    }
    
    return results;
}
