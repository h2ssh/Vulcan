/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     train_area_classifiers.cpp
* \author   Collin Johnson
*
* train_area_classifiers is a program to train leave-one-out classifiers from a collection of labeled maps. The
* leave-one-out classifiers can be trained for both full and incremental maps. In all cases, if maps begin with the
* same substring, then they will be left out of the training data. This allows for multiple floors of a building to
* be in the set of labeled maps without polluting the classifier by training on a different floor of a building, which
* will surely have many of the same properties (width of hallways, etc).
*
* Each map has its data stored in multiple files all associated with the map's filename. The naming scheme for maps
* must follow this format:
*
*   'building''floor'(_'incremental number')
*
*   - The building name must not contain numbers.
*   - The floor must be a number.
*   - The incremental number must be separated by an underscore and be a number.
*
* For example, the third floor of the EECS building is eecs3. The thirtieth map in an incremental sequence of
* maps of the third floor of the EECS building is eecs3_30.
*
* The input to the train_area_classifiers program is an .lbl file, which has the following format (per area_labels.h):
*
*       map_name1 map_directory1
*       map_name2 map_directory2
*               .
*               .
*               .
*       map_nameN map_directoryN
*
* The map_name is assumed to end in .lpm and thus the file extension should be left off. For simplicity, don't put any
* underscores in the 'building' name.
*
* The map_directory should be the absolute directory of the map file.
*
* train_area_classifiers is run using the following command-line:
*
*   ./train_area_classifier --labels 'maps.lbl' --output-dir 'path_to_output_directory' --config-file 'local_topo_hssh.cfg'
*/

#include "hssh/local_topological/training/area_labels.h"
#include "hssh/local_topological/training/hypothesis_classifier_test.h"
#include "hssh/local_topological/training/labeled_boundary_data.h"
#include "hssh/local_topological/training/local_topo_area_editor.h"
#include "hssh/local_topological/area_detection/gateways/gateway_classifier.h"
#include "hssh/local_topological/area_detection/gateways/classifier_based_generator.h"
#include "hssh/local_topological/area_detection/labeling/boundary_classifier.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_classifier.h"
#include "hssh/local_topological/params.h"
#include "utils/command_line.h"
#include "utils/config_file.h"

using namespace vulcan;
using namespace vulcan::hssh;

////////// Command line arguments ////////////
const std::string kLabelsArg("labels");
const std::string kOutputDirArg("output-dir");
const std::string kAllArg("train-all");
const std::string kGwyArg("train-gwy");
const std::string kAreaArg("train-area");

//////// File name pieces ////////
const std::string kIncrementalExt("_incr");
const std::string kFullExt("");


struct ClassificationProblem
{
    std::string testBuildingName;
    std::string fileExt;

    std::vector<MapLabels*> testMaps;
    std::vector<MapLabels*> trainingMaps;
};


std::vector<ClassificationProblem> create_classification_problems(LabelsFile::BuildingLabels& labels);

void compute_gateway_features(LabelsFile::BuildingLabels& labels);
void learn_gateway_classifier(ClassificationProblem& problem, const std::string& outputDir);

void compute_area_features(LabelsFile::BuildingLabels& labels);
void learn_hypothesis_classifier(ClassificationProblem& problem, const std::string& outputDir);
void learn_boundary_classifier(ClassificationProblem& problem, const std::string& outputDir);


int main(int argc, char** argv)
{
    std::vector<utils::command_line_argument_t> arguments;
    arguments.push_back({utils::kConfigFileArgument, "Configuration file for calculating local topo features", true, "local_topo_hssh.cfg"});
    arguments.push_back({kLabelsArg, "File containing the map names to be used for classification", false, ""});
    arguments.push_back({kOutputDirArg, "Directory in which to save the learned classifiers", true, "."});
    arguments.push_back({kAllArg, "Train both gateway and area classifiers.", true, ""});
    arguments.push_back({kGwyArg, "Train the gateway classifier.", true, ""});
    arguments.push_back({kAreaArg, "Train the area classifier.", true, ""});

    utils::CommandLine cmdLine(argc, argv, arguments);
    if(!cmdLine.verify())
    {
        return -1;
    }

    utils::ConfigFile config(cmdLine.configName());
    local_topology_params_t params(config);
    LabelsFile labels = load_labels_file(cmdLine.argumentValue(kLabelsArg), params);

    std::cout << "INFO: Loaded data for " << labels.fullMaps.size() << " full buildings and "
        << labels.incrementalMaps.size() << " incremental buildings.\n";

    auto fullProblems = create_classification_problems(labels.fullMaps);
//     auto incrementalProblems = create_classification_problems(labels.incrementalMaps);

    if(cmdLine.argumentExists(kAllArg) || cmdLine.argumentExists(kGwyArg))
    {
        compute_gateway_features(labels.fullMaps);
//         compute_gateway_features(labels.incrementalMaps);

        for(auto& full : fullProblems)
        {
            std::cout << "\n\nINFO: Learning gateway classifier for full " << full.testBuildingName << "...\n\n\n";
            full.fileExt = kFullExt;
            learn_gateway_classifier(full, cmdLine.argumentValue(kOutputDirArg));
        }

//         for(auto& incr : incrementalProblems)
//         {
//             std::cout << "\n\nINFO: Learning gateway classifier for incremental " << incr.testBuildingName << "...\n\n\n";
//             incr.fileExt = kIncrementalExt;
//             learn_gateway_classifier(incr, cmdLine.argumentValue(kOutputDirArg));
//         }
    }

    if(cmdLine.argumentExists(kAllArg) || cmdLine.argumentExists(kAreaArg))
    {
        compute_area_features(labels.fullMaps);
//     compute_area_features(labels.incrementalMaps);

        for(auto& full : fullProblems)
        {
            std::cout << "\n\nINFO: Learning full area classifier for " << full.testBuildingName << "...\n\n\n";
            full.fileExt = kFullExt;
            learn_hypothesis_classifier(full, cmdLine.argumentValue(kOutputDirArg));
            learn_boundary_classifier(full, cmdLine.argumentValue(kOutputDirArg));
        }

//     for(auto& incr : incrementalProblems)
//     {
//         std::cout << "\n\nINFO: Learning incremental area classifier for " << incr.testBuildingName << "...\n\n\n";
//         incr.fileExt = kIncrementalExt;
//         learn_hypothesis_classifier(incr, cmdLine.argumentValue(kOutputDirArg));
//         learn_boundary_classifier(incr, cmdLine.argumentValue(kOutputDirArg));
//     }
    }

    return 0;
}


std::vector<ClassificationProblem> create_classification_problems(LabelsFile::BuildingLabels& labels)
{
    // Get each of the building names
    std::vector<std::string> buildings;
    std::transform(labels.begin(),
                   labels.end(),
                   std::back_inserter(buildings),
                   [](const std::pair<std::string, LabelsFile::LabelsVec>& buildingLabels) {
        return buildingLabels.first;
    });

    std::vector<ClassificationProblem> problems;

    // For each building name, create a new ClassificationProblem. The problem for each building will consist of
    // all other buildings being training data and this building being test data
    for(auto& b : buildings)
    {
        ClassificationProblem problem;
        problem.testBuildingName = b;

        // Go through all the building data
        for(auto& l : labels)
        {
            // If the building is equivalent to the current test building, add the data to the test data
            if(l.first == b)
            {
                for(auto& map : l.second)
                {
                    problem.testMaps.push_back(&map);
                }
            }
            // Otherwise the data is part of the training set
            else
            {
                for(auto& map : l.second)
                {
                    problem.trainingMaps.push_back(&map);
                }
            }
        }

        problems.push_back(std::move(problem));
    }

    return problems;
}


void compute_gateway_features(LabelsFile::BuildingLabels& labels)
{
    for(auto& bldg : labels)
    {
        for(auto& floor : bldg.second)
        {
            compute_gateway_features(floor);
        }
    }
}


void learn_gateway_classifier(ClassificationProblem& problem, const std::string& outputDir)
{
    LabeledGatewayData trainingData;
    for(auto& labels : problem.trainingMaps)
    {
        trainingData.addExamples(labels->gatewayData);
    }

    auto classifier = GatewayClassifier::LearnClassifier(trainingData);
    std::ostringstream filename;
    filename << outputDir << '/' << problem.testBuildingName << problem.fileExt;

    if(!classifier->save(filename.str()))
    {
        std::cerr << "ERROR: Failed to save learned gateway classifier to " << filename.str() << '\n';
    }

    // Now create a gateway generator for each of the test maps so the area features are computed correctly
    for(auto& map : problem.testMaps)
    {
        map->editor->setGatewayGenerator(std::make_unique<ClassifierBasedGenerator>(3, filename.str()));
    }

}


void compute_area_features(LabelsFile::BuildingLabels& labels)
{
    for(auto& bldg : labels)
    {
        for(auto& floor : bldg.second)
        {
            compute_area_and_boundary_features(floor);
        }
    }
}


void learn_hypothesis_classifier(ClassificationProblem& problem, const std::string& outputDir)
{
    LabeledAreaData trainingData;
    for(auto& labels : problem.trainingMaps)
    {
        trainingData.addExamples(labels->initialData);
        trainingData.addExamples(labels->simplifiedData);
    }

    auto classifier = HypothesisClassifier::LearnClassifier(trainingData);

    std::ostringstream filename;
    filename << outputDir << '/' << problem.testBuildingName << problem.fileExt;

    if(!classifier->save(filename.str()))
    {
        std::cerr << "ERROR: Failed to save learned area classifier to " << filename.str() << '\n';
    }
}


void learn_boundary_classifier(ClassificationProblem& problem, const std::string& outputDir)
{
    LabeledBoundaryData trainingData;
    for(auto& labels : problem.trainingMaps)
    {
        trainingData.addExamples(labels->boundaryData);
    }

    auto classifier = BoundaryClassifier::LearnClassifier(trainingData);
    std::ostringstream filename;
    filename << outputDir << '/' << problem.testBuildingName << problem.fileExt;

    if(!classifier->save(filename.str()))
    {
        std::cerr << "ERROR: Failed to save learned boundary classifier to " << filename.str() << '\n';
    }
}
