/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     linear.cpp
* \author   Collin Johnson
*
* Definition of learn_linear_classifier.
*/

#include "utils/liblinear.h"
#include "core/float_comparison.h"
#include "core/vector.h"
#include <map>

namespace vulcan
{
namespace utils
{

// namespace {
// std::vector<std::pair<int, double>> calculate_label_weights(const IntMatrix& labels);
// }


model* learn_linear_classifier(const Matrix& features, const IntMatrix& labels)
{
    // Convert the feature data into an linear_problem so linear_train can be called
    std::vector<double> linearLabels;
    std::vector<LinearFeatures> linearNodes;

    for(std::size_t n = 0; n < features.n_cols; ++n)
    {
        LinearFeatures nodes;
        features_to_nodes(features.col(n), nodes);

        linearNodes.push_back(nodes);
        linearLabels.push_back(labels(n));
    }

    std::vector<feature_node*> nodeStarts;
    for(auto& n : linearNodes)
    {
        nodeStarts.push_back(n.data());
    }

//     auto weights = calculate_label_weights(labels);

    assert(nodeStarts.size() == linearLabels.size());

    problem problem;
    problem.l = linearLabels.size();    // number of examples
    problem.n = features.n_rows;        // number of features
    problem.y = linearLabels.data();
    problem.x = nodeStarts.data();
    problem.bias = 0.0;

    // Assign the parameters to use for the learning -- using defaults taken from train.c in liblinear
    parameter param;
    param.solver_type = L2R_LR;
//     param.solver_type = L1R_LR;
    param.C = 1;
    param.eps = 0.01;
    param.p = 0.1;
    param.nr_weight = 0;//weights.size();
    param.init_sol = 0;

//     param.weight_label = new int[param.nr_weight];
//     param.weight = new double[param.nr_weight];
//
//     for(int n = 0; n < param.nr_weight; ++n)
//     {
//         param.weight_label[n] = weights[n].first;
//         param.weight[n] = weights[n].second;
//     }

    double minC = 0.125;  // setting less than 0 has it automatically find the best starting value
    double maxC = 2048.0;   // using value from train.c
    int numCrossValidationFolds = 5;    // use value from train.c

    // Output parameters of the training process
    double bestC = 0.0;     // best C value to use
    double bestRate = 0.0;  // best success rate (1.0-error rate)

    std::cout << "INFO:learn_linear_classifier: Searching for best parameter C in range:\n";

    find_parameter_C(&problem, &param, numCrossValidationFolds, minC, maxC, &bestC, &bestRate);

    std::cout << "\tBest C:" << bestC << " % correct:" << (bestRate * 100.0) << '\n';

    param.C = bestC;
    model* trainedModel = train(&problem, &param);

    // Due to the trained model storing data from the problem, the original pointer can't be returned. Instead,
    // save the model to a temp file and reload it so it allocates the memory itself and makes it safe to pass
    // ownership onward to someone else
    const char* kDummyModelFile = "dummy_model.linear";

    if(save_model(kDummyModelFile, trainedModel) != 0)
    {
        perror("ERROR: learn_linear_classifier: Failed to save temporary model");
        free_and_destroy_model(&trainedModel);
        return nullptr;
    }
    else
    {
        auto loadedModel = load_model(kDummyModelFile);

        if(loadedModel == nullptr)
        {
            std::cerr << "ERROR: learn_linear_classifier: Failed to load temporary model.\n";
        }

        free_and_destroy_model(&trainedModel);
        return loadedModel;
    }
}


void features_to_nodes(const Vector& features, LinearFeatures& nodes)
{
    for(int featIdx = 0; featIdx < static_cast<int>(features.n_rows); ++featIdx)
    {
        nodes.push_back(feature_node{featIdx + 1, features(featIdx)});
    }

    // The end of the nodes are marked by an index of -1
    nodes.push_back(feature_node{-1, 0});
}


// namespace
// {
//
// std::vector<std::pair<int, double>> calculate_label_weights(const IntMatrix& labels)
// {
//     std::map<int, double> counts;
//
//     for(arma::uword n = 0; n < labels.n_rows; ++n)
//     {
//         if(counts.find(labels[n]) == counts.end())
//         {
//             counts[labels[n]] = 0.0;
//         }
//         else
//         {
//             counts[labels[n]] += 1.0;
//         }
//     }
//
//     // The weight to assign will make the weights of the total number of labels for a given instance sum to 1.
//     std::vector<std::pair<int, double>> weights;
//     for(auto& c : counts)
//     {
//         weights.emplace_back(c.first, (1.0 - (c.second / labels.n_rows)));
//         std::cout << "Weight " << weights.back().first << ": " << weights.back().second << '\n';
//     }
//
//     return weights;
// }
//
// }

} // namespace utils
} // namespace vulcan
