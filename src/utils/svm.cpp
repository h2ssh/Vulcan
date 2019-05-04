/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     svm.cpp
* \author   Collin Johnson
* 
* Definition of learn_svm_classifier.
*/

#include <utils/svm.h>
#include <core/float_comparison.h>
#include <core/vector.h>
#include <boost/range/iterator_range.hpp>
#include <future>
#include <map>
#include <vector>
#include <utility>

namespace vulcan
{
namespace utils
{
    
struct SVMParamAccuracy
{
    double C;
    double gamma;
    double accuracy;

    SVMParamAccuracy(double C, double gamma)
    : C(C)
    , gamma(gamma)
    {
    }
};

using ParamVec = std::vector<SVMParamAccuracy>;

std::vector<std::pair<int, double>> calculate_label_weights(const IntMatrix& labels);
svm_parameter select_best_parameters(svm_problem* problem, const std::vector<std::pair<int, double>>& weights);
void evaluate_params(ParamVec::iterator begin, ParamVec::iterator end, svm_problem* problem, svm_parameter params);
int svm_classifier_accuracy(double* truth, double* estimated, int size);
    

svm_model* learn_svm_classifier(const Matrix& features, const IntMatrix& labels)
{
    // Convert the feature data into an svm_problem so svm_train can be called
    std::vector<double> svmLabels;
    std::vector<SVMFeatures> svmNodes;
    
    for(std::size_t n = 0; n < features.n_cols; ++n)
    {
        SVMFeatures nodes;
        features_to_svm_nodes(features.col(n), nodes);
        
        svmNodes.push_back(nodes);
        svmLabels.push_back(labels(n));
    }
    
    std::vector<svm_node*> nodeStarts;
    for(auto& n : svmNodes)
    {
        nodeStarts.push_back(n.data());
    }
    
    assert(nodeStarts.size() == svmLabels.size());
    
    svm_problem problem;
    problem.l = svmLabels.size();
    problem.y = svmLabels.data();
    problem.x = nodeStarts.data();
    
    // Assign the parameters to use for the learning
    auto trainingParams = select_best_parameters(&problem, calculate_label_weights(labels));
    
    // When training the final model, the probability estimates are needed, so ensure they are turned on
    trainingParams.probability = 1;

    svm_model* trainedModel = svm_train(&problem, &trainingParams);
    
    // Due to the trained model storing data from the svm_problem, the original pointer can't be returned. Instead,
    // save the model to a temp file and reload it so it allocates the memory itself and makes it safe to pass
    // ownership onward to someone else
    const char* kDummyModelFile = "dummy_svm_model.svm";
    
    if(svm_save_model(kDummyModelFile, trainedModel) != 0)
    {
        perror("ERROR: learn_svm_classifier: Failed to save temporary model");
        svm_free_and_destroy_model(&trainedModel);
        svm_destroy_param(&trainingParams);
        return nullptr;
    }
    else
    {
        auto loadedModel = svm_load_model(kDummyModelFile);
        
        if(loadedModel == nullptr)
        {
            std::cerr << "ERROR: learn_svm_classifier: Failed to load temporary model.\n";
        }
        
        svm_free_and_destroy_model(&trainedModel);
        svm_destroy_param(&trainingParams);
        return loadedModel;
    }
}


void features_to_svm_nodes(const Vector& features, SVMFeatures& nodes)
{
    for(int featIdx = 0; featIdx < static_cast<int>(features.n_rows); ++featIdx)
    {
        nodes.push_back(svm_node{featIdx + 1, features(featIdx)});
    }
    
    // The end of the nodes are marked by an index of -1
    nodes.push_back(svm_node{-1, 0});
}


std::vector<std::pair<int, double>> calculate_label_weights(const IntMatrix& labels)
{
    std::map<int, double> counts;

    for(arma::uword n = 0; n < labels.n_rows; ++n)
    {
        if(counts.find(labels[n]) == counts.end())
        {
            counts[labels[n]] = 0.0;
        }
        else
        {
            counts[labels[n]] += 1.0;
        }
    }

    // The weight to assign will make the weights of the total number of labels for a given instance sum to 1.
    std::vector<std::pair<int, double>> weights;
    for(auto& c : counts)
    {
        weights.emplace_back(c.first, (1.0 - (c.second / labels.n_rows)));
    }

    return weights;
}


svm_parameter select_best_parameters(svm_problem* problem, const std::vector<std::pair<int, double>>& weights)
{
    // Default parameters snagged from svm-train
    svm_parameter params;
    
    // default values
    params.svm_type = C_SVC;
    params.kernel_type = RBF;
    params.degree = 3;
    params.gamma = 0;    // 1/num_features
    params.coef0 = 0;
    params.nu = 0.5;
    params.cache_size = 1000;
    params.C = 1;
    params.eps = 1e-3;
    params.p = 0.1;
    params.shrinking = 1;
    params.probability = 0;
    params.nr_weight = weights.size();

    // Assign weights if they are present
    if(params.nr_weight > 0)
    {
        params.weight_label = new int[params.nr_weight];
        params.weight = new double[params.nr_weight];

        for(int n = 0; n < params.nr_weight; ++n)
        {
            params.weight_label[n] = weights[n].first;
            params.weight[n] = weights[n].second;
        }
    }
    // Otherwise null them out
    else
    {
        params.weight_label = NULL;
        params.weight = NULL;
    }
    
    ParamVec allParams;
    for(double g = -5.0, gEnd = 5.0; g <= gEnd; g += 1.0)
    {
        allParams.emplace_back(1.0, std::pow(10.0, g));

        // There is no coeff0 for RBF
//         for(double c = -4.0, cEnd = 4.0; c <= cEnd; c += 1.0)
//         {
//             allParams.emplace_back(std::pow(10.0, c), std::pow(10.0, g));
//         }
    }

    // Random because different sets of parameters take wildly different amounts of computation
    std::random_device rd;
    std::mt19937 generator(rd());
    std::shuffle(allParams.begin(), allParams.end(), generator);
    
    // Create up to four threads for finding the isovists. Assign 1/4th of the work to each thread.
    std::size_t kMaxThreads = 4;
    const int kNumThreads = std::min(kMaxThreads, (allParams.size() / kMaxThreads) + 1);    
    std::size_t paramsPerThread = (allParams.size() / kNumThreads) + 1; // Add one to the positions/thread to account for truncating due to integer division
    // The last thread will take up to 3 fewer isovist calculations
    
    // Launch the threads
    std::vector<std::future<void>> asyncParams;
    for(int n = 0; n < kNumThreads; ++n)
    {
        std::size_t start = paramsPerThread * n;
        std::size_t end   = std::min(paramsPerThread * (n + 1), allParams.size());
        
        asyncParams.push_back(std::async(std::launch::async, 
                                            evaluate_params,
                                            allParams.begin() + start,
                                            allParams.begin() + end,
                                            problem,
                                            params));
    }
    
    for(auto& f : asyncParams)
    {
        f.get();
    }
    
    // Sort in descending order of accuracy. Tie goes to the lower C, as softer margins are generally better.
    std::sort(allParams.begin(), allParams.end(), [](SVMParamAccuracy lhs, SVMParamAccuracy rhs) {
        return (lhs.accuracy > rhs.accuracy) || ((lhs.accuracy == rhs.accuracy) && (lhs.gamma < rhs.gamma));
    });

    std::cout << "SVM parameter accuracy: (C, gamma) -> accuracy\n";
    for(auto p : allParams)
    {
        std::cout << '(' << p.C << ',' << p.gamma << ") -> " << p.accuracy << "%\n";
    }
    
    params.C = allParams.front().C;
    params.gamma = allParams.front().gamma;
    
    return params;
}


void evaluate_params(ParamVec::iterator begin, ParamVec::iterator end, svm_problem* problem, svm_parameter params)
{
    std::vector<double> validationResults(problem->l);
    
    for(auto& p : boost::make_iterator_range(begin, end))
    {
        params.C = p.C;
        params.gamma = p.gamma;
        
        svm_cross_validation(problem, &params, 5, validationResults.data());
        p.accuracy = svm_classifier_accuracy(problem->y, validationResults.data(), problem->l);
    }
}


int svm_classifier_accuracy(double* truth, double* estimated, int size)
{
    int numCorrect = 0;
    for(int n = 0; n < size; ++n)
    {
        if(absolute_fuzzy_equal(truth[n], estimated[n]))
        {
            ++numCorrect;
        }
    }
    
    return (numCorrect / static_cast<double>(size)) * 100.0;
}
    
} // namespace utils
} // namespace vulcan
