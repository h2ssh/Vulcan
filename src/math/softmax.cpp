/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     softmax.cpp
* \author   Collin Johnson
* 
* Definition of functions for softmax regression and classification.
*/

#include <math/softmax.h>
#include <iostream>
#include <future>
#include <thread>
#include <cassert>

namespace vulcan 
{
namespace math
{

using LambdaIter = std::vector<double>::const_iterator;

struct LambdaResult
{
    double lambda;
    int numCorrect;
    double totalProbability;
    Matrix classifier;

    bool operator<(const LambdaResult& rhs)
    {
        // Tie goes to the result with a higher total probability for the correct answer
        return (numCorrect > rhs.numCorrect) || (numCorrect == rhs.numCorrect && totalProbability > rhs.totalProbability);
    }
};


// Size x: n+1   Size thetas: k-by-n+1  Size y: k, zeros except the appropriate label
double softmax_cost_and_gradient(const Vector& x, const Matrix& thetas, int y, Matrix& gradient);
bool converged(const Matrix& prevCost, const Matrix& cost, double threshold);
LambdaResult find_best_lambda(LambdaIter begin,
                            LambdaIter end,
                            const Matrix& trainingFeatures,
                            const IntMatrix& trainingLabels,
                            const Matrix& testFeatures,
                            const IntMatrix& testLabels,
                            const int numClasses,
                            softmax_regression_options_t options);
LambdaResult softmax_regression_plus_test(const Matrix& trainingFeatures,
                                         const IntMatrix& trainingLabels,
                                         const Matrix& testFeatures,
                                         const IntMatrix& testLabels,
                                         const int numClasses,
                                         const softmax_regression_options_t& options);



Matrix softmax_regression(const Matrix& x, 
                          const IntMatrix& y, 
                          const int numClasses, 
                          const softmax_regression_options_t options)
{
    const int kDimensions = x.n_rows;
    const int kNumExamples = x.n_cols;
    
    std::array<Matrix, 2> thetas;
    thetas[0] = thetas[1] = Matrix(numClasses, kDimensions);
    thetas[0].zeros();
    thetas[1].zeros();
    
    std::array<Matrix, 2> grads;
    grads[0] = grads[1] = Matrix(numClasses, kDimensions);
    
    int prevIndex = 0;
    int currIndex = 1;
    
    int numIterations = 0;
    
    bool hasConverged = false;

    const double kRegularizeLambda = options.l2Lambda / kNumExamples;
    Matrix regularizationVector(numClasses, kDimensions);
    
    do
    {
        std::swap(prevIndex, currIndex);
        
        auto& prevThetas = thetas[prevIndex];
        auto& currThetas = thetas[currIndex];
        
        auto& prevGrad = grads[prevIndex];
        auto& currGrad = grads[currIndex];
        
        currGrad.zeros();
        
        double cost = 0.0;
        
        for(int n = 0; n < kNumExamples; ++n)
        {
            cost += softmax_cost_and_gradient(x.col(n), prevThetas, y(n, 0), currGrad);
        }
        
        currGrad /= -kNumExamples;
        currGrad += prevThetas * options.weightDecay;
        
        cost = (-cost / kNumExamples) + ((options.weightDecay / 2) * accu(prevThetas % prevThetas));

        regularizationVector = kRegularizeLambda * prevThetas;
        regularizationVector.col(0).zeros();  // regularization doesn't apply to the first weight
        
        // Apply L2 regularization in the gradient descent update step
        currThetas   = prevThetas - (options.learningRate * (regularizationVector + currGrad));
        hasConverged = converged(prevGrad, currGrad, options.convergenceThreshold);
        
        if((hasConverged || (numIterations % 1000 == 0)) && options.verbose)
        {
            std::cout << "Iteration " << numIterations << ": Converged? " << hasConverged << " Error:" << cost 
                      << " Grad:" << accu(abs(currGrad)) << std::endl;
        }
            
        ++numIterations;
        
    } while(!hasConverged && (numIterations < options.maxIterations || options.maxIterations == 0));
    
    return thetas[currIndex];
}


Matrix softmax_regression_with_search(const Matrix&    features,
                                      const IntMatrix& labels,
                                      int              numClasses,
                                      softmax_regression_options_t options,
                                      double lambdaMin,
                                      double lambdaMax,
                                      double lambdaStep)
{
    // Split data into a holdout set and a training set -- train on training and test against the holdout to determine
    // the best lambda value

    const int kHoldoutRatio = 50;
    // minus 1 is here because the divide vs. mod. If n_cols % 50 == 0, it won't get there due to 0-based indexing, so
    // shift n_cols to show what the for-loop will see for column values below.
    int numHoldouts = ((features.n_cols - 1) / kHoldoutRatio) + 1;          // use 2% of the data as holdout for determining best value of lambda
    Matrix holdoutFeatures(features.n_rows, numHoldouts);
    IntMatrix holdoutLabels(numHoldouts, 1);

    Matrix trainingFeatures(features.n_rows, features.n_cols - numHoldouts);
    IntMatrix trainingLabels(labels.n_rows - numHoldouts, 1);

    int holdoutIndex = 0;
    int trainingIndex = 0;

    for(arma::uword n = 0; n < features.n_cols; ++n)
    {
        if(n % kHoldoutRatio == 0)
        {
            assert(holdoutIndex < numHoldouts);
            holdoutFeatures.col(holdoutIndex) = features.col(n);
            holdoutLabels(holdoutIndex, 0) = labels(n, 0);
            ++holdoutIndex;
        }
        else
        {
            trainingFeatures.col(trainingIndex) = features.col(n);
            trainingLabels(trainingIndex, 0) = labels(n, 0);
            ++trainingIndex;
        }
    }

    // Create the lambdas that will be searched
    lambdaMin = std::max(lambdaMin, 0.0);
    lambdaMax = std::max(lambdaMax, lambdaMin);     // can't have a negative lambda
    lambdaStep = std::abs(lambdaStep);

    std::size_t numLambdas = std::ceil((lambdaMax - lambdaMin) / lambdaStep) + 1;        // always search at least lambda = lambdaMin

    // Create up to four threads for searching the space.
    std::size_t kMaxThreads = 4;
    const int kNumThreads = std::min(kMaxThreads, numLambdas);
    std::size_t lambdasPerThread = std::max((numLambdas / kNumThreads), std::size_t(1));
    numLambdas = kNumThreads * lambdasPerThread;

    std::vector<double> lambdas(numLambdas);  // Run as many lambdas as possible to fill up full CPU time
    std::iota(lambdas.begin(), lambdas.end(), 0.0);       // fill with [0, numLambdas]
    std::transform(lambdas.begin(), lambdas.end(), lambdas.begin(), [lambdaStep, lambdaMin](double lambda) {
       return (lambda * lambdaStep) + lambdaMin;  // convert integer index to the appropriate lambda value in [lambdaMin, lambdaMax]
    });

    std::cout << "INFO: softmax_regression_with_search: Performing search of lambdas [" << lambdas.front() << ','
        << lambdas.back() << "] step:" << lambdaStep << " Training examples:" << trainingFeatures.n_cols
        << " Holdouts:" << numHoldouts << '\n';

    std::cout << "INFO: softmax_regression_with_search: Running search using " << kNumThreads << " threads with "
        << lambdasPerThread << " lambdas per thread. Num lambdas:" << lambdas.size() << '\n';

    // Launch the threads
    std::vector<std::future<LambdaResult>> asyncParams;
    for(int n = 0; n < kNumThreads; ++n)
    {
        std::size_t start = lambdasPerThread * n;
        std::size_t end   = std::min(lambdasPerThread * (n + 1), numLambdas);

        asyncParams.push_back(std::async(std::launch::async,
                                         find_best_lambda,
                                         lambdas.begin() + start,
                                         lambdas.begin() + end,
                                         trainingFeatures,
                                         trainingLabels,
                                         holdoutFeatures,
                                         holdoutLabels,
                                         numClasses,
                                         options
        ));
    }

    // Wait until all the tasks have finished
    std::vector<LambdaResult> results;
    for(auto& f : asyncParams)
    {
        results.push_back(f.get());
    }

    // Select the best result
    auto bestResult = std::max_element(results.begin(), results.end());

    assert(bestResult != results.end());
    return bestResult->classifier;
}


std::pair<int, double> softmax_max_likelihood(const Vector& features, const Matrix& classifier)
{
    auto distribution = softmax_classify(features, classifier);

    arma::uword index;
    double maxLikelihood = distribution.max(index);
    return std::make_pair(static_cast<int>(index), maxLikelihood);
}


Vector softmax_classify(const Vector& features, const Matrix& classifier)
{
    Vector thetaDotX = classifier * features;
    
    // To prevent overflow, subtract off the largest of the exponent values
    thetaDotX -= thetaDotX.max();
    
    // e^trans(theta) * x
    thetaDotX = exp(thetaDotX);
    
    // Normalize all the values
    thetaDotX /= sum(thetaDotX);
    
    return thetaDotX;
}


double softmax_cost_and_gradient(const Vector& x, const Matrix& thetas, int y, Matrix& gradient)
{
    assert(y >= 0 && y < static_cast<int>(thetas.n_rows));

    Vector labels(thetas.n_rows);
    labels.zeros();
    labels(y) = 1;
    
    Vector thetaDotX = thetas * x;
    
    // To prevent overflow, subtract off the largest of the exponent values
    arma::uword maxIndex;
    thetaDotX -= thetaDotX.max(maxIndex);
    
    // e^trans(theta) * x
    thetaDotX = exp(thetaDotX);
    
    // Normalize all the values
    thetaDotX /= sum(thetaDotX);
    
    // Add the cost for each of the theta vectors at the same time
    gradient += (labels - thetaDotX) * x.t();
    
//     std::cout << "Sample error:" << std::log(thetaDotX(y)) << " Est.prob:" << thetaDotX(y) << '\n';
    return std::log(thetaDotX(y));
}


bool converged(const Matrix& prevCost, const Matrix& cost, double threshold)
{
    return std::abs(arma::accu(abs(cost))) < threshold;
}


LambdaResult find_best_lambda(LambdaIter begin,
                            LambdaIter end,
                            const Matrix& trainingFeatures,
                            const IntMatrix& trainingLabels,
                            const Matrix& testFeatures,
                            const IntMatrix& testLabels,
                            const int numClasses,
                            softmax_regression_options_t options)
{
    std::vector<LambdaResult> lambdaResults;

    std::for_each(begin, end, [&](double lambda) {
        options.l2Lambda = lambda;
        lambdaResults.push_back(
            softmax_regression_plus_test(
                trainingFeatures,
                trainingLabels,
                testFeatures,
                testLabels,
                numClasses,
                options
        ));
    });

    return *std::max_element(lambdaResults.begin(), lambdaResults.end());
}


LambdaResult softmax_regression_plus_test(const Matrix& trainingFeatures,
                                         const IntMatrix& trainingLabels,
                                         const Matrix& testFeatures,
                                         const IntMatrix& testLabels,
                                         const int numClasses,
                                         const softmax_regression_options_t& options)
{
    // Calculate the classifier
    LambdaResult result;
    result.classifier = softmax_regression(trainingFeatures, trainingLabels, numClasses, options);
    result.lambda = options.l2Lambda;
    result.numCorrect = 0;
    result.totalProbability = 0.0;

    // Find the results
    for(arma::uword n = 0; n < testFeatures.n_cols; ++n)
    {
        auto testResult = softmax_max_likelihood(testFeatures.col(n), result.classifier);

        if(testResult.first == testLabels(n, 0))
        {
            ++result.numCorrect;
            result.totalProbability += testResult.second;
        }
    }

    std::cout << "Softmax with lambda = " << result.lambda << ", correct = " << result.numCorrect << " prob = "
        << result.totalProbability << '\n';

    return result;
}

} // namespace math
} // namespace vulcan
