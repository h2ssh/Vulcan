/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     boosting.h
* \author   Collin Johnson
*
* Definition of AdaBoostClassifier.
*/

#include "utils/boosting.h"
#include "utils/float_io.h"
#include <fstream>
#include <sstream>

namespace vulcan
{
namespace utils
{

Vector initialize_weights(const IntVector& labels);
void classify_examples(const IntVector& labels,
                       const Matrix& features,
                       const DecisionStump& stump,
                       Vector& results);
double compute_stump_weight(const Vector& results, const Vector& weights);
void reweight_examples(double stumpWeight, const Vector& results, Vector& weights);


std::unique_ptr<AdaBoostClassifier> AdaBoostClassifier::LearnClassifier(const IntVector& labels,
                                                                        const Matrix& features,
                                                                        int maxClassifiers)
{
    const int kNumStumps = maxClassifiers;

    Vector weights = initialize_weights(labels);
    Vector results(labels.n_elem);
    std::vector<DecisionStump> stumps;

    int lastFeatIdx = -1;

    for(int n = 0; n < kNumStumps; ++n)
    {
        std::vector<int> ignore;
        if(lastFeatIdx != -1)
        {
            ignore.push_back(lastFeatIdx);
        }
        auto newStump = DecisionStump::LearnStump(labels, weights, features, ignore);

        if(newStump)
        {
            classify_examples(labels, features, *newStump, results);
            double stumpWeight = compute_stump_weight(results, weights);

            if(stumpWeight > 0.0)
            {
                reweight_examples(stumpWeight, results, weights);
                newStump->setWeight(stumpWeight);
                stumps.push_back(*newStump);
                lastFeatIdx = newStump->featureIndex();

                std::cout << "Created stump " << (n + 1) << " of " << kNumStumps << '\n';
            }
            else
            {
                std::cout << "Decision stump learning converged -- can't find another weak learner.\n";
                break;
            }
        }
        else
        {
            std::cerr << "WARNING: Failed to create a stump!\n";
        }
    }

    if(!stumps.empty())
    {
        return std::unique_ptr<AdaBoostClassifier>(new AdaBoostClassifier(stumps));
    }
    else
    {
        return nullptr;
    }
}


AdaBoostClassifier::AdaBoostClassifier(const std::string& filename)
{
    bool success = load(filename);
    assert(success);
}


double AdaBoostClassifier::classify(const Vector& features) const
{
    double totalWeight = 0.0;

    for(auto& stump : stumps_)
    {
        totalWeight += stump.classify(features);
    }

    // Logistic Correction from Friedman et al. (2000) to get a reasonably calibrated probability
    return 1.0 / (1.0 + std::exp(-2.0 * totalWeight));

//     return 0.5 * ((totalWeight * weightNormalizer_) + 1.0);
}


bool AdaBoostClassifier::load(const std::string& filename)
{
    std::ifstream in(filename);

    int numClassifiers = 0;
    in >> numClassifiers;

    assert(numClassifiers > 0);

    std::vector<DecisionStump> loadedStumps;
    std::string line;
    while(loadedStumps.size() < static_cast<std::size_t>(numClassifiers))
    {
        if(std::getline(in, line))
        {
            std::istringstream stumpIn(line);
            DecisionStump stump;
            if(stump.load(stumpIn))
            {
                loadedStumps.push_back(stump);
            }
        }
        else
        {
            std::cerr << "ERROR::AdaBoostClassifier: Expected to load " << numClassifiers << " stumps but only found "
                << loadedStumps.size() << " stumps in " << filename << '\n';
            return false;
        }
    }

    weightNormalizer_ = load_floating_point(in);
    stumps_ = std::move(loadedStumps);

    return true;
}


bool AdaBoostClassifier::save(const std::string& filename) const
{
    /*
    * The file format is:
    *
    *   num_classifiers
    *   weak_classifier_0
    *       .
    *       .
    *       .
    *   weak_classifier_N
    *   weightNormalizer_
    */

    std::ofstream out(filename);

    out << stumps_.size() << '\n';
    for(auto& stump : stumps_)
    {
        stump.save(out);
        out << '\n';
    }

    save_floating_point(out, weightNormalizer_);
    out << '\n';
    return out.good();
}


AdaBoostClassifier::AdaBoostClassifier(std::vector<DecisionStump> stumps)
: stumps_(std::move(stumps))
{
    double totalWeight = 0.0;
    for(auto& s : stumps_)
    {
        totalWeight += s.weight();
    }

    assert(totalWeight > 0.0);
    weightNormalizer_ = 1.0 / totalWeight;
}


Vector initialize_weights(const IntVector& labels)
{
    Vector weights(labels.n_elem);
    weights.ones();

    arma::uvec posIdx = arma::find(labels > 0);
    arma::uvec negIdx = arma::find(labels < 0);

    std::cout << "Num pos examples:" << posIdx.n_elem << " Num neg examples:" << negIdx.n_elem << '\n';

    // If there are more positives than negatives, increase the weight for negative examples
    if(posIdx.n_elem > negIdx.n_elem)
    {
        double negWeight = static_cast<double>(posIdx.n_elem) / negIdx.n_elem;
        weights.elem(negIdx) *= negWeight;

        std::cout << "Applying weight to negative examples: " << negWeight << '\n';
    }
    // If there are more negatives than positives, increase the weight for positive examples
    else if(posIdx.n_elem < negIdx.n_elem)
    {
        double posWeight = static_cast<double>(negIdx.n_elem) / posIdx.n_elem;
        weights.elem(posIdx) *= posWeight;

        std::cout << "Applying weight to positive examples: " << posWeight << '\n';
    }

    weights /= arma::accu(weights);   // make weights sum to 1
    return weights;
}


void classify_examples(const IntVector& labels,
                       const Matrix& features,
                       const DecisionStump& stump,
                       Vector& results)
{
    int numPosCorrect = 0;
    int numNegCorrect = 0;
    int numPosWrong = 0;
    int numNegWrong = 0;

    // Mark = 1 for a correct label and -1 for an incorrect label
    for(arma::uword n = 0; n < labels.n_elem; ++n)
    {
        // Classification > 0 means it was a positive classification, so if positive and label was positive,
        // then a correct match was made
        double classification = stump.classify(features.col(n));
        if(((classification > 0.0) && (labels(n) > 0))
            || ((classification < 0.0) && (labels(n) < 0)))
        {
            results(n) = 1.0;
        }
        else
        {
            results(n) = -1.0;
        }

        if(results(n) < 0)
        {
            if(labels(n) > 0)
            {
                ++numPosWrong;
            }
            else
            {
                ++numNegWrong;
            }
        }
        else
        {
            if(labels(n) > 0)
            {
                ++numPosCorrect;
            }
            else
            {
                ++numNegCorrect;
            }
        }
    }

    std::cout << "Stump got " << numPosCorrect << " of " << (numPosCorrect + numPosWrong) << " pos correct and "
        << numNegCorrect << " of " << (numNegCorrect + numNegWrong) << " neg correct.";
}


double compute_stump_weight(const Vector& results, const Vector& weights)
{
    // Stump error is the sum of incorrect weights divided by total weights
    double error = arma::accu(weights.elem(find(results < 0)));
    error /= arma::accu(weights);

    if((error > 0.0) && (error < 1.0))
    {
        std::cout << "Stump error rate:" << error << " Weight: " << std::log((1 - error) / error) << '\n';
        return 0.5 * std::log((1 - error) / error);
    }
    else
    {
        std::cerr << "ERROR: Invalid error rate: " << error << " Shouldn't get all right or wrong.\n";
        return 0.0;
    }
}


void reweight_examples(double stumpWeight, const Vector& results, Vector& weights)
{
    Vector stumpExponent = results * -stumpWeight;
    weights %= arma::exp(stumpExponent);
    weights /= arma::accu(weights);
}

} // namespace utils
} // namespace vulcan
