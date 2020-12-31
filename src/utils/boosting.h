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
* Declaration of AdaBoostClassifier, which can be learned via the LearnClassifier method.
*/

#ifndef UTILS_BOOSTING_H
#define UTILS_BOOSTING_H

#include "utils/decision_stump.h"
#include "core/matrix.h"
#include "core/vector.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace utils
{

class AdaBoostClassifier;
using AdaBoostModelPtr = std::shared_ptr<AdaBoostClassifier>;

/**
* AdaBoostClassifier is a simple AdaBoost classifier using decision stumps.
*/
class AdaBoostClassifier
{
public:

    /**
    * LearnClassifier learns a classifier for the provided data. The data is a feature vector and a binary label.
    *
    * \param    labels          Binary labels for the data
    * \param    features        Feature vectors for each example
    * \param    maxClassifiers  Maximum number of classifiers to learns
    * \return   A learned AdaBoostClassifier.
    */
    static std::unique_ptr<AdaBoostClassifier> LearnClassifier(const IntVector& labels,
                                                               const Matrix& features,
                                                               int maxClassifiers = 200);

    // Allow default construction
    AdaBoostClassifier(void) = default;

    /**
    * Constructor for AdaBoostClassifier.
    *
    * Load a classifier from a file.
    *
    * \param    filename            Filename with the saved boosting classifier
    */
    explicit AdaBoostClassifier(const std::string& filename);

    /**
    * classify classifies an example. The learned classifier is applied to the feature vector and the probability of
    * the feature being a positive classificaton is returned.
    *
    * \param    features            Feature vector to classify
    * \return   Value in range [0, 1] indicating how likely it is to be a good example.
    */
    double classify(const Vector& features) const;

    /**
    * load loads a classifier from the given file.
    *
    * \param    filename        Classifier to load
    * \return   True if the classifier was loaded successfully.
    */
    bool load(const std::string& filename);

    /**
    * save saves a classifier to a file.
    *
    * \param    filename        Name to save classifier in
    * \return   True if saved successfully.
    */
    bool save(const std::string& filename) const;

    // Iteration over the boosted decision stump classifiers
    std::size_t size(void) const { return stumps_.size(); }
    std::vector<DecisionStump>::const_iterator begin(void) const { return stumps_.begin(); }
    std::vector<DecisionStump>::const_iterator end(void) const { return stumps_.end(); }

private:

    AdaBoostClassifier(std::vector<DecisionStump> stumps);

    std::vector<DecisionStump> stumps_;
    double weightNormalizer_;           // normalizer to apply to all weights
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_BOOSTING_H
