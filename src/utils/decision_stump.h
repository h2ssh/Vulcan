/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_stump.h
* \author   Collin Johnson
*
* Declaration of DecisionStump.
*/

#ifndef UTILS_DECISION_STUMP_H
#define UTILS_DECISION_STUMP_H

#include <core/matrix.h>
#include <core/vector.h>
#include <iosfwd>
#include <memory>

namespace vulcan
{
namespace utils
{

/**
* DecisionStump
*/
class DecisionStump
{
public:

    /**
    * LearnStump takes a sets of labels (+1, -1), weights associated with the labels, and features for each labeled
    * example and converts them into a single decision stump that maximizes the weighted classification accuracy. The
    * error term to minimize is the sum of the weights of misclassified examples. If the number of positive and
    * negative examples is not approximately equal, this approach may not work particularly well. Balancing the weights
    * of the two classes should be sufficient to get good stump-learning performance though.
    *
    * \param    labels          Labels for each of the examples
    * \param    weights         Weights for each example
    * \param    features        Feature vectors for the examples. The features for an example are a column vector
    * \param    ignoreFeatures  Features to be ignored because they are overused or deemed uninformative
    * \return   A learned DecisionStump. nullptr is somehow the learning process fails.
    */
    static std::unique_ptr<DecisionStump> LearnStump(const IntVector& labels,
                                                     const Vector& weights,
                                                     const Matrix& features,
                                                     const std::vector<int>& ignoreFeatures);

    /**
    * classify classifies a feature vector using the binary decision encoded in the stump.
    *
    * \param    features        Feature vector to classify
    * \return   The assigned weight if it is classified as an example. 0 if not an example.
    */
    double classify(const Vector& features) const;

    /**
    * setWeight sets the weight associated with this stump. It is the value returned for positive classifications.
    *
    * \param    weight          Weight to assigned
    * \pre  weight > 0.0
    */
    void setWeight(double weight);

    /**
    * weight retrieves the weight assigned to this stump.
    */
    double weight(void) const;

    /**
    * threshold retrieves the decision boundary threshold for this stump.
    */
    double threshold(void) const { return threshold_; }

    /**
    * featureIndex retrieves the feature index associated with the stump.
    */
    int featureIndex(void) const { return index_; }

    /**
    * load loads a DecisionStump from the provided stream.
    *
    * \param    in          Input stream
    * \return   True if loaded successfully.
    */
    bool load(std::istream& in);

    /**
    * save saves the DecisionStump to the provided stream.
    *
    * \param    out         Output stream
    * \return   True if saved successfully.
    */
    bool save(std::ostream& out) const;

private:

    int index_;             ///< Feature index for this stump
    double threshold_;      ///< Threshold of the decision boundary
    double trueWeight_;     ///< Weight to return when value >= threshold_
    double falseWeight_;    ///< Weight to return when value < threshold_
    int polarity_;          ///< features[index_] > threshold_, is it a positive example (+1) or negative example (-1)
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_DECISION_STUMP_H
