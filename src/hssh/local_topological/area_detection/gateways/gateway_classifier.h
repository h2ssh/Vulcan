/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateway_classifier.h
* \author   Collin Johnson
*
* Declaration of GatewayClassifier.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_GATEWAY_CLASSIFIER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_GATEWAY_CLASSIFIER_H

#include <core/vector.h>
#include <memory>

namespace vulcan
{
namespace utils { class FeatureVector; }
namespace utils { class AdaBoostClassifier; }
namespace hssh
{

class LabeledGatewayData;

/**
* GatewayClassification contains information about the classification for a gateway. The classification indicates
* whether or not a gateway should be there and an estimated probability that there's a gateway at the particular
* location.
*
* The flag is included in addition to the probability because the threshold for a gateway being accepted/ignored is
* determined by the learning algorithm to maximize the recall of the classification.
*/
struct GatewayClassification
{
    bool isGateway;
    double probability;
};


/**
* GatewayClassifier is a learned classifier for determining whether or not a gateway should appear at the certain cell
* or not.
*/
class GatewayClassifier
{
public:

    enum ClassifierType
    {
        logistic,
        svm,
        adaboost,
    };

    /**
    * LearnClassifier learns the classification model from labeled gateway data. The labeled data needs to include
    * positive and negative examples of gateway locations in the skeleton.
    *
    * \param    examples            Labeled examples of where gateways should and shouldn't be located
    * \return   The learned model based on the examples for use in finding and classifying gateways.
    */
    static std::unique_ptr<GatewayClassifier> LearnClassifier(const LabeledGatewayData& examples);

    /**
    * Constructor for GatewayClassifier.
    *
    * Create a GatewayClassifier based on the saved model.
    *
    * \param    filename        Filename base for the saved classifier model
    */
    GatewayClassifier(const std::string& filename);

    /**
    * setThreshold changes the threshold for a positive vs. negative gateway classification.
    */
    void setThreshold(double threshold) { threshold_ = threshold; }

    /**
    * classifyHypothesis determines the maximum likelihood type of the provided hypothesis.
    *
    * \param   features            Features to be classified
    * \param   type                Type of classifier to be used -- both SVM and logistic regression are available
    * \return  The best type for the hypothesis
    */
    GatewayClassification classifyGateway(const utils::FeatureVector& features,
                                          ClassifierType type) const;

    /**
    * save saves the parameters used by the classifier to the specified file.
    *
    * \param    filename            Name of the file in which the classifier should be saved
    * \return   True if saving was successful.
    */
    bool save(const std::string& filename) const;

    /**
    * load loads the parameters used by the classifier from the specified file.
    *
    * \param    filename            Name of the file from which to load the stored classifier
    * \return   True if loading was successful.
    */
    bool load(const std::string& filename);

private:

    double threshold_;
    std::shared_ptr<utils::AdaBoostClassifier> adaboostModel_;

    // Private constructor for use with LearnClassifier
    GatewayClassifier(std::unique_ptr<utils::AdaBoostClassifier>&& adaboostModel);

    GatewayClassification adaboostClassification(const Vector& features) const;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_GATEWAYS_GATEWAY_CLASSIFIER_H
