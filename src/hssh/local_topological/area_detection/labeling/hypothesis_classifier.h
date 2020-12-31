/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     hypothesis_classifier.h
 * \author   Collin Johnson
 *
 * Declaration of HypothesisClassifier.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_CLASSIFIER_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_CLASSIFIER_H

#include "hssh/local_topological/area_detection/labeling/type_distribution.h"
#include "utils/boosting.h"

namespace vulcan
{
namespace hssh
{

class HypothesisFeatures;
class LabeledAreaData;


/**
 * HypothesisClassifier is a classifier that only calculates the likelihood of a given type in one-against-all
 * fashion, instead of a single multi-class classification. Thus, this classifier will assign appropriateness values
 * more akin to see how similiar a new observation is compared to the training examples.
 */
class HypothesisClassifier
{
public:
    /**
     * LearnClassifier performs supervised learning for learning a classifier for the different types of
     * areas. The classifier learned selects between the full ontology of path, decision point, and destination.
     *
     * \pre      At least five examples of each type of area (decision, dest, path) in the examples.
     * \param    examples            Examples from which to learn a classifier
     * \return   A classifier trained to distinguish between path, dest, and decision.
     */
    static std::unique_ptr<HypothesisClassifier> LearnClassifier(const LabeledAreaData& examples);

    /**
     * Constructor for HypothesisClassifier.
     *
     * \param    filename        Name of the file containing the stored classifier information
     */
    HypothesisClassifier(const std::string& filename);

    /**
     * classifyHypothesis determines the maximum likelihood type of the provided hypothesis.
     *
     * \param    features            Features to be classified
     * \return   The best type for the hypothesis
     */
    HypothesisType classify(const HypothesisFeatures& features) const;

    /**
     * calculateDistribution calculates the full probability distribution for all types that can be assigned to the
     * hypothesis.
     *
     * \param    features            Features being classified
     * \return   Full probability distribution across possible types for the hypothesis.
     */
    HypothesisTypeDistribution calculateDistribution(const HypothesisFeatures& features) const;

    /**
     * calculateRawDistribution calculates the unnormalized distribution for each hypothesis type.
     * This unnormalized distribution can be used to see how much certainty actually exists about an hypothesis.
     * If none of the probabilities are very high, then it's an extremely uncertain area and thus shouldn't affect
     * the sampling algorithm much.
     *
     * \param    features            Features being classified
     * \return   Unnormalized probability distribution across possible types for the hypothesis.
     */
    HypothesisTypeDistribution calculateRawDistribution(const HypothesisFeatures& features) const;

    /**
     * calculateRawBoostingDistribution calculates just the unnormalized boosting distribution for each type.
     */
    HypothesisTypeDistribution calculateRawBoostingDistribution(const HypothesisFeatures& features) const;

    /**
     * calculateRawIsovistDistribution calculates just the unnormalized isovist distribution for each type.
     */
    HypothesisTypeDistribution calculateRawIsovistDistribution(const HypothesisFeatures& features) const;

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
    struct type_classifier_t
    {
        HypothesisType type;   ///< Type that this instance is classifier for

        utils::AdaBoostModelPtr areaClassifier;      ///< AdaBoost w/decision stumps classifier
        utils::AdaBoostModelPtr isovistClassifier;   ///< AdaBoost w/decision stumps classifier for isovists
    };

    type_classifier_t pathClassifier_;
    type_classifier_t destClassifier_;
    type_classifier_t decisionClassifier_;

    HypothesisClassifier(type_classifier_t&& pathClassifier,
                         type_classifier_t&& destClassifier,
                         type_classifier_t&& decisionClassifier);

    Vector boostingVotes(const Vector& features) const;         // unnormalized
    Vector isovistVotes(const Matrix& isovistFeatures) const;   // unnormalized

    bool saveClassifier(const type_classifier_t& classifier,
                        const std::string& baseName,
                        const std::string& isovistBaseName,
                        const std::string& typeName) const;
    bool loadClassifier(const std::string& baseName,
                        const std::string& isovistBaseName,
                        const std::string& typeName,
                        type_classifier_t& classifier);
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_LIKELIHOOD_ONLY_CLASSIFIER_H
