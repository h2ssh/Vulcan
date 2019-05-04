/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     evaluator.h
* \author   Collin Johnson
*
* Definition of AreaHypothesisEvaluator.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREAS_SUBGRAPH_EVALUATOR_H
#define HSSH_LOCAL_TOPOLOGICAL_AREAS_SUBGRAPH_EVALUATOR_H

#include <hssh/local_topological/area_detection/labeling/hypothesis_type.h>
#include <hssh/local_topological/area_detection/labeling/type_distribution.h>
#include <memory>

namespace vulcan
{
namespace hssh
{

class AreaHypothesisBoundary;
class HypothesisClassifier;
class BoundaryClassifier;
class HypothesisFeatures;
struct hypothesis_evaluator_params_t;

/**
* AreaHypothesisEvaluator provides the probability distributions for the optimization. The classifiers to be used
* are stored here.
*/
class AreaHypothesisEvaluator
{
public:

    /**
    * Constructor for AreaHypothesisEvaluator.
    *
    * \param    mapName             Name of the map in which the robot is operating
    */
    AreaHypothesisEvaluator(const std::string& mapName);

    /**
    * Destructor for AreaHypothesisEvaluator.
    */
    ~AreaHypothesisEvaluator(void);

    /**
    * calculateLikelihood calculates the likelihood of a hypothesis. The likelihood calculation is calculated
    * as described in the class description.
    *
    * \param    features            Features of the hypothesis to find likelihood for
    * \param    type                Type of the hypothesis associated with the features
    * \return   Likelihood of the hypothesis.
    */
    double calculateLikelihood(const HypothesisFeatures& features, HypothesisType type) const;

    /**
    * calculatePrior calculates the prior of a particular boundary existing. A boundary prior
    * looks at the type of the adjacent areas, providing a prior that they coexist given the environments the
    * robot has encountered.
    *
    * \param    boundary        Boundary to be evaluated
    * \return   Prior probability of this boundary combination.
    */
    double calculatePrior(const AreaHypothesisBoundary& boundary, bool isOn) const;
    double calculatePrior(HypothesisType typeA, HypothesisType typeB, bool isOn) const;
    double calculateLogPrior(HypothesisType typeA, HypothesisType typeB, bool isOn) const;

    /**
    * calculateMaximumLikelihoodType calculates the maximum likelihood for a type using the features only. No constraints are
    * considered during the calculation.
    */
    HypothesisType calculateMaximumLikelihoodType(const HypothesisFeatures& features) const;

    /**
    * calculateTypeDistribution calculates the distribution across each label type.
    */
    HypothesisTypeDistribution calculateTypeDistribution(const HypothesisFeatures& features) const;

    /**
    * fullBoundaryDistribution retrieves the full distribution across all boundary relations.
    */
    BoundaryTypeDistribution fullBoundaryDistribution(void) const;

private:

    std::unique_ptr<HypothesisClassifier> classifier_;
    std::unique_ptr<BoundaryClassifier> boundaryClassifier_;

    double pathSegmentLikelihood  (const HypothesisFeatures& features) const;
    double destinationLikelihood  (const HypothesisFeatures& features) const;
    double decisionPointLikelihood(const HypothesisFeatures& features) const;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREAS_SUBGRAPH_EVALUATOR_H
