/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     evaluator.cpp
* \author   Collin Johnson
*
* Implementation of AreaHypothesisEvaluator.
*/

#include <hssh/local_topological/area_detection/labeling/evaluator.h>
#include <hssh/local_topological/area_detection/labeling/boundary.h>
#include <hssh/local_topological/area_detection/labeling/boundary_classifier.h>
#include <hssh/local_topological/area_detection/labeling/hypothesis_classifier.h>
#include <hssh/local_topological/params.h>
#include <cassert>

namespace vulcan
{
namespace hssh
{

//////////////////////////////// AreaHypothesisEvaluator /////////////////////////////////////////////

AreaHypothesisEvaluator::AreaHypothesisEvaluator(const std::string& mapName)
: classifier_(std::make_unique<HypothesisClassifier>(mapName))
, boundaryClassifier_(std::make_unique<AreaBoundaryClassifier>())
{
    boundaryClassifier_->load(mapName);
}


AreaHypothesisEvaluator::~AreaHypothesisEvaluator(void)
{
    // For std::unique_ptr
}


double AreaHypothesisEvaluator::calculateLikelihood(const HypothesisFeatures& features, HypothesisType type) const
{
    switch(type)
    {
    case HypothesisType::kArea:      // treat the area like a path so the initial assignment will still have an evaluation
    case HypothesisType::kPath:
        return pathSegmentLikelihood(features);

    case HypothesisType::kDecision:
        return decisionPointLikelihood(features);

    case HypothesisType::kDest:
        return destinationLikelihood(features);

    default:
        assert(!"ERROR: Trying to calculate likelihood of unknown type!");
    }

    return 0.0;
}


double AreaHypothesisEvaluator::calculatePrior(const AreaHypothesisBoundary& boundary, bool isOn) const
{
    return boundaryClassifier_->classifyBoundary(boundary.getBoundaryType(boundary.getHypotheses()[0]),
                                                 boundary.getBoundaryType(boundary.getHypotheses()[1]),
                                                 isOn);
}


double AreaHypothesisEvaluator::calculatePrior(HypothesisType typeA, HypothesisType typeB, bool isOn) const
{
    return boundaryClassifier_->classifyBoundary(typeA, typeB, isOn);
}


double AreaHypothesisEvaluator::calculateLogPrior(HypothesisType typeA, HypothesisType typeB, bool isOn) const
{
    return boundaryClassifier_->classifyBoundaryLog(typeA, typeB, isOn);
}


HypothesisType AreaHypothesisEvaluator::calculateMaximumLikelihoodType(const HypothesisFeatures& features) const
{
    return classifier_->classify(features);
}


HypothesisTypeDistribution AreaHypothesisEvaluator::calculateTypeDistribution(const HypothesisFeatures& features) const
{
    return classifier_->calculateDistribution(features);
}


BoundaryTypeDistribution AreaHypothesisEvaluator::fullBoundaryDistribution(void) const
{
    return boundaryClassifier_->boundaryDistribution();
}


double AreaHypothesisEvaluator::pathSegmentLikelihood(const HypothesisFeatures& features) const
{
    auto distribution = classifier_->calculateDistribution(features);
    return distribution.path;
}


double AreaHypothesisEvaluator::destinationLikelihood(const HypothesisFeatures& features) const
{
    auto distribution = classifier_->calculateDistribution(features);
    return distribution.destination;
}


double AreaHypothesisEvaluator::decisionPointLikelihood(const HypothesisFeatures& features) const
{
    auto distribution = classifier_->calculateDistribution(features);
    return distribution.decision;
}

} // namespace hssh
} // namespace vulcan
