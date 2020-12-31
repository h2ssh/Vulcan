/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     probability_evaluator.cpp
 * \author   Collin Johnson
 *
 * Definition of HypothesisProbabilityEvaluator.
 */

#include "hssh/global_topological/mapping/probability_evaluator.h"
#include "hssh/global_topological/mapping/likelihood_evaluator.h"
#include "hssh/global_topological/mapping/prior_evaluator.h"
#include "hssh/global_topological/state.h"
#include <cassert>
#include <iostream>

#define DEBUG_PROBABILITY

namespace vulcan
{
namespace hssh
{

HypothesisProbabilityEvaluator::HypothesisProbabilityEvaluator(
  std::vector<std::unique_ptr<HypothesisLikelihoodEvaluator>> likelihoodEvaluators,
  std::unique_ptr<HypothesisPriorEvaluator> priorEvaluator)
: likelihoodEvaluators_(std::move(likelihoodEvaluators))
, priorEvaluator_(std::move(priorEvaluator))
{
}


HypothesisProbabilityEvaluator::~HypothesisProbabilityEvaluator(void)
{
    // For std::unique_ptr
}


TopoMapProbability HypothesisProbabilityEvaluator::calculateProbability(const TopologicalState& hypothesis,
                                                                        const MetricMapCache& places)
{
    TopoMapProbability probability;

    double measurementLogLikelihood = 0.0;

    for (auto& eval : likelihoodEvaluators_) {
        measurementLogLikelihood = eval->calculateLogLikelihood(hypothesis, places);

        probability.measurementLogLikelihoods.push_back(measurementLogLikelihood);
        probability.logLikelihood += measurementLogLikelihood;
    }

    probability.logPrior = priorEvaluator_->calculateLogPrior(hypothesis);
    probability.logPosterior = probability.logLikelihood + probability.logPrior;

#ifdef DEBUG_PROBABILITY
    std::cout << "Likelihood:" << probability.logLikelihood << " Prior:" << probability.logPrior
              << " Posterior:" << probability.logPosterior << '\n';
#endif

    return probability;
}

}   // namespace hssh
}   // namespace vulcan
