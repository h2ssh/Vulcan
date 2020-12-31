/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     probability_evaluator.h
* \author   Collin Johnson
*
* Declaration of HypothesisProbabilityEvaluator.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PROBABILITY_EVALUATOR_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PROBABILITY_EVALUATOR_H

#include "hssh/global_topological/map_probability.h"
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{
    
class HypothesisLikelihoodEvaluator;
class HypothesisPriorEvaluator;
class MetricMapCache;
struct TopologicalState;

/**
* HypothesisProbabilityEvaluator evaluates the probability of a map hypothesis being the correct map
* given the measurements gathered by the robot and the actions taken by the robot up to the
* current time.
*
* The desired probability is p(map | measurements, actions). Using Bayes' Law, we can instead calculate:
*
*   p(map | measurements, actions) ~ p(measurements | map, actions) * p(map | actions)
*
* where:
*   - p(measurements | map, actions) is the measurement likelihood given the current map
*   - p(map | actions) is the prior likelihood that such a map could exist given the actions take by the robot
*
* The measurement likelihood is calculated by some collection of HypothesisLikelihoodEvaluator instances. The evaluators are
* assumed to be independent so the Naive Bayes assumption can be used:
*   p(measurements | map, actions) = p(measurement_A | map, actions) * ... * p(measurement_N)
*
* The prior is calculated by an instance of HypothesisPriorEvaluator.
*
* NOTE: Right now, only a likelihood is calculated for a hypothesis. Our formulation actually makes a prior potentially
*       unnecessary. However, the structure is in-place for supporting a prior should the need arise in the future.
*
* The interface for HypothesisProbabilityEvaluator is a single method:
*
*   hypothesis_probability_t calculateProbability(const TopologicalMapHypothesis& map, const topology_measurements_t& measurements);
*       - calculate the probability of this map given the most recent set of measurements
*
* The configuration parameters for HypothesisProbabilityEvaluator are:
*
*   [HypothesisProbabilityEvaluatorParameters]
*   likelihood_evaluators = a comma-separated list of likelihood evaluators to be used
*
*/
class HypothesisProbabilityEvaluator
{
public:

    /**
    * Constructor for HypothesisProbabilityEvaluator.
    *
    * \param    likelihoodEvaluators            Evaluators to use for a map likelihood
    * \param    priorEvaluator                  Evaluator to use for the prior
    */
    HypothesisProbabilityEvaluator(std::vector<std::unique_ptr<HypothesisLikelihoodEvaluator>> likelihoodEvaluators,
                                   std::unique_ptr<HypothesisPriorEvaluator> priorEvaluator);
    
    /**
    * Destructor for HypothesisProbabilityEvaluator.
    */
    ~HypothesisProbabilityEvaluator(void);

    /**
    * calculateProbability calculates the probability of a map hypothesis given the current measurements.
    *
    * \param    hypothesis      Hypothesis to be evaluated
    * \param    places          Stored local place models
    * \return   The calculated probability of the hypothesis.
    */
    TopoMapProbability calculateProbability(const TopologicalState& hypothesis, const MetricMapCache& places);

private:

    std::vector<std::unique_ptr<HypothesisLikelihoodEvaluator>> likelihoodEvaluators_;
    std::unique_ptr<HypothesisPriorEvaluator> priorEvaluator_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PROBABILITY_EVALUATOR_H
