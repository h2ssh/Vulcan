/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     likelihood_evaluator.h
 * \author   Collin Johnson
 *
 * Declaration of HypothesisLikelihoodEvaluator interface and create_topo_map_likelihood_evaluator() factory.
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LIKELIHOOD_EVALUATOR_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LIKELIHOOD_EVALUATOR_H

#include <memory>
#include <string>

namespace vulcan
{
namespace hssh
{

class HypothesisLikelihoodEvaluator;
class MetricMapCache;
struct TopologicalState;
struct hypothesis_probability_evaluator_params_t;

/**
 * create_hypothesis_likelihood_evaluator is a factory function for creating new instances of
 * HypothesisLikelihoodEvaluator. The current subclasses of HypothesisLikelihoodEvaluator are defined
 * in likelihood_evaluator_impl.h.
 *
 * NOTE: If an unknown type is requested, the factory will fail immediately.
 *
 * \param    type            Type of HypothesisLikelihoodEvaluator to be created
 * \param    params          Parameters for the likelihood evaluator
 * \return   A new instance of HypothesisLikelihoodEvaluator.
 */
std::unique_ptr<HypothesisLikelihoodEvaluator>
  create_hypothesis_likelihood_evaluator(const std::string& type,
                                         const hypothesis_probability_evaluator_params_t& params);

/**
 * HypothesisLikelihoodEvaluator is an interface for subclasses that evaluate the likelihood of a particular
 * TopologicalMapHypothesis. Each likelihood evaluator implementation is assumed to be independent
 * of other evaluators, thereby allowing use of the Naive Bayes assumption when calculating the
 * overall likelihood of a map, given the measurements.
 *
 * The likelihood evaluator interface is a single method:
 *
 *   - calculateLogLikelihood : calculate the likelihood of a map hypothesis using the most recent set of measurements
 */
class HypothesisLikelihoodEvaluator
{
public:
    virtual ~HypothesisLikelihoodEvaluator(void) { }

    /**
     * calculateLogLikelihood calculates the likelihood of a map hypothesis using the most recently arrived set of
     * topology measurements. The calculation should be p(measurements | map)
     *
     * \param    state                   Topological state being evaluated
     * \param    places                  Place manager that maintains the local place models
     * \return   Likelihood of the measurements given this map.
     */
    virtual double calculateLogLikelihood(const TopologicalState& state, const MetricMapCache& places) = 0;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LIKELIHOOD_EVALUATOR_H
