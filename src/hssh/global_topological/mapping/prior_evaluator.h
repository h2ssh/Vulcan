/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     prior_evaluator.h
* \author   Collin Johnson
*
* Declaration of HypothesisPriorEvaluator interface and create_hypothesis_prior_evaluator() factory function.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PRIOR_EVALUATOR_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PRIOR_EVALUATOR_H

#include <memory>
#include <string>

namespace vulcan
{
namespace hssh
{

class HypothesisPriorEvaluator;
class TopologicalState;
struct hypothesis_probability_evaluator_params_t;

/**
* create_hypothesis_prior_evaluator is a factory function for creating new instances of HypothesisPriorEvaluator.
*
* NOTE: If an unknown type is provided, the function will crash immediately.
*
* \param    type            HypothesisPriorEvaluator instance to create
* \param    params          Parameters for the prior evaluator
* \return   The desired instance of HypothesisPriorEvaluator.
*/
std::unique_ptr<HypothesisPriorEvaluator> create_hypothesis_prior_evaluator(const std::string&                               type,
                                                                            const hypothesis_probability_evaluator_params_t& params);

/**
* HypothesisPriorEvaluator is an interface for subclasses that calculate the prior for a
* map. Only the map is needed for calculation of the prior because the actions for a map
* hypothesis are unique to each hypothesis, so the hypothesis itself provides the necessary
* information for evaluation of the prior.
*
* The interface consists of a single method:
*
*   double calculateLogPrior(const TopologicalMapHypothesis& map)
*       - calculate the prior for the map given the actions taken so far
*/
class HypothesisPriorEvaluator
{
public:

    virtual ~HypothesisPriorEvaluator(void) { }

    /**
    * calculateLogPrior calculates the prior for a map hypothesis given the actions taken up to the
    * current time. The equation calculated is p(map | actions).
    *
    * \param    state           Map for which to calculate the prior
    * \return   Prior probability for the map.
    */
    virtual double calculateLogPrior(const TopologicalState& state) = 0;
};

} // namespace hssh 
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PRIOR_EVALUATOR_H
