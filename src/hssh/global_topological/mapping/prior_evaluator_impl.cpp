/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     prior_evaluator_impl.cpp
 * \author   Collin Johnson
 *
 * Definition of the various HypothesisPriorEvaluator subclasses and the create_topo_map_prior_evaluator() factory.
 */

#include "hssh/global_topological/mapping/prior_evaluator_impl.h"
#include "hssh/global_topological/state.h"

namespace vulcan
{
namespace hssh
{

///////////////////////////////// DirichletPriorEvaluator definition ////////////////////////////////////////////
DirichletPriorEvaluator::DirichletPriorEvaluator(const dirichlet_prior_evaluator_params_t& params) : params(params)
{
}


double DirichletPriorEvaluator::calculateLogPrior(const TopologicalState& state)
{
    // TODO
    return 0.0;
}

///////////////////////////////// BayesianInformationCriterionEvaluator definition
///////////////////////////////////////////////
BayesianInformationCriterionEvaluator::BayesianInformationCriterionEvaluator(
  const bayesian_information_criterion_evaluator_params_t& params)
: params(params)
{
}


double BayesianInformationCriterionEvaluator::calculateLogPrior(const TopologicalState& state)
{
    // Need to always have at least 1 here, so the log is a valid number.
    return -1.0 * state.map->numPlaces() * std::log(std::max(state.numPlaceVisits, 1));
}

}   // namespace hssh
}   // namespace vulcan
