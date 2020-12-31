/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     prior_evaluator_impl.h
* \author   Collin Johnson
*
* Declaration of subclasses of HypothesisPriorEvaluator:
*
*   - UniformPriorEvaluator
*   - DirichletPriorEvaluator
*   - BayesianInformationCriterionEvaluator
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PRIOR_EVALUATOR_IMPL_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PRIOR_EVALUATOR_IMPL_H

#include "hssh/global_topological/mapping/prior_evaluator.h"
#include "hssh/global_topological/params.h"

namespace vulcan
{
namespace hssh
{

const std::string UNIFORM_PRIOR_EVALUATOR_TYPE                 ("uniform");
const std::string DIRICHLET_PRIOR_EVALUATOR_TYPE               ("dirichlet");
const std::string BAYESIAN_INFORMATION_CRITERION_EVALUATOR_TYPE("bayesian-information");

/**
* UniformPriorEvaluator
*/
class UniformPriorEvaluator : public HypothesisPriorEvaluator
{
public:

    /**
    * Constructor for UniformPriorEvaluator.
    */
    UniformPriorEvaluator(void) {}

    /////   HypothesisPriorEvaluator interface   /////
    double calculateLogPrior(const TopologicalState& state) override { return 0.0; }
};

/**
* DirichletPriorEvaluator
*/
class DirichletPriorEvaluator : public HypothesisPriorEvaluator
{
public:

    /**
    * Constructor for DirichletPriorEvaluator.
    *
    * \param    params          Parameters for the constants used in the prior
    */
    DirichletPriorEvaluator(const dirichlet_prior_evaluator_params_t& params);

    /////   HypothesisPriorEvaluator interface   /////
    double calculateLogPrior(const TopologicalState& state) override;

private:

    dirichlet_prior_evaluator_params_t params;
};

/**
* BayesianInformationCriterionEvaluator
*/
class BayesianInformationCriterionEvaluator : public HypothesisPriorEvaluator
{
public:

    /**
    *  Constructor for BayesianInformationCriterionEvaluator.
    *
    * \param    params          Parameters with the constants for the information criterion
    */
    BayesianInformationCriterionEvaluator(const bayesian_information_criterion_evaluator_params_t& params);

    /////   HypothesisPriorEvaluator interface   /////
    double calculateLogPrior(const TopologicalState& state) override;

private:

    bayesian_information_criterion_evaluator_params_t params;
};

}
}

#endif // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_PRIOR_EVALUATOR_IMPL_H
