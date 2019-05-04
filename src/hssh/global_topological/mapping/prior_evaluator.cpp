/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     prior_evaluator.cpp
* \author   Collin Johnson
*
* Definition of create_hypothesis_prior_evaluator() factory function.
*/

#include <hssh/global_topological/mapping/prior_evaluator.h>
#include <hssh/global_topological/mapping/prior_evaluator_impl.h>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace hssh
{

std::unique_ptr<HypothesisPriorEvaluator> create_hypothesis_prior_evaluator(const std::string&                               type,
                                                                            const hypothesis_probability_evaluator_params_t& params)
{
    if(type == UNIFORM_PRIOR_EVALUATOR_TYPE)
    {
        return std::unique_ptr<HypothesisPriorEvaluator>(new UniformPriorEvaluator());
    }
    else if(type == DIRICHLET_PRIOR_EVALUATOR_TYPE)
    {
        return std::unique_ptr<HypothesisPriorEvaluator>(new DirichletPriorEvaluator(params.dirichletParams));
    }
    else if(type == BAYESIAN_INFORMATION_CRITERION_EVALUATOR_TYPE)
    {
        return std::unique_ptr<HypothesisPriorEvaluator>(new BayesianInformationCriterionEvaluator(params.bayesianInformationParams));
    }
    else
    {
        std::cerr<<"ERROR:Unknown hypothesis prior type: "<<type<<" Exiting...\n";
        assert(false);
    }

    return std::unique_ptr<HypothesisPriorEvaluator>();
}

}
}
