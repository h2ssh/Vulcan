/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     likelihood_evaluator.cpp
* \author   Collin Johnson
*
* Definition of create_hypothesis_likelihood_evaluator factory.
*/

#include "hssh/global_topological/mapping/likelihood_evaluator.h"
#include "hssh/global_topological/mapping/likelihood_evaluator_impl.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace hssh
{

std::unique_ptr<HypothesisLikelihoodEvaluator> create_hypothesis_likelihood_evaluator(const std::string&                               type,
                                                                                      const hypothesis_probability_evaluator_params_t& params)
{
    if(type == EDGE_LENGTH_EVALUATOR_TYPE)
    {
        return std::unique_ptr<HypothesisLikelihoodEvaluator>(new EdgeLengthEvaluator(params.edgeLengthParams));
    }
    else if(type == LPM_MATCH_EVALUATOR_TYPE)
    {
        return std::unique_ptr<HypothesisLikelihoodEvaluator>(new LPMMatchEvaluator(params.lpmMatchParams));
    }
    else if(type == PLACE_LAYOUT_COMPATIBILITY_EVALUATOR_TYPE)
    {
        return std::unique_ptr<HypothesisLikelihoodEvaluator>(new PlaceLayoutCompatibilityEvaluator(params.compatibilityParams));
    }
    else if(type == CHI_LIKELIHOOD_EVALUATOR_TYPE)
    {
        return std::unique_ptr<HypothesisLikelihoodEvaluator>(new ChiLikelihoodEvaluator(params.chiLikelihoodParams));
    }
    else
    {
        std::cerr << "ERROR:Unknown hypothesis likelihood evaluator: " << type << " Exiting..." << std::endl;
        assert(false);
    }

    return std::unique_ptr<HypothesisLikelihoodEvaluator>();
}

} // namespace hssh
} // namespace vulcan
