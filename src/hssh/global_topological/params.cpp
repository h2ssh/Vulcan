/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.cpp
* \author   Collin Johnson
*
* Definition of the various parsers for the params structs for the global_topo_hssh module.
*/

#include "hssh/global_topological/params.h"
#include "utils/config_file.h"

namespace vulcan
{
namespace hssh
{

const std::string MODULE_HEADING("GlobalTopoParameters");

const std::string QUEUE_HEADING("GlobalTopoDataQueueParameters");
const std::string POSE_DIST_KEY("distance_between_pose_events");

const std::string GEN_QUEUE_HEADING("GeneratorQueueParameters");
const std::string QUEUE_TYPE_KEY("queue_type");

const std::string HYP_GEN_HEADING("HypothesisGeneratorParameters");
const std::string GEN_TYPE_KEY("generator_type");

const std::string TOPOLOGICAL_MAPPER_HEADING("TopologicalMapperParameters");
const std::string LOCALIZER_TYPE_KEY        ("localizer_type");

const std::string LAZY_EVALUATION_HEADING("LazyEvaluationMapperParameters");
const std::string MIN_TO_EXPAND_KEY      ("min_hypotheses_to_expand");
const std::string USE_HEURISTIC_KEY      ("use_heuristic");

const std::string MAP_OPTIMIZER_HEADING("MapOptimizerParameters");
const std::string OPTIMIZER_TYPE_KEY   ("optimizer_type");

const std::string LEV_MAR_OPTIMIZER_HEADING("LevMarOptimizerParameters");
const std::string MAX_OPT_ITERS_KEY        ("max_iterations");
const std::string MU_KEY                   ("initial_mu");
const std::string STOP_THRESHOLD_KEY       ("stop_threshold");

const std::string PROBABILITY_EVALUATOR_HEADING("HypothesisProbabilityEvaluatorParameters");
const std::string LIKELIHOOD_EVALUTORS_KEY     ("likelihood_evaluators");
const std::string PRIOR_EVALUATOR_KEY          ("prior_evaluator");


global_topo_params_t::global_topo_params_t(const utils::ConfigFile& config)
: queueParams(config)
{
}


global_topo_data_queue_params_t::global_topo_data_queue_params_t(const utils::ConfigFile& config)
: distanceBetweenPoseEvents(config.getValueAsFloat(QUEUE_HEADING, POSE_DIST_KEY))
{
}


lazy_evaluation_mapper_params_t load_lazy_evaluation_params(const utils::ConfigFile& config)
{
    lazy_evaluation_mapper_params_t params;

    params.minHypothesesToExpand = config.getValueAsUInt32(LAZY_EVALUATION_HEADING, MIN_TO_EXPAND_KEY);
    params.useHeuristic          = config.getValueAsBool  (LAZY_EVALUATION_HEADING, USE_HEURISTIC_KEY);

    return params;
}


hypothesis_generator_params_t::hypothesis_generator_params_t(const utils::ConfigFile& config)
: generatorType(config.getValueAsString(HYP_GEN_HEADING, GEN_TYPE_KEY))
{
    assert(!generatorType.empty());
}


generator_queue_params_t::generator_queue_params_t(const utils::ConfigFile& config)
: queueType(config.getValueAsString(GEN_QUEUE_HEADING, QUEUE_TYPE_KEY))
{
    assert(!queueType.empty());
}


hypothesis_probability_evaluator_params_t::hypothesis_probability_evaluator_params_t(const utils::ConfigFile& config)
: likelihoodEvaluators(config.getValueAsString(PROBABILITY_EVALUATOR_HEADING, LIKELIHOOD_EVALUTORS_KEY))
, priorEvaluator(config.getValueAsString(PROBABILITY_EVALUATOR_HEADING, PRIOR_EVALUATOR_KEY))
, edgeLengthParams(config)
, lpmMatchParams(config)
, compatibilityParams(config)
, chiLikelihoodParams(config)
, dirichletParams(config)
, bayesianInformationParams(config)
{
    assert(!likelihoodEvaluators.empty());
    assert(!priorEvaluator.empty());
}


edge_length_evaluator_params_t::edge_length_evaluator_params_t(const utils::ConfigFile& config)
{

}


lpm_match_evaluator_params_t::lpm_match_evaluator_params_t(const utils::ConfigFile& config)
{

}


place_layout_compatibility_evaluator_params_t::place_layout_compatibility_evaluator_params_t(const utils::ConfigFile& config)
{

}


chi_likelihood_evaluator_params_t::chi_likelihood_evaluator_params_t(const utils::ConfigFile& config)
{

}


dirichlet_prior_evaluator_params_t::dirichlet_prior_evaluator_params_t(const utils::ConfigFile& config)
{

}


bayesian_information_criterion_evaluator_params_t::bayesian_information_criterion_evaluator_params_t(const utils::ConfigFile& config)
{

}


map_optimizer_params_t::map_optimizer_params_t(const utils::ConfigFile& config)
: type(config.getValueAsString(MAP_OPTIMIZER_HEADING, OPTIMIZER_TYPE_KEY))
, levMarParams(config)
{
}


lev_mar_optimizer_params_t::lev_mar_optimizer_params_t(const utils::ConfigFile& config)
: maxIterations(config.getValueAsUInt32(LEV_MAR_OPTIMIZER_HEADING, MAX_OPT_ITERS_KEY))
, initialMu(config.getValueAsDouble(LEV_MAR_OPTIMIZER_HEADING, MU_KEY))
, stopThreshold(config.getValueAsDouble(LEV_MAR_OPTIMIZER_HEADING, STOP_THRESHOLD_KEY))
{
}

} // namespace hssh
} // namespace vulcan
