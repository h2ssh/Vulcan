/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     params.h
* \author   Collin Johnson
*
* Definition of the params structs for the global_topo_hssh module.
*/

#ifndef HSSH_GLOBAL_TOPOLOGICAL_PARAMS_H
#define HSSH_GLOBAL_TOPOLOGICAL_PARAMS_H

#include <string>

namespace vulcan
{
namespace utils { class ConfigFile; }

namespace hssh
{

struct global_topo_data_queue_params_t
{
    float distanceBetweenPoseEvents;
    
    global_topo_data_queue_params_t(const utils::ConfigFile& config);
    global_topo_data_queue_params_t(void) = default;
};

struct hypothesis_generator_params_t
{
    std::string generatorType;
    
    hypothesis_generator_params_t(const utils::ConfigFile& config);
    hypothesis_generator_params_t(void) = default;
};

struct generator_queue_params_t
{
    std::string queueType;
    
    generator_queue_params_t(const utils::ConfigFile& config);
    generator_queue_params_t(void) = default;
};

struct lev_mar_optimizer_params_t
{
    size_t maxIterations;
    double initialMu;
    double stopThreshold;
    
    lev_mar_optimizer_params_t(const utils::ConfigFile& config);
    lev_mar_optimizer_params_t(void) = default;
};

struct map_optimizer_params_t
{
    std::string type;
    
    lev_mar_optimizer_params_t levMarParams;
    
    map_optimizer_params_t(const utils::ConfigFile& config);
    map_optimizer_params_t(void) = default;
};

struct edge_length_evaluator_params_t
{
    edge_length_evaluator_params_t(const utils::ConfigFile& config);
    edge_length_evaluator_params_t(void) = default;
};

struct lpm_match_evaluator_params_t
{
    lpm_match_evaluator_params_t(const utils::ConfigFile& config);
    lpm_match_evaluator_params_t(void) = default;
};

struct place_layout_compatibility_evaluator_params_t
{
    place_layout_compatibility_evaluator_params_t(const utils::ConfigFile& config);
    place_layout_compatibility_evaluator_params_t(void) = default;
};

struct chi_likelihood_evaluator_params_t
{
    chi_likelihood_evaluator_params_t(const utils::ConfigFile& config);
    chi_likelihood_evaluator_params_t(void) = default;
};

struct dirichlet_prior_evaluator_params_t
{
    dirichlet_prior_evaluator_params_t(const utils::ConfigFile& config);
    dirichlet_prior_evaluator_params_t(void) = default;
};

struct bayesian_information_criterion_evaluator_params_t
{
    bayesian_information_criterion_evaluator_params_t(const utils::ConfigFile& config);
    bayesian_information_criterion_evaluator_params_t(void) = default;
};

struct hypothesis_probability_evaluator_params_t
{
    std::string likelihoodEvaluators;
    std::string priorEvaluator;

    edge_length_evaluator_params_t                edgeLengthParams;
    lpm_match_evaluator_params_t                  lpmMatchParams;
    place_layout_compatibility_evaluator_params_t compatibilityParams;
    chi_likelihood_evaluator_params_t             chiLikelihoodParams;

    dirichlet_prior_evaluator_params_t                dirichletParams;
    bayesian_information_criterion_evaluator_params_t bayesianInformationParams;
    
    hypothesis_probability_evaluator_params_t(const utils::ConfigFile& config);
    hypothesis_probability_evaluator_params_t(void) = default;
};

struct lazy_evaluation_mapper_params_t
{
    uint32_t minHypothesesToExpand;
    bool     useHeuristic;

    hypothesis_probability_evaluator_params_t probabilityParams;
};

struct topological_mapper_params_t
{
    std::string optimizerType;
    std::string localizerType;

    map_optimizer_params_t optimizerParams;

    lazy_evaluation_mapper_params_t lazyEvaluationParams;
};

struct global_topo_params_t
{
    global_topo_data_queue_params_t queueParams;
    
    global_topo_params_t(const utils::ConfigFile& config);
    global_topo_params_t(void) = default;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_GLOBAL_TOPOLOGICAL_PARAMS_H
