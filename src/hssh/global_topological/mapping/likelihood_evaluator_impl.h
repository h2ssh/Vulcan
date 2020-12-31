/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     likelihood_evaluator_impl.h
 * \author   Collin Johnson
 *
 * Declaration of subclasses of HypothesisLikelihoodEvaluator:
 *
 *   - EdgeLengthEvaluator
 *   - LPMMatchEvaluator
 *   - PlaceLayoutCompatibilityEvaluator
 *   - ChiLikelihood
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LIKELIHOOD_EVALUATOR_IMPL_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LIKELIHOOD_EVALUATOR_IMPL_H

#include "hssh/global_topological/mapping/likelihood_evaluator.h"
#include "hssh/global_topological/params.h"

namespace vulcan
{
namespace hssh
{

const std::string EDGE_LENGTH_EVALUATOR_TYPE("edge-length");
const std::string LPM_MATCH_EVALUATOR_TYPE("lpm-match");
const std::string PLACE_LAYOUT_COMPATIBILITY_EVALUATOR_TYPE("place-layout-compatibility");
const std::string CHI_LIKELIHOOD_EVALUATOR_TYPE("chi-likelihood");

/**
 * EdgeLengthEvaluator evaluates the likelihood of the length of the edge traversed to
 * reach the current place. The map contains an estimate of the mean edge length for each
 * edge. Using this distribution, the likelihood of measuring the new edge length can
 * be easily calculated assuming a zero-mean Gaussian distribution.
 */
class EdgeLengthEvaluator : public HypothesisLikelihoodEvaluator
{
public:
    /**
     * Constructor for EdgeLengthEvaluator.
     *
     * \param    params          Parameters for evaluating likelihood of measured edge lengths
     */
    EdgeLengthEvaluator(const edge_length_evaluator_params_t& params);

    // HypothesisLikelihoodEvaluator interface
    double calculateLogLikelihood(const TopologicalState& state, const MetricMapCache& places) override;

private:
    edge_length_evaluator_params_t params;
};

/**
 * LPMMatchEvaluator calculates the likelihood of two LPMs being the same. The map matching finds the optimal overlap
 * between the stored LPM at a place and the LPM from the current visit.
 *
 * TODO: How exactly is this calculated?
 */
class LPMMatchEvaluator : public HypothesisLikelihoodEvaluator
{
public:
    /**
     * Constructor for LPMMatchEvaluator.
     *
     * \param    params          Parameters for evaluating the likelihood of an LPM match
     */
    LPMMatchEvaluator(const lpm_match_evaluator_params_t& params);

    // HypothesisLikelihoodEvaluator interface
    double calculateLogLikelihood(const TopologicalState& state, const MetricMapCache& places) override;

private:
    lpm_match_evaluator_params_t params;
};

/**
 * PlaceLayoutCompatibilityEvaluator
 *
 * TODO: Figured out exactly how this gets calculated.
 */
class PlaceLayoutCompatibilityEvaluator : public HypothesisLikelihoodEvaluator
{
public:
    /**
     * Constructor for PlaceLayoutCompatibilityEvaluator.
     *
     * \param    params          Parameters for evaluating the compatibility of the map's layout
     */
    PlaceLayoutCompatibilityEvaluator(const place_layout_compatibility_evaluator_params_t& params);

    // HypothesisLikelihoodEvaluator interface
    double calculateLogLikelihood(const TopologicalState& state, const MetricMapCache& places) override;

private:
    place_layout_compatibility_evaluator_params_t params;
};

/**
 * ChiLikelihoodEvaluator calculates the likelihood of a Chi value for a given map hypothesis. The Chi value contains
 * the optimized position of each place in the a single global reference frame. The lambda value from the current event
 * can then be compared against the expected Chi to determine how well the values actually match.
 *
 * TODO: Once the exact form of Chi is determined, explain this better
 */
class ChiLikelihoodEvaluator : public HypothesisLikelihoodEvaluator
{
public:
    /**
     * Constructor for ChiLikelihoodEvaluator.
     *
     * \param    params          Parameters for evaluating the fit of the chi values to the measured lambdas
     */
    ChiLikelihoodEvaluator(const chi_likelihood_evaluator_params_t& params);

    // HypothesisLikelihoodEvaluator interface
    double calculateLogLikelihood(const TopologicalState& state, const MetricMapCache& places) override;

private:
    chi_likelihood_evaluator_params_t params;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_LIKELIHOOD_EVALUATOR_IMPL_H
