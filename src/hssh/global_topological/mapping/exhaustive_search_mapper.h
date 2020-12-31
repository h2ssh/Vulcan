/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     exhaustive_search_mapper.h
 * \author   Collin Johnson
 *
 * Declaration of ExhaustiveSearchMapper.
 */

#ifndef HSSH_GLOBAL_TOPOLOGICAL_MAPPING_EXHAUSTIVE_SEARCH_MAPPER_H
#define HSSH_GLOBAL_TOPOLOGICAL_MAPPING_EXHAUSTIVE_SEARCH_MAPPER_H

#include "hssh/global_topological/mapping/topological_mapper.h"
#include <boost/shared_ptr.hpp>
#include <fstream>
#include <memory>
#include <vector>

namespace vulcan
{
namespace hssh
{

class LargeScaleStar;
class LocalPlace;
class TopologicalLocalizer;
class MapOptimizer;
class TopologicalMapHypothesis;
struct Lambda;
struct place_connection_t;
struct topological_mapper_params_t;

const std::string EXHAUSTIVE_SEARCH_MAPPER_TYPE("exhaustive-search");

/**
 * ExhaustiveSearchMapper implements an exhaustive tree-of-maps algorithm for determining the topological map
 * of the robot's environment. The mapping algorithm creates all possible map hypotheses on each update.
 *
 * This algorithm makes three simplifying assumptions:
 *
 *   A) All places are detected. There are no false negatives.
 *   B) When a place is found, all paths are detected. The detected topology of a place is the correct topology.
 *   C) Maps are planar.
 *
 * The parameters for the tree-of-maps mapper are:
 *
 *   [ExhaustiveSearchMapperParameters]
 *   optimizer_type = type of MapOptimizer to use for determining Chi
 */
class ExhaustiveSearchMapper : public TopologicalMapper
{
public:
    /**
     * Constructor for ExhaustiveSearchMapper.
     *
     * \param    params          Parameters for calculating map probabilities
     * \param    manager         Place manager to use for accessing the LocalPlaces
     */
    ExhaustiveSearchMapper(const topological_mapper_params_t& params, MetricMapCache& manager);

    // TopologicalMapper interface
    virtual void updateMap(const topology_action_t& action, const topology_measurements_t& measurements);

    virtual TopoMapPtr getUsableHypothesis(void) const;

private:
    void localizeHypotheses(const topology_action_t& action);
    void pruneInvalidHypotheses(const LargeScaleStar& actionStar);
    void createNewHypotheses(const LargeScaleStar& actionStar, const topology_measurements_t& measurements);
    void calculateHypothesesProbability(const topology_measurements_t& measurements);

    bool isValidHypothesis(const TopoMapPtr& hypothesis, const LargeScaleStar& placeStar);


    std::vector<TopoMapPtr> mapsToEvaluate;   // hypotheses created on current update

    std::unique_ptr<TopologicalLocalizer> localizer;
    std::unique_ptr<MapOptimizer> optimizer;

    size_t numHypothesesExpanded;
    size_t numHypothesesEvaluated;
    std::ofstream evaluationLog;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_GLOBAL_TOPOLOGICAL_MAPPING_EXHAUSTIVE_SEARCH_MAPPER_H
