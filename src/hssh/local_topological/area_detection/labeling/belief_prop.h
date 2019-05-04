/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     belief_prop.h
* \author   Collin Johnson
*
* Declaration of functions for running belief propagation:
*
*   - belief_propagation : standard sum-product algorithm on a tree-structure factor graph.
*   - loopy_belief_propagation : loopy sum-product algorithm on a general graph with loops.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_BELIEF_PROP_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_BELIEF_PROP_H

namespace vulcan
{
namespace hssh
{

enum class UpdateType;
class FactorGraph;

/**
* belief_propagation runs belief propagation on the provided tree-structured graph.
*
* After running belief propagation, the marginal for every Variable can be computed.
*
* \param    type            Type of belief prop to run -- sum-prod or max-prod
*/
void belief_propagation(FactorGraph& graph, UpdateType type);

/**
* loopy_belief_propagation runs belief propagation on the general graph with loops.
*
* After running belief propagation, the marginal for every Variable can be computed.
*
* \param    graph           Graph on which to run loopy belief prop
* \param    type            Type of belief prop to run -- sum-prod or max-prod
*/
void loopy_belief_propagation(FactorGraph& graph, UpdateType type);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_BELIEF_PROP_H
