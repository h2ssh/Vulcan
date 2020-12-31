/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     hypothesis_factor_graph.h
* \author   Collin Johnson
*
* Declaration of functions and structs for HypothesisGraph -> FactorGraph conversion:
*
*   - HypothesisFactorGraph : holds the FactorGraph of the HypGraph and a mapping of the variables in the FG to the
*       boundaries and hypotheses in the HypGraph.
*   - convert_hypothesis_graph_to_factor_graph : convert the hypothesis graph to its corresponding factor graph
*       representation
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_FACTOR_GRAPH_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_FACTOR_GRAPH_H

#include "hssh/local_topological/area_detection/labeling/factor_graph.h"
#include "hssh/local_topological/area_detection/labeling/hypothesis_graph.h"

namespace vulcan
{
namespace hssh
{

class BoundaryClassifier;
class HypothesisClassifier;

struct HypothesisFactorGraph
{
    FactorGraph graph;
    std::unordered_map<int, AreaHypothesisBoundary*> varToBoundary;
    std::unordered_map<int, AreaHypothesis*> varToHyp;
};

/**
* convert_hypothesis_graph_to_factor_graph converts a HypothesisGraph into the corresponding factor graph model for
* the area and boundary distributions. The provided evaluator is used to provide the model for those distributions.
*
* \param    hypGraph            HypothesisGraph with all areas and boundaries
* \param    hypClassifier       Classifier for computing distribution across label types
* \param    bndClassifier       Classifier for computing distribution across boundary relations
* \return   HypothesisFactorGraph containing the factor graph and proper mapping of graph variables back to their
*   corresponding boundaries and hypotheses.
*/
HypothesisFactorGraph convert_hypothesis_graph_to_factor_graph(HypothesisGraph& hypGraph,
                                                               const HypothesisClassifier& hypClassifier,
                                                               const BoundaryClassifier& bndClassifier);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_HYPOTHESIS_FACTOR_GRAPH_H
