/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     factor_graph.h
* \author   Collin Johnson
* 
* Declaration of FactorGraph.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_GRAPH_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_GRAPH_H

#include "hssh/local_topological/area_detection/labeling/factor.h"
#include "hssh/local_topological/area_detection/labeling/factor_edge.h"
#include "hssh/local_topological/area_detection/labeling/variable.h"
#include <vector>

namespace vulcan 
{
namespace hssh 
{

/**
* FactorGraph is a simple representation of a factor graph consisting of:
* 
*   - Variables
*   - Factors
*   - FactorEdges
*/
class FactorGraph
{
public:
    
    using VarIter = std::vector<Variable::Ptr>::iterator;
    using FactorIter = std::vector<Factor::Ptr>::iterator;
    using EdgeIter = std::vector<FactorEdge::Ptr>::iterator;
    
    /**
    * Constructor for FactorGraph.
    * 
    * \param    variables           Variable nodes in the graph
    * \param    factors             Factor nodes in the graph
    * \param    edges               Edges along which messages are passed
    */
    FactorGraph(std::vector<Variable::Ptr> variables, 
                std::vector<Factor::Ptr> factors,
                std::vector<FactorEdge::Ptr> edges);
    
    FactorGraph(void) = default;

    // Iterate over the entities in the graph
    VarIter beginVars(void) { return vars_.begin(); }
    VarIter endVars(void) { return vars_.end(); }
    std::size_t sizeVars(void) { return vars_.size(); }
    
    FactorIter beginFactors(void) { return factors_.begin(); }
    FactorIter endFactors(void) { return factors_.end(); }
    std::size_t sizeFactors(void) { return factors_.size(); }
    
    EdgeIter beginEdges(void) { return edges_.begin(); }
    EdgeIter endEdges(void) { return edges_.end(); }
    std::size_t sizeEdges(void) { return edges_.size(); }
    
private:
    
    std::vector<Variable::Ptr> vars_;
    std::vector<Factor::Ptr> factors_;
    std::vector<FactorEdge::Ptr> edges_;
};

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_GRAPH_H
