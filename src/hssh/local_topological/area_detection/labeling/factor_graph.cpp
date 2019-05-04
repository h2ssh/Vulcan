/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     factor_graph.cpp
* \author   Collin Johnson
* 
* Definition of FactorGraph.
*/

#include <hssh/local_topological/area_detection/labeling/factor_graph.h>

namespace vulcan 
{
namespace hssh 
{

FactorGraph::FactorGraph(std::vector<Variable::Ptr> variables, 
                         std::vector<Factor::Ptr> factors,
                         std::vector<FactorEdge::Ptr> edges)
: vars_(variables)
, factors_(factors)
, edges_(edges)
{
}
    
} // namespace hssh
} // namespace vulcan
