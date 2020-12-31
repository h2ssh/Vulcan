/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     factor_graph_utils.cpp
* \author   Collin Johnson
* 
* Definition of utility functions for factor graph processing:
* 
*   - should_send_message_on_edge
*/

#include "hssh/local_topological/area_detection/labeling/factor_graph_utils.h"

namespace vulcan 
{
namespace hssh 
{

bool should_send_message_on_edge(FactorEdge* edge, MsgDir direction, const std::vector<FactorEdge*>& edges)
{
    int freshCount = 0;
    
    for(FactorEdge* other : edges)
    {
        // Every other edge must be set, and then a message can send to the desired edge
        if(other != edge)
        {
            // If an edge is unset, then definitely can't send a message
            if(other->status(direction) == MsgStatus::unset)
            {
                return false;
            }
            else if(other->status(direction) == MsgStatus::fresh)
            {
                ++freshCount;
            }
        }
    }
    
    return (freshCount > 0) // Need at least one fresh message
        || ((edges.size() == 1) && (edges.front()->status(direction) == MsgStatus::fresh)); // or a fresh msg and single edge
}

} // namespace hssh
} // namespace vulcan
