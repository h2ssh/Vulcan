/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     factor_graph_utils.h
* \author   Collin Johnson
* 
* Declaration of utility functions for factor graph processing:
* 
*   - should_send_message_on_edge : check if a new fresh message can be sent along the given edge
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_GRAPH_UTILS_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_GRAPH_UTILS_H

#include <hssh/local_topological/area_detection/labeling/factor_edge.h>

namespace vulcan 
{
namespace hssh 
{

/**
* should_send_message_on_edge checks if a message can be sent along the edge in the specified direction. A message
* should be sent if no unset messages exist and at least one fresh message exists.
* 
* \param    edge            Edge to check if a message can be sent
* \param    direction       Direction in which the message will be sent
* \param    edges           All edges for the node
* \return   True if no unset messages exist and at least one fresh message.
*/
bool should_send_message_on_edge(FactorEdge* edge, MsgDir direction, const std::vector<FactorEdge*>& edges);

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_LABELING_FACTOR_GRAPH_UTILS_H
