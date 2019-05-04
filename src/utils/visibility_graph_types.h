/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     visibility_graph_types.h
* \author   Collin Johnson
*
* Definition of utility types for the VisibilityGraph implementation.
*/

#ifndef UTILS_VISIBILITY_GRAPH_TYPES_H
#define UTILS_VISIBILITY_GRAPH_TYPES_H

#include <core/point.h>
#include <vector>

namespace vulcan
{
namespace utils
{

using VisGraphVertex = Point<int>;
using VisVertexIter = std::vector<utils::VisGraphVertex>::const_iterator;

}
}

#endif // UTILS_VISIBILITY_GRAPH_TYPES_H
