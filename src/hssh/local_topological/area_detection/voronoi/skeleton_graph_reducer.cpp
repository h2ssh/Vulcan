/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     skeleton_graph_reducer.cpp
* \author   Collin Johnson
*
* Definition of create_skeleton_graph_reducer factory.
*/

#include <hssh/local_topological/area_detection/voronoi/skeleton_graph_reducer.h>
#include <hssh/local_topological/area_detection/voronoi/all_paths_graph_reducer.h>
#include <iostream>
#include <cassert>

namespace vulcan
{
namespace hssh
{
    
// Static factory for creating reducers
std::unique_ptr<SkeletonGraphReducer> create_skeleton_graph_reducer(const std::string& reducerName)
{
    if(reducerName == ALL_PATHS_GRAPH_REDUCER_TYPE)
    {
        return std::unique_ptr<SkeletonGraphReducer>(new AllPathsGraphReducer());
    }
    else
    {
        std::cerr<<"ERROR: Unknown SkeletonGraphReducer type: "<<reducerName<<std::endl;
        assert(false);
    }
    
    return std::unique_ptr<SkeletonGraphReducer>();
}

}
}
