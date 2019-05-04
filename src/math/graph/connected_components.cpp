/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     connected_components.cpp
* \author   Collin Johnson
* 
* Definition of ConnectedComponents.
*/

#include <math/graph/connected_components.h>
#include <stack>

namespace vulcan
{
namespace math
{
    
std::vector<PathGraph> ConnectedComponents::connectedComponents(const PathGraph& graph) const
{
    std::vector<PathGraph> subgraphs;
    
    visited.clear();
    
    auto& vertices = graph.getVertices();
    
    for(auto& vertex : vertices)
    {
        if(visited.find(vertex) == visited.end())
        {
            subgraphs.push_back(PathGraph());
            
            depthFirstSearch(vertex, subgraphs.back());
        }
    }
    
    return subgraphs;
}


void ConnectedComponents::depthFirstSearch(path_vertex_t* node, PathGraph& graph) const
{
    std::stack<path_vertex_t*> stack;
    
    stack.push(node);
    
    while(!stack.empty())
    {
        path_vertex_t* top = stack.top();
        
        graph.addVertex(top);
        visited.insert(top);
        stack.pop();
        
        for(uint8_t n = 0; n < top->numAdjacent; ++n)
        {
            if(visited.find(top->adjacent[n]) == visited.end())
            {
                stack.push(top->adjacent[n]);
            }
        }
    }
}
    
}
}
