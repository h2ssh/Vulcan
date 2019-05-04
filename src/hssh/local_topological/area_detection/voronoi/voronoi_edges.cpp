/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     voronoi_edges.cpp
* \author   Collin Johnson
* 
* Definition of VoronoiEdges.
*/

#include <hssh/local_topological/area_detection/voronoi/voronoi_edges.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_utils.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>

namespace vulcan
{
namespace hssh 
{
    
VoronoiEdges::VoronoiEdges(const VoronoiSkeletonGrid& skeleton, uint8_t mask)
{
    CellSet visitedCells;
    CellVector edgeCells;
    
    for(std::size_t y = 0; y < skeleton.getHeightInCells(); ++y)
    {
        for(std::size_t x = 0; x < skeleton.getWidthInCells(); ++x)
        {
            cell_t cell(x, y);
            // If this is a new cell we're encountering
            if((skeleton.getClassificationNoCheck(x, y) & mask) && 
                (visitedCells.find(cell) == visitedCells.end()))
            {
                // Then extract the edge for it
                edgeCells.clear();
                extract_edge_from_skeleton(cell, skeleton, edgeCells);
                visitedCells.insert(edgeCells.begin(), edgeCells.end());
                
                // All the cells along the edge should have the appropriate index marked to map back to the edge
                for(auto c : edgeCells)
                {
                    cellToEdge_[c] = edges_.size();
                }
                
                edges_.emplace_back(edgeCells);
            }
        }
    }
}


VoronoiEdges::const_iterator VoronoiEdges::findEdgeForCell(cell_t cell) const
{
    auto edgeIt = cellToEdge_.find(cell);
    return (edgeIt != cellToEdge_.end()) ? edges_.begin() + edgeIt->second : edges_.end();
}
    
} // namespace hssh
} // namespace vulcan
