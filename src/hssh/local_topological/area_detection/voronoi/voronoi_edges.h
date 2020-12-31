/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     voronoi_edges.h
 * \author   Collin Johnson
 *
 * Declaration of VoronoiEdges.
 */

#ifndef HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_VORONOI_VORONOI_EDGES_H
#define HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_VORONOI_VORONOI_EDGES_H

#include "hssh/types.h"
#include <vector>

namespace vulcan
{
namespace hssh
{

class VoronoiSkeletonGrid;

/**
 * VoronoiEdges extracts and maintains each individual edge in the Voronoi skeleton. An edge is an ordered sequence of
 * cells between two junction points in the skeleton.
 */
class VoronoiEdges
{
public:
    using const_iterator = std::vector<CellVector>::const_iterator;

    /**
     * Constructor for VoronoiEdges.
     *
     * \param    skeleton        Skeleton from which to extract the edges
     * \param    mask            Mask to apply when considering the edges
     */
    VoronoiEdges(const VoronoiSkeletonGrid& skeleton, uint8_t mask);

    /**
     * findEdgeForCell finds the edges associated with a particular cell.
     *
     * \param    cell        Cell to find the edge for
     * \return   An iterator to the appropriate edge. end() if no edge exists for the cell.
     */
    const_iterator findEdgeForCell(cell_t cell) const;

    // Iteration support over the edges in the skeleton.
    std::size_t size(void) const { return edges_.size(); }
    bool empty(void) const { return edges_.empty(); }
    const_iterator begin(void) const { return edges_.begin(); }
    const_iterator end(void) const { return edges_.end(); }

private:
    std::vector<CellVector> edges_;
    CellToIntMap cellToEdge_;
};

}   // namespace hssh
}   // namespace vulcan

#endif   // HSSH_LOCAL_TOPOLOGICAL_AREA_DETECTION_VORONOI_VORONOI_EDGES_H
