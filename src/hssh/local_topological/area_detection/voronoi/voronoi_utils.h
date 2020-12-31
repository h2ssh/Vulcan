/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     voronoi_utils.h
* \author   Collin Johnson
*
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_VORONOI_UTILS_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_VORONOI_UTILS_H

#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "core/line.h"
#include "core/point.h"
#include <array>
#include <deque>
#include <set>
#include <vector>
#include <unordered_map>
#include <cstdint>

namespace vulcan
{

namespace utils { struct ray_trace_range_t; }

namespace hssh
{

using SourceToCellsMap = std::unordered_map<cell_t, std::vector<cell_t>, PointHash<cell_idx_t>>;
using NeighborArray    = std::array<cell_t, 8>;

enum cell_connectivity_t
{
    FOUR_WAY,               // Look up down, left right
    EIGHT_WAY,              // Look along the diagonals as well
    FOUR_THEN_EIGHT_WAY     // If not connected four way in a direction, then check the diagonals
};

enum trace_end_cause_t
{
    TRACE_FRONTIER,     // The trace ended at a frontier cell
    TRACE_JUNCTION,     // The trace ended at a junction cell
    TRACE_DISTANCE,     // The trace went the MAX_TRACE_CELLS distance
    TRACE_DEAD_END      // The trace ran out of cells
};

struct voronoi_trace_cells_t
{
    std::deque<cell_t> points;         // points[0] == skeletonCell
    trace_end_cause_t   endCause;
};

struct voronoi_trace_t
{
    cell_t                             skeletonCell;
    std::vector<voronoi_trace_cells_t> traces;
};

struct voronoi_directions_t
{
    double left;
    double right;

    double leftDist;        ///< Distance along skeleton the left direction was computed
    double rightDist;       ///< Distance along skeleton the right direction was computed
};

/**
* extract_cells_with_mask finds all cells with the given mask in the grid.
*
* \param    grid            Grid from which to extract the cells
* \param    mask            Mask for the cells to extract
* \param    step            Step to take when iterating through the grid -- (optional, default = 1)
* \return   All cells matching the mask in the grid
*/
std::vector<cell_t> extract_cells_with_mask(const VoronoiSkeletonGrid& grid, uint8_t mask, int step = 1);

/**
* extract_source_cells creates a source_cells_t for each source cell in a VoronoiSkeletonGrid.
*
* \param    grid            Grid from which to extract the source cells
* \param    skeletonMask    Mask indicating valid classification of the skeleton cells to consider
* \return   A map of source cell -> skeleton cells.
*/
SourceToCellsMap extract_source_cells(const VoronoiSkeletonGrid& grid, uint8_t skeletonMask);

/**
* cell_vector_perimeter computes the discretized perimeter of a linearly-sorted collection of cells in the grid.
*/
float cell_vector_perimeter(CellConstIter begin, CellConstIter end, float metersPerCell);

/**
* num_source_cells_with_classification checks the source cells of a skeleton cell to see if it is one of the
* masked types.
*
* \param    cell            Skeleton cell to check
* \param    mask            Mask to apply to source cells
* \param    grid            Grid in which the cell exists.
* \return   Number of source cells with the given classification.
*/
int num_source_cells_with_classification(cell_t cell, uint8_t mask, const VoronoiSkeletonGrid& grid);

/**
* neighbor_cells_with_classification does a 4-way or 8-way search for the cells neighboring a cell
* that satisify the specified mask. To retrieve all neighboring skeleton cells, for example,
* one would set the mask as SKELETON_CELL_SKELETON.
*
* \param    cell                Cell for which to find the neighbors
* \param    classification      Classification mask (grid.getClassification() & classification) is the operation
* \param    grid                Grid in which to do the search
* \param    connectivity        Type of connectivity to check for
* \param    neighbors           Array in which to store the neighbors
* \return   The number of neighbors with the classification.
*/
std::size_t neighbor_cells_with_classification(const cell_t&              cell,
                                               uint8_t                    classification,
                                               const VoronoiSkeletonGrid& grid,
                                               cell_connectivity_t        connectivity,
                                               NeighborArray&             neighbors);

/**
* neighbor_cells_with_classification does a 4-way or 8-way search for the cells neighboring a cell
* that is equal to the specified classification. This is useful when dealing with frontier cells, which are
* associated with a number of underlying types
*
* \param    cell                Cell for which to find the neighbors
* \param    classification      Classification  (grid.getClassification() == classification) is the operation
* \param    grid                Grid in which to do the search
* \param    connectivity        Type of connectivity to check for
* \param    neighbors           Array in which to store the neighbors
* \return   The number of neighbors with the classification.
*/
std::size_t neighbor_cells_equal_classification(const cell_t&              cell,
                                                uint8_t                    classification,
                                                const VoronoiSkeletonGrid& grid,
                                                cell_connectivity_t        connectivity,
                                                NeighborArray&             neighbors);

/**
* num_neighbor_cells_with_classification does a 4-way or 8-way search for the cells neighboring a cell
* that satisify the specified mask. This function just counts the number of neighbors, while the above function provides
* them as output to the user.
*
* \param    cell                Cell for which to find the neighbors
* \param    classification      Classification mask (grid.getClassification() & classification) is the operation
* \param    grid                Grid in which to do the search
* \param    connectivity        Type of connectivity to check for
* \return   The number of neighbors with the classification.
*/
int num_neighbor_cells_with_classification(const cell_t&              cell,
                                           uint8_t                    classification,
                                           const VoronoiSkeletonGrid& grid,
                                           cell_connectivity_t        connectivity);

/**
* trace_ray_to_cell traces a ray starting from the specified point, along the given angle. The ray endpoint is determined
* when (grid.getClassification() & mask) || endpoint_is_edge_of_map().
*
* \param    angle           Direction along which to trace the ray
* \param    start           Starting point of the ray
* \param    mask            Mask to be satisfied for the endpoint of the ray
* \param    grid            Grid through which the ray should be traced
* \return   Endpoint of the ray.
*/
cell_t trace_ray_to_cell(float                      angle,
                         const Point<int>&    start,
                         uint8_t                    mask,
                         const VoronoiSkeletonGrid& grid);

/**
* trace_rays_in_range traces the rays in the specified range until a non-island mask cell is encountered. This function
* is for convenience in calling trace_ray_to_cell multiple times.
*
* If the defined range is for > 2*pi range, the cutoff will be after 2*pi is angle subtends by ray area.
*
* \param    range           Range of rays to be traced
* \param    start           Start point of the rays
* \param    mask            Mask specifying the cells of interest
* \param    grid            Grid in which to trace
* \return   Vector of cell endpoints for the rays.
*/
std::vector<cell_t> trace_rays_in_range(const utils::ray_trace_range_t& range,
                                        const Point<int>&         start,
                                        uint8_t                         mask,
                                        const VoronoiSkeletonGrid&      grid);


/**
* trace_voronoi_graph traces the Voronoi graph to the left and right of the provided cell.
*
* The first cell in each trace is the starting cell.
*
* \param    cell                        Cell at which to start the trace
* \param    grid                        Grid containing the graph
* \param    maxTraceLength              Maximum distance to follow a trace
* \param    numJunctionsBeforeHalt      Number of junctions to branch in before halting the search (optional, default = 0)
* \param    followOnlyReduced           Flag indicating if only reduced skeleton cells should be followed (optiona, default = true)
* \return   The individual traces along the various branches of the skeleton within the specified distance and number
*   of branches of the starting cell.
*/
voronoi_trace_t trace_voronoi_graph(const cell_t&              cell,
                                    const VoronoiSkeletonGrid& grid,
                                    int                        maxTraceLength,
                                    int                        numJunctionsBeforeHalt = 0,
                                    bool                       followOnlyReduced = true);

/**
* extract_edge_from_skeleton extracts all cells along an edge of the rEVG from the skeleton. The cells will be ordered
* in sequence along the edge, though which endpoint comes first is undetermined.
*
* If start is a junction point, then the edge length is 0, as this function doesn't follow multiple branches. If
* multiple branches are needed, used trace_voronoi_graph.
*
* \param[in]    start       Starting point of the search for the edges
* \param[in]    grid        Skeleton grid through which to search
* \param[out]   edgeOut     Output iterator in which to store the edge cells.
* \return   One-past-the-end iterator for the cells along the edge.
*/
int extract_edge_from_skeleton(cell_t start, const VoronoiSkeletonGrid& grid, CellVector& edgeOut);

/**
* voronoi_direction calculates the direction the Voronoi graph is pointing as it moves through the
* specified skeleton cell.
*
* The direction is calculated using the following logic:
*
*   - The skeleton is explored to the left and right of the starting cell
*   - If either branch contains no cells, the angleHint is used for the direction
*   - Otherwise, each direction is a line fit to the trace of the Voronoi in that direction.
*
* Left corresponds to above a vertical piece of the skeleton in the grid's coordinate system.
*
* The returned directions point AWAY from the skeleton cell. They are the direction of a vector with the
* tail on skeleton cell.
*
* \param    skeletonCell            Cell on the skeleton for which to find the direction
* \param    grid                    Grid in which the skeleton was extracted
* \param    angleHint               Hint at the angle of the Voronoi skeleton -- the returned direction is the closest to this angle
*                                   Any value outside [-pi,pi] will be ignored. (optional, default = don't use it)
* \return   Direction of a line fit to the Voronoi graph passing through the skeleton cell on the left and right sides.
*/
voronoi_directions_t voronoi_direction(const cell_t& skeletonCell, const VoronoiSkeletonGrid& grid, double angleHint = 1000.0);

/**
* voronoi_direction calculates the direction the Voronoi graph is pointing as it moves through the
* specified skeleton cell.
*
* The logic is the same as above, but in this case, the trace for the skeleton cell has already been performed.
*
* \param    trace                   Trace of the skeleton for which to find the direction
* \param    angleHint               Hint at the angle of the Voronoi skeleton -- the returned direction is the closest to this angle
*                                   Any value outside [-pi,pi] will be ignored. (optional, default = don't use it)
* \return   Direction of a line fit to the Voronoi graph passing through the skeleton cell on the left and right sides.
*/
voronoi_directions_t voronoi_direction(const voronoi_trace_t& trace, double angleHint = 1000.0);

} // namespace hssh
} // namespace vulcan

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_VORONOI_UTILS_H
