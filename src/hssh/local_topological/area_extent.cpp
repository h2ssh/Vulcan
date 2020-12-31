/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_extent.cpp
* \author   Collin Johnson
*
* Definition of AreaExtent.
*/

#include "hssh/local_topological/area_extent.h"
#include "hssh/local_topological/gateway.h"
#include "hssh/local_topological/voronoi_skeleton_grid.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_topological/area_detection/gateways/gateway_utils.h"
#include "math/geometry/convex_hull.h"
#include "math/geometry/shape_fitting.h"
#include "utils/ray_tracing.h"
#include "utils/algorithm_ext.h"
#include <deque>
#include <numeric>
#include <cassert>

namespace vulcan
{
namespace hssh
{

struct extent_search_state_t
{
    CellSet visited;
    CellSet boundaryCells;
    std::unordered_map<int, CellVector> gatewayToBoundaryCells;
    std::deque<cell_t> queue;
    const VoronoiSkeletonGrid* grid;
};


float discretized_polygon_perimeter(const math::Polygon<double>& polygon, const VoronoiSkeletonGrid& grid);

std::tuple<float, float> flood_fill_extent(const std::vector<Gateway>&      gateways,
                                           CellVector                       startCells,
                                           const VoronoiSkeletonGrid&       grid,
                                           std::vector<Point<double>>& cells);
int extent_side_of_gateway(const Gateway& gateway, CellVector& startCells, const VoronoiSkeletonGrid& grid);
void add_gateway_boundary_cells_to_queue(const Gateway& gateway, extent_search_state_t& state);
void add_boundary_adjacent_cells_to_queue(const Gateway& gateway, int extentSide, extent_search_state_t& state);
std::tuple<float, float> expand_cell(cell_t cell, extent_search_state_t& state);
std::tuple<float, float> expand_skeleton_cell(cell_t skeleton, extent_search_state_t& state);
std::tuple<float, float> perimeter_stats(cell_t cell, const VoronoiSkeletonGrid& grid);
bool neighbor_is_closer_to_wall(cell_t cell, cell_t neighbor, const VoronoiSkeletonGrid& grid);
bool has_boundary_neighbor(cell_t cell, extent_search_state_t& state);
bool is_on_map_boundary(cell_t cell, extent_search_state_t& state);
bool is_on_another_boundary(cell_t cell, int boundaryId, extent_search_state_t& state);


AreaExtent::AreaExtent(const std::vector<Gateway>& gateways, cell_t startCell, const VoronoiSkeletonGrid& grid)
: havePolygon_(false)
, perimeter_(0.0f)
, area_(0.0f)
, frontierRatio_(0.0f)
, cellsPerMeter_(grid.cellsPerMeter())
{
    CellVector start{startCell};
    growExtent(gateways, start, grid);
}


AreaExtent::AreaExtent(const std::vector<const AreaExtent*>& extents)
: havePolygon_(false)
, perimeter_(0.0f)
, area_(0.0f)
, frontierRatio_(0.0f)
{
    // Calculate the merged properties -- area, perimeter, frontier ratio
    std::size_t numCells = 0;
    float frontierPerimeter = 0.0f;
    for(auto& e : extents)
    {
        assert(e);
        numCells          += e->cells_.size();
        frontierPerimeter += e->perimeter_ * e->frontierRatio_;
        perimeter_        += e->perimeter_;
        area_             += e->area_;
        cellsPerMeter_     = e->cellsPerMeter_;
    }

    frontierRatio_ = frontierPerimeter / perimeter_;

    // Copy the cells from each of the extents
    cells_.reserve(numCells);
    for(auto& e : extents)
    {
        cells_.insert(cells_.end(), e->cells_.begin(), e->cells_.end());
    }

    std::sort(cells_.begin(), cells_.end());
    utils::erase_unique(cells_);

    // Once cells have been found, the new center and boundaries can be found
    calculateCenter();
    calculateBoundaries();
}


AreaExtent::AreaExtent(const std::vector<Gateway>& gateways,
                       const CellVector&           skeletonCells,
                       const VoronoiSkeletonGrid&  grid)
: havePolygon_(false)
, perimeter_(0.0f)
, area_(0.0f)
, frontierRatio_(0.0f)
, cellsPerMeter_(grid.cellsPerMeter())
{
    growExtent(gateways, skeletonCells, grid);

    // Sanity check for easier debugging if somehow one of the flood fills leaks out of the area.
    if(area_ > 100.0f)
    {
        int numToShow = std::min(skeletonCells.size(), 5ul);
        std::cout << "INFO:AreaExtent: Found a big area:" << area_ << "m^2. Start cells:\n";
        std::copy(skeletonCells.begin(), skeletonCells.begin() + numToShow, std::ostream_iterator<cell_t>(std::cout, ","));
        std::cout << '\n';
    }
}


double AreaExtent::hullPerimeter(const VoronoiSkeletonGrid& grid) const
{
    return discretized_polygon_perimeter(polygonBoundary(math::ReferenceFrame::GLOBAL), grid);
}


extent_compactness_t AreaExtent::compactness(void) const
{
    // All measurements are in cells here

    /*
     * Normalized discretized compactness:
     *
     *   ndc = (c_d - c_dmin) / (c_dmax - c_dmin),
     *
     * where c_d = contact perimeter in cells , c_dmin = n - 1, c_dmax = 2(n - sqrt(n)), n = num cells
     * contact perimeter = (4n - perimeter in cells) / 2
     */

    extent_compactness_t compactness;
    int numCells = cells_.size();
    int perimInCells = perimeter_ * cellsPerMeter_;

    compactness.circularity = (4.0 * M_PI * numCells) / std::pow(perimInCells, 2);
    compactness.ndc = (numCells - (perimInCells / 4)) / (numCells - std::sqrt(numCells));

    /**
    * Actual vs. min/max comparisons require:
    *
    * P_max = 2 * (area + 1)  for eight-way connected
    * P_min = three cases with: int rta = sqrt(numCells)
    *
    *   if rta * rta == numCells:
    *       4 * rta
    *   if rta * rta + rta < a:
    *       4(rta + 1)
    *   if rta * rta + rta > a:
    *       4rta + 2
    *
    * A_max = two cases based on rta:
    *   if rta * rta == numCells:
    *       (perim / 4) ^ 2
    *   else
    *       ((perim^2 / 4) - 1) / 4
    *
    * A_min = (perim - 2) / 2
    */

    double aMin = (perimInCells - 2) / 2.0;     // cells in a single line, each cell would contribute 2 cells to
                                                // perimeter except ends, which would add three, hence subtracing 2
    double pMax = 2.0 * (numCells - 1);         // same logic, as with a-min. lined out cells each are two minus a
                                                // couple to take care of the ends
    int rta = std::sqrt(numCells);

    double pMin = 4.0 * rta;                    // minimum perim would be a square, so sqrt(area) would be side length
    double aMax = std::pow(perimInCells / 4, 2.0);  // max area also a square, so it would have size length perim/4

    // Make minor corrections if not even number of cells under consideration
    if(rta * rta != numCells)
    {
        if((rta * rta) + rta > numCells)
        {
            pMin = (4.0 * rta) + 2;
        }
        else // if((rta * rta) + rta  < numCells
        {
            pMin = 4.0 * (rta + 1);
        }

        aMax = 0.25 * ((perimInCells * perimInCells / 4.0) - 1);
    }

    compactness.aMin = aMin / numCells;
    compactness.aMax = numCells / aMax;
    compactness.pMin = pMin / perimInCells;
    compactness.pMax = perimInCells / pMax;

    return compactness;
}


math::Polygon<double> AreaExtent::polygonBoundary(math::ReferenceFrame frame) const
{
    if(!havePolygon_)
    {
        polygon_     = math::convex_hull<double>(cells_.cbegin(), cells_.cend());
        havePolygon_ = true;
    }

    if(frame == math::ReferenceFrame::GLOBAL)
    {
        return polygon_;
    }
    else
    {
        auto poly = polygon_;
        poly.rotate(-center_.theta);
        poly.translate(-center_.x, -center_.y);
        return poly;
    }
}


math::Rectangle<double> AreaExtent::rectangleBoundary(math::ReferenceFrame frame) const
{
    if(frame == math::ReferenceFrame::GLOBAL)
    {
        return rectangle_;
    }
    else
    {
        auto rect = rectangle_;
        rect.rotate(-center_.theta);
        rect.translate(-center_.x, -center_.y);
        return rect;
    }
}


void AreaExtent::changeReferenceFrame(const pose_t& transformToFrame)
{
    center_ = center_.transformToNewFrame(transformToFrame);
}


void AreaExtent::setOrientation(double orientation)
{
    center_.theta = orientation;
}


bool AreaExtent::contains(const Point<double>& point, math::ReferenceFrame frame) const
{
    auto pointInAreaFrame = point;

    if(frame == math::ReferenceFrame::LOCAL)
    {
        pointInAreaFrame = transform(point, center_.x, center_.y, center_.theta);
    }

    // If it isn't within the rectangle, then it can't be in the area for sure.
    if(!rectangle_.contains(pointInAreaFrame))
    {
        return false;
    }

    if(!havePolygon_)
    {
        polygon_     = math::convex_hull<double>(cells_.cbegin(), cells_.cend());
        havePolygon_ = true;
    }

    return polygon_.contains(pointInAreaFrame);
}


bool AreaExtent::cellContains(const Point<double>& position) const
{
    const double kError = 1e-6;

    assert(cellsPerMeter_ > 0.0);
    double metersPerCell = (1.0 / cellsPerMeter_) + kError;

    for(auto cell : cells_)
    {
        if(((position.x - cell.x) >= -kError)
            && ((position.x - cell.x) <= metersPerCell)
            && ((position.y - cell.y) >= -kError)
            && ((position.y - cell.y) <= metersPerCell))
        {
            return true;
        }
    }

    return false;
}


void AreaExtent::growExtent(const std::vector<Gateway>& gateways,
                            const CellVector&           skeletonCells,
                            const VoronoiSkeletonGrid&  grid)
{
    std::tie(perimeter_, frontierRatio_) = flood_fill_extent(gateways, skeletonCells, grid, cells_);
    area_ = cells_.size() * grid.metersPerCell() * grid.metersPerCell();

    calculateCenter();
    calculateBoundaries();
}


void AreaExtent::calculateCenter(void)
{
    auto centerPos = std::accumulate(cells_.begin(), cells_.end(), Point<double>(0.0,0.0));

    if(!cells_.empty())
    {
        center_.x     = centerPos.x / cells_.size();
        center_.y     = centerPos.y / cells_.size();
        center_.theta = 0.0f;

        // Round to the nearest cell-unit
        center_.x = static_cast<int>(center_.x * cellsPerMeter_) / cellsPerMeter_;
        center_.y = static_cast<int>(center_.y * cellsPerMeter_) / cellsPerMeter_;
    }
}


void AreaExtent::calculateBoundaries(void)
{
//     polygon_   = math::convex_hull(cells_);
    rectangle_ = math::axis_aligned_bounding_rectangle<double>(cells_.cbegin(), cells_.cend());
}


float discretized_polygon_perimeter(const math::Polygon<double>& polygon, const VoronoiSkeletonGrid& grid)
{
    CellVector cells;

    for(auto vertIt = polygon.begin(), endIt = polygon.end() - 1; vertIt != endIt; ++vertIt)
    {
        auto nextIt = vertIt + 1;
        auto vertCell = utils::global_point_to_grid_cell_round(*vertIt, grid);
        auto nextCell = utils::global_point_to_grid_cell_round(*nextIt, grid);

        utils::find_cells_along_line(Line<double>(vertCell, nextCell), grid, std::back_inserter(cells));
    }

    // Don't double-count the ends
    utils::erase_unique(cells);
    return cell_vector_perimeter(cells.begin(), cells.end(), grid.metersPerCell());
}


std::tuple<float, float> flood_fill_extent(const std::vector<Gateway>&      gateways,
                                           CellVector                       startCells,
                                           const VoronoiSkeletonGrid&       grid,
                                           std::vector<Point<double>>& cells)
{
    extent_search_state_t state;
    state.grid = &grid;

    float perimeter = 0.0f;

    // Add the cells along the gateway boundary to the queue
    for(auto& g : gateways)
    {
        add_gateway_boundary_cells_to_queue(g, state);
        perimeter += gateway_cell_perimeter(g, grid.metersPerCell());
    }

    // Add the cells adjacent to each boundary to the queue
    for(auto& g : gateways)
    {
        add_boundary_adjacent_cells_to_queue(g, extent_side_of_gateway(g, startCells, grid), state);
    }

    // After adding all gateway boundary cells, go through the queue and remove any cells that ended up as part of
    // the boundary. As the boundaries are enqueued one at a time, close gateways could have adjacent cells that
    // are actually part of another gateway's boundary cells
    utils::erase_remove_if(state.queue, [&state](cell_t cell) {
        return state.boundaryCells.find(cell) != state.boundaryCells.end();
    });

    for(auto& c : startCells)
    {
        // Only enqueue cells that aren't marked state.visited. The visited cells here are on the gateway boundary.
        // Need to not enqueue those, otherwise we'll be jumping across the gateway to our doom
        if(state.visited.find(c) == state.visited.end())
        {
            state.queue.push_back(c);
        }
    }

    state.visited.insert(startCells.begin(), startCells.end());

    float frontierPerimeter = 0.0f;
    float cellPerimeter = 0.0f;
    float cellFrontier = 0.0f;

    while(!state.queue.empty())
    {
        cell_t next = state.queue.front();
        state.queue.pop_front();

        // If on the map boundary, don't expand it
        if(is_on_map_boundary(next, state))
        {
            std::tie(cellPerimeter, cellFrontier) = perimeter_stats(next, *state.grid);
        }
        // Otherwise treat skeleton
        else if(state.grid->getClassification(next) & SKELETON_CELL_SKELETON)
        {
            std::tie(cellPerimeter, cellFrontier) = expand_skeleton_cell(next, state);
        }
        // and free cells differently
        else
        {
            std::tie(cellPerimeter, cellFrontier) = expand_cell(next, state);
        }

        perimeter += cellPerimeter + cellFrontier;
        frontierPerimeter += cellFrontier;
    }

    cells.reserve(state.visited.size());
    std::transform(state.visited.begin(), state.visited.end(), std::back_inserter(cells), [&grid](cell_t cell) {
        return utils::grid_point_to_global_point(cell, grid);
    });

    if(perimeter > 0.0f)
    {
        return std::make_tuple(perimeter, frontierPerimeter / perimeter);
    }
    else
    {
        std::cerr << "WARNING: AreaExtent: Extent with no perimeter detected. Gateways:\n";
        std::copy(gateways.begin(), gateways.end(), std::ostream_iterator<Gateway>(std::cerr, ",  "));
        std::cerr << '\n';
        return std::make_tuple(0.0f, 0.0f);
    }
}


void add_gateway_boundary_cells_to_queue(const Gateway& gateway, extent_search_state_t& state)
{
    CellVector gatewayCells(gateway.beginCells(), gateway.endCells());

    // Remove the boundary cells that aren't in free space
    utils::erase_remove_if(gatewayCells, [state](cell_t cell) {
        return !(state.grid->getClassification(cell) & SKELETON_CELL_FREE);
    });

    state.visited.insert(gatewayCells.begin(), gatewayCells.end());
    state.boundaryCells.insert(gatewayCells.begin(), gatewayCells.end());
    state.gatewayToBoundaryCells[gateway.id()] = gatewayCells;
}


void add_boundary_adjacent_cells_to_queue(const Gateway& gateway, int extentSide, extent_search_state_t& state)
{
    if(extentSide == 0)
    {
        std::cout << "ERROR: Can't determine which gateway cells to add. Gateway:" << gateway.boundary() << '\n';
        return;
    }

    // For each cell along the gateway, add the cell that is on the extent side. This will ensure that the whole area
    // gets scanned. Because the only way to always move down requires starting from a skeleton cell, in cases where
    // a gateway chops the skeleton (but not reduced skeleton) somewhere other than at a junction, following the
    // gradient is no longer possible. Thus, cells along the gateway need to be added for the start

    // The direction to add is the normal to the boundary pointing into the area. If sin(angle) > cos(angle), then add
    // update y because there's one or more x per y, and vice versa. Sign is determined by the sign of the cos/sin
    double extentNormal = angle_sum(direction(gateway.boundary()), std::copysign(M_PI_2, extentSide));
    int xIncr = 0;
    int yIncr = 0;
    if(std::abs(std::sin(extentNormal)) > 0.5)
    {
        yIncr = std::copysign(1.0, std::sin(extentNormal));
    }
    else
    {
        xIncr = std::copysign(1.0, std::cos(extentNormal));
    }

    for(auto cell : state.gatewayToBoundaryCells[gateway.id()])
    {
        // Ignore cells that belong to multiple boundaries, as they will have different extent sides and likely
        // cause the extent to leak
        if(is_on_another_boundary(cell, gateway.id(), state))
        {
            continue;
        }

        // Otherwise, see if the neighbor hasn't been visited and is in free space, so that it is safe to be added
        // to the search queue
        cell_t neighbor(cell.x + xIncr, cell.y + yIncr);

        if((state.grid->getClassification(neighbor.x, neighbor.y) & SKELETON_CELL_FREE)
            && (state.visited.find(neighbor) == state.visited.end()))
        {
            state.visited.insert(neighbor);
            state.queue.push_back(neighbor);

            neighbor.x += xIncr;
            neighbor.y += yIncr;

            if((state.grid->getClassification(neighbor.x, neighbor.y) & SKELETON_CELL_FREE)
                && (state.visited.find(neighbor) == state.visited.end()))
            {
                state.visited.insert(neighbor);
                state.queue.push_back(neighbor);
            }
        }
    }
}


int extent_side_of_gateway(const Gateway& gateway, CellVector& startCells, const VoronoiSkeletonGrid& grid)
{
    const int kDecisionThreshold = 3;

    // The extent side of the gateway is whichever side the majority of the start cells nearest the gateway vote
    // for. Take a winner once a threshold number of cells agree.

    std::sort(startCells.begin(), startCells.end(), [&gateway](cell_t lhs, cell_t rhs) {
        return distance_to_line_segment(lhs, gateway.cellBoundary()) <
            distance_to_line_segment(rhs, gateway.cellBoundary());
    });

    int extentSide = 0;

    assert(gateway.sizeCells() > 0);

    for(auto cell : startCells)
    {
        extentSide += gateway.isCellToLeft(cell);

        if(std::abs(extentSide) > kDecisionThreshold)
        {
            break;
        }
    }

//     bool debug = (gateway.skeletonCell() == cell_t(562, 1276))
//         || (gateway.skeletonCell() == cell_t(591, 1315));
//
//     if(debug)
//     {
//         std::cout << "Cells for gateway: " << gateway.skeletonCell() << " :: " << gateway.cellBoundary() << ":\n";
//         for(int n = 0; n < 20; ++n)
//         {
//             std::cout << startCells[n] << " left? " << gateway.isCellToLeft(startCells[n]) << '\n';
//         }
//     }

    return extentSide;
}


// Return the perimeter of the cell
std::tuple<float, float> expand_cell(cell_t cell, extent_search_state_t& state)
{
    // Allow pure free space to expand 8-way when not on the boundary because sometimes the gradient is along a
    // diagonal, especially in corners. These cells aren't always marked as skeleton once the distance is low
    // enough, so allow growing there if needed.
    cell_connectivity_t connectivity = has_boundary_neighbor(cell, state) ? FOUR_WAY : EIGHT_WAY;

    // Only expand to free cells as the skeleton within an area must intersect with the boundary and will already
    // be included in the starting cells at the boundary.
    NeighborArray neighbors;
    int numNeighbors = neighbor_cells_equal_classification(cell,
                                                           SKELETON_CELL_FREE,
                                                           *state.grid,
                                                           connectivity,
                                                           neighbors);
    for(int n = 0; n < numNeighbors; ++n)
    {
        if((state.visited.find(neighbors[n]) == state.visited.end())
            && neighbor_is_closer_to_wall(cell, neighbors[n], *state.grid))
        {
            state.visited.insert(neighbors[n]);
            state.queue.push_back(neighbors[n]);
        }
    }

    return perimeter_stats(cell, *state.grid);
}


std::tuple<float, float> expand_skeleton_cell(cell_t skeleton, extent_search_state_t& state)
{
    NeighborArray neighbors;
    int numNeighbors = neighbor_cells_with_classification(skeleton,
                                                          SKELETON_CELL_SKELETON | SKELETON_CELL_FREE,
                                                          *state.grid,
                                                          EIGHT_WAY,
                                                          neighbors);
    bool isBoundarySkeleton = has_boundary_neighbor(skeleton, state);

    for(int n = 0; n < numNeighbors; ++n)
    {
        if((state.visited.find(neighbors[n]) == state.visited.end())
            && (!has_boundary_neighbor(neighbors[n], state) || !isBoundarySkeleton))
        {
            state.visited.insert(neighbors[n]);
            state.queue.push_back(neighbors[n]);
        }
    }

    return perimeter_stats(skeleton, *state.grid);
}


std::tuple<float, float> perimeter_stats(cell_t cell, const VoronoiSkeletonGrid& grid)
{
    // A cell is on the parameter if it is neighbors with an unknown or occupied cell. If the cell also has a frontier,
    // then it's a frontier as well. Count just the number of cells to avoid big changes in perimeter that depend
    // on map rotation.
    int numOccupied = num_neighbor_cells_with_classification(cell, SKELETON_CELL_OCCUPIED | SKELETON_CELL_UNKNOWN, grid, FOUR_WAY);
    int numFrontier = num_neighbor_cells_with_classification(cell, SKELETON_CELL_FRONTIER, grid, FOUR_WAY);
    return std::make_tuple((numOccupied > 0) ? grid.metersPerCell() : 0.0f,
                           (numFrontier > 0) ? grid.metersPerCell() : 0.0f);
}


bool neighbor_is_closer_to_wall(cell_t cell, cell_t neighbor, const VoronoiSkeletonGrid& grid)
{
    // A neighbor is closer to the wall if the parent is a skeleton cell.
    // Or its distance from the wall is less than the parent's distance from the wall
    bool isCloserToWall = grid.getObstacleDistance(neighbor.x, neighbor.y) <
        grid.getObstacleDistance(cell.x, cell.y);
    bool isCellSkeleton = grid.getClassification(cell.x, cell.y) & SKELETON_CELL_SKELETON;

    return isCloserToWall || isCellSkeleton;
}


bool has_boundary_neighbor(cell_t cell, extent_search_state_t& state)
{
    // Confirm that none of the four-way connected neighbors is along the boundary. This keeps cells from skipping
    // across the boundary near the skeleton.
    return (state.boundaryCells.find(cell_t(cell.x + 1, cell.y)) != state.boundaryCells.end())
        || (state.boundaryCells.find(cell_t(cell.x - 1, cell.y)) != state.boundaryCells.end())
        || (state.boundaryCells.find(cell_t(cell.x, cell.y + 1)) != state.boundaryCells.end())
        || (state.boundaryCells.find(cell_t(cell.x, cell.y - 1)) != state.boundaryCells.end());
}


bool is_on_map_boundary(cell_t cell, extent_search_state_t& state)
{
    // Either hitting the edge of the map or the skeleton is hitting frontier, in which case that also is the edge
    // of the visible map
    return (cell.x == 0)
        || (cell.y == 0)
        || (cell.x == static_cast<int>(state.grid->getWidthInCells() - 1))
        || (cell.y == static_cast<int>(state.grid->getHeightInCells() - 1))
        || (state.grid->getClassification(cell) & SKELETON_CELL_FRONTIER);
}


bool is_on_another_boundary(cell_t cell, int boundaryId, extent_search_state_t& state)
{
    // Check all sets of boundary cells that aren't the current boundary to see if this cell is on another boundary
    for(auto& cells : state.gatewayToBoundaryCells)
    {
        if((cells.first != boundaryId) && utils::contains(cells.second, cell))
        {
            return true;
        }
    }

    // Didn't find the cell on any other boundary, but it might be on the edge of the map
    return is_on_map_boundary(cell, state);
}

} // namespace hssh
} // namespace vulcan
