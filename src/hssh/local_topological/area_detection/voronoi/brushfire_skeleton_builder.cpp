/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     brushfire_skeleton_builder.cpp
* \author   Collin Johnson
*
* Definition of BrushfireSkeletonBuilder.
*/

#include "hssh/local_topological/area_detection/voronoi/brushfire_skeleton_builder.h"
#include "hssh/local_topological/area_detection/voronoi/island_detector.h"
#include "hssh/local_topological/area_detection/voronoi/voronoi_utils.h"
#include "hssh/local_metric/lpm.h"
#include "core/pose.h"
#include "utils/algorithm_ext.h"
#include "utils/timestamp.h"
#include <iostream>
#include <cassert>

#define DEBUG_TIME

namespace vulcan
{
namespace hssh
{

const uint8_t LPM_FREE     = kFreeOccGridCell | kUnknownOccGridCell | kDynamicOccGridCell;
const uint8_t LPM_OCCUPIED = kOccupiedOccGridCell | kHazardOccGridCell | kLimitedVisibilityOccGridCell; // | kDynamicOccGridCell;

const VoronoiDist MAXIMUM_SEARCH_DISTANCE = std::numeric_limits<VoronoiDist>::max();

// Helpers for running the search
bool is_cell_obstacle_boundary(cell_t cell, const LocalPerceptualMap& map);
bool is_cell_frontier         (cell_t cell, const LocalPerceptualMap& map);
bool is_cell_on_map_edge      (cell_t cell, const LocalPerceptualMap& map);
bool is_cell_unknown          (cell_t cell, const LocalPerceptualMap& map);
bool is_cell_free_space       (cell_t cell, const LocalPerceptualMap& map);
bool is_cell_occupied         (cell_t cell, const LocalPerceptualMap& map);
bool is_cell_skeleton         (cell_t cell, const VoronoiSkeletonGrid& grid);
bool are_cells_separated      (cell_t first, cell_t second);
bool isnt_clump               (cell_t skeleton, const VoronoiSkeletonGrid& grid);

inline unsigned int set_index(cell_t cell, int width)
{
    return cell.x + cell.y*width;
}


inline VoronoiDist dist_to_origin(cell_t cell, cell_t origin)
{
    return (cell.x - origin.x)*(cell.x - origin.x) + (cell.y - origin.y)*(cell.y - origin.y);
}


BrushfireSkeletonBuilder::BrushfireSkeletonBuilder(const skeleton_builder_params_t& params)
: SkeletonBuilder(params)
, cellAnchors(0)
, numUpdates(0)
, sumInitializeTimeUs(0)
, sumEnqueueTimeUs(0)
, sumBrushfireTimeUs(0)
, sumPruneTimeUs(0)
{
}


BrushfireSkeletonBuilder::~BrushfireSkeletonBuilder(void)
{
}


void BrushfireSkeletonBuilder::extractSkeleton(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid, IslandDetector& islands)
{
    assert((map.getWidthInCells() <= grid.getWidthInCells()) && (map.getHeightInCells() <= grid.getHeightInCells()));

    int64_t startTime = utils::system_time_us();
    initializeSearch(grid);
    sumInitializeTimeUs += utils::system_time_us() - startTime;

    startTime = utils::system_time_us();
    enqueueObstacleBoundaries(map, grid, islands);
    sumEnqueueTimeUs += utils::system_time_us() - startTime;

    startTime = utils::system_time_us();
    runBrushfire(grid);
    sumBrushfireTimeUs += utils::system_time_us() - startTime;

    startTime = utils::system_time_us();
    pruneSkeleton(grid);
    sumPruneTimeUs += utils::system_time_us() - startTime;

    ++numUpdates;

#ifdef DEBUG_TIME
    std::cout<<"DEBUG:Brushfire::timing:Initialize:"<<(sumInitializeTimeUs/numUpdates/1000)
             <<"ms Enqueue:"<<(sumEnqueueTimeUs/numUpdates/1000)
             <<"ms Brushfire:"<<(sumBrushfireTimeUs/numUpdates/1000)
             <<"ms Prune:"<<(sumPruneTimeUs/numUpdates/1000)<<"ms\n";
#endif
}

// Steps for the brushfire algorithm
void BrushfireSkeletonBuilder::initializeSearch(VoronoiSkeletonGrid& grid)
{
    searchQueue.clear();
    unprunedSkeletonCells.clear();

    std::size_t numGridCells = grid.getWidthInCells() * grid.getHeightInCells();

    if(cellAnchors.size() < numGridCells)
    {
        cellAnchors.resize(numGridCells);
    }

    grid.reset(MAXIMUM_SEARCH_DISTANCE, static_cast<uint8_t>(SKELETON_CELL_UNKNOWN));

    // Clear out all previously stored anchor information, but preserve the memory to help minimize allocation over time
    for(auto& vec : cellAnchors)
    {
        vec.clear();
    }
}


void BrushfireSkeletonBuilder::enqueueObstacleBoundaries(const LocalPerceptualMap& map, VoronoiSkeletonGrid& grid, IslandDetector& islands)
{
    // Two passes through the grid are required because the first pass initializes the occupied cells that are
    // then used by the IslandDetector for detecting which occupied regions are actually islands

    int width  = map.getWidthInCells();
    int height = map.getHeightInCells();

    cell_t  cell;

    int numFreeCells = 0;

    for(cell.y = 0; cell.y < height; ++cell.y)
    {
        for(cell.x = 0; cell.x < width; ++cell.x)
        {
            // Any type of cell can be a frontier, so always make this check
            uint8_t classification = is_cell_frontier(cell, map) ? SKELETON_CELL_FRONTIER : 0;

            if(is_cell_free_space(cell, map))
            {
                grid.setClassification(cell.x, cell.y, classification | SKELETON_CELL_FREE);
                ++numFreeCells;
            }
            else if(is_cell_occupied(cell, map))
            {
                grid.setClassification(cell.x, cell.y, classification | SKELETON_CELL_OCCUPIED);
            }
            else if(classification & SKELETON_CELL_FRONTIER)
            {
                // Frontiers on the edge of map should be marked free as they lead to somewhere new
                if(is_cell_on_map_edge(cell, map))
                {
                    grid.setClassification(cell.x, cell.y, classification | SKELETON_CELL_FREE);
                }
                else
                {
                    grid.setClassification(cell.x, cell.y, classification | SKELETON_CELL_UNKNOWN);
                }
            }
        }
    }

    for(cell.y = 0; cell.y < height; ++cell.y)
    {
        for(cell.x = 0; cell.x < width; ++cell.x)
        {
            uint8_t classification = grid.getClassification(cell.x, cell.y);

            // Eliminate all islands from the grid. Then nothing in the future needs to deal with them
            // because they won't exist
            if((classification & ~SKELETON_CELL_FREE) && islands.cellIsOnIsland(cell, grid))
            {
                grid.setClassification(cell.x, cell.y, SKELETON_CELL_FREE);
                ++numFreeCells;
            }
            else if(classification & (SKELETON_CELL_OCCUPIED | SKELETON_CELL_FRONTIER))
            {
                if(classification & SKELETON_CELL_FRONTIER)
                {
                    frontierPoints->push_back(cell);
                }

                // Frontiers on the edge of the map should not be expanded
                if(((classification & SKELETON_CELL_FRONTIER) && !is_cell_on_map_edge(cell, map)) ||
                    ((classification & SKELETON_CELL_OCCUPIED) && is_cell_obstacle_boundary(cell, map)))
                {
                    addObstacleCell(cell, grid);
                }
                else if(classification & SKELETON_CELL_OCCUPIED)
                {
                    // If the obstacle isn't on a boundary, then turn it into an unknown cell because it doesn't matter
                    grid.setClassification(cell.x, cell.y, SKELETON_CELL_UNKNOWN | (classification & SKELETON_CELL_FRONTIER));
                }
            }
        }
    }

    std::cout << "INFO: BrushfireSkeletonBuilder: The map contains " << numFreeCells << " free cells.\n";
}


void BrushfireSkeletonBuilder::addObstacleCell(cell_t cell, VoronoiSkeletonGrid& grid)
{
    enqueueFreeCell(brushfire_node_t(cell, 0, cell), grid);
}


void BrushfireSkeletonBuilder::runBrushfire(VoronoiSkeletonGrid& grid)
{
    brushfire_node_t nodeToExpand;

    // The grid-based coastal distance depends on the map, so it needs to be set for each update
    setCoastalDistance(grid.metersPerCell());

    while(searchQueue.size())
    {
        nodeToExpand = searchQueue.extract();

        if(~grid.getClassificationNoCheck(nodeToExpand.parent.x, nodeToExpand.parent.y) & SKELETON_CELL_SKELETON)
        {
            expandCell(nodeToExpand, grid);
        }
    }
}


void BrushfireSkeletonBuilder::expandCell(const brushfire_node_t& node, VoronoiSkeletonGrid& grid)
{
    // If this node is further from the obstacle than the closest known distance, it is a collision, but further
    // expansion should not happen because the closest obstacle is already being expanded
    VoronoiDist dist = grid.getObstacleDistance(node.cell.x, node.cell.y);

    // This is the first visit to the cell
    if(dist == MAXIMUM_SEARCH_DISTANCE)
    {
        grid.setObstacleDistance(node.cell.x, node.cell.y, node.distance);
        cellAnchors[set_index(node.cell, grid.getWidthInCells())].push_back(node.origin);
    }
    // The search has already been here, so handle the collision and carry on
    else
    {
        handleCollisionCell(node, grid);
        return;
    }

    NeighborArray neighbors;
    int numNeighbors = neighbor_cells_with_classification(node.cell, SKELETON_CELL_FREE, grid, EIGHT_WAY, neighbors);

    for(int n = 0; n < numNeighbors; ++n)
    {
        // Don't fold back on self
        if(neighbors[n] == node.parent)
        {
            continue;
        }

        // Add all neighbors that are further from the origin than the node being expanded
        VoronoiDist neighborDist = dist_to_origin(neighbors[n], node.origin);

        auto neighborNode = brushfire_node_t(neighbors[n], neighborDist, node.cell, node.origin);

        // If the neighbor is closer to a cell than the current closest object, it should replace the other obstacle
        // Thus, it is enqueued and will be popped off the queue when appropriate
        if(neighborDist < grid.getObstacleDistance(neighbors[n].x, neighbors[n].y))
        {
            enqueueFreeCell(neighborNode, grid);
        }
    }
}


void BrushfireSkeletonBuilder::enqueueFreeCell(const brushfire_node_t& node, VoronoiSkeletonGrid& grid)
{
    searchQueue.insert(node);
}


void BrushfireSkeletonBuilder::handleCollisionCell(const brushfire_node_t& node, VoronoiSkeletonGrid& grid)
{
    // A collision occurs when a previously visited cell is visited by another growing brushfire
    // When the collision occurs, two cells are involved, the cell in the queue, A, and that cell's parent cell, B,
    // At least A or B is a skeleton cell (except if their origins are adjacent -- for noise reduction purposes)

    const auto& anchors = cellAnchors[set_index(node.cell, grid.getWidthInCells())];
    if(utils::contains_if(anchors, [&node](cell_t cell) { return !are_cells_separated(cell, node.origin);})) {
        return;
    }

    bool isSkeleton = false;

    for(auto& origin : anchors)
    {
        VoronoiDist storedDist = grid.getObstacleDistance(node.cell.x, node.cell.y);

        if((node.distance > 1) || (storedDist > 1))
        {
            addSkeletonCell(node.cell, node.origin, grid);
            addSkeletonCell(node.cell, origin, grid);
            isSkeleton = true;
        }
    }

    if(isSkeleton)
    {
        cellAnchors[set_index(node.cell, grid.getWidthInCells())].push_back(node.origin);
    }
}


void BrushfireSkeletonBuilder::addSkeletonCell(cell_t cell, cell_t origin, VoronoiSkeletonGrid& grid)
{
    // If the skeleton cell hasn't already been marked as a skeleton by a previous collision,
    // then add it to the unpruned cells and mark it in the grid
    if(!is_cell_skeleton(cell, grid))
    {
        unprunedSkeletonCells.push_back(std::make_pair(cell, dist_to_origin(cell, origin)));
        grid.addClassification(cell.x, cell.y, SKELETON_CELL_SKELETON);
    }
}


void BrushfireSkeletonBuilder::pruneSkeleton(VoronoiSkeletonGrid& grid)
{
    // Sort unpruned skeleton cells in ascending order of distance
    std::sort(unprunedSkeletonCells.begin(), unprunedSkeletonCells.end(),
              [](const CellDistPair& lhs, const CellDistPair& rhs) { return lhs.second < rhs.second; });

    skeletonPoints->reserve(unprunedSkeletonCells.size());

    // For each skeleton cell
    for(auto c : unprunedSkeletonCells)
    {
        // If it is an essential part of the thinned skeleton, then add it to the final skeleton points
        if(isnt_clump(c.first, grid))
        {
            skeletonPoints->push_back(c.first);
            // Save the origin as one of the skeleton's anchors
            (*skeletonAnchors)[c.first] = cellAnchors[set_index(c.first, grid.getWidthInCells())];
        }
        // Otherwise, remove the classification of the cell as being part of the skeleton
        else
        {
            grid.removeClassification(c.first.x, c.first.y, SKELETON_CELL_SKELETON);
        }
    }
}


void BrushfireSkeletonBuilder::setCoastalDistance(float cellScale)
{
    double requestedCoastalDist = std::pow(skeletonParams.coastalDistance / cellScale, 2);

    coastalDistanceInCells = requestedCoastalDist > MAXIMUM_SEARCH_DISTANCE ? MAXIMUM_SEARCH_DISTANCE :
        requestedCoastalDist;

    std::cout << "INFO: BrushfireSkeletonBuilder: Coastal distance: " << skeletonParams.coastalDistance << "m, "
        << coastalDistanceInCells << " cells^2.\n";
}


// Helpers for initializing the brushfire
bool is_cell_obstacle_boundary(cell_t cell, const LocalPerceptualMap& map)
{
    if(is_cell_on_map_edge(cell, map))
    {
        return true;
    }

    for(int y = -1; y <= 1; ++y)
    {
        for(int x = -1; x <= 1; ++x)
        {
            if(map.getCellType(cell.x+x, cell.y+y) & LPM_FREE)
            {
                return true;
            }
        }
    }

    return false;
}


bool is_cell_frontier(cell_t cell, const LocalPerceptualMap& map)
{
    // on the border and free? definitely a frontier
    if(is_cell_on_map_edge(cell, map) && is_cell_free_space(cell, map))
    {
        return true;
    }

    // Mark isolated occupied cells as frontiers as well because their isolation means the robot doesn't exactly
    // know where that wall is going to end up.
    bool isUnknownCell = is_cell_unknown(cell, map);
    int numOccupiedNeighbors = 0;

    for(int y = -1; y <= 1; ++y)
    {
        for(int x = -1; x <= 1; ++x)
        {
            if((x == 0 && y == 0) // if it is the same cell, then don't check it
                || (!map.isCellInGrid(cell_t(cell.x + x, cell.y + y)))) // or if the cell isn't in the map
            {
                continue;
            }

            if(isUnknownCell && is_cell_free_space(cell_t(cell.x+x, cell.y+y), map))
            {
                return true;
            }
            else if(is_cell_occupied(cell_t(cell.x+x, cell.y+y), map))
            {
                ++numOccupiedNeighbors;
            }
        }
    }

    return is_cell_occupied(cell, map) && (numOccupiedNeighbors == 0);
}


bool is_cell_on_map_edge(cell_t cell, const LocalPerceptualMap& map)
{
    int width  = map.getWidthInCells()-1;
    int height = map.getHeightInCells()-1;
    return (cell.x == 0) || (cell.y == 0) || (cell.x == width) || (cell.y == height);
}


// Helpers for doing the main search
bool is_cell_unknown(cell_t cell, const LocalPerceptualMap& map)
{
    return map.getCost(cell) == 127;
}

bool is_cell_free_space(cell_t cell, const LocalPerceptualMap& map)
{
    return (map.getCost(cell) < 127) && !(map.getCellType(cell) & LPM_OCCUPIED);
}

bool is_cell_occupied(cell_t cell, const LocalPerceptualMap& map)
{
    return (map.getCost(cell) > 127) || (map.getCellType(cell) & LPM_OCCUPIED);
}


bool is_cell_skeleton(cell_t cell, const VoronoiSkeletonGrid& grid)
{
    return grid.getClassification(cell.x, cell.y) & SKELETON_CELL_SKELETON;
}


bool are_cells_separated(cell_t first, cell_t second)
{
    return (abs(first.x - second.x) > 2) || (abs(first.y - second.y) > 2);
}


bool isnt_clump(cell_t skeleton, const VoronoiSkeletonGrid& grid)
{
    // Ignore any cells on the edge of the map
    if((skeleton.x == 0) || (skeleton.y == 0) || (skeleton.x + 1 >= static_cast<int>(grid.getWidthInCells())) ||
        (skeleton.y + 1 >= static_cast<int>(grid.getHeightInCells())))
    {
        return true;
    }

    // Create a bitmask to represent the neighbors going around the cell
    // Start at the left and go around counterclockwise to enumerate the neighbors
    std::array<bool, 8> isSkeleton;
    isSkeleton[0] = grid.getClassification(skeleton.x-1, skeleton.y)   & SKELETON_CELL_SKELETON;
    isSkeleton[1] = grid.getClassification(skeleton.x-1, skeleton.y-1) & SKELETON_CELL_SKELETON;
    isSkeleton[2] = grid.getClassification(skeleton.x,   skeleton.y-1) & SKELETON_CELL_SKELETON;
    isSkeleton[3] = grid.getClassification(skeleton.x+1, skeleton.y-1) & SKELETON_CELL_SKELETON;
    isSkeleton[4] = grid.getClassification(skeleton.x+1, skeleton.y)   & SKELETON_CELL_SKELETON;
    isSkeleton[5] = grid.getClassification(skeleton.x+1, skeleton.y+1) & SKELETON_CELL_SKELETON;
    isSkeleton[6] = grid.getClassification(skeleton.x,   skeleton.y+1) & SKELETON_CELL_SKELETON;
    isSkeleton[7] = grid.getClassification(skeleton.x-1, skeleton.y+1) & SKELETON_CELL_SKELETON;

    /*
    * Pattern 1:
    *
    * Check if part of a diagonal line
    *
    *       S F .
    *       F S .
    *       . . .
    *
    * The four rotations of this pattern correspond to the following indices in the skeleton mask:
    *
    *      0-F 1-T 2-F
    *      2-F 3-T 4-F
    *      4-F 5-T 6-F
    *      6-F 7-T 0-F
    */

    if((!isSkeleton[0] && isSkeleton[1] && !isSkeleton[2]) ||
        (!isSkeleton[2] && isSkeleton[3] && !isSkeleton[4]) ||
        (!isSkeleton[4] && isSkeleton[5] && !isSkeleton[6]) ||
        (!isSkeleton[6] && isSkeleton[7] && !isSkeleton[0]))
    {
        return true;
    }

    /*
    * Pattern 2:
    *
    * Check if part of a straight line
    *
    *       . F .
    *       S S S
    *       . F .
    *
    * The two rotations of this pattern correspond to the following indices in the skeleton mask:
    *
    *      0-T 2-F 4-T 6-F
    *      0-F 2-T 4-F 6-T
    */

    if((isSkeleton[0] && !isSkeleton[2] && isSkeleton[4] && !isSkeleton[6]) ||
        (!isSkeleton[0] && isSkeleton[2] && !isSkeleton[4] && isSkeleton[6]))
    {
        return true;
    }

    /*
    * Pattern 3:
    *
    * Check if the center of a 4-way junction (not necessarily needed, but looks nicer!)
    *
    *       . S .
    *       S S S
    *       . S .
    *
    * This pattern corresponds to the following indices in the skeleton mask:
    *
    *      0-T 2-T 4-T 6-T
    */

    if(isSkeleton[0] && isSkeleton[2] && isSkeleton[4] && isSkeleton[6])
    {
        return true;
    }

    return false;
}

} // namespace hssh
} // namespace vulcan
