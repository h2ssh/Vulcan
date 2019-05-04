/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     island_detector.cpp
* \author   Collin Johnson
*
* Definition of IslandDetector.
*/

#include <hssh/local_topological/area_detection/voronoi/island_detector.h>
#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/area_detection/voronoi/voronoi_utils.h>
#include <utils/disjoint_set_forest.h>
#include <core/point.h>
#include <iostream>

// #define DEBUG_CARDINAL_SEARCH
// #define DEBUG_ISLANDS

namespace vulcan
{
namespace hssh
{

const unsigned int LARGE_STRUCTURE_SET = 0;
const unsigned int ISLAND_SET          = 1;

const uint8_t OCCUPIED_CELL_MASK = SKELETON_CELL_OCCUPIED | SKELETON_CELL_UNKNOWN | SKELETON_CELL_FRONTIER;


bool vertical_path_exists_to_map_edge  (cell_t start, int yIncrement, int yEnd, const VoronoiSkeletonGrid& grid);
bool horizontal_path_exists_to_map_edge(cell_t start, int xIncrement, int xEnd, const VoronoiSkeletonGrid& grid);

// Helper to get the appropriate set index for a given cell
unsigned int cell_index_component(cell_t cell, const VoronoiSkeletonGrid& grid)
{
    // The first two indices are reserved for LARGE_STRUCTURE_SET and ISLAND_SET
    return cell.x + cell.y*grid.getWidthInCells() + 2;
}


IslandDetector::IslandDetector(int maxIslandAreaInCells)
: maxIslandArea(maxIslandAreaInCells)
{
}


void IslandDetector::reset(void)
{
    if(islands.get() != 0)
    {
        islands->reset();
    }
}


bool IslandDetector::cellIsOnIsland(cell_t cell, const VoronoiSkeletonGrid& grid) const
{
    createSetForestForGridIfNeeded(grid);

    // Cardinal direction search is fast, only width+height iterations
    // If cardinal direction search fails, then do a more complete search that is width*height complexity at worst
    return (grid.getClassification(cell.x, cell.y) & OCCUPIED_CELL_MASK) &&
    // !checkCardinalDirections(cell, grid) &&
           (findCellComponent(cell, grid) == ISLAND_SET);
}


void IslandDetector::createSetForestForGridIfNeeded(const VoronoiSkeletonGrid& grid) const
{
    if((islands.get() == 0) ||
       (grid.getWidthInCells()*grid.getHeightInCells() + 2u != islands->size()))
    {
        islands.reset(new utils::DisjointSetForest(grid.getWidthInCells()*grid.getHeightInCells() + 2));
    }
}


bool IslandDetector::checkCardinalDirections(cell_t cell, const VoronoiSkeletonGrid& grid) const
{
    // Do a quick check for to see if there is a clear connection to the edge of the map
    return horizontal_path_exists_to_map_edge(cell, 1, grid.getWidthInCells()-1, grid) ||
           horizontal_path_exists_to_map_edge(cell, -1, 0, grid)                ||
           vertical_path_exists_to_map_edge(cell, 1, grid.getHeightInCells()-1, grid)  ||
           vertical_path_exists_to_map_edge(cell, -1, 0, grid);
}


unsigned int IslandDetector::findCellComponent(cell_t cell, const VoronoiSkeletonGrid& grid) const
{
    searchQueue.clear();

    searchQueue.push_back(cell);

    int componentSize = 0;

    // If the component has reached the necessary size for a large structure, then exit the search immediately
    while(!searchQueue.empty() && (componentSize < maxIslandArea) &&
          (islands->findSet(cell_index_component(cell, grid)) != islands->findSet(LARGE_STRUCTURE_SET)) &&
          (islands->findSet(cell_index_component(cell, grid)) != islands->findSet(ISLAND_SET)))
    {
        cell_t currentCell = searchQueue.front();

        expandCell(currentCell, grid);
        processCell(currentCell, grid);

        assert(grid.getClassification(currentCell.x, currentCell.y) != SKELETON_CELL_FREE);

        ++componentSize;

        searchQueue.pop_front();
    }

    unsigned int cellSet           = islands->findSet(cell_index_component(cell, grid));
    unsigned int largeStructureSet = islands->findSet(LARGE_STRUCTURE_SET);

    if((cellSet != largeStructureSet) && (componentSize < maxIslandArea))
    {
        islands->setUnion(cellSet, ISLAND_SET);
        cellSet = ISLAND_SET;

#ifdef DEBUG_ISLANDS
        std::cout<<"INFO:IslandDetector:Found island with "<<componentSize<<" cells containing "<<cell<<'\n';
#endif
    }
    else if(cellSet != largeStructureSet)
    {
        // The component was larger than the maximum island size, so it is a large structure
        islands->setUnion(cellSet, LARGE_STRUCTURE_SET);
        cellSet = LARGE_STRUCTURE_SET;

#ifdef DEBUG_ISLANDS
        std::cout<<"INFO:IslandDetector:Found large structure with "<<componentSize<<" cells containing "<<cell<<'\n';
#endif
    }

    assert(islands->findSet(LARGE_STRUCTURE_SET) != islands->findSet(ISLAND_SET));

    return cellSet;
}


void IslandDetector::expandCell(cell_t cell, const VoronoiSkeletonGrid& grid) const
{
    unsigned int  cellIndex = cell_index_component(cell, grid);
    NeighborArray neighbors;
    
    std::size_t numNeighbors = neighbor_cells_with_classification(cell, OCCUPIED_CELL_MASK, grid, FOUR_THEN_EIGHT_WAY, neighbors);
    
    for(std::size_t n = 0; n < numNeighbors; ++n)
    {
        mergeAndEnqueueNeighbor(neighbors[n], cellIndex, grid);
    }
}


void IslandDetector::processCell(cell_t cell, const VoronoiSkeletonGrid& grid) const
{
    // Check to see if the cell is at the boundary of the grid. If so, merge it with the large structure set
//     if(cell.x == 0 || cell.x == grid.getWidthInCells()-1 || cell.y == 0 || cell.y == grid.getHeightInCells()-1)
//     {
//         islands->setUnion(cell_index_component(cell, grid), LARGE_STRUCTURE_SET);
//     }
}


void IslandDetector::mergeAndEnqueueNeighbor(cell_t neighbor, unsigned int parentIndex, const VoronoiSkeletonGrid& grid) const
{
    unsigned int neighborIndex = cell_index_component(neighbor, grid);

    if(islands->findSet(parentIndex) != islands->findSet(neighborIndex))
    {
        searchQueue.push_back(neighbor);

        islands->setUnion(parentIndex, neighborIndex);
    }
}


// Search along the cell path until a free cell is encountered. If the end of the search succeeds, then a path exists
// to the edge of the map along the desired direction
bool vertical_path_exists_to_map_edge(cell_t start, int yIncrement, int yEnd, const VoronoiSkeletonGrid& grid)
{
    for(int y = start.y; y != yEnd; y += yIncrement)
    {
        if(!(grid.getClassification(start.x, y) & OCCUPIED_CELL_MASK))
        {
            return false;
        }
    }

#ifdef DEBUG_CARDINAL_SEARCH
    std::cout<<"INFO:IslandDetector:Found vertical path from "<<start<<" to edge of grid\n";
#endif

    return true;
}


bool horizontal_path_exists_to_map_edge(cell_t start, int xIncrement, int xEnd, const VoronoiSkeletonGrid& grid)
{
    for(int x = start.x; x != xEnd; x += xIncrement)
    {
        if(!(grid.getClassification(x, start.y) & OCCUPIED_CELL_MASK))
        {
            return false;
        }
    }

#ifdef DEBUG_CARDINAL_SEARCH
    std::cout<<"INFO:IslandDetector:Found horizontal path from "<<start<<" to edge of grid\n";
#endif

    return true;
}

} // namespace hssh
} // namespace vulcan
