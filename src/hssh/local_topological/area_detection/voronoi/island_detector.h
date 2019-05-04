/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     island_detector.h
* \author   Collin Johnson
*
* Declaration of IslandDetector.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISLAND_DETECTOR_H
#define HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISLAND_DETECTOR_H

#include <hssh/types.h>
#include <cstdint>
#include <deque>
#include <memory>

namespace vulcan
{
namespace utils { class DisjointSetForest; }
namespace hssh
{

class VoronoiSkeletonGrid;

/**
* IslandDetector is used to find small islands in the place grid. These islands are small
* enough to be easily circumnavigated, and as such, should not be included in the local
* topology of a place.
*
* Island detection is a connected components search. An island is a connected component
* less than some size and not connected to the edge of the map. The edge of the map condition
* exists because the majority of the map edge in enclosed environments belongs to the
* underlying static structure.
*/
class IslandDetector
{
public:

    /**
    * Constructor for IslandDetector.
    *
    * \param    maxIslandAreaInCells        Maximum number of cells in a connected components for it to be considered an island
    */
    IslandDetector(int maxIslandAreaInCells);

    /**
    * reset resets the island detector for a new map.
    */
    void reset(void);

    /**
    * cellIsOnIsland checks the provided cell to see if it belongs to an island component.
    *
    * \param    cell            Cell to be checked
    * \param    grid            Grid in which the cell exists
    * \return   True if the cell is found on an island. False otherwise.
    */
    bool cellIsOnIsland(cell_t cell, const VoronoiSkeletonGrid& grid) const;

private:

    mutable std::shared_ptr<utils::DisjointSetForest> islands;
    mutable std::deque<cell_t>                        searchQueue;

    int maxIslandArea;

    // Disallow copying
    IslandDetector(const IslandDetector& toCopy)         = delete;
    IslandDetector& operator=(const IslandDetector& rhs) = delete;

    // Can make all methods const because cellIsOnIsland is a const method in that repeated calls will always produce the same results
    
    // Initialization
    void createSetForestForGridIfNeeded(const VoronoiSkeletonGrid& grid) const;

    // A fast check before performing a full search to see if the cell is connected to the edge of the map
    // by a vertical or horizontal line
    bool checkCardinalDirections(cell_t cell, const VoronoiSkeletonGrid& grid) const;

    // Methods for the full BFS of the cell's component
    unsigned int findCellComponent(cell_t cell, const VoronoiSkeletonGrid& grid) const;
    void expandCell               (cell_t cell, const VoronoiSkeletonGrid& grid) const;
    void processCell              (cell_t cell, const VoronoiSkeletonGrid& grid) const;
    void mergeAndEnqueueNeighbor  (cell_t neighbor, unsigned int parentIndex, const VoronoiSkeletonGrid& grid) const;
};

}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_GATEWAYS_ISLAND_DETECTOR_H
