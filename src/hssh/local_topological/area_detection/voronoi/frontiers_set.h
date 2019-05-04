/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     frontiers_set.h
* \author   Collin Johnson
* 
* Declaration of FrontiersSet.
*/

#ifndef HSSH_LOCAL_TOPOLOGICAL_VORONOI_FRONTIERS_SET_H
#define HSSH_LOCAL_TOPOLOGICAL_VORONOI_FRONTIERS_SET_H

#include <hssh/local_topological/voronoi_skeleton_grid.h>
#include <hssh/local_topological/frontier.h>
#include <map>
#include <vector>

namespace vulcan
{
namespace utils { class DisjointSetForest; }

namespace hssh
{
    
/**
* FrontiersSet builds frontiers and maintains the relationship between a particular frontier
* cell and its corresponding frontier. Each frontier consists of the connected frontier cells
* from the VoronoiSkeletonGrid.
* 
* 
*/
class FrontiersSet
{
public:
    
    static const int INVALID_FRONTIER = -1;
    
    /**
    * Constructor for FrontiersSet.
    * 
    * \param    frontierCells               The frontier cells in the Voronoi skeleton
    * \param    farFromOccupiedDistance     Minimum distance a cell needs to be from the edge of a frontier to be considered far from the frontier
    */
    FrontiersSet(const VoronoiSkeletonGrid& grid, float farFromOccupiedDistance);
    
    /**
    * getCellFrontierId retrieves the id of the frontier to which the provided cell belongs.
    * 
    * \param    frontierCell        Cell for which to retrieve the id
    * \return   The associated frontier. Frontier with id == INVALID_FRONTIER if this cell doesn't belong to a frontier.
    */
    Frontier getCellFrontier(cell_t frontierCell) const;
    
    /**
    * isFarFromOccupied checks to see if the provided cell is far from an occupied cell.
    * 
    * \param    frontierCell        Cell to check for frontier status
    * \return   True if the cell belongs to a frontier and is far from the edge of the frontier, as deemed by the farFromOccupiedDistance
    *           provided in the constructor.
    */
    bool isFarFromOccupied(cell_t frontierCell) const;
    
    /**
    * getFrontiers retrieves all frontiers in the set.
    */
    std::vector<Frontier> getFrontiers(void) const { return frontiers; }
    
private:
    
    std::vector<Frontier> frontiers;
    std::map<cell_t, int>   cellToFrontierIndex;
    
    int farFromOccupiedDistance;
    
    void growFrontier(cell_t start, CellSet& addedCells, std::size_t frontierIndex, const VoronoiSkeletonGrid& grid);
};
    
}
}

#endif // HSSH_LOCAL_TOPOLOGICAL_VORONOI_FRONTIERS_SET_H
