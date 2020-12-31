/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_grid_utils.h
* \author   Collin Johnson
*
* Declaration of utilities for NavigationGrid:
*
*   - GridPath : defines a path through a grid
*   - minimum_cost_path : find the minimum cost path from the position to the goal
*/

#ifndef MPEPC_GRIDS_NAVIGATION_GRID_UTILS_H
#define MPEPC_GRIDS_NAVIGATION_GRID_UTILS_H

#include "core/point.h"
#include <vector>

namespace vulcan
{
namespace mpepc
{

class NavigationGrid;

/**
* GridPath defines a path through the grid. Both the grid cells and global position of each vertex in the path
* is provided.
*/
class GridPath
{
public:

    /**
    * Default constructor for GridPath.
    *
    * Create an empty path.
    */
    GridPath(void) = default;

    /**
    * Constructor for GridPath.
    *
    * Create a path from cells.
    *
    * \param    cells           Cells in the path
    * \param    grid            Grid in which the path is defined
    */
    GridPath(const std::vector<Point<int>>& cells, const NavigationGrid& grid);

    /**
    * Constructor for GridPath.
    *
    * Create a path from positions.
    *
    * \param    positions       Positions in the path
    * \param    grid            Grid in which the path is defined
    */
    GridPath(const std::vector<Point<float>>& positions, const NavigationGrid& grid);

    /**
    * simplify creates a new GridPath where all co-linear positions have been removed so only the final vertices
    * remain, thereby explicitly encoding the edges vs implicitly encoding them via adjacent cells.
    */
    GridPath simplify(void) const;

    // Iterator access to the grid path
    std::size_t size(void) const { return cells_.size(); }
    bool empty(void) const { return cells_.empty(); }
    std::vector<Point<int>>::const_iterator beginCell(void) const { return cells_.begin(); }
    std::vector<Point<int>>::const_iterator endCell(void) const { return cells_.end(); }

    std::vector<Point<float>>::const_iterator beginPosition(void) const { return positions_.begin(); }
    std::vector<Point<float>>::const_iterator endPosition(void) const { return positions_.end(); }

private:

    // INVARIANT: cells_.size() == positions_.size()
    std::vector<Point<int>> cells_;
    std::vector<Point<float>> positions_;
};

/**
* minimum_cost_path follows the gradient of the NavigationGrid from the provided starting position to the goal by
* following the gradient of the navigation function.
*
* \param    start           Starting position of the path (global coordinates)
* \param    navFunc         Navigation function to the goal
* \return   The path from the start to the goal encoded in the navigation function. If there's no path, then the
*   returned path will be empty.
*/
GridPath minimum_cost_path(Point<float> start, const NavigationGrid& navFunc);

} // namespace mpepc
} // namespace vulcan

#endif // MPEPC_GRIDS_NAVIGATION_GRID_UTILS_H
