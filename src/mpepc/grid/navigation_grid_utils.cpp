/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_grid_utils.cpp
* \author   Collin Johnson
*
* Definition of utilities for NavigationGrid:
*
*   - GridPath
*   - minimum_cost_path
*/

#include <mpepc/grid/navigation_grid_utils.h>
#include <mpepc/grid/navigation_grid.h>

namespace vulcan
{
namespace mpepc
{

////////////////////////   GridPath definition   ///////////////////////////////
GridPath::GridPath(const std::vector<Point<int>>& cells, const NavigationGrid& grid)
: cells_(cells)
{
    std::transform(cells_.begin(), cells_.end(), std::back_inserter(positions_), [&grid](auto cell) {
        return grid.cellToPosition(cell);
    });
}


GridPath::GridPath(const std::vector<Point<float>>& positions, const NavigationGrid& grid)
: positions_(positions)
{
    std::transform(positions_.begin(), positions_.end(), std::back_inserter(cells_), [&grid](auto position) {
        return grid.positionToCell(position);
    });
}


GridPath GridPath::simplify(void) const
{
    // TODO
    return *this;
}


///////////////////////   Utility function definitions   ////////////////////////

GridPath minimum_cost_path(Point<float> start, const NavigationGrid& navFunc)
{
    // Empty path if the point isn't in the grid
    if(!navFunc.isPointInGrid(start))
    {
        return GridPath();
    }

    std::vector<Point<int>> path;
    auto nextCell = navFunc.positionToCell(start);
    float minCost = navFunc.getCostToGo(nextCell);

    // Upper-bound on path is to visit every single cell
    for(std::size_t n = 0; n < navFunc.getWidthInCells() * navFunc.getHeightInCells(); ++n)
    {
        path.push_back(nextCell);

        float bestCost = minCost;
        auto bestCell = nextCell;
        // Select next best cell
        for(int y = -1; y <= 1; ++y)
        {
            for(int x = -1; x <= 1; ++x)
            {
                Point<int> neighbor(nextCell.x + x, nextCell.y + y);
                float costToGo = navFunc.getCostToGo(neighbor);

                if(costToGo < bestCost)
                {
                    bestCost = costToGo;
                    bestCell = neighbor;
                }
            }
        }

        // If the cell moved, then not at a minimum
        if(bestCell != nextCell)
        {
            nextCell = bestCell;
            minCost = bestCost;
        }
        // If no lower-cost cells then reached the minimum and thus the goal
        else
        {
            break;
        }
    }

    return GridPath(path, navFunc);
}

} // namespace mpepc
} // namespace vulcan
