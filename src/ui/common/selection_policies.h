/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     selection_policies.h
* \author   Collin Johnson
* 
* Definition of implementations of the SelectionPolicy concept for the GridObjectSelector:
* 
*   - ExactCellSelector : matches only the exact cell
*   - NearestCellSelector : matches the nearest cell in the object cells
*/

#ifndef UI_COMMON_SELECTION_POLICIES_H
#define UI_COMMON_SELECTION_POLICIES_H

#include <core/point.h>
#include <algorithm>
#include <vector>

namespace vulcan
{
namespace ui
{
    
using Cell = Point<int>;
using ObjectCells = std::vector<Cell>;

/**
* ExactCellSelector always returns the exact cell that was found.
*/
struct ExactCellSelector
{
    Cell operator()(Cell cell, const ObjectCells& objectCells)
    {
        return cell;
    }
};

/**
* NearestCellSelector returns the nearest cell to cell in objectCells.
*/
struct NearestCellSelector
{
    Cell operator()(Cell cell, const ObjectCells& objectCells)
    {
        if(objectCells.empty())
        {
            return cell;
        }
        
        return *std::min_element(objectCells.begin(), objectCells.end(), [&cell](Cell lhs, Cell rhs) {
            return distance_between_points(lhs, cell) < distance_between_points(rhs, cell);
        });
    }
};

} // namespace ui
} // namespace vulcan

#endif // UI_COMMON_SELECTION_POLICIES_H
