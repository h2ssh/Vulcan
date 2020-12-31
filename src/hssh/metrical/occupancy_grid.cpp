/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     occupancy_grid.cpp
* \author   Collin Johnson
*
* Definition of the OccupancyGrid.
*/

#include "hssh/metrical/occupancy_grid.h"

namespace vulcan
{
namespace hssh
{

const uint8_t MAX_CELL_COST     = 255u;
const uint8_t INITIAL_CELL_COST = MAX_CELL_COST / 2;


OccupancyGrid::OccupancyGrid(std::size_t               widthInCells,
                             std::size_t               heightInCells, 
                             float                     cellsToMeters, 
                             const Point<float>& globalCenter,
                             uint8_t                   occupiedCellCost, 
                             uint8_t                   freeCellCost)
: costGrid(widthInCells, heightInCells, cellsToMeters, globalCenter, INITIAL_CELL_COST)
, typeGrid(widthInCells, heightInCells, cellsToMeters, globalCenter, kUnobservedOccGridCell)
, occupiedCellCostThreshold(occupiedCellCost)
, freeCellCostThreshold(freeCellCost)
{
}


// Accessors
uint8_t OccupancyGrid::getMaxCellCost(void) const
{
    return MAX_CELL_COST;
}

// Mutators
void OccupancyGrid::setGridSizeInCells(std::size_t width, std::size_t height)
{
    costGrid.setGridSizeInCells(width, height);
    typeGrid.setGridSizeInCells(width, height);
}


void OccupancyGrid::setBottomLeft(const Point<float>& newBottom)
{
    costGrid.setBottomLeft(newBottom);
    typeGrid.setBottomLeft(newBottom);
}


void OccupancyGrid::setMetersPerCell(float gridScale)
{
    costGrid.setMetersPerCell(gridScale);
    typeGrid.setMetersPerCell(gridScale);
}


// Methods for modifying position of the grid
void OccupancyGrid::changeGlobalCenter(const Point<float>& newGlobalCenter, uint8_t initialCost)
{
    if(newGlobalCenter != costGrid.getGlobalCenter())
    {
        costGrid.changeGlobalCenter(newGlobalCenter, initialCost);
        typeGrid.changeGlobalCenter(newGlobalCenter, kUnobservedOccGridCell);
    }
}


void OccupancyGrid::changeBoundary(const math::Rectangle<float>& boundary)
{
    costGrid.changeBoundary(boundary, INITIAL_CELL_COST);
    typeGrid.changeBoundary(boundary, kUnobservedOccGridCell);
}


void OccupancyGrid::rotate(float radians)
{
    costGrid = utils::transform_grid(costGrid, INITIAL_CELL_COST, 0, 0, radians);
    typeGrid = utils::transform_grid(typeGrid, kUnobservedOccGridCell, 0, 0, radians);
}


void OccupancyGrid::setCost(const Point<int>& cell, uint8_t cost)
{
    costGrid.setValue(cell.x, cell.y, cost);
    setCellType(cell, cost);
}


uint8_t OccupancyGrid::updateCost(const Point<int>& cell, int8_t change)
{
    if(isCellInGrid(cell))
    {
        return updateCostNoCheck(cell, change);
    }
    else
    {
        return 0;
    }
}


void OccupancyGrid::reset(void)
{
    costGrid.reset(INITIAL_CELL_COST);
    typeGrid.reset(kUnobservedOccGridCell);
}


void OccupancyGrid::setCostNoCheck(const Point<int>& cell, uint8_t cost)
{
    costGrid.setValueNoCheck(cell.x, cell.y, cost);
    setCellTypeNoCheck(cell, cost);
}


uint8_t OccupancyGrid::updateCostNoCheck(const Point<int>& cell, int8_t change)
{
    // No need to do anything if the cost isn't actually changing
    if(!change)
    {
        return costGrid.getValueNoCheck(cell.x, cell.y);
    }

    // Need int16_t explicit here, as the cell values don't update correctly otherwise, overflow results in
    // values shooting to 0
    int16_t newCost = static_cast<int16_t>(costGrid.getValueNoCheck(cell.x, cell.y)) + change;

    if(newCost < 0)
    {
        newCost = 0;
    }
    else if(newCost > static_cast<int16_t>(MAX_CELL_COST))
    {
        newCost = MAX_CELL_COST;
    }
    
    auto cellType = setCellTypeNoCheck(cell, static_cast<uint8_t>(newCost));
    
    // If one of the permanent types, then always max cost because these types might be drawn in by hand
    // and they might not be visible features of the environment
    if(cellType & kPermanentOccGridCell)
    {
        newCost = getMaxCellCost();
    }
    
    costGrid.setValueNoCheck(cell.x, cell.y, static_cast<uint8_t>(newCost));

    return newCost;
}


cell_type_t OccupancyGrid::setCellType(const Point<int>& cell, uint8_t cost)
{
    if(typeGrid.isCellInGrid(cell))
    {
        cell_type_t cellType = determineCellType(cost, cell);
        typeGrid.setValueNoCheck(cell.x, cell.y, cellType);
        return cellType;
    }
    // Any cell not in the grid must be unobserved
    else
    {
        return kUnobservedOccGridCell;
    }
    
}


cell_type_t OccupancyGrid::setCellTypeNoCheck(const Point<int>& cell, uint8_t cost)
{
    cell_type_t cellType = determineCellType(cost, cell);
    typeGrid.setValueNoCheck(cell.x, cell.y, cellType);
    return cellType;
}


cell_type_t OccupancyGrid::determineCellType(uint8_t cost, const Point<int>& cell)
{
    /*
    * NOTE: The quasi-static designation isn't great. In general, we need observations over a longer timespan to
    * determine quasi-staticness. However, at this shorter time scale, the quasi-static exists to hopefully allowed doors
    * previously seen as open to be seen now as quasi-static, which will let the local_topo_hssh still identify the
    * appropriate gateways.
    */
    
    cell_type_t currentType = typeGrid.getValueNoCheck(cell.x, cell.y);
    cell_type_t newType     = kUnobservedOccGridCell;

    if(cost <= freeCellCostThreshold)
    {
        newType = kFreeOccGridCell;
    }
    else // cost > freeCellCostThreshold
    {
        // If a cell is above the free threshold and it was once free, then it must be dynamic, as long as it hasn't
        // hit the maximum cost threshold.
        if((currentType & (kFreeOccGridCell | kDynamicOccGridCell)) && (cost < MAX_CELL_COST))
        {
            newType = kDynamicOccGridCell;
        }
        // Once the maximum cost threshold is hit, then the cell has been occupied for awhile. We optionally transition
        // to quasi-static at this point, depending on the neighboring cells.
        else if((currentType & kDynamicOccGridCell) && (cost == MAX_CELL_COST))
        {
            // If a dynamic cell hits the max cost and it is adjacent to a wall, then it is quite possibly a door or
            // moving furniture, so turn it into a quasi-static cell.
            if(hasOccupiedNeighbor(cell))
            {
                newType = kQuasiStaticOccGridCell;
            }
            // If the dynamic cell is floating in free space, then it is most likely a person or something similar, so
            // leave it to be tracked by the object tracker
            else
            {
                newType = kDynamicOccGridCell;
            }
        }
        // If the cell has crossed the occupied threshold at this point, then it was not a free or dynamic cell.
        // Thus, it is either occupied, or is a cell that transitioned to quasi-static and should remain as such.
        else if(cost >= occupiedCellCostThreshold)
        {
            // Once quasi-static, then leave a cell quasi-static forever.
            if(currentType & kQuasiStaticOccGridCell)
            {
               newType = kQuasiStaticOccGridCell;
            }
            // Everything else should be occupied
            else
            {
                newType = kOccupiedOccGridCell;
            }
        }
        // Otherwise, it is a cell that was occupied or quasi-static, but now has gone below the occupied threshold
        else // cost < occupiedCellCostThreshold
        {
            // Quasi-static cells transition back to dynamic, as we still had strong evidence the cell was free at some
            // point of time in the past.
            if(currentType & kQuasiStaticOccGridCell)
            {
                newType = kDynamicOccGridCell;
            }
            // Otherwise, the cell is turning into something, probably free, but it should go back to unknown for now.
            // These transitions often are the result of localization uncertainty.
            else
            {
                newType = kUnknownOccGridCell;
            }
        }
    }
    
    // If limited visibility, can't also be dynamic
    if(currentType & kLimitedVisibilityOccGridCell)
    {
        newType &= ~kDynamicOccGridCell;
        newType |= kOccupiedOccGridCell;
    }

    // Make sure any permanently assigned flags don't change, even if the underlying cost goes to free or dynamic
    return (currentType & kPermanentOccGridCell) | newType;
}


bool OccupancyGrid::hasOccupiedNeighbor(const Point<int>& cell)
{
    const uint8_t kOccupiedMask = kOccupiedOccGridCell | kQuasiStaticOccGridCell | kLimitedVisibilityOccGridCell;
    
    if((cell.x > 0) && (typeGrid.getValueNoCheck(cell.x-1, cell.y) & kOccupiedMask))
    {
        return true;
    }
    
    int lastCellX = typeGrid.getWidthInCells() > 0 ? typeGrid.getWidthInCells() - 1 : 0;

    if((cell.x < lastCellX) && (typeGrid.getValueNoCheck(cell.x+1, cell.y) & kOccupiedMask))
    {
        return true;
    }

    if((cell.y > 0) && (typeGrid.getValueNoCheck(cell.x, cell.y-1) & kOccupiedMask))
    {
        return true;
    }
    
    int lastCellY = typeGrid.getHeightInCells() > 0 ? typeGrid.getHeightInCells() - 1 : 0;

    if((cell.y < lastCellY) && (typeGrid.getValueNoCheck(cell.x, cell.y+1) & kOccupiedMask))
    {
        return true;
    }

    return false;
}


} // namespace hssh
} // namespace vulcan
