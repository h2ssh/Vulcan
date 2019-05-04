/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_grid.cpp
* \author   Jong Jin Park and Collin Johnson
*
* Definition of NavigationGrid and NavigationGridBuilder.
*/

#include <mpepc/grid/navigation_grid.h>
#include <math/interpolation.h>

namespace vulcan
{
namespace mpepc
{

float computeNavGridOffset(float fraction); // helper function for interpolation

////// Grid ///////////////////////////////////////////////////////////////////////////////////////////////////
NavigationGrid::NavigationGrid(void)
: timestamp_(0)
, id_(2<<30)
, maxGridCostToGo_(0)
{
}


void NavigationGrid::setGridCost(const Point<int>& cell, int32_t gridCost)
{
    if(isCellInGrid(cell.x, cell.y))
    {
        setGridCostNoCheck(cell, gridCost);
    }
}


void NavigationGrid::setGridCostNoCheck(const Point<int>& cell, int32_t gridCost)
{
    setValueNoCheck(cell.x, cell.y, gridCost);

    if(gridCost > maxGridCostToGo_)
    {
        maxGridCostToGo_ = gridCost;
    }
}


float NavigationGrid::getCostToGo(const Point<int>& cell) const
{
    if(isCellInGrid(cell))
    {
        return (getValueNoCheck(cell.x, cell.y) * metersPerCell()) / kStraightCellDist;
    }
    else
    {
        // return some large value for cells out of bound.
        std::cout<<"WARNING: NavigationGrid: Cell out of bound. Cell location at "<<'('<<cell.x<<','<<cell.y<<')'<<"\n";
        return getMaxCostToGo();
    }
}


float NavigationGrid::getCostToGo(const Point<float>& position) const
{
    // grid indices of the cell the queried point falls in
    Point<int> cell = utils::global_point_to_grid_cell(position, *this);

    // metric location of the bottom left corner of the cell
    Point<float> bottomLeftCorner = utils::grid_point_to_global_point(cell, *this);

    // fractional location of the point within the cell.
    float fractionX = (position.x - bottomLeftCorner.x) * cellsPerMeter();
    float fractionY = (position.y - bottomLeftCorner.y) * cellsPerMeter();

    // fractional location determines which cells to interpolate with
    int leftIndex   = cell.x - 1 + static_cast<int>(round(fractionX));
    int bottomIndex = cell.y - 1 + static_cast<int>(round(fractionY));

    // fractional location from the bottom left cell to be interpolated
    float xOffset = computeNavGridOffset(fractionX);
    float yOffset = computeNavGridOffset(fractionY);
    Point<float> offset(xOffset, yOffset);

    // values of the cells to be interpolated
    float values[4];
    values[0] = getCostToGo(Point<int>(leftIndex,   bottomIndex));
    values[1] = getCostToGo(Point<int>(leftIndex+1, bottomIndex));
    values[2] = getCostToGo(Point<int>(leftIndex,   bottomIndex+1));
    values[3] = getCostToGo(Point<int>(leftIndex+1, bottomIndex+1));

    return math::unit_bilinear_interpolation(offset, values);
}


float NavigationGrid::getMaxCostToGo(void) const
{
    return maxGridCostToGo_ / static_cast<float>(kStraightCellDist);
}


bool NavigationGrid::isPointInGrid(Point< float > position, float margin_m) const
{
    if(getWidthInMeters() < 2.0f * margin_m)
    {
        return false;
    }

    Point<int> cell = positionToCell(position);
    int width = getWidthInCells();
    int height = getHeightInCells();
    int marginInCells = margin_m / metersPerCell();

    return (cell.x < (width - marginInCells)) &&
           (cell.x > marginInCells) &&
           (cell.y < (height - marginInCells)) &&
           (cell.y > marginInCells);
}


Point<int> NavigationGrid::positionToCell(const Point<float>& position) const
{
    return utils::global_point_to_grid_cell(position, *this);
}


Point<float> NavigationGrid::cellToPosition(const Point<int>& cell) const
{
    Point<float> cellCenter;
    cellCenter.x = getBottomLeft().x + (cell.x + 0.5)*metersPerCell();
    cellCenter.y = getBottomLeft().y + (cell.y + 0.5)*metersPerCell();

    return cellCenter;
}


void NavigationGrid::resetCosts(int32_t cost)
{
    reset(cost);
    maxGridCostToGo_ = 0;
}


float computeNavGridOffset(float fraction)
{
    float offset = (fraction < 0.5f) ? (fraction + 0.5f) : (fraction - 0.5f);

    // clamping to prevent floating point error
    if(offset > 1.0f)
    {
        std::cout<<"WARNING: NavigationGrid: Offset out of bound: "<<offset<<"\n";
        return 1.0f;
    }
    else if(offset < 0.0f)
    {
        std::cout<<"WARNING: NavigationGrid: Offset out of bound: "<<offset<<"\n";
        return 0.0f;
    }
    else
    {
        return offset;
    }

    return offset;
}

} // namespace mpepc
} // namespace vulcan
