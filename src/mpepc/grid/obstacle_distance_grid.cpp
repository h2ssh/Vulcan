/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     obstacle_distance_grid.cpp
* \author   Jong Jin Park and Collin Johnson
*
* Definition of ObstacleDistanceGrid and ObstacleDistanceGridBuilder.
*/

#include "mpepc/grid/obstacle_distance_grid.h"
#include "math/interpolation.h"

#define DEBUG_OBSTACLE_DISTANCE_GRID

namespace vulcan
{
namespace mpepc
{

float computeOffset(float fraction); // helper function for interpolation

////// Grid ///////////////////////////////////////////////////////////////////////////////////////////////////
ObstacleDistanceGrid::ObstacleDistanceGrid(void)
: timestamp_(0)
, id_(2<<30)
, maxGridDist_(0)
{
}


float ObstacleDistanceGrid::getObstacleDistance(const Point<int>& cell) const
{
    if(isCellInGrid(cell))
    {
        return (getValueNoCheck(cell.x, cell.y) * metersPerCell()) / kStraightCellDist;
    }
    else
    {
        return 0.0f; // treat all cells out of bound as obstacles.
    }
}


float ObstacleDistanceGrid::getObstacleDistance(const Point<float>& position) const
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
    float xOffset = computeOffset(fractionX);
    float yOffset = computeOffset(fractionY);
    Point<float> offset(xOffset, yOffset);
    
    // values of the cells to be interpolated
    float values[4];
    values[0] = getObstacleDistance(Point<int>(leftIndex,   bottomIndex));
    values[1] = getObstacleDistance(Point<int>(leftIndex+1, bottomIndex));
    values[2] = getObstacleDistance(Point<int>(leftIndex,   bottomIndex+1));
    values[3] = getObstacleDistance(Point<int>(leftIndex+1, bottomIndex+1));
    
    return math::unit_bilinear_interpolation(offset, values);
}


void ObstacleDistanceGrid::setGridDist(const Point<int>& cell, int32_t gridDist)
{ 
    if(isCellInGrid(cell.x, cell.y))
    {
        setGridDistNoCheck(cell, gridDist);
    }
}


void ObstacleDistanceGrid::setGridDistNoCheck(const Point<int>& cell, int32_t gridDist)
{ 
    setValueNoCheck(cell.x, cell.y, gridDist); 
    
    if(gridDist > maxGridDist_)
    {
        maxGridDist_ = gridDist;
    }
}


bool ObstacleDistanceGrid::isPositionInFreeSpace(const Point<float> position, float margin_m) const
{
    Point<int> cell = positionToCell(position); // do an approximate check with cells
    
    return getObstacleDistance(cell) > margin_m;
}


Point<int> ObstacleDistanceGrid::positionToCell(const Point<float>& position) const
{
    return utils::global_point_to_grid_cell(position, *this);
}


Point<float> ObstacleDistanceGrid::cellToPosition(const Point<int>& cell) const
{
    Point<float> cellCenter;
    cellCenter.x = getBottomLeft().x + ((cell.x + 0.5) * metersPerCell());
    cellCenter.y = getBottomLeft().y + ((cell.y + 0.5) * metersPerCell());
    
    return cellCenter;
}


float computeOffset(float fraction)
{
    float offset = (fraction < 0.5f) ? (fraction + 0.5f) : (fraction - 0.5f);
    
    // clamping to prevent floating point error
    if(offset > 1.0f)
    {
        std::cout<<"WARNING: ObstacleDistanceGrid: Offset out of bound: "<<offset<<"\n";
        return 1.0f;
    }
    else if(offset < 0.0f)
    {
        std::cout<<"WARNING: ObstacleDistanceGrid: Offset out of bound: "<<offset<<"\n";
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
