/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "hssh/local_metric/lpm.h"
#include "hssh/metrical/mapping/mapping_params.h"
#include <iostream>

namespace vulcan
{
namespace hssh
{

LocalPerceptualMap::LocalPerceptualMap(void) : timestamp(0), mapId(-1), frameIndex(0)
{
}


LocalPerceptualMap::LocalPerceptualMap(const lpm_params_t& gridParams, const Point<float>& globalCenter)
: OccupancyGrid(gridParams.width,
                gridParams.height,
                gridParams.scale,
                globalCenter,
                gridParams.occupiedCellCost,
                gridParams.freeCellCost)
, timestamp(0)
, mapId(-1)
, frameIndex(0)
{
}


LocalPerceptualMap::LocalPerceptualMap(std::size_t widthInCells,
                                       std::size_t heightInCells,
                                       float cellsToMeters,
                                       const Point<float>& globalCenter,
                                       uint8_t occupiedCellCost,
                                       uint8_t freeCellCost)
: OccupancyGrid(widthInCells, heightInCells, cellsToMeters, globalCenter, occupiedCellCost, freeCellCost)
, timestamp(0)
, mapId(-1)
, frameIndex(0)
{
}


LocalPerceptualMap::LocalPerceptualMap(const OccupancyGrid& grid) : OccupancyGrid(grid), frameIndex(0)
{
}


void LocalPerceptualMap::changeReferenceFrame(const pose_t& newReferenceFrame)
{
    //     costGrid = utils::transform_grid(costGrid, INITIAL_CELL_COST, newReferenceFrame.x, newReferenceFrame.y,
    //     newReferenceFrame.theta); typeGrid = utils::transform_grid(typeGrid, kUnobservedOccGridCell,
    //     newReferenceFrame.x, newReferenceFrame.y, newReferenceFrame.theta);
    //
    //     std::cout<<"Changing frame to "<<newReferenceFrame<<'\n';
    //
    //     ++frameIndex;
    //     transformFromLastFrame = newReferenceFrame;

    std::cerr << "STUB!  LocalPerceptualMap::changeReferenceFrame\n";
}

}   // namespace hssh
}   // namespace vulcan
