/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     place_grid.cpp
* \author   Collin Johnson
*
* Definition of VoronoiSkeletonGrid.
*/

#include <hssh/local_topological/voronoi_skeleton_grid.h>

namespace vulcan
{
namespace hssh
{
    
std::vector<cell_t> filter_cells(const std::vector<cell_t>& cells, uint8_t mask, const ClassificationGrid& classificationGrid);


VoronoiSkeletonGrid::VoronoiSkeletonGrid(void)
{
}


VoronoiSkeletonGrid::VoronoiSkeletonGrid(std::size_t width, std::size_t height, float scale)
    : distanceGrid      (width, height, scale, Point<float>(0, 0))
    , classificationGrid(width, height, scale, Point<float>(0, 0))
{
}


VoronoiSkeletonGrid::VoronoiSkeletonGrid(const DistanceGrid& distanceGrid, const ClassificationGrid& classificationGrid)
: distanceGrid(distanceGrid)
, classificationGrid(classificationGrid)
{
}


SourceCells VoronoiSkeletonGrid::getSourceCells(int x, int y) const
{
    auto sourcesIt = skeletonSources.find(cell_t(x, y));
    
    if(sourcesIt != skeletonSources.end())
    {
        return sourcesIt->second;
    }
    
    return SourceCells();
}


void VoronoiSkeletonGrid::filterVoronoiInformation(uint8_t classificationMask)
{
    std::vector<cell_t> filteredJunctions = filter_cells(junctions, classificationMask, classificationGrid);
    std::vector<cell_t> filteredDeadEnds  = filter_cells(deadEnds,  classificationMask, classificationGrid);
    
    junctions = std::move(filteredJunctions);
    deadEnds  = std::move(filteredDeadEnds);
}


void VoronoiSkeletonGrid::setGridSize(std::size_t width, std::size_t height)
{
    distanceGrid.setGridSizeInCells(width, height);
    classificationGrid.setGridSizeInCells(width, height);
}


void VoronoiSkeletonGrid::setScale(float scale)
{
    distanceGrid.setMetersPerCell(scale);
    classificationGrid.setMetersPerCell(scale);
}


void VoronoiSkeletonGrid::setBottomLeft(const Point<float>& bottomLeft)
{
    distanceGrid.setBottomLeft(bottomLeft);
    classificationGrid.setBottomLeft(bottomLeft);
}


void VoronoiSkeletonGrid::setObstacleDistance(int x, int y, VoronoiDist distance)
{
    distanceGrid(x, y) = distance;
}


void VoronoiSkeletonGrid::setClassification(int x, int y, uint8_t classification)
{
    classificationGrid(x, y) = classification;
}


void VoronoiSkeletonGrid::addClassification(int x, int y, skeleton_cell_type_t classification)
{
    classificationGrid(x, y) |= static_cast<uint8_t>(classification);
}


void VoronoiSkeletonGrid::removeClassification(int x, int y, skeleton_cell_type_t classification)
{
    classificationGrid(x, y) &= ~static_cast<uint8_t>(classification);
}


void VoronoiSkeletonGrid::reset(VoronoiDist distance, uint8_t classification)
{
    distanceGrid.reset(distance);
    classificationGrid.reset(classification);
}


std::vector<cell_t> filter_cells(const std::vector<cell_t>& cells, uint8_t mask, const ClassificationGrid& classificationGrid)
{
    std::vector<cell_t> filtered;
    
    for(auto cell : cells)
    {
        if(classificationGrid.getValue(cell.x, cell.y) & mask)
        {
            filtered.push_back(cell);
        }
    }
    
    return filtered;
}

} // namespace hssh
} // namespace vulcan
