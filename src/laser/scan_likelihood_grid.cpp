/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     scan_likelihood_grid.cpp
* \author   Collin Johnson
*
* Definition of ScanLikelihoodGrid.
*/

#include "laser/scan_likelihood_grid.h"
#include "math/univariate_gaussian.h"

#include <iomanip>
#include <iostream>
#include <cmath>
#include <cassert>

#define DEBUG_RASTER
// #define DEBUG_FINE_GRID
// #define DEBUG_COARSE_GRID

namespace vulcan
{
namespace laser
{

const float RASTER_MAHALANOBIS_MAX = 1.0f;
const int   SKIP_INTERVAL = 2;

// Helper functions and structs
struct rectangle_bounds_t
{
    rectangle_bounds_t(void) : minX(1000000), maxX(-1000000), minY(1000000), maxY(-1000000) { }

    float minX;
    float maxX;

    float minY;
    float maxY;
};


std::pair<float, float> find_scan_min   (const std::vector<Point<float>>& scan, const utils::CellGrid<uint8_t>& grid);
rectangle_bounds_t      find_scan_bounds(const std::vector<Point<float>>& scan);

uint8_t find_max_cell_value_in_region(const Point<uint16_t>& regionStart, uint8_t regionSize, const utils::CellGrid<uint8_t>& grid);

void display_raster(uint8_t raster[51][51], uint8_t rasterWidth);
void display_likelihood_grid(const utils::CellGrid<uint8_t>& grid);


inline float mahalanobis(float value, float mean, float variance)
{
    return (value-mean)*(value-mean)/variance;
}


ScanLikelihoodGrid::ScanLikelihoodGrid(uint16_t widthInCells, uint16_t heightInCells, float metersPerCell)
    : rasterSize(0)
    , grid(widthInCells, heightInCells, metersPerCell, Point<float>(0, 0))
{
}


void ScanLikelihoodGrid::setLaserVariance(float variance)
{
    createRaster(variance);
}


void ScanLikelihoodGrid::setReferenceScan(const std::vector<Point<float>>& scan)
{
    assert(rasterSize > 0);   // there must be a raster in order to create a scan grid
    assert(!scan.empty());    // there must be a scan to rasterize

    grid.reset(0);

    setReferenceScanOrigin(scan);

    for(size_t x = 0; x < scan.size(); x += SKIP_INTERVAL)
    {
        rasterizeScanPoint(scan[x]);
    }

    #ifdef DEBUG_FINE_GRID
    display_likelihood_grid(grid);
    #endif
}


void ScanLikelihoodGrid::setLikelihoodSearchBaseScan(const std::vector<Point<float>>& baseScan)
{
    if(baseScan.size()/SKIP_INTERVAL != baseLikelihoodScanPoints.size())
    {
        baseLikelihoodScanPoints.resize(baseScan.size()/SKIP_INTERVAL);
    }

    for(int i = baseLikelihoodScanPoints.size(); --i >= 0;)
    {
        baseLikelihoodScanPoints[i].x = lrint(baseScan[i*SKIP_INTERVAL].x * grid.cellsPerMeter()) - referenceScanOrigin.x;
        baseLikelihoodScanPoints[i].y = lrint(baseScan[i*SKIP_INTERVAL].y * grid.cellsPerMeter()) - referenceScanOrigin.y;
    }
}


uint32_t ScanLikelihoodGrid::calculateTransformLikelihood(float deltaX, float deltaY) const
{
    uint32_t likelihood = 0;

    int xShift = lrint(deltaX*grid.cellsPerMeter());
    int yShift = lrint(deltaY*grid.cellsPerMeter());

    for(size_t i = 0; i < baseLikelihoodScanPoints.size(); ++i)
    {
        likelihood += grid.getValue(baseLikelihoodScanPoints[i].x+xShift, baseLikelihoodScanPoints[i].y+yShift);
    }

    return likelihood;
}


float ScanLikelihoodGrid::createCoarseGrid(float coarseMetersPerCell, ScanLikelihoodGrid& coarseGrid) const
{
    assert(coarseMetersPerCell > grid.metersPerCell());

    // Adjust the coarse resolution to be a multiple of the fine resolution
    uint8_t fineCellsPerCoarseCell = lrint(coarseMetersPerCell * grid.cellsPerMeter());

    coarseMetersPerCell = grid.metersPerCell() * fineCellsPerCoarseCell;

    uint16_t coarseWidth  = grid.getWidthInCells()/fineCellsPerCoarseCell;
    uint16_t coarseHeight = grid.getHeightInCells()/fineCellsPerCoarseCell;

    coarseGrid.grid.setGridSizeInCells(coarseWidth, coarseHeight);
    coarseGrid.grid.setMetersPerCell(coarseMetersPerCell);
    coarseGrid.grid.reset(0);

    Point<uint16_t> coarsePoint;
    Point<uint16_t> fineGridRegionStart;

    for(coarsePoint.x = 0; coarsePoint.x < coarseWidth; ++coarsePoint.x)
    {
        for(coarsePoint.y = 0; coarsePoint.y < coarseHeight; ++coarsePoint.y)
        {
            fineGridRegionStart.x = coarsePoint.x * fineCellsPerCoarseCell;
            fineGridRegionStart.y = coarsePoint.y * fineCellsPerCoarseCell;

            uint8_t maxCellValue = find_max_cell_value_in_region(fineGridRegionStart, fineCellsPerCoarseCell, grid);

            coarseGrid.grid.setValueNoCheck(coarsePoint.x, coarsePoint.y, maxCellValue);
        }
    }

    coarseGrid.referenceScanOrigin.x = referenceScanOrigin.x / fineCellsPerCoarseCell;
    coarseGrid.referenceScanOrigin.y = referenceScanOrigin.y / fineCellsPerCoarseCell;

    #ifdef DEBUG_COARSE_GRID
    std::cout<<"DEBUG: coarse_grid: Origin: "<<coarseGrid.referenceScanOrigin<<'\n';
    display_likelihood_grid(coarseGrid.grid);
    #endif

    return coarseMetersPerCell;
}


void ScanLikelihoodGrid::setReferenceScanOrigin(const std::vector<Point<float>>& scan)
{
    std::pair<float, float> scanMin = find_scan_min(scan, grid);

    referenceScanOrigin.x = (scanMin.first * grid.cellsPerMeter()) - rasterSize;

    // y value is the min scan value, so shift forward enough in y so this value can be placed on the grid
    referenceScanOrigin.y = (scanMin.second * grid.cellsPerMeter()) - rasterSize;

    #ifdef DEBUG_RASTER
    std::cout<<"DEBUG: ref_scan_origin: scan_range: ("<<scanMin.first<<','<<scanMin.second<<")"
             <<" origin: ("<<referenceScanOrigin<<'\n';
    #endif
}


void ScanLikelihoodGrid::rasterizeScanPoint(const  Point<float>& scanPoint)
{
    Point<int16_t> scanPointInCells;

    scanPointInCells.x = lrint(scanPoint.x*grid.cellsPerMeter()) - referenceScanOrigin.x;
    scanPointInCells.y = lrint(scanPoint.y*grid.cellsPerMeter()) - referenceScanOrigin.y;

    if((scanPointInCells.x < 0 || (static_cast<std::size_t>(scanPointInCells.x) >= grid.getWidthInCells())) ||
        (scanPointInCells.y < 0 || (static_cast<std::size_t>(scanPointInCells.y) >= grid.getHeightInCells())))
    {
        return;
    }

    int16_t minX = std::max(0, scanPointInCells.x - rasterSize/2);
    int16_t minY = std::max(0, scanPointInCells.y - rasterSize/2);

    uint16_t maxX = std::min(static_cast<int>(grid.getWidthInCells()),  minX + rasterSize);
    uint16_t maxY = std::min(static_cast<int>(grid.getHeightInCells()), minY + rasterSize);

    Point<uint16_t> gridCell;

    for(gridCell.x = minX; gridCell.x < maxX; ++gridCell.x)
    {
        for(gridCell.y = minY; gridCell.y < maxY; ++gridCell.y)
        {
            if(scanPointRaster[gridCell.x-minX][gridCell.y-minY] > grid.getValueNoCheck(gridCell.x, gridCell.y))
            {
                grid.setValueNoCheck(gridCell.x, gridCell.y, scanPointRaster[gridCell.x-minX][gridCell.y-minY]);
            }
        }
    }
}


void ScanLikelihoodGrid::createRaster(float variance)
{
    // Raster will extend to three sigmas in a circle around the scan point
    setRasterSize(variance);

    fillRasterArray(variance);

    #ifdef DEBUG_RASTER
    display_raster(scanPointRaster, rasterSize);
    #endif
}


void ScanLikelihoodGrid::setRasterSize(float variance)
{
    float stdDev = sqrt(variance);

    rasterSize = static_cast<uint8_t>((2*stdDev*sqrt(RASTER_MAHALANOBIS_MAX)) * grid.cellsPerMeter()) + 1;

    // The rasterSize always needs to be odd in order to center it at the scan point. If it is
    // even, then there will be a bias in the rasterization
    if(rasterSize % 2 == 0)
    {
        ++rasterSize;
    }

    if(rasterSize > MAX_RASTER_SIZE)
    {
        std::cerr<<"WARNING: ScanLikelihoodGrid: Raster size was larger than MAX_RASTER_SIZE. Consider increasing MAX_RASTER_SIZE constant. Preferred size: "<<(int16_t)rasterSize<<std::endl;

        rasterSize = MAX_RASTER_SIZE;
    }
}


void ScanLikelihoodGrid::fillRasterArray(float variance)
{
    Point<int> rasterCenter(rasterSize/2, rasterSize/2);

    math::UnivariateGaussianDistribution likelihoodDistribution(0, variance);

    // The rasterNormalizer is used to get the likelihood of the mean of the distribution
    // so that the peak of the raster values is MAX_RASTER_VALUE and then falls off accordingly with
    // the value of the Gaussian distribution
    float rasterNormalizer = likelihoodDistribution.likelihood(0);

    for(int x = rasterSize; --x >= 0;)
    {
        for(int y = rasterSize; --y >= 0;)
        {
            float radius = distance_between_points(rasterCenter, Point<int>(x, y));

            radius *= grid.metersPerCell();

            if(mahalanobis(radius, 0, variance) <= RASTER_MAHALANOBIS_MAX)
            {
                scanPointRaster[x][y] = static_cast<uint8_t>(MAX_RASTER_VALUE * (1.0/rasterNormalizer * likelihoodDistribution.likelihood(radius)));
            }
            else
            {
                scanPointRaster[x][y] = 0;
            }
        }
    }
}


std::pair<float, float> find_scan_min(const std::vector<Point<float>>& scan, const utils::CellGrid<uint8_t>& grid)
{
//     rectangle_bounds_t bounds = find_scan_bounds(scan);

    return std::make_pair(-grid.getHeightInCells()*grid.metersPerCell()/2, -grid.getWidthInCells()*grid.metersPerCell()/2);
}


rectangle_bounds_t find_scan_bounds(const std::vector<Point<float>>& scan)
{
    rectangle_bounds_t bounds;

    for(int i = scan.size(); --i >= 0;)
    {
        if(scan[i].x < bounds.minX)
        {
            bounds.minX = scan[i].x;
        }
        if(scan[i].x > bounds.maxX)
        {
            bounds.maxX = scan[i].x;
        }

        if(scan[i].y < bounds.minY)
        {
            bounds.minY = scan[i].y;
        }
        if(scan[i].y > bounds.maxY)
        {
            bounds.maxY = scan[i].y;
        }
    }

    return bounds;
}


uint8_t find_max_cell_value_in_region(const Point<uint16_t>& regionStart, uint8_t regionSize, const utils::CellGrid<uint8_t>& grid)
{
    Point<uint16_t> regionEnd;

    regionEnd.x = std::min(regionStart.x+regionSize, static_cast<int>(grid.getWidthInCells()));
    regionEnd.y = std::min(regionStart.y+regionSize, static_cast<int>(grid.getHeightInCells()));

    uint8_t maxValue = 0;

    Point<uint16_t> pointToCheck;

    for(pointToCheck.x = regionStart.x; pointToCheck.x < regionEnd.x; ++pointToCheck.x)
    {
        for(pointToCheck.y = regionStart.y; pointToCheck.y < regionEnd.y; ++pointToCheck.y)
        {
            if(grid.getValueNoCheck(pointToCheck.x, pointToCheck.y) > maxValue)
            {
                maxValue = grid.getValueNoCheck(pointToCheck.x, pointToCheck.y);
            }
        }
    }

    return maxValue;
}


void display_raster(uint8_t raster[51][51], uint8_t rasterWidth)
{
    std::cout<<"ScanLikelihoodGrid: Raster width: "<<static_cast<uint16_t>(rasterWidth)<<"\n";

    for(int i = 0; i < rasterWidth; ++i)
    {
        for(int j = 0; j < rasterWidth; ++j)
        {
            std::cout<<static_cast<uint16_t>(raster[i][j])<<' ';
        }

        std::cout<<'\n';
    }

    std::cout<<std::endl;
}


void display_likelihood_grid(const utils::CellGrid<uint8_t>& grid)
{
    std::cout<<"ScanLikelihoodGrid: Grid:\n\n";

    Point<uint16_t> cell;

    for(cell.y = grid.getHeightInCells()-1; cell.y > 0; --cell.y)
    {
        for(cell.x = 0; cell.x < grid.getWidthInCells(); ++cell.x)
        {
            std::cout<<std::setw(3)<<static_cast<uint16_t>(grid.getValueNoCheck(cell.x, cell.y))<<' ';
        }

        std::cout<<'\n';
    }

    std::cout<<"\n\n"<<std::endl;
}

} // namespace laser
} // namespace vulcan
