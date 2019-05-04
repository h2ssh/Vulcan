/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     laser_scan_rasterizer.cpp
* \author   Collin Johnson
*
* Definition of LaserScanRasterizer.
*/

#include <hssh/metrical/mapping/laser_scan_rasterizer.h>
#include <core/angle_functions.h>
#include <math/geometry/shape_fitting.h>
#include <core/pose.h>
#include <laser/moving_laser_scan.h>
#include <hssh/metrical/mapping/mapping_params.h>
#include <iostream>
#include <cassert>
#include <cmath>
#include <cstring>

// #define DEBUG_BOUNDARY

namespace vulcan
{
namespace hssh
{

// max_range_interval_t is used for filling in the gaps for max-range readings
struct max_range_interval_t
{
    float startRange = 0.0f;
    float rangeStep = 0.0f;

    std::size_t beginIndex = 0;
    std::size_t endIndex = 0;
};


// Helpers for doing the ray tracing
max_range_interval_t find_max_range_interval(std::size_t                   startIndex,
                                             const laser::MovingLaserScan& scan,
                                             float                         maxDistance);


LaserScanRasterizer::LaserScanRasterizer(float cellScale, const laser_scan_rasterizer_params_t& params)
: raster(cellScale)
, params(params)
{
    int rasterWidth  = 2 * ceil(params.maxLaserDistance / cellScale);//+1?
    int rasterHeight = 2 * ceil(params.maxLaserDistance / cellScale);

    assert(rasterWidth * rasterHeight > 0);

    raster.setGridSizeInCells(rasterWidth, rasterHeight);
}


void LaserScanRasterizer::rasterizeScan(const laser::MovingLaserScan& scan,
                                        const Point<float>&     lpmReferenceFrame,
                                        bool                          initialUpdate)
{
    initializeUpdate(initialUpdate);
    adjustMaxDistanceRanges(scan);
    calculateScanBoundary(scan, lpmReferenceFrame);
    updateBufferWithLaserData(scan);
}


void LaserScanRasterizer::initializeUpdate(bool initialUpdate)
{

    // The initial update will burn the state into the map, thereby allowing immediate localization without
    // a warmup period for the LPM
    if(initialUpdate)
    {
        occupiedCellCost = params.initialOccupiedCostChange;
        freeCellCost     = params.initialFreeCostChange;
    }
    else
    {
        occupiedCellCost = params.occCellCostChange;
        freeCellCost     = params.freeCellCostChange;
    }
}


void LaserScanRasterizer::adjustMaxDistanceRanges(const laser::MovingLaserScan& scan)
{
    max_range_interval_t interval;
    adjustedRanges_.resize(scan.size());

    for(std::size_t n = 0; n < scan.size(); ++n)
    {
        if(scan[n].range <= params.maxLaserDistance)
        {
            adjustedRanges_[n] = scan[n].range;
        }
        else // if(scan[n].range > params.maximumUpdateDistance)
        {
            // If n isn't within the current range, then a new one needs to be created
            if((n < interval.beginIndex) || (n > interval.endIndex))
            {
                interval = find_max_range_interval(n, scan, params.maxLaserDistance);
            }

            adjustedRanges_[n] = interval.startRange + ((n - interval.beginIndex) * interval.rangeStep);
        }
    }
}


void LaserScanRasterizer::calculateScanBoundary(const laser::MovingLaserScan& scan, 
                                                const Point<float>&     lpmReferenceFrame)
{
    // The bottomLeft and topRight both need to be set to the laser position because all rays start at the laser, so it 
    // must included in the raster, even if all rays only go in front or behind it due to field-of-view
    Point<float> bottomLeft = scan[0].position;
    Point<float> topRight   = scan[0].position;

    for(std::size_t n = 0; n < scan.size(); ++n)
    {
        // Skip invalid ranges
        if((scan[n].range < 0.0f) || (scan[n].range > params.maxLaserDistance))
        {
            continue;
        }
        
        auto scanPoint = scan[n].endpoint;

        if(scanPoint.x < bottomLeft.x)
        {
            bottomLeft.x = scanPoint.x;
        }

        if(scanPoint.y < bottomLeft.y)
        {
            bottomLeft.y = scanPoint.y;
        }

        if(scanPoint.x > topRight.x)
        {
            topRight.x = scanPoint.x;
        }

        if(scanPoint.y > topRight.y)
        {
            topRight.y = scanPoint.y;
        }
    }

    // After the coordinates are found, expand the dimensions by one cell's width to ensure that truncation or
    // other float->int conversions don't eliminate a scan from the map
    bottomLeft.x -= raster.metersPerCell();
    bottomLeft.y -= raster.metersPerCell();
    topRight.x   += raster.metersPerCell();
    topRight.y   += raster.metersPerCell();

    // Find the tile position of the raster bottom left in the LPM's reference frame
    int tileX = std::floor((bottomLeft.x - lpmReferenceFrame.x) * raster.tilesPerMeter());
    int tileY = std::floor((bottomLeft.y - lpmReferenceFrame.y) * raster.tilesPerMeter());
    
    // Set the bottom left of the raster to be relative to the lpm reference frame to ensure the scan raster cells
    // are exactly aligned to the LPM's cells. If they are slightly out of alignment, mayhem ensues
    // Implicit conversion does the desired truncation of the floating point value
    // which is why this calculation changes the position of the bottom left
    Point<float> updatedBottom(lpmReferenceFrame.x + (tileX * raster.metersPerTile()), 
                                     lpmReferenceFrame.y + (tileY * raster.metersPerTile()));

    // The updatedBottom might end up shifted by a cell. If so, then shift one cell to the left to make sure
    // all the rays fit properly.
    if(updatedBottom.x > bottomLeft.x)
    {
        updatedBottom.x -= raster.metersPerTile();
    }

    if(updatedBottom.y > bottomLeft.y)
    {
        updatedBottom.y -= raster.metersPerTile();
    }

    assert(updatedBottom.x <= bottomLeft.x);
    assert(updatedBottom.y <= bottomLeft.y);

    raster.setBottomLeft(updatedBottom);
    raster.setGridSizeInCells(ceil((topRight.x - updatedBottom.x) * raster.cellsPerMeter()),
                              ceil((topRight.y - updatedBottom.y) * raster.cellsPerMeter()));
    raster.reset(0);
    

#ifdef DEBUG_BOUNDARY
    std::cout << "LaserScanRasterizer:Boundary:" << raster.getBoundary() << " Width:" << raster.getWidthInCells() 
              << " Height:" << raster.getHeightInCells() << '\n';
#endif
}


void LaserScanRasterizer::updateBufferWithLaserData(const laser::MovingLaserScan& scan)
{
    /*
    * The grid and the update grid need to have the same center. If they don't have the same center, then the 
    * discretization of map cells will cause discretization errors that are worse than just the normal discretization 
    * error that is expected with the lpm. Don't want to compound things and make them worse than they already are.
    */

    max_range_interval_t interval;

    for(std::size_t i = 0; i < scan.size(); ++i)
    {
        auto laserCell = utils::global_point_to_grid_point(scan[i].position, raster);
        
        if((scan[i].range > 0) && (scan[i].range <= params.maxLaserDistance))
        {
            scoreRay(scan[i].range, angle_to_point(scan[i].position, scan[i].endpoint), laserCell);
        }
        else if(scan[i].range > params.maxLaserDistance)
        {
            scoreRay(adjustedRanges_[i], angle_to_point(scan[i].position, scan[i].endpoint), laserCell);
        }
    }

    for(std::size_t i = 0; i < scan.size(); ++i)
    {
        if((scan[i].range > 0.1) && (scan[i].range <= params.maxLaserDistance))
        {
            scoreEndpoint(scan[i].endpoint);
        }
    }
}


void LaserScanRasterizer::scoreRay(float radius, float angle, const Point<float>& laserCell)
{
    // Note that the laserCell is a float. This allows the scanner to be placed anywhere within the grid cell. This
    // is important because it can help avoid small discrepencies in the map from occurring because the summation might hit
    // different cells with the scale of the map
    // Coarsely trace the ray based on the cell scale. This update WILL miss some cells, but they would be less important
    // anyway as the beam only glances by them.
    // Everything until the final discretized position is going to be considered free space
    
    int numIncrements = static_cast<int>(radius * raster.cellsPerMeter()) + 1;

    // If the radius is too long, then just go until the update distance threshold
    if(radius > params.maxLaserDistance)
    {
        numIncrements = static_cast<int>(params.maxLaserDistance * raster.cellsPerMeter());
    }

    // Just using the cos/sin here because each cell size is 1 as the math here happens in cell-space
    const double deltaX = std::cos(angle);
    const double deltaY = std::sin(angle);
    Point<int> rayCell;

    ////////////////////////////////
    // NOTE: This truncation approach only works because grid coordinates are always positive. If grid coordinates go
    // negative, then a floor function needs to be used to perform correct rounding
    ////////////////////////////////
    for(int i = 0; i < numIncrements; ++i)
    {
        rayCell.x = static_cast<int>(laserCell.x + i*deltaX);
        rayCell.y = static_cast<int>(laserCell.y + i*deltaY);

        // Only mark a cell as free if it hasn't already been marked as occupied because the occupied
        // cells shouldn't be erased - they take precedence
        if(raster.isCellInGrid(rayCell))
        {
            const auto minValue = std::numeric_limits<int8_t>::min();

            auto& value = raster(rayCell.x, rayCell.y);
            if(value >  minValue + freeCellCost)
            {
                value -= freeCellCost;
            }
            else
            {
                value = minValue;
            }
        }
    }
}


void LaserScanRasterizer::scoreEndpoint(const Point<float>& endpoint)
{
    auto cell = utils::global_point_to_grid_cell(endpoint, raster);

    if(raster.isCellInGrid(cell.x, cell.y))
    {
        auto& value = raster(cell.x, cell.y);

        if(value < 0)
        {
            value = occupiedCellCost;
        }
        else
        {
            const auto maxValue = std::numeric_limits<int8_t>::max();

            if(value < maxValue - occupiedCellCost)
            {
                value += occupiedCellCost;
            }
            else
            {
                value = maxValue;
            }
        }
    }
}


max_range_interval_t find_max_range_interval(std::size_t                   startIndex, 
                                             const laser::MovingLaserScan& scan, 
                                             float                         maxDistance)
{
    max_range_interval_t interval;

    interval.beginIndex = startIndex;

    std::size_t endIndex = startIndex;

    while((endIndex < scan.size()) && (scan[endIndex].range > maxDistance))
    {
        ++endIndex;
    }

    interval.endIndex = endIndex;

    // Must be at least one valid reading, otherwise crash HARD
    assert((interval.beginIndex) > 0 || (interval.endIndex+1 < scan.size()));

    // Find the ranges that border the interval. Take the minimum as it is a more conservative estimate
    // which really ends up being important
    float beginRange = (interval.beginIndex > 0) ? scan[interval.beginIndex-1].range : scan[interval.endIndex].range;
    float endRange   = (interval.endIndex < scan.size()) ? scan[interval.endIndex].range : beginRange;
    
    beginRange = std::min(beginRange, endRange);
    endRange = beginRange;

    beginRange = std::max(beginRange - 0.5f, 0.9f * beginRange);
    endRange = std::max(endRange - 0.5f, 0.9f * endRange);

    interval.startRange = beginRange;
    
    // Setup the linear interpolation parameter for the max range interval
    if(endIndex > startIndex)
    {
        interval.rangeStep = (endRange - beginRange) / (interval.endIndex - interval.beginIndex);
    }
    
    return interval;
}

} // namespace hssh
} // namespace vulcan
