/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     glass_map.cpp
* \author   Collin Johnson
*
* Definition of GlassMap.
*/

#include "hssh/metrical/glass_map.h"
#include "hssh/metrical/mapping/glass_map_utils.h"
#include "utils/timestamp.h"
#include <boost/algorithm/clamp.hpp>
#include <boost/range/iterator_range.hpp>
#include <cstdlib>

// #define DEBUG_REGION_FILTER

namespace vulcan
{
namespace hssh
{

const int kNumAngleBins = 1440;
const double kDefaultMaxRange = 3.0;
const cell_type_t kHitCellTypes = kOccupiedOccGridCell | kDynamicOccGridCell | kQuasiStaticOccGridCell;
const int8_t kMaxHit = 127;
const int8_t kMaxMiss = -127;


inline int round_to_multiple_of_16(int value)
{
    // Integer division will toss out any remainder, then mulitply by 16 gets back to nearest multiple of 16
    return (value / 16) * 16;
}


GlassMap::GlassMap(void)
: numAngleBins_(kNumAngleBins)
, hitGrid_("hit", 0.314, kDefaultMaxRange, numAngleBins_)
, flattenedGrid_(100, 100, 0.314, Point<float>(0.0f, 0.0f), 200, 0)
, hitCount_(100, 100, 0.314, Point<float>(0.0f, 0.0f), 0)
, missCount_(100, 100, 0.314, Point<float>(0.0f, 0.0f), 0)
, intensityGrid_(100, 100, 0.314, Point<float>(0.0f, 0.0f), 0)
{
    assert(kNumAngleBins == round_to_multiple_of_16(kNumAngleBins));
    setGridOffset();
}


GlassMap::GlassMap(float                     maxLaserRange,
                   int                       numAngleBins,
                   int8_t                    flattenHitThreshold,
                   int8_t                    flattenMissThreshold,
                   std::size_t               gridWidth,
                   std::size_t               gridHeight,
                   float                     gridScale,
                   const Point<float>& globalCenter)
: numAngleBins_(round_to_multiple_of_16(numAngleBins))
, hitThreshold_(std::max(int8_t(1), flattenHitThreshold))
, missThreshold_(std::min(int8_t(-1), flattenMissThreshold))
, hitGrid_(gridWidth, gridHeight, gridScale, globalCenter, maxLaserRange, numAngleBins_, "hit")
, flattenedGrid_(gridWidth, gridHeight, gridScale, globalCenter, 200, 0)
, hitCount_(gridWidth, gridHeight, gridScale, globalCenter, 0)
, missCount_(gridWidth, gridHeight, gridScale, globalCenter, 0)
, intensityGrid_(gridWidth, gridHeight, gridScale, globalCenter, 0)
{
    std::cout << "Created Glass Map with hit threshold " << static_cast<int>(flattenHitThreshold)
        << " and miss threshold " << static_cast<int>(flattenMissThreshold) << '\n';

    setGridOffset();
}


void GlassMap::setGridSizeInCells(std::size_t width, std::size_t height)
{
    flattenedGrid_.setGridSizeInCells(width, height);
    hitCount_.setGridSizeInCells(width, height);
    missCount_.setGridSizeInCells(width, height);
    intensityGrid_.setGridSizeInCells(width, height);

    setGridOffset();
}


void GlassMap::setMetersPerCell(float gridScale)
{
    flattenedGrid_.setMetersPerCell(gridScale);
    hitCount_.setMetersPerCell(gridScale);
    missCount_.setMetersPerCell(gridScale);
    intensityGrid_.setMetersPerCell(gridScale);
}


void GlassMap::setBottomLeft(const Point<float>& bottomLeft)
{
    flattenedGrid_.setBottomLeft(bottomLeft);
    hitCount_.setBottomLeft(bottomLeft);
    missCount_.setBottomLeft(bottomLeft);
    intensityGrid_.setBottomLeft(bottomLeft);

    setGridOffset();
}


void GlassMap::changeBoundary(const math::Rectangle<float>& newBoundary)
{
    flattenedGrid_.changeBoundary(newBoundary);
    hitCount_.changeBoundary(newBoundary, 0);
    missCount_.changeBoundary(newBoundary, 0);
    intensityGrid_.changeBoundary(newBoundary, 0);
    hitGrid_.changeBoundary(newBoundary);
    setGridOffset();
}


void GlassMap::recenterActiveRegion(const Point<float>& position)
{
    hitGrid_.recenterActiveRegion(position);
}


void GlassMap::reset(void)
{
    flattenedGrid_.reset();
    hitCount_.reset(0);
    missCount_.reset(0);
    intensityGrid_.reset(0);
    hitGrid_.reset();
}


bool GlassMap::haveOverlappingAngles(int x1, int y1, int x2, int y2) const
{
    // When searching, looking for matches in a slightly wider region than cell-to-cell +/-1 cell will make the match
    // more robust to localization noise
    auto firstIt = beginBin(x1, y1);
    auto firstEnd = endBin(x1, y1);
    auto secondIt = beginBin(x2, y2);
    auto secondEnd = endBin(x2, y2);

    if ((isHit(*firstIt) && isHit(*(secondEnd - 1))) || (isHit(*(firstEnd - 1)) && isHit(*secondIt)))
    {
        return true;
    }

    for(; (firstIt != firstEnd) && (secondIt != secondEnd); ++firstIt, ++secondIt)
    {
        if(isHit(*firstIt) && isHit(*secondIt))
//             || ((*(firstIt - 1) >= hitThreshold_) && (*secondIt >= hitThreshold_))
//             || ((*(firstIt + 1) >= hitThreshold_) && (*secondIt >= hitThreshold_)))
        {
            return true;
        }
    }

    firstIt = beginBin(x1, y1);
    firstEnd = endBin(x1, y1);
    secondIt = beginBin(x2, y2);
    secondEnd = endBin(x2, y2);
    ++secondIt;

    for(; (firstIt != firstEnd) && (secondIt != secondEnd); ++firstIt, ++secondIt)
    {
        if(isHit(*firstIt) && isHit(*secondIt))
        {
            return true;
        }
    }

    firstIt = beginBin(x1, y1);
    firstEnd = endBin(x1, y1);
    secondIt = beginBin(x2, y2);
    secondEnd = endBin(x2, y2);
    ++firstIt;

    for(; (firstIt != firstEnd) && (secondIt != secondEnd); ++firstIt, ++secondIt)
    {
        if(isHit(*firstIt) && isHit(*secondIt))
        {
            return true;
        }
    }

    return false;
}


int GlassMap::angleToBin(double angle) const
{
    assert(angle >= 0.0);
    if(angle >= 2*M_PI)
    {
        angle -= 2*M_PI;
    }
    assert(angle < 2*M_PI);

    constexpr double kOneOverTwoPi = 1.0 / (2 * M_PI);
    return angle * numAngleBins_ * kOneOverTwoPi;
}


double GlassMap::binToAngle(int bin) const
{
    assert(bin >= 0);
    assert(bin < static_cast<int>(numAngleBins_));

    const double kBinAngleWidth = 2 * M_PI / numAngleBins_;
    return bin * kBinAngleWidth;
}


bool GlassMap::observedCell(int x, int y, int angleBin, uint16_t intensity)
{
    auto flatCell = glassToFlatMap(x, y);
    if(intensityGrid_.isCellInGrid(flatCell))
    {
        intensityGrid_(flatCell.x, flatCell.y) = std::max(intensityGrid_(flatCell.x, flatCell.y), intensity);
    }

    haveIntensity_ |= intensity > 0;    // any intensity measurement clamps to using intensity for the map

    if(hitGrid_.isCellInActiveRegion(x, y) && (hitGrid_(x, y, angleBin) < kMaxHit))
    {
        // Once a miss, always a miss. Otherwise if not below that threshold yet, see what we can see.
        if(hitGrid_(x, y, angleBin) > kMaxMiss)
        {
            ++hitGrid_(x, y, angleBin);
            // If the threshold is hit then we've switched to occupied
            if((hitGrid_(x, y, angleBin) == hitThreshold_)
                && hitCount_.isCellInGrid(flatCell))
            {
                ++hitCount_(flatCell.x, flatCell.y);
                return true;
            }
            // Had been a miss, but now has switched to unknown
            else if((hitGrid_(x, y, angleBin) == (missThreshold_ + 1)) && (missCount_.isCellInGrid(flatCell)))
            {
                --missCount_(flatCell.x, flatCell.y);
            }
        }
    }

    return false;
}


bool GlassMap::missedCell(int x, int y, int angleBin)
{
    if(hitGrid_.isCellInActiveRegion(x, y) && (hitGrid_(x, y, angleBin) > kMaxMiss))
    {
        --hitGrid_(x, y, angleBin);
        // If the value slips one below then it has changed from occupied to free
        auto flatCell = glassToFlatMap(x, y);
        if((hitGrid_(x, y, angleBin) == (hitThreshold_ - 1)) && hitCount_.isCellInGrid(flatCell))
        {
            --hitCount_(flatCell.x, flatCell.y);
            return true;
        }
        else if((hitGrid_(x, y, angleBin) == missThreshold_) && (missCount_.isCellInGrid(flatCell)))
        {
            ++missCount_(flatCell.x, flatCell.y);
        }
    }

    return false;
}


void GlassMap::reflectedCell(int x, int y, int angleBin)
{
    if(hitGrid_.isCellInActiveRegion(x, y) && (hitGrid_(x, y, angleBin) > kMaxMiss))
    {
        // If the value was a hit and we're resetting it, then reduce the hit count
        auto flatCell = glassToFlatMap(x, y);
        if((hitGrid_(x, y, angleBin) >= hitThreshold_) && hitCount_.isCellInGrid(flatCell))
        {
            --hitCount_(flatCell.x, flatCell.y);
        }
        else if((hitGrid_(x, y, angleBin) <= missThreshold_) && missCount_.isCellInGrid(flatCell))
        {
            --missCount_(flatCell.x, flatCell.y);
        }
        hitGrid_(x, y, angleBin) = 0;
    }
}


void GlassMap::flattenActiveRegion(int minVisibleRange)
{
    // Get the active region in which changes might have occurred and flatten just those
    flattenRegion(active_region_in_flat_map(*this), minVisibleRange);
}


void GlassMap::flattenFullMap(int minVisibleRange)
{
    math::Rectangle<int> fullMapRegion(Point<int>(0, 0),
                                       Point<int>(hitCount_.getWidthInCells(), hitCount_.getHeightInCells()));
    flattenRegion(fullMapRegion, minVisibleRange);
}


void GlassMap::filterDynamicObjectsFromActiveRegion(int minHighlyVisibleRange, uint16_t minGlassIntensity)
{
    // To filter, change all occupied cells to dynamic. Flip cells back to occupied after they confirmed to be
    // attached to a highly visible cell
    minHighlyVisibleRange = std::max(minHighlyVisibleRange, 1); // must be visible from at least one angle

    int64_t computationStartTimeUs = utils::system_time_us();

    auto activeRegion = activeRegionInCells();
    auto flatActiveRegion = active_region_in_flat_map(*this);

    std::deque<Point<int>> searchQueue;

    // Set all occupied and dynamic to be dynamic, which allows easily knowing if a particular cell has been visited
    // or not
    for(int y = flatActiveRegion.bottomLeft.y; y < flatActiveRegion.topRight.y; ++y)
    {
        for(int x = flatActiveRegion.bottomLeft.x; x < flatActiveRegion.topRight.x; ++x)
        {
            if(flattenedGrid_.getCellTypeNoCheck(x, y) & kHitCellTypes)
            {
                flattenedGrid_.setTypeNoCheck(Point<int>(x, y), kDynamicOccGridCell);
            }
        }
    }

    // For all highly visible cells, add them to the initial search queue and flip them back to being occupied
    // because the search starts here
    for(int y = flatActiveRegion.bottomLeft.y; y < flatActiveRegion.topRight.y; ++y)
    {
        for(int x = flatActiveRegion.bottomLeft.x; x < flatActiveRegion.topRight.x; ++x)
        {
            // If the intensity is greater than the min glass intensity, definitely glass
            // If the hit count is higher than the min visible range, definitely highly visible
            // If there's been a hit and no misses, then the hit count might be highly visible, so mark as occupied
            if((haveIntensity_ && (intensityGrid_(x, y) > minGlassIntensity) && (hitCount_(x, y) > 0))
                || (!haveIntensity_ && (hitCount_(x, y) >= minHighlyVisibleRange))
                || ((hitCount_(x, y) > 0) && (missCount_(x, y) == 0)))
            {
                searchQueue.push_back(Point<int>(x, y));
                flattenedGrid_.setTypeNoCheck(Point<int>(x, y), kOccupiedOccGridCell);
            }
        }
    }

    // Use an 8-way search
    const int xDeltas[8] = { 1, -1, 0, 0, -1, -1, 1, 1 };
    const int yDeltas[8] = { 0, 0, 1, -1, -1, 1, -1, 1 };

    // Queue up cells that sit right outside the active region. If any of them are occupied and are adjacent in (x,y)
    // to a cell in the active region, then add that active region cell to the queue. This change causes the walls saved
    // during a previous dynamic search to still exist even when they fall outside the active region
    for(int y = flatActiveRegion.bottomLeft.y-1; y <= flatActiveRegion.topRight.y; ++y)
    {
        for(int x = flatActiveRegion.bottomLeft.x-1; x <= flatActiveRegion.topRight.x; ++x)
        {
            // Ignore cells that...
            if(is_cell_in_region(Point<int>(x, y), flatActiveRegion)  // are in the active region
                || !flattenedGrid_.isCellInGrid(Point<int>(x, y))     // aren't in the flat map
                || (flattenedGrid_.getCellTypeNoCheck(x, y) != kOccupiedOccGridCell))   // aren't occupied
            {
                continue;
            }

            for(int n = 0; n < 8; ++n)
            {
                Point<int> adjacentCell(x + xDeltas[n], y + yDeltas[n]);
                // Ensure the cell is in the active region and it is dynamic -- if so, then it put it on the queue
                if(is_cell_in_region(adjacentCell, flatActiveRegion)
                    && flattenedGrid_.getCellType(adjacentCell) == kDynamicOccGridCell)
                {
                    flattenedGrid_.setTypeNoCheck(adjacentCell, kOccupiedOccGridCell);
                    searchQueue.push_back(adjacentCell);
                }
            }
        }
    }

    while(!searchQueue.empty())
    {
        auto cell = searchQueue.front();
        searchQueue.pop_front();

        auto angleCell = cell - glassToFlatCellOffset_;

        for(int n = 0; n < 8; ++n)
        {
            Point<int> adjacentCell(cell.x + xDeltas[n], cell.y + yDeltas[n]);
            auto adjacentAngleCell = adjacentCell - glassToFlatCellOffset_;
            // Ensure the cell is in the map
            if(!is_cell_in_region(adjacentCell, flatActiveRegion)
                || !is_cell_in_region(adjacentAngleCell, activeRegion))
            {
                continue;
            }

            // Skip anything that isn't dynamic because it is either free or already be identified in the search.
            if(flattenedGrid_.getCellTypeNoCheck(adjacentCell) != kDynamicOccGridCell)
            {
                continue;
            }

            // If the cells are also adjacent across angles, then add the cell to the queue and mark as part of
            // the occupied portion of the environment.
            if(haveOverlappingAngles(angleCell.x, angleCell.y, adjacentAngleCell.x, adjacentAngleCell.y))
            {
                flattenedGrid_.setTypeNoCheck(adjacentCell, kOccupiedOccGridCell);
                searchQueue.push_back(adjacentCell);
            }
        }
    }

    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    std::cout << "Filtering time: " << computationTimeUs << "us\n";
}


void GlassMap::filterDynamicObjectsFromFullMap(int minHighlyVisibleRange, uint16_t minGlassIntensity)
{
    // Find the number of x and y increments in the active regions to fit the whole map
    // If the radius is larger than the grid, make the increment just the center of the grid
    const double xRegionIncrement = (hitGrid_.activeRegionRadius() < hitGrid_.getWidthInMeters()) ?
        hitGrid_.activeRegionRadius() : hitGrid_.getWidthInMeters() / 2.0;
    // Always at least one increment
    const int numXIncrements = static_cast<int>(hitGrid_.getWidthInMeters() / hitGrid_.activeRegionRadius()) + 1;

    // If the radius is larger than the grid, make the increment just the center of the grid
    const double yRegionIncrement = (hitGrid_.activeRegionRadius() < hitGrid_.getHeightInMeters()) ?
        hitGrid_.activeRegionRadius() : hitGrid_.getHeightInMeters() / 2.0;
    // Always at least one increment
    const int numYIncrements = static_cast<int>(hitGrid_.getHeightInMeters() / hitGrid_.activeRegionRadius()) + 1;

    // Before beginning to shift, save the original active center so it can be restored at the end
    auto originalRegionCenter = hitGrid_.activeRegionCenter();

#ifdef DEBUG_REGION_FILTER
    std::cout << "DEBUG: GlassMap::filterDynamicObjectsFromFullMap: Num increments (x,y): ("
        << numXIncrements << ',' << numYIncrements << ") Increment size: (" << xRegionIncrement << ','
        << yRegionIncrement << ")\n";
#endif

    // Create all the regions and filter one-by-one
    for(int y = 0; y < numYIncrements; ++y)
    {
        Point<float> activeCenter = hitGrid_.getBottomLeft();
        activeCenter.y += (y + 1) * yRegionIncrement; // add one to account for shifting from bottom left to center

        for(int x = 0; x < numXIncrements; ++x)
        {
            activeCenter.x += xRegionIncrement;
            hitGrid_.recenterActiveRegion(activeCenter);

#ifdef DEBUG_REGION_FILTER
            std::cout << "Filtering region: Center:" << activeCenter << " Boundary:"
                << hitGrid_.getActiveRegion() << '\n';
#endif

            filterDynamicObjectsFromActiveRegion(minHighlyVisibleRange, minGlassIntensity);
        }
    }

    // Restore the original active region
    hitGrid_.recenterActiveRegion(originalRegionCenter);
}


void GlassMap::setGridOffset(void)
{
    // The glass map and flattened grid likely don't have the exact same boundary, since one
    // snaps to tiles not cells. This offset will align the flattened and glass cells for storage.
    auto glassToFlatOffset = hitGrid_.getBottomLeft() - flattenedGrid_.getBottomLeft();
    glassToFlatCellOffset_.x = std::lrint(glassToFlatOffset.x * flattenedGrid_.cellsPerMeter());
    glassToFlatCellOffset_.y = std::lrint(glassToFlatOffset.y * flattenedGrid_.cellsPerMeter());
}


Point<int> GlassMap::glassToFlatMap(int x, int y)
{
    return Point<int>(x + glassToFlatCellOffset_.x, y + glassToFlatCellOffset_.y);
}


void GlassMap::flattenRegion(math::Rectangle<int> boundaryInCells, int minVisibleRange)
{
    minVisibleRange = std::max(minVisibleRange, 1);             // must be visible from at least one angle

    int64_t computationStartTimeUs = utils::system_time_us();

    const int kHitIncrement = 128 / 5;

    for(int y = boundaryInCells.bottomLeft.y; y < boundaryInCells.topRight.y; ++y)
    {
        for(int x = boundaryInCells.bottomLeft.x; x < boundaryInCells.topRight.x; ++x)
        {
            uint8_t prevCost = flattenedGrid_.getCostNoCheck(Point<int>(x, y));
            // If there's a hit, then mark full odds.
            // If there's a miss, then mark no odds.
            // Otherwise, leave it as unknown.
            uint8_t cost = 127;

            if(hitCount_(x, y) >= minVisibleRange)
            {
                cost = (hitCount_(x, y) * kHitIncrement > 128) ? 255 : 127 + hitCount_(x, y) * kHitIncrement;
            }
            else if(missCount_(x, y) >= minVisibleRange)
            {
                cost = 0;
            }

            // If the cost is unchanged, then no need to do anything
            if(prevCost != cost)
            {
                cell_type_t type = flattenedGrid_.getCellTypeNoCheck(x, y);
                flattenedGrid_.setCostNoCheck(Point<int>(x, y), cost);

                // Keep the type for occupied cells the same so as not to erase the results of the dynamic object filter
                if(cost > 127)
                {
                    type = (type & kHitCellTypes) ? type : kOccupiedOccGridCell;    // if type not set, just go with occupied
                    flattenedGrid_.setTypeNoCheck(Point<int>(x, y), type);
                }
                // If the cost has gone to 127, then set to unknown. Otherwise removed reflections get marked dynamic
                else if(cost == 127)
                {
                    flattenedGrid_.setTypeNoCheck(Point<int>(x, y), kUnknownOccGridCell);
                }
                else // if(cost < 127)
                {
                    flattenedGrid_.setTypeNoCheck(Point<int>(x, y), kFreeOccGridCell);
                }

                assert((cost < 128) || (flattenedGrid_.getCellTypeNoCheck(x, y) & kHitCellTypes));
            }
        }
    }

    int64_t computationTimeUs = utils::system_time_us() - computationStartTimeUs;
    std::cout << "Flattening time: " << computationTimeUs << "us\n";
}


void GlassMap::flushCacheToDisk(void) const
{
    hitGrid_.flush();
}

} // namespace hssh
} // namespace vulcan
