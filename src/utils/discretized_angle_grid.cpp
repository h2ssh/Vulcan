/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     discretized_angle_grid.cpp
* \author   Paul Foster and Collin Johnson
*
* Definition of DiscretizedAngleGrid.
*/

#include <utils/discretized_angle_grid.h>
#include <utils/algorithm_ext.h>
#include <utils/compression.h>
#include <utils/stub.h>
#include <utils/timestamp.h>
#include <boost/filesystem.hpp>
#include <boost/algorithm/clamp.hpp>
#include <algorithm>
#include <sstream>
#include <cassert>
#include <cstdio>
#include <cstring>

// #define DEBUG_BOUNDARY
// #define DEBUG_TILES

namespace vulcan
{
namespace utils
{

const int kTileSize = 16;
const std::string kTileBaseDirectory = "tiles/";

std::string tile_name(const std::string& gridName, Point<double> tilePosition);


const DiscretizedAngleGrid::BinIndex DiscretizedAngleGrid::kUnassignedBin = -1;


DiscretizedAngleGrid::DiscretizedAngleGrid(std::string name,
                                           double metersPerCell,
                                           double activeRadius,
                                           std::size_t numAngleBins)
: DiscretizedAngleGrid(100, 100, metersPerCell, Point<float>(0, 0), activeRadius, numAngleBins, name)
{
}


DiscretizedAngleGrid::DiscretizedAngleGrid(std::size_t widthInCells,
                                           std::size_t heightInCells,
                                           double metersPerCell,
                                           Point<double> globalCenter,
                                           double activeRadius,
                                           std::size_t numAngleBins,
                                           std::string name)
: name_(name)
, kTileWidth_(kTileSize)
, kAngleBinsPerCell_(numAngleBins)
, kAngleBinsPerTile_(kTileWidth_ * kTileWidth_ * kAngleBinsPerCell_)
, cellsPerMeter_(1.0 / metersPerCell)
, metersPerCell_(metersPerCell)
, cells_(widthInCells, heightInCells, metersPerCell, globalCenter, kUnassignedBin)
{
    // Increase the active radius by 3 tile widths because the robot just stays within 1 tile of the center, so need
    // the extra amount to ensure the desired active radius is always valid and the laser pokes up a litle from where
    // the robot center is
    const int kActiveRadiusBuffer = 3;

    assert(metersPerCell_ > 0.0);
    assert(kTileSize == cells_.getTileSize());

    cellsPerMeter_ = 1.0 / metersPerCell_;

    // Determine the number of bins that need to be allocated.
    // The activeRadius is the width in tiles. The width of the region is 2*radius + 1 to give a little slop, plus
    // account for the robot's position. Each tile contains tilewidth * tilewidth cells, so tilewidth^2 * anglebins
    // are needed for each tile.
    activeRegionRadius_ = (round_up_to_nearest_n(activeRadius * cellsPerMeter_, kTileWidth_) / kTileWidth_)
        + kActiveRadiusBuffer;
    int maxActiveWidth = (2 * activeRegionRadius_) + 1;

    int numActiveTiles = (maxActiveWidth + 1) * (maxActiveWidth + 1);
    angleBins_.resize(numActiveTiles * kAngleBinsPerTile_, 0);

    // Create available bins for each tile
    for(int n = 0; n < numActiveTiles; ++n)
    {
        availableBins_.push_back(n * kAngleBinsPerTile_);
    }

    activeCenter_ = globalCenter;
    activeRegion_ = calculateActiveRegion(globalCenter);
    setActiveBins();
}


void DiscretizedAngleGrid::changeBoundary(const math::Rectangle<double>& newBoundary)
{
    // When changing the boundary, need to worry if the active region changes. All other cells are just marked as
    // unavailable, so their shifting doesn't affect the map. If an active cells shifts of the map, then its bin needs
    // to be added back to the available bins

    if(newBoundary == cells_.getBoundary())
    {
        return;
    }

    // In the easy case, the active region is wholely contained in the new boundary, so the shift doesn't change which
    // bins are available, so everything just works
    // Otherwise, need to save the current active bins and after changing the boundary, determine which ones have
    // fallen off the map and make them available again

    using ActiveTileBin = std::pair<Point<int>, int64_t>;
    std::vector<ActiveTileBin> activeBins;
    for(auto activeIt = beginActiveTiles(), activeEnd = endActiveTiles(); activeIt != activeEnd; ++activeIt)
    {
        activeBins.emplace_back(Tile(activeIt.x(), activeIt.y()), *activeIt);
    }

    auto oldBottomLeft = cells_.getBottomLeft();

    cells_.changeBoundary(newBoundary, kUnassignedBin);

    auto gridShift = oldBottomLeft - cells_.getBottomLeft();
    int xShift = std::round(gridShift.x * cells_.tilesPerMeter());
    int yShift = std::round(gridShift.y * cells_.tilesPerMeter());

    for(auto& a : activeBins)
    {
        if((a.first.x + xShift < 0) || (a.first.x + xShift >= static_cast<int>(cells_.getWidthInTiles()))
            || (a.first.y + yShift < 0) || (a.first.y + yShift >= static_cast<int>(cells_.getHeightInTiles())))
        {
            assert(a.second != kUnassignedBin);
            availableBins_.push_back(a.second);
            clearBin(a.second);
        }

        int numSame = std::count_if(activeBins.begin(), activeBins.end(), [a](ActiveTileBin b) {
            return b.second == a.second;
        });

        if(numSame != 1)
        {
            std::cout << "ERROR: DiscretizedAngleGrid: Duplicate bin assigned:" << a.first << "->"
                << a.second << '\n';
            assert(numSame == 1);
        }
    }

    // After the boundary change, the active region needs to be changed to reflect the new positioning within the map
    // It doesn't need to be recentered through because the same information is stored, just in slightly different
    // position, cell-wise
    activeRegion_ = calculateActiveRegion(activeCenter_);

    // Sort bins so close tiles are as close in memory as possible
    std::sort(availableBins_.begin(), availableBins_.end());

    // If the active region has grown, then some tiles will need to be assigned a bin. Can't use recenter because
    // it will also try save the non-existent (now) cells to file.
    for(auto activeIt = beginActiveTiles(), activeEnd = endActiveTiles(); activeIt != activeEnd; ++activeIt)
    {
        if(*activeIt == kUnassignedBin)
        {
            assignTileBins(Tile(activeIt.x(), activeIt.y()));
        }
    }

#ifdef DEBUG_BOUNDARY
    std::cout << "DEBUG: DiscretizedAngleGrid: Assigned boundary:" << newBoundary << " Final boundary:"
        << cells_.getBoundary() << '\n';
#endif

    eraseOffMapTiles();

    verifyActiveRegion();
}


bool DiscretizedAngleGrid::recenterActiveRegion(Point<float> robotPosition)
{
    // If the robot tile is within 1 of the center tile, then don't recenter. Otherwise, need to shift to make the
    // robot tile the center tile.
    auto robotTileOffset = position_to_tile(robotPosition, cells_) - activeRegion_.center();

    if((std::abs(robotTileOffset.x) <= 1) && (std::abs(robotTileOffset.y) <= 1))
    {
#ifdef DEBUG_BOUNDARY
        std::cout << "DEBUG: DiscretizedAngleGrid: Center offset not large enough. Not recentering.\n";
#endif
        return false;
    }

    auto newActiveRegion = calculateActiveRegion(robotPosition);

    // If pushed up against the boundary of the map, even if a shift is needed, there might not be anywhere for the
    // region to grow, so it will still be unchanged for the off-center robot position.
    if(newActiveRegion == activeRegion_)
    {
#ifdef DEBUG_BOUNDARY
        std::cout << "DEBUG: DiscretizedAngleGrid: Active region unchanged. Not recentering.\n";
#endif
        return false;
    }

    saveNewInactiveBins(newActiveRegion);
    loadNewActiveBins(newActiveRegion);

    activeCenter_ = robotPosition;
    activeRegion_ = newActiveRegion;

    verifyActiveRegion();

#ifdef DEBUG_BOUNDARY
    std::cout << "DEBUG: DiscretizedAngleGrid: Assigned new active region: Tiles:" << activeRegion_ << " Cells: "
        << " Cells:" << getActiveRegionInCells() << '\n';
#endif

    return true;
}


math::Rectangle<float> DiscretizedAngleGrid::getActiveRegion(void) const
{
    auto activeOrigin = grid_point_to_global_point(tile_origin_cell(activeRegion_.bottomLeft, cells_), cells_);
    return math::Rectangle<float>(activeOrigin,
                                  Point<float>(activeOrigin.x + activeWidth() * cells_.metersPerTile(),
                                                     activeOrigin.y + activeHeight() * cells_.metersPerTile()));
}


math::Rectangle<int> DiscretizedAngleGrid::getActiveRegionInCells(void) const
{
    auto activeOrigin = tile_origin_cell(activeRegion_.bottomLeft, cells_);
    return math::Rectangle<int>(activeOrigin,
                                Point<int>(activeOrigin.x + (activeWidth() * cells_.getTileSize()),
                                                 activeOrigin.y + (activeHeight() * cells_.getTileSize())));
}


bool DiscretizedAngleGrid::isCellInActiveRegion(Point<int> cell) const
{
    return isCellInActiveRegion(cell.x, cell.y);
}


bool DiscretizedAngleGrid::isCellInActiveRegion(int x, int y) const
{
    return (cells_.isCellInGrid(x, y)) ? (cells_(x, y) != kUnassignedBin) : false;
}


DiscretizedAngleGrid::TileIter DiscretizedAngleGrid::beginActiveTiles(void)
{
    return TileIter(activeRegion_.bottomLeft.x, activeRegion_.bottomLeft.y, activeWidth(), cells_);
}


DiscretizedAngleGrid::TileIter DiscretizedAngleGrid::endActiveTiles(void)
{
    return TileIter(activeRegion_.bottomLeft.x,
                    activeRegion_.bottomLeft.y + activeHeight(),
                    activeWidth(),
                    cells_);
}


void DiscretizedAngleGrid::reset(void)
{
    std::fill(angleBins_.begin(), angleBins_.end(), 0);
    resetCachedTiles();
}


void DiscretizedAngleGrid::flush(void)
{
    for(int tileY = activeRegion_.bottomLeft.y; tileY <= activeRegion_.topRight.y; ++tileY)
    {
        for(int tileX = activeRegion_.bottomLeft.x; tileX <= activeRegion_.topRight.x; ++tileX)
        {
            tileWrite(Tile(tileX, tileY));
        }
    }
}


DiscretizedAngleGrid::TileRegion DiscretizedAngleGrid::calculateActiveRegion(Point<float> center)
{
    using namespace boost::algorithm;

    auto centerTile = position_to_tile(center, cells_);
    return TileRegion(Tile(clamp(centerTile.x - activeRegionRadius_, 0, static_cast<int>(cells_.getWidthInTiles() - 1)),
                           clamp(centerTile.y - activeRegionRadius_, 0, static_cast<int>(cells_.getHeightInTiles() - 1))),
                      Tile(clamp(centerTile.x + activeRegionRadius_, 0, static_cast<int>(cells_.getWidthInTiles() - 1)),
                           clamp(centerTile.y + activeRegionRadius_, 0, static_cast<int>(cells_.getHeightInTiles() - 1))));
}


void DiscretizedAngleGrid::setActiveBins(void)
{
    for(auto activeIt = beginActiveTiles(), activeEnd = endActiveTiles(); activeIt != activeEnd; ++activeIt)
    {
        assignTileBins(Tile(activeIt.x(), activeIt.y()));
    }
}


void DiscretizedAngleGrid::saveNewInactiveBins(const TileRegion& newActiveRegion)
{
    // Write out the newly inactive tiles and return their bins to the available bins
    int writecount = 0;
    for(int tileY = activeRegion_.bottomLeft.y; tileY <= activeRegion_.topRight.y; ++tileY)
    {
        for(int tileX = activeRegion_.bottomLeft.x; tileX <= activeRegion_.topRight.x; ++tileX)
        {
            // If this tile that was loaded with the old origin is no longer in the grid with the changed origin
            // then it should be cached to disk
            if((tileX < newActiveRegion.bottomLeft.x) || (tileX > newActiveRegion.topRight.x)
                || (tileY < newActiveRegion.bottomLeft.y) || (tileY > newActiveRegion.topRight.y))
            {
                tileWrite(Tile(tileX, tileY));
                freeTileBins(Tile(tileX, tileY));
                // If successfully wrote, then cache the position that was written
                cachedTilePositions_.push_back(tile_origin_position(Tile(tileX, tileY), cells_));
                writecount++;
            }
        }
    }
}


void DiscretizedAngleGrid::loadNewActiveBins(const TileRegion& newActiveRegion)
{
    // Sort bins so close tiles are as close in memory as possible
    std::sort(availableBins_.begin(), availableBins_.end());

    int readcount = 0;
    for(int tileY = newActiveRegion.bottomLeft.y; tileY <= newActiveRegion.topRight.y; ++tileY)
    {
        for(int tileX = newActiveRegion.bottomLeft.x; tileX <= newActiveRegion.topRight.x; ++tileX)
        {
            // If the tile in the new region wasn't in the old region, its prior state should be loaded from disk
            if((tileX < activeRegion_.bottomLeft.x) || (tileX > activeRegion_.topRight.x)
                || (tileY < activeRegion_.bottomLeft.y) || (tileY > activeRegion_.topRight.y))
            {
                assignTileBins(Tile(tileX, tileY));
                tileRead(Tile(tileX, tileY));
                readcount++;
            }
        }
    }
}


void DiscretizedAngleGrid::eraseOffMapTiles(void)
{
    std::sort(cachedTilePositions_.begin(), cachedTilePositions_.end());
    erase_unique(cachedTilePositions_);

    auto currentBoundary = cells_.getBoundary();

#ifdef DEBUG_TILES
    std::cout << "DEBUG: DiscretizedAngleGrid: Checking for cached tiles in " << currentBoundary << '\n';
#endif

    // Find all cached tiles that are no longer on the map and remove the file for each of them
    for(auto tile : cachedTilePositions_)
    {
        if(currentBoundary.contains(tile))
        {
            continue;
        }

        std::string tileName = tile_name(name_, tile);
        bool success = boost::filesystem::remove(tileName);

        if(!success)
        {
            std::cerr << "WARNING: DiscretizedAngleGrid: Failed to remove tile: " << tileName << " It didn't exist.\n";
        }
    }

    // Erase them from the list of cached tiles
    erase_remove_if(cachedTilePositions_, [currentBoundary](Point<double> p) {
        return !currentBoundary.contains(p);
    });
}


void DiscretizedAngleGrid::freeTileBins(Tile tile)
{
    tileClear(tile);

    auto originCell = tile_origin_cell(tile, cells_);
    availableBins_.push_back(cells_(originCell.x, originCell.y));

    // When iterating through a single tile, can just use the unsafe iterator for TiledCellGrid
    std::fill(cells_.begin(tile.x, tile.y), cells_.end(tile.x, tile.y), kUnassignedBin);
}


void DiscretizedAngleGrid::assignTileBins(Tile tile)
{
    if(availableBins_.empty())
    {
        std::cerr << "ERROR: DiscretizedAngleGrid: Out of available bins!\n";
        assert(!availableBins_.empty());
    }

    BinIndex nextBin = availableBins_.front();
    availableBins_.pop_front();

    // When iterating through a single tile, can just use the unsafe iterator for TiledCellGrid
    auto tileStart = cells_.begin(tile.x, tile.y);
    auto tileEnd = cells_.end(tile.x, tile.y);
    while(tileStart != tileEnd)
    {
        *tileStart = nextBin;
        nextBin += kAngleBinsPerCell_;
        ++tileStart;
    }
}


void DiscretizedAngleGrid::clearBin(BinIndex binStart)
{
    std::fill(angleBins_.begin() + binStart, angleBins_.begin() + binStart + kAngleBinsPerTile_, 0);
}

///////////////////////////////////// I/O //////////////////////////////////////////////////////

void DiscretizedAngleGrid::resetCachedTiles(void)
{
    // Create the directory for storing the saved tiles
    std::ostringstream directories;
    directories << kTileBaseDirectory << name_ << '/';
    assert(directories.str() != "/");
    boost::filesystem::remove_all(directories.str());
    boost::filesystem::create_directories(directories.str());
}


void DiscretizedAngleGrid::tileWrite(Tile tile)
{
    auto tilePosition = tile_origin_position(tile, cells_);
    std::string tileName = tile_name(name_, tilePosition);
    FILE* fp = fopen(tileName.c_str(), "wb");

    if(fp)
    {
        const std::size_t kTileBytes = kAngleBinsPerTile_ * sizeof(Value);
        auto tileOrigin = tile_origin_cell(tile, cells_);
        auto tileStartBin = cells_(tileOrigin.x, tileOrigin.y);

        std::size_t compression = 0;
        if(compression)
        {
            PRINT_STUB("DiscretizedAngleGrid::tileWrite compressed data");
    //         diskBuffer_.reserve(bsz);
    //         diskBuffer_.clear();
    //         compress(tileBuffer_.begin(), tileBuffer_.end(), std::back_inserter(diskBuffer_));
    //         writesize = diskBuffer_.size();
    //
    //         fwrite(&compression,sizeof(std::size_t),1,fp);
    //         fwrite(&writesize,sizeof(std::size_t),1,fp);
    //         fwrite(diskBuffer_.data(),1,writesize,fp);
        }
        else
        {
            fwrite(&compression, sizeof(std::size_t), 1, fp);
            fwrite(&kTileBytes, sizeof(std::size_t), 1, fp);
            fwrite(angleBins_.data() + tileStartBin, 1, kTileBytes, fp);
        }

        fclose(fp);
    }
    else
    {
        std::cerr << "ERROR:DiscretizedAngleGrid: Failed to open tile cache file:" << tileName << '\n';
        perror("errno:");
    }
}


void DiscretizedAngleGrid::tileRead(Tile tile)
{
    std::string tileFilename = tile_name(name_, tile_origin_position(tile, cells_));
    FILE* fp = fopen(tileFilename.c_str(), "rb");

    if(fp)
    {
        std::size_t compression;
        if(fread(&compression,sizeof(std::size_t),1,fp))
        {
            // do nothing on error...
        }
        std::size_t readsize;
        if(fread(&readsize,sizeof(std::size_t),1,fp))
        {
            // do nothing on error
        }

        //Data is uncompressed
        if(!compression)
        {
            const std::size_t kTileBytes = kAngleBinsPerTile_ * sizeof(Value);
            auto tileOrigin = tile_origin_cell(tile, cells_);
            auto tileStartBin = cells_(tileOrigin.x, tileOrigin.y);

            assert(readsize == kTileBytes);
            std::size_t count = fread(angleBins_.data() + tileStartBin, 1, readsize, fp);
            assert(count == readsize);
        }
        else
        {  //Data is RL compressed
            PRINT_STUB("DiscretizedAngleGrid::tileRead compressed data");
//             diskBuffer_.resize(readsize);
//             if(fread(diskBuffer_.data(),1,readsize,fp));
//
//             tileBuffer_.reserve(bsz);
//             tileBuffer_.clear();
//             decompress(diskBuffer_.begin(), diskBuffer_.end(), std::back_inserter(tileBuffer_));
//             assert(tileBuffer_.size() == bsz);
        }
    }
    else
    {
//         std::cerr << "WARNING: DiscretizedAngleGrid: Failed to read tile from " << tileFilename << " Tile:" << tile <<
//             " Origin:" << cells_.getBottomLeft() << " Dim:" << cells_.getWidthInTiles() << 'x' << cells_.getHeightInTiles() << '\n';
//         perror("errno:");

        tileClear(tile);
    }

    if(fp)
    {
        fclose(fp);
    }
}


void DiscretizedAngleGrid::tileClear(Tile tile)
{
    // Zero out the values stored in all angle bins associated with the tile
    auto tileOrigin = tile_origin_cell(tile, cells_);
    auto tileBinStart = cells_(tileOrigin.x, tileOrigin.y);
    clearBin(tileBinStart);
}


void DiscretizedAngleGrid::verifyActiveRegion(void)
{
    for(auto activeIt = beginActiveTiles(), activeEnd = endActiveTiles(); activeIt != activeEnd; ++activeIt)
    {
        if(*activeIt == kUnassignedBin)
        {
            std::cout << "ERROR: DiscretizedAngleGrid: Active region:" << activeRegion_ << '\n'
                << "Unassigned active:" << Tile(activeIt.x(), activeIt.y()) << '\n';
            assert(*activeIt != kUnassignedBin);
        }

        auto unassignedIt = std::find(cells_.begin(activeIt.x(), activeIt.y()),
                                      cells_.end(activeIt.x(), activeIt.y()),
                                      kUnassignedBin);
        if(unassignedIt != cells_.end(activeIt.x(), activeIt.y()))
        {
            auto badCellIt = activeIt.begin();
            std::advance(badCellIt, std::distance(cells_.begin(activeIt.x(), activeIt.y()), unassignedIt));
            std::cout << "ERROR: DiscretizedAngleGrid: Active region:" << activeRegion_ << '\n'
                << "Unassigned active:" << Tile(activeIt.x(), activeIt.y())
                << " Cell:" << badCellIt.x() << ',' << badCellIt.y() << '\n';
            assert(unassignedIt == cells_.end(activeIt.x(), activeIt.y()));
        }
    }
}


std::string tile_name(const std::string& gridName, Point<double> tilePosition)
{
    // The tile is stored based on its global position in millimeters. Storing based on position allows for the boundary
    // to change and come back. Because the grid always shifts in units of tiles, indexing based on the tile origin will
    // always yield a repeatable location for loading the tile data
    int xName = std::round(tilePosition.x * 100.0);
    int yName = std::round(tilePosition.y * 100.0);

    std::ostringstream filename;
    filename << kTileBaseDirectory << gridName << "/Tile_X-" << xName << "_Y-" << yName << "_Sz-" << kTileSize
        << ".tile";
    return filename.str();
}

} // namespace utils
} // namespace vulcan
