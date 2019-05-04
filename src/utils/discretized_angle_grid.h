/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     discretized_angle_grid.h
* \author   Paul Foster and Collin Johnson
*
* Declaration of DiscretizedAngleGrid.
*/

#ifndef UTILS_DISCRETIZED_ANGLE_GRID_H
#define UTILS_DISCRETIZED_ANGLE_GRID_H

#include <utils/tiled_cell_grid.h>
#include <utils/grid_iterators.h>
#include <core/point.h>
#include <cereal/access.hpp>
#include <cereal/types/deque.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cstdint>

namespace vulcan
{
namespace utils
{

/**
* DiscretizedAngleGrid is an occupancy grid that stores the angle from which a particular (x, y) cell was seen. These
* angles are discretized into a fixed number of bins. Thus, the occupancy grid has three dimensions, (x, y, theta).
* 
* The three-dimension grid potentially holds a massive amount of data, more than can fit readily in RAM. Therefore, the
* DiscretizedAngleGrid caches tiles of data to disk and loads them as needed. On construction, the dimensions of the 
* loaded area can be specified. As the robot moves through the environment, the map will be cached to disk as needed. 
* If the robot drives over a previously visited portion of the map, then it will reload the cached tiles of the map. If
* a tile falls outside the map boundary, it will be erased.
* 
* The active region of the grid is a square centered around the robot. The active region is always less than or equal
* to the overall grid boundary. The active region of the grid is specified as a radius that extends around the robot.
* The active boundary will always have:  active.width >= radius*2.
* 
* Like all Grids, the bottom left corner of the boundary is cell (0,0). However, this cell is unlikely to be part of
* the active region. The active region can be queried and used via the following methods:
* 
*   - recenterActiveRegion : recenter the active region around the robot's current pose
*   - getActiveRegion : the boundary of the active region of the map
*   - isCellInActiveRegion : check if a cell falls into the active region
*   - beginActiveTiles : first tile in the active region
*   - endActiveTiles : one-past-the-end tile in the active region
* 
* For best use, the bins should be the same angular resolution as the laser used for building the map (see Paul's paper
* for details).
*/
class DiscretizedAngleGrid
{
public:

//     using Value = uint16_t;
    using Value = int8_t;
    using BinConstIter = const Value*; //std::vector<Value>::const_iterator;
    using BinIter = Value*; //std::vector<Value>::iterator;
    using TileIter = TiledGridIterator<int64_t, TiledCellGrid<int64_t, 16>>;
    
    /**
    * Default constructor for DiscretizedAngleGrid.
    * 
    * \param    metersPerCell           Meters per cell
    * \param    activeRadius            Minimum radius of the active region to maintain around the robot
    * \param    numAngleBins            Number of angle bins to use
    */
    explicit DiscretizedAngleGrid(std::string name = "default",
                                  double metersPerCell = 0.05, 
                                  double activeRadius = 3.0, 
                                  std::size_t numAngleBins = 1440);

    /**
    * Constructor for DiscretizedAngleGrid.
    * 
    * \param    widthInCells            Initial width of the grid
    * \param    heightInCells           Initial height of the grid
    * \param    metersPerCell           Meters per cell
    * \param    globalCenter            Initial center of the grid
    * \param    activeRadius            Minimum radius of the active region to maintain around the robot
    * \param    numAngleBins            Number of angle bins to use
    * \param    name                    Name to use for the cached tiles
    */
    DiscretizedAngleGrid(std::size_t widthInCells, 
                         std::size_t heightInCells, 
                         double metersPerCell, 
                         Point<double> globalCenter,
                         double activeRadius,
                         std::size_t numAngleBins, 
                         std::string name);
    
    // Grid concept interface
    std::size_t getWidthInCells(void) const  { return cells_.getWidthInCells(); }
    double      getWidthInMeters(void) const { return cells_.getWidthInMeters(); }

    std::size_t getHeightInCells(void) const  { return cells_.getHeightInCells(); }
    double      getHeightInMeters(void) const { return cells_.getHeightInMeters(); }

    std::size_t numAngleBins(void) const { return kAngleBinsPerCell_; }
    
    float metersPerCell(void) const { return cells_.metersPerCell();}
    float cellsPerMeter(void) const { return cells_.cellsPerMeter(); }

    math::Rectangle<double> getBoundary(void) const { return cells_.getBoundary(); }
    Point<double> getGlobalCenter(void) const { return cells_.getGlobalCenter(); }
    Point<double> getBottomLeft(void) const { return cells_.getBottomLeft(); }

    bool isCellInGrid(const Point<int>& cell) const { return isCellInGrid(cell.x, cell.y); }
    bool isCellInGrid(int x, int y) const { return cells_.isCellInGrid(x, y); }
    
    // Mutators
    void setMetersPerCell(float gridScale) { metersPerCell_ = gridScale; cellsPerMeter_ = 1.0 / gridScale; }
    void setBottomLeft   (const Point<float>& bottomLeft) { cells_.setBottomLeft(bottomLeft); }
    
    /**
    * changeBoundary changes the shape and boundary of the DiscretizedAngleGrid. The reshaped area is specified as a
    * new metric boundary for the DiscretizedAngleGrid. The parts of the current grid that fall into the reshaped area
    * will be copied into the appropriate location, and the rest of the cells will be set to the default value.
    *
    * \param    newBoundary         Reshaped boundary for the DiscretizedAngleGrid
    */
    void changeBoundary(const math::Rectangle<double>& newBoundary);
    
    // Angle-specific access
    /**
    * recenterActiveRegion recenters the active region around the current robot position. The active region doesn't
    * always need to be recentered, so sometimes this operation does nothing.
    * 
    * If a recentering occurs, all iterators are invalidated.
    * 
    * \param    robotPosition           Position around which to center the active region
    * \return   True if the active region was recentered.
    */
    bool recenterActiveRegion(Point<float> robotPosition);
    
    /**
    * getActiveRegion retrieves the boundary of the active region of the grid.
    */
    math::Rectangle<float> getActiveRegion(void) const;
    math::Rectangle<int> getActiveRegionInCells(void) const;
    double activeRegionRadius(void) const { return activeRegionRadius_ * cells_.metersPerTile(); }
    Point<float> activeRegionCenter(void) { return activeCenter_; }
    
    /**
    * isCellInActiveRegion checks if the specified cell falls in the active region of the grid.
    */
    bool isCellInActiveRegion(Point<int> cell) const;
    bool isCellInActiveRegion(int x, int y) const;
    
    /**
    * beginActiveTiles retrieves an iterator to the first active tile in the grid.
    * 
    * This iterator is invalidated if a call to recenterActiveRegion returns true or by a call to changeBoundary.
    */
    TileIter beginActiveTiles(void);
    
    /**
    * endActiveTiles retrieves an iterator to the one-past-the-end active tile in the grid.
    * 
    * The end iterator isn't cheap to create, so it should be saved rather than called on each iteration.
    * 
    * This iterator is invalidated if a call to recenterActiveRegion returns true or by a call to changeBoundary.
    */
    TileIter endActiveTiles(void);
    
    /**
    * numActiveTiles retrieves the number of active tiles.
    */
    std::size_t numActiveTiles(void) const { return activeRegion_.width() * activeRegion_.height(); }
    
    // Iterator access to angle bins
    /**
    * beginBin retrieves the beginning iterator for angle bin at the (x,y) cell. If the cell isn't loaded, then
    * beginBin == endBin.
    */
    BinConstIter beginBin(int x, int y) const { return angleBins_.data() + cells_(x, y); }
    BinIter beginBin(int x, int y) { return angleBins_.data() + cells_(x, y); }
    
    /**
    * beginBin retrieves the beginning iterator for angle bin for the cell specified in the TileIter.
    */
    BinConstIter beginBin(const TileIter& iter) const { return angleBins_.data() + *iter; }
    
    /**
    * endBin retrieves the end iterator for angle bin at the (x,y) cell. If the cell isn't loaded, then
    * beginBin == endBin.
    */
    BinConstIter endBin(int x, int y) const { return beginBin(x, y) + kAngleBinsPerCell_; }
    BinIter endBin(int x, int y) { return beginBin(x, y) + kAngleBinsPerCell_; }
    
    /**
    * endBin retrieves the one-past-the-end iterator for angle bin for the cell specified in the TileIter.
    */
    BinConstIter endBin(const TileIter& iter) const { return beginBin(iter) + kAngleBinsPerCell_; }
    
    /**
    * Direct access to the loaded angle bins.
    * 
    * \pre  cell(x, y) is in an active tile
    * \pre  isCellInGrid(x, y) == true
    * \pre  theta >= 0 && theta < numAngleBins
    * \param    x       x-position of the cell
    * \param    y       y-position of the cell
    * \param    theta   Angle bin to access
    * \return   Reference to the value of the specified angle bin.
    */
    Value& operator()(int x, int y, int theta)
    {
        BinIndex binStart = cells_(x, y);
        assert(binStart >= 0);
        assert(binStart < static_cast<BinIndex>(angleBins_.size()));
        return angleBins_[binStart + theta];
    }
    
    /**
    * Direct access to the loaded angle bins.
    * 
    * \pre  cell(x, y) is in an active tile
    * \pre  isCellInGrid(x, y) == true
    * \pre  theta >= 0 && theta < numAngleBins
    * \param    x       x-position of the cell
    * \param    y       y-position of the cell
    * \param    theta   Angle bin to access
    * \return   Value of the specified angle bin.
    */
    Value operator()(int x, int y, int theta) const
    {
        BinIndex binStart = cells_(x, y);
        assert(binStart < static_cast<BinIndex>(angleBins_.size()));
        return angleBins_[binStart + theta];
    }

    /**
    * reset zeroes out the map and erases any cached blocks.
    */
    void reset(void);
    
    /**
    * flush flushes all cached data to disk.
    */
    void flush(void);
    
private:

    using Tile = Point<int>;
    using TileRegion = math::Rectangle<int>;
    using Cell = Point<int>;
    using BinIndex = int64_t;
    
    static const BinIndex kUnassignedBin;
   
    std::string name_;
    std::size_t kTileWidth_;
    std::size_t kAngleBinsPerCell_;
    std::size_t kAngleBinsPerTile_;

    // coordinate system
    double cellsPerMeter_;
    double metersPerCell_;
    
    TiledCellGrid<BinIndex, 16> cells_;
    std::vector<Value> angleBins_;
    std::deque<BinIndex> availableBins_;    // starting bins for an entire tile's angle bins
    
    Point<float> activeCenter_;       // robot position that resulted in current center of the active region
    TileRegion activeRegion_;               // The active region includes both sides, so width + 1 = number of tiles
    int activeRegionRadius_;                // In tiles!
    
    // Keep track of the position of all tiled currently cached to disk. When the map boundary changes, can then easily
    // determine which cached tiles aren't in the map anymore and can be erased.
    std::vector<Point<double>> cachedTilePositions_;
    
    TileRegion calculateActiveRegion(Point<float> center);
    void setActiveBins(void);   // assign a bin to each cell in the active region that doesn't already have one
    int activeWidth(void) const { return activeRegion_.width() + 1; }
    int activeHeight(void) const { return activeRegion_.height() + 1; }
    
    void saveNewInactiveBins(const TileRegion& newActiveRegion);
    void loadNewActiveBins(const TileRegion& newActiveRegion);
    void eraseOffMapTiles(void);
    
    void freeTileBins(Tile tile);
    void assignTileBins(Tile tile);
    void clearBin(BinIndex binStart);
    
    void resetCachedTiles(void);
    void tileWrite(Tile tile);
    void tileRead(Tile tile);
    void tileClear(Tile tile);
    
    void verifyActiveRegion(void);

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar( name_,
            kTileWidth_,
            kAngleBinsPerCell_,
            kAngleBinsPerTile_,
            cellsPerMeter_,
            metersPerCell_,
            cells_,
            angleBins_,
            availableBins_,
            activeCenter_,
            activeRegion_,
            activeRegionRadius_,
            cachedTilePositions_
        );
    }
};

} // namespace utils
} // namespace vulcan

#endif // UTILS_DISCRETIZED_ANGLE_GRID_H
