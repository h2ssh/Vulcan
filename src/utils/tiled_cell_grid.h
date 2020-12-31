/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tilted_cell_grid.h
* \author   Collin Johnson
* 
* Definition of TiledCellGrid.
*/

#ifndef UTILS_TILED_CELL_GRID_H
#define UTILS_TILED_CELL_GRID_H

#include "utils/cell_grid_utils.h"
#include "utils/tiled_cell_grid_utils.h"
#include "core/point.h"
#include "math/geometry/rectangle.h"
#include <cereal/access.hpp>
#include <cereal/types/vector.hpp>
#include <cstring>
#include <iostream>

// #define DEBUG_CHANGE_FRAME
// #define DEBUG_RESHAPE

namespace vulcan
{
namespace utils
{
  
/**
* TiledCellGrid is a cell grid that stores the underlying values in small tiles. This alternate memory layout can be
* more efficient in some circumstances than the normal cell grid layout that stores the grid in row-major order.
*/
template <typename T, int N = 8>
class TiledCellGrid
{
public:

    using Iter = typename std::vector<T>::iterator;
    
    // Methods for creating/destroying the grid
    TiledCellGrid(void);
    
    /**
    * Constructor to create a new TiledCellGrid.
    *
    * \param    gridScale           Meters per grid cell
    */
    TiledCellGrid(double gridScale);
    
    /**
    * Constructor to create a new TiledCellGrid.
    *
    * \param    widthInCells        Width of the grid in cells
    * \param    heightInCells       Height of the grid in cells
    * \param    metersPerCell       Meters per grid cell
    * \param    globalCenter        Global center of the map
    * \param    initialValue        Initial value to assign to each grid cell (optional)
    */
    TiledCellGrid(std::size_t widthInCells, 
                  std::size_t heightInCells, 
                  double metersPerCell, 
                  Point<double> globalCenter, 
                  T initialValue = T());
    
    TiledCellGrid(const TiledCellGrid& gridToCopy) = default;
    TiledCellGrid(TiledCellGrid&& toMove) = default;
    TiledCellGrid& operator=(const TiledCellGrid& rhs) = default;
    TiledCellGrid& operator=(TiledCellGrid&& rhs) = default;
    
    // Methods for accessing grid parameters
    int64_t getTimestamp(void) const { return timestamp; }
    int32_t getId(void)        const { return id; }
    
    // Width and height are maintained as the tile width/height, not cell width/height
    std::size_t getWidthInTiles(void)  const { return width; }
    std::size_t getWidthInCells(void)  const { return width*N; }
    double       getWidthInMeters(void) const { return width * metersPerTile(); }
    
    std::size_t getHeightInTiles(void)  const { return height; }
    std::size_t getHeightInCells(void)  const { return height*N; }
    double       getHeightInMeters(void) const { return height * metersPerTile(); }
    
    math::Rectangle<double> getBoundary(void) const { return boundary; }
    
    double metersPerCell(void) const { return metersPerCell_;}
    double cellsPerMeter(void) const { return cellsPerMeter_;}
    double metersPerTile(void) const { return metersPerCell_ * N; }
    double tilesPerMeter(void) const { return cellsPerMeter_ / N; }
    int   getTileSize (void) const { return N; }
    
    Point<double> getGlobalCenter(void) const;
    Point<double> getBottomLeft(void) const { return boundary.bottomLeft; }
    
    // Methods for mutating the grid
    
    void setTimestamp      (int64_t newTime) { timestamp = newTime; }
    void setId             (int32_t mapId) { id = mapId; }
    void setGridSizeInCells(std::size_t width, std::size_t height);
    void setMetersPerCell  (double gridScale) { metersPerCell_ = gridScale; cellsPerMeter_ = 1.0 / gridScale; }
    void setBottomLeft     (const Point<double>& bottomLeft) { updateBoundary(bottomLeft); }
    
    // Methods for modifying position of the grid
    /**
    * changeBoundary changes the shape and boundary of the TiledCellGrid. The reshaped area is specified as a new metric
    * boundary for the TiledCellGrid. The parts of the current grid that fall into the reshaped area will be copied
    * into the appropriate location, and the rest of the cells will be set to the default value.
    *
    * \param    newBoundary         Reshaped boundary for the TiledCellGrid
    * \param    initialValue        Value to assign to the new cells that appear in the grid (optional)
    */
    void changeBoundary(const math::Rectangle<double>& newBoundary, T initialValue = T());
    
    /**
    * shiftGrid shifts the grid the specified number of cells.
    *
    * \param    deltaX              Change in cells along the x-coordinate
    * \param    deltaY              Change in cells along the y-coordinate
    * \param    initialValue        Value to assign to the new values that will appear in the grid
    */
    void shiftGrid(int deltaX, int deltaY, T initialValue = T());
    
    /**
    * changeGlobalCenter moves the global center of the grid to the specified
    * position. Changing the center will shift the grid the appropriate number
    * of cells.
    *
    * \param    newGlobalCenter     New global center for the grid
    * \param    initialCost         Cost to assign to the new values that will appear in the grid
    */
    void changeGlobalCenter(const Point<double>& newGlobalCenter, T initialValue = T());
    
    /**
    * reset resets all cells into the grid to value
    *
    * \param    value           Value to assign all the cells
    */
    void reset(T value = T());
    
    // Benefit of unsigned is no need to check against 0. Those values just shoot to the moon!
    bool isCellInGrid(const Point<int>& cell) const { return isCellInGrid(cell.x, cell.y); }
    bool isCellInGrid(int x, int y)                 const { return (x >= 0) && (y >= 0) && (static_cast<std::size_t>(x) < width*N) && (static_cast<std::size_t>(y) < height*N); }
    
    /**
    * cellIndex retrieves the index of the cell in the grid. It is assumed (x, y) actually exists in the grid, i.e.
    * isCellInGrid(x, y) == true. If not, undetermined behavior, likely a crash, will happen.
    * 
    * \param    x       x-index of desired cell
    * \param    y       y-index of desired cell
    * \return   Index of the cell in the grid.
    */
    int cellIndex(int x, int y) const { return xIndices[x] + yIndices[y]; }
    
    // Methods for manipulating the costs in the grid
    // No check is an unsafe, but faster method for accessing a cell, as it avoids bounds checks
    T getValue       (int x, int y) const;
    T getValueNoCheck(int x, int y) const { return gridValues[cellIndex(x, y)]; }
    
    void setValue       (int x, int y, const T& value);
    void setValueNoCheck(int x, int y, const T& value) { gridValues[cellIndex(x, y)] = value; }
    
    // Overload the function operator to allow matrix-style access, i.e. grid(x, y) = something or something = grid(x, y)
    T&       operator()(int x, int y)       { return gridValues[cellIndex(x, y)]; }
    const T& operator()(int x, int y) const { return gridValues[cellIndex(x, y)]; }
    
    // Basic iteration over a single tile
    // DANGER: BE VERY CAREFUL HERE!
    Iter begin(int tileX, int tileY) { return gridValues.begin() + cellIndex(tileX * N, tileY * N); }
    Iter end(int tileX, int tileY) { return begin(tileX, tileY) + (N * N); }
    
private:
    
    int64_t timestamp;
    int32_t id;
    
    std::vector<T> gridValues;          ///< The actual grid -- stored in row-column order
    
    std::size_t width;                  ///< Width of the grid in tiles
    std::size_t height;                 ///< Height of the grid in tiles
    double      metersPerCell_;         
    double      cellsPerMeter_;
    
    // Pre-cached indices for x-y coordinates
    std::vector<int> xIndices;
    std::vector<int> yIndices;
    
    math::Rectangle<double> boundary;    ///< Global metric boundary of the area covered by the grid
    
    
    void setTileValues (int tileX, int tileY, T value);
    void copyTileValues(int toX, int toY, int fromX, int fromY, const TiledCellGrid<T, N>& from);
    void calculateCachedIndices(int width, int height);
    
    Point<double> bottomLeftFromCenter(const Point<double>& center);
    void               updateBoundary(const Point<double>& newBottomLeft);

    // Serialization support
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar ( timestamp,
             id,
             gridValues,
             width,
             height,
             metersPerCell_,
             cellsPerMeter_,
             xIndices,
             yIndices,
             boundary
        );
    }
};


/**
* transform_grid transforms the grid to be in the newly specified reference frame. The grid is rotated to
* align to the new axes and then all coordinates for the grid are transformed to be relative to the new
* reference frame.
* 
* \param[in]    grid            Grid to be transformed
* \param[in]    initialValue    Value to assign to the cells that pop into existance when the map rotates
* \param[in]    xRef            x coordinate of new reference frame
* \param[in]    yRef            y coordinate of new reference frame
* \param[in]    thetaRef        Orientation of x-axis in new reference frame
* \return   A new grid oriented to the new axes and with coordinates relative to reference
*/
template <typename T, int N>
TiledCellGrid<T, N> transform_grid(const TiledCellGrid<T, N>& grid, T initialValue, double xRef, double yRef, double thetaRef)
{
    /*
    * Need to find the new width and height of the grid after rotation. Find the radius of the grid.
    * Take the angle of the radius and rotate it by theta to find where the radius cell lies in the
    * rotated grid. These will be the maxima to determine the width and height of the new grid.
    */
    
    double halfWidth  = grid.getWidthInCells() / 2.0f;
    double halfHeight = grid.getHeightInCells() / 2.0f;
    
    double radiusAngle = atan2(halfHeight, halfWidth);
    double radius      = ceil(sqrt(halfWidth*halfWidth + halfHeight*halfHeight));
    
    int newHalfWidth  = ceil(radius * std::max(std::abs(cos(radiusAngle - thetaRef)), std::abs(cos(-radiusAngle - thetaRef))));
    int newHalfHeight = ceil(radius * std::max(std::abs(sin(radiusAngle - thetaRef)), std::abs(sin(-radiusAngle - thetaRef))));
    
#ifdef DEBUG_CHANGE_FRAME
    std::cout<<"INFO:transform_grid:Old dim:("<<halfWidth<<','<<halfHeight<<") New dim:("<<newHalfWidth<<','<<newHalfHeight<<")\n";
#endif
    
    TiledCellGrid<T, N> rotated(grid);
    rotated.setGridSizeInCells(2*newHalfWidth, 2*newHalfHeight);
    rotated.reset(initialValue);
    
    Point<double> originalCell;
    Point<double> rotatedCell;
    
    int centerX = grid.getWidthInCells() / 2;
    int centerY = grid.getHeightInCells() / 2;
    
    // Go through each cell in the rotated grid to ensure no holes crop up in the new image rather than only converting
    // the old lpm cell-by-cell into the new one.
    for(uint16_t y = 0; y < rotated.getHeightInCells(); ++y)
    {
        rotatedCell.y = y - newHalfHeight;
        
        for(uint16_t x = 0; x < rotated.getWidthInCells(); ++x)
        {
            rotatedCell.x  = x - newHalfWidth;
            originalCell   = rotate(rotatedCell, thetaRef);  // rotating  rotated->original, so angle is not negative!
            originalCell.x += centerX;
            originalCell.y += centerY;
            
            if(grid.isCellInGrid(originalCell))
            {
                rotated.setValueNoCheck(x, y, grid.getValueNoCheck(originalCell.x, originalCell.y));
            }
        }
    }
    
    // After copying the cells, need to adjust the bottom left corner to be in the new reference frame
    Point<double> oldCenter = grid.getGlobalCenter();
    
    // Use the cell dimensions/2 in order to make sure the bo            ttom left is an integral number of cells from the center, which it needs to be
    rotated.setBottomLeft(Point<double>(oldCenter.x - newHalfWidth*rotated.metersPerCell()  - xRef,
                                             oldCenter.y - newHalfHeight*rotated.metersPerCell() - yRef));
    
#ifdef DEBUG_CHANGE_FRAME
    std::cout<<"INFO:transform_grid:Old BL:"<<grid.getBottomLeft()<<" New BL:"<<rotated.getBottomLeft()<<'\n';
#endif
    
    return rotated;
}


// Implementation of the CellGrid interface for the non-one-liner methods
template <typename T, int N>
TiledCellGrid<T, N>::TiledCellGrid(void)
: width(0)
, height(0)
, metersPerCell_(0.05) // have a non-zero scale so global_point_to_grid_cell on a default constructed cell grid doesn't divide by 0
, cellsPerMeter_(1.0 / metersPerCell_)
{
    // Don't worry about setting all parameters, just those that concern general operation
}


template <typename T, int N>
TiledCellGrid<T, N>::TiledCellGrid(double metersPerCell)
: width(0)
, height(0)
, metersPerCell_(metersPerCell)
, cellsPerMeter_(1.0 / metersPerCell_)
{
}


template <typename T, int N>
TiledCellGrid<T, N>::TiledCellGrid(std::size_t widthInCells, 
                                   std::size_t heightInCells, 
                                   double metersPerCell, 
                                   Point<double> globalCenter, 
                                   T initialValue)
: width(round_up_to_nearest_n(widthInCells, N) / N)
, height(round_up_to_nearest_n(heightInCells, N) / N)
, metersPerCell_(metersPerCell)
{
    assert(width > 0);
    assert(height > 0);
    assert(metersPerCell_ > 0.0);
    
    cellsPerMeter_ = 1.0 / metersPerCell_;
    
    // Calculate the boundary in two parts because truncation can lead to inaccurate results otherwise.
    // ALL calculations MUST be done relative to the establish bottomLeft corner, otherwise the implied truncations
    // from the casts to integers result in broken maps
    updateBoundary(bottomLeftFromCenter(globalCenter));
    
    gridValues.resize(width * N * height * N);
    reset(initialValue);
    
    calculateCachedIndices(width*N, height*N);
}


// Methods for mutating the grid
template <typename T, int N>
Point<double> TiledCellGrid<T, N>::getGlobalCenter(void) const
{
    return Point<double>(boundary.bottomLeft.x + (width*N/2)*metersPerCell_,
                              boundary.bottomLeft.y + (height*N/2)*metersPerCell_);
}


template <typename T, int N>
void TiledCellGrid<T, N>::setGridSizeInCells(std::size_t width, std::size_t height)
{
    if(this->width == width && this->height == height)
    {
        return;
    }
    
    // Make sure the height is a multiple of N
    width  = round_up_to_nearest_n(width,  N);
    height = round_up_to_nearest_n(height, N);
    gridValues.resize(width * height);
    
    this->width  = width / N;
    this->height = height / N;
    
    calculateCachedIndices(width, height);
    updateBoundary(boundary.bottomLeft);
}


// Methods for modifying position of the grid
template <typename T, int N>
void TiledCellGrid<T, N>::changeBoundary(const math::Rectangle<double>& newBoundary, T initialValue)
{
    if((newBoundary.bottomLeft == boundary.bottomLeft) && (newBoundary.topRight == boundary.topRight))
    {
        return;
    }
    
    auto newOriginTile = position_to_tile(newBoundary.bottomLeft, *this);
    auto newEndTile = position_to_tile(newBoundary.topRight, *this);
    
    // New dimensions needs to include all of the end tile, so jump to one beyond to incorporate the full end tile
    std::size_t newWidth  = newEndTile.x - newOriginTile.x + 1;
    std::size_t newHeight = newEndTile.y - newOriginTile.y + 1;
    
    math::Rectangle<double> adjustedBoundary(tile_origin_position(newOriginTile, *this),
                                            tile_origin_position(Point<int>(newOriginTile.x + newWidth, 
                                                                                  newOriginTile.y + newHeight), 
                                                                 *this));
    
#ifdef DEBUG_RESHAPE
    std::cout << "Changing boundary from " << boundary << " to " << adjustedBoundary << '\n';
#endif

    if(adjustedBoundary.intersection(boundary).area() != 0)
    {
        const boundary_intersection_t coords(boundary, 
                                             adjustedBoundary, 
                                             tilesPerMeter(), 
                                             std::make_pair(width, height), 
                                             std::make_pair(newWidth, newHeight));
        
        int xShift = coords.updateStartCell.x - coords.gridStartCell.x;
        int yShift = coords.updateStartCell.y - coords.gridStartCell.y;
        
        assert(absolute_fuzzy_equal(adjustedBoundary.bottomLeft.x, boundary.bottomLeft.x - xShift*metersPerTile()));
        assert(absolute_fuzzy_equal(adjustedBoundary.bottomLeft.y, boundary.bottomLeft.y - yShift*metersPerTile()));
        
#ifdef DEBUG_RESHAPE
        std::cout<<"Boundaries: Old:"<<boundary<<" New:"<<adjustedBoundary<<'\n';
        std::cout<<"DEBUG:CellGrid::changeBoundary: coords:"<<coords.updateWidth<<'x'<<coords.updateHeight<<" old:"<<width<<'x'<<height<<" new:"<<newWidth<<'x'<<newHeight
                    <<" old bl:"<<boundary.bottomLeft<<" new bl:"<<adjustedBoundary.bottomLeft<<" shift:"<<xShift<<'x'<<yShift<<'\n';
#endif
        
        if((coords.updateWidth == 0) && (coords.updateHeight == 0))
        {
            return;
        }
        
        assert(coords.updateWidth <= newWidth);
        assert(coords.updateWidth <= width);
        assert(coords.gridStartCell.x + coords.updateWidth <= width);
        assert(coords.updateStartCell.x + coords.updateWidth <= newWidth);
        
        TiledCellGrid<T, N> newGrid(newWidth*N, newHeight*N, metersPerCell_, adjustedBoundary.center(), initialValue);
        newGrid.updateBoundary(adjustedBoundary.bottomLeft);
        
        assert(newGrid.getWidthInTiles() == newWidth);
        assert(newGrid.getHeightInTiles() == newHeight);
        
        for(std::size_t y = 0; y < coords.updateHeight; ++y)
        {
            for(std::size_t x = 0; x < coords.updateWidth; ++x)
            {
                newGrid.copyTileValues(coords.updateStartCell.x + x, 
                                       coords.updateStartCell.y + y, 
                                       coords.gridStartCell.x + x, 
                                       coords.gridStartCell.y + y, 
                                       *this);
            }
        }
        
        *this = std::move(newGrid);
    }
    else
    {
        reset();
        changeGlobalCenter(adjustedBoundary.center());
    }
    
    assert(this->boundary == adjustedBoundary);
}


template <typename T, int N>
void TiledCellGrid<T, N>::shiftGrid(int deltaX, int deltaY, T initialValue)
{
    /*
    * For the shift, both x and y shifts are handled at the same time. The idea here is to start
    * at the end of the shifted region and move backwards through it so that no temporaries need
    * to be created.
    *
    * First, shift the (x, y) values, then fill in the blanks with initialValue afterward.
    */
    
    // If there isn't any shifting actually occurring, then just return
    if((deltaX == 0) && (deltaY == 0))
    {
        return;
    }
    
    grid_shift_params_t xShift(deltaX, width);
    grid_shift_params_t yShift(deltaY, height);
    
    for(int y = yShift.shiftStart; y != yShift.shiftEnd; y += yShift.shiftIncrement)
    {
        for(int x = xShift.shiftStart; x != xShift.shiftEnd; x += xShift.shiftIncrement)
        {
            copyTileValues(x, y, x - xShift.shift, y - yShift.shift, *this);
        }
    }
    
    // Fill in the blanks below the starting y-coordinate, so copy in the initial value
    // all the way across grid
    for(int y = yShift.zeroStart; y != yShift.zeroEnd; y += yShift.shiftIncrement)
    {
        for(int x = 0; x < xShift.maxValue; ++x)
        {
            setTileValues(x, y, initialValue);
        }
    }
    
    // Fill in the blanks left of the starting x-coordinate, so copy in the initial value
    // all the way up the grid
    for(int y = 0; y < yShift.maxValue; ++y)
    {
        for(int x = xShift.zeroStart; x != xShift.zeroEnd; x += xShift.shiftIncrement)
        {
            setTileValues(x, y, initialValue);
        }
    }
}


template <typename T, int N>
void TiledCellGrid<T, N>::changeGlobalCenter(const Point<double>& newGlobalCenter, T initialValue)
{
    // Changing the global center requires shifting the whole grid to reflect the new position
    Point<double> newBottomLeft = bottomLeftFromCenter(newGlobalCenter);
    
    if(newBottomLeft != boundary.bottomLeft)
    {
        std::pair<int, int> cellsToShift = calc_cells_to_shift(boundary.bottomLeft, newBottomLeft, metersPerTile());
        
        shiftGrid(cellsToShift.first, cellsToShift.second, initialValue);
        
        // The new bottom left is not based on the newBottomLeft because the grid only shifts in whole cell increments
        // as a result, the center only moves the shift*cellScale() in each direction. Doing otherwise results in a
        // gradual drift in the map as the small shift errors accumulate
        newBottomLeft.x = boundary.bottomLeft.x - cellsToShift.first * metersPerTile();
        newBottomLeft.y = boundary.bottomLeft.y - cellsToShift.second * metersPerTile();
        
#ifdef DEBUG_RESHAPE
        std::cout<<"DEBUG:TiledCellGrid::changeGlobalCenter: Old:"<<boundary.bottomLeft<<" New:"<<newBottomLeft<<" Shift:"<<cellsToShift.first<<','<<cellsToShift.second<<'\n';
#endif
        
        updateBoundary(newBottomLeft);
    }
}


template <typename T, int N>
void TiledCellGrid<T, N>::reset(T initialValue)
{
    std::fill(gridValues.begin(), gridValues.end(), initialValue);
}


// Methods for manipulating the costs in the grid
template <typename T, int N>
T TiledCellGrid<T, N>::getValue(int cellX, int cellY) const
{
    if(isCellInGrid(cellX, cellY))
    {
        return getValueNoCheck(cellX, cellY);
    }
    else
    {
        return 0;
    }
}


template <typename T, int N>
void TiledCellGrid<T, N>::setValue(int cellX, int cellY, const T& value)
{
    if(isCellInGrid(cellX, cellY))
    {
        setValueNoCheck(cellX, cellY, value);
    }
}


template <typename T, int N>
void TiledCellGrid<T, N>::setTileValues(int tileX, int tileY, T value)
{
    int tileStart = cellIndex(tileX*N, tileY*N);
    std::fill(gridValues.begin() + tileStart, gridValues.begin() + tileStart + (N*N), value);
}


template <typename T, int N>
void TiledCellGrid<T, N>::copyTileValues(int toX, int toY, int fromX, int fromY, const TiledCellGrid<T, N>& from)
{
    int toIndex   = cellIndex(toX*N, toY*N);
    int fromIndex = from.cellIndex(fromX*N, fromY*N);

    std::copy(from.gridValues.begin() + fromIndex,
              from.gridValues.begin() + fromIndex + (N*N),
              gridValues.begin() + toIndex);
}


template <typename T, int N>
void TiledCellGrid<T, N>::calculateCachedIndices(int width, int height)
{
    xIndices.resize(width);
    yIndices.resize(height);
    
    for(int x = 0; x < width; ++x)
    {
        xIndices[x] = N*N*(x/N) + (x%N);
    }
    
    for(int y = 0; y < height; ++y)
    {
        yIndices[y] = N*width*(y/N) + (y%N)*N;
    }
}


template <typename T, int N>
Point<double> TiledCellGrid<T, N>::bottomLeftFromCenter(const Point<double>& center)
{
    return Point<double>(center.x - getWidthInMeters()/2.0f, center.y - getHeightInMeters()/2.0f);
}

template <typename T, int N>
void TiledCellGrid<T, N>::updateBoundary(const Point<double>& newBottomLeft)
{
    boundary = math::Rectangle<double>(newBottomLeft,
                                      Point<double>(newBottomLeft.x + getWidthInMeters(), newBottomLeft.y + getHeightInMeters()));
}

    
} // namespace utils
} // namespace vulcan

#endif // UTILS_TILED_CELL_GRID_H
