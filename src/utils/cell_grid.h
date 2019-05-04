/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     cell_grid.h
* \author   Collin Johnson
*
* Declaration and definition of CellGrid template.
*/

#ifndef UTILS_CELL_GRID_H
#define UTILS_CELL_GRID_H

#include <core/point.h>
#include <math/geometry/rectangle.h>
#include <core/float_comparison.h>
#include <utils/cell_grid_utils.h>
#include <utils/compression.h>
#include <cereal/cereal.hpp>
#include <cereal/access.hpp>
#include <cassert>
#include <cstdint>
#include <vector>


// #define DEBUG_RESHAPE

#ifdef DEBUG_RESHAPE
#include <iostream>
#endif

namespace vulcan
{
namespace utils
{

/**
* CellGrid is a generic grid implementation that maintains a cost value for each square. The cost can be used to
* represent any number of concepts that require a discretized grid of the robot's surround.
*
* The CellGrid exists in the global coordinate frame. The parameters used to describe the grid are the global
* coordinate of the bottom left cell, the number of meters per cell, and the width and height of the grid in cells.
*
* All operations in the CellGrid are in grid coordinates. Helper functions are provided to convert between global and
* grid coordinates.
*/
template <typename T>
class CellGrid
{
public:

    // Methods for creating/destroying the grid
    CellGrid(void);

    /**
    * Constructor to create a new CellGrid.
    *
    * \param    metersPerCell           Meters per grid cell
    */
    CellGrid(double metersPerCell);

    /**
    * Constructor to create a new CellGrid.
    *
    * \param    gridWidth           Width of the grid in cells
    * \param    gridHeight          Height of the grid in cells
    * \param    metersPerCell       Meters per grid cell
    * \param    globalCenter        Global center of the map
    * \param    initialValue        Initial value to assign to each grid cell (optional)
    */
    CellGrid(std::size_t gridWidth, std::size_t gridHeight, double metersPerCell, const Point<float>& globalCenter, T initialValue = T());

    /**
    * Constructor to create a new CellGrid.
    *
    * \param    origin              Origin of the grid in global coordinates
    * \param    width               Width in meters
    * \param    height              Height in meters
    * \param    cellsPerMeter       Number of cells per unit of the global coordinates
    * \param    initialValue        Value to assign to cells
    */
    CellGrid(const Point<float>& origin, double width, double height, double cellsPerMeter, T initialValue = T());

    CellGrid(const CellGrid& gridToCopy);
    CellGrid(CellGrid&& toMove);
    CellGrid& operator=(const CellGrid& rhs);
    CellGrid& operator=(CellGrid&& rhs);
    ~CellGrid(void);

    // Methods for accessing grid parameters
    std::size_t getWidthInCells(void) const  { return width_; }
    float       getWidthInMeters(void) const { return width_ * metersPerCell_; }

    std::size_t getHeightInCells(void) const  { return height_; }
    float       getHeightInMeters(void) const { return height_ * metersPerCell_; }

    math::Rectangle<float> getBoundary(void) const { return boundary_; }

    double metersPerCell(void) const { return metersPerCell_;}
    double cellsPerMeter(void) const { return cellsPerMeter_; }

    Point<float> getGlobalCenter(void) const;
    Point<float> getBottomLeft(void) const { return boundary_.bottomLeft; }

    // Methods for mutating the grid
    void setGridSizeInCells(std::size_t width, std::size_t height);
    void setMetersPerCell  (float metersPerCell) { metersPerCell_ = metersPerCell; cellsPerMeter_ = 1.0f / metersPerCell; }
    void setBottomLeft     (const Point<float>& bottomLeft) { updateBoundary(bottomLeft); }

    // Methods for modifying position of the grid
    /**
    * changeBoundary changes the shape and boundary of the CellGrid. The reshaped area is specified as a new metric
    * boundary for the CellGrid. The parts of the current grid that fall into the reshaped area will be copied
    * into the appropriate location, and the rest of the cells will be set to the default value.
    *
    * \param    newBoundary         Reshaped boundary for the CellGrid
    * \param    initialValue        Value to assign to the new cells that appear in the grid (optional)
    */
    void changeBoundary(const math::Rectangle<float>& newBoundary, T initialValue = T());

    /**
    * changeGlobalCenter moves the global center of the grid to the specified
    * position. Changing the center will shift the grid the appropriate number
    * of cells.
    *
    * \param    newGlobalCenter     New global center for the grid
    * \param    initialValue        Cost to assign to the new values that will appear in the grid
    */
    void changeGlobalCenter(const Point<float>& newGlobalCenter, T initialValue = T());

    /**
    * reset resets all cells into the grid to value
    *
    * \param    value           Value to assign all the cells
    */
    void reset(T value = T());

    // Benefit of unsigned is no need to check against 0. Those values just shoot to the moon!
    bool isCellInGrid(const Point<int>& cell) const { return isCellInGrid(cell.x, cell.y); }
    bool isCellInGrid(int x, int y)                 const { return (x >= 0) && (y >= 0) && (static_cast<std::size_t>(x) < width_) && (static_cast<std::size_t>(y) < height_); }

    // Methods for manipulating the costs in the grid
    // No check is an unsafe, but faster method for accessing a cell, as it avoids bounds checks
    T getValue       (int x, int y) const;
    T getValueNoCheck(int x, int y) const { return cells_[cell_to_index(x, y, width_)]; }

    void setValue       (int x, int y, const T& value);
    void setValueNoCheck(int x, int y, const T& value) { cells_[cell_to_index(x, y, width_)] = value; }

    // Overload the function operator to allow matrix-style access, i.e. grid(x, y) = something or something = grid(x, y)
    T&       operator()(int x, int y)       { return cells_[cell_to_index(x, y, width_)]; }
    const T& operator()(int x, int y) const { return cells_[cell_to_index(x, y, width_)]; }

private:

    T*          cells_;                 ///< The actual grid -- stored in row-column order
    std::size_t cellBufferSize_;        ///< Size of the allocated buffer

    std::size_t width_;                 ///< Width of the grid in cells
    std::size_t height_;                ///< Height of the grid in cells
    float       metersPerCell_;
    float       cellsPerMeter_;

    math::Rectangle<float> boundary_;    ///< Global metric boundary of the area covered by the grid

    void               shiftGrid           (int deltaX, int deltaY, T initialValue = T());
    Point<float> bottomLeftFromCenter(const Point<float>& center);
    void               adjustBottomLeft    (int deltaX, int deltaY);    // change bottom left, but don't touch cells
    void               updateBoundary      (const Point<float>& newBottomLeft);

    // Code for serialization
    /////// No message_traits because CellGrids can't be sent on their own. They are just part of some other class. ////
    friend class cereal::access;

    template <class Archive>
    void save(Archive& ar, const unsigned int version) const
    {
        ar( cellBufferSize_,
            cereal::binary_data(cells_, sizeof(T)*cellBufferSize_), // buffer size is number of values of T, not raw bytes
            width_,
            height_,
            metersPerCell_,
            cellsPerMeter_,
            boundary_);
    }

    template <class Archive>
    void load(Archive& ar, const unsigned int version)
    {
        std::size_t bufferSize;
        ar(bufferSize);

        if(cellBufferSize_ != bufferSize)
        {
            delete [] cells_;
            cells_ = nullptr;
        }

        if(!cells_)
        {
            cells_ = new T[bufferSize];
        }

        cellBufferSize_ = bufferSize;

        ar( cereal::binary_data(cells_, sizeof(T)*cellBufferSize_), // buffer size is number of values of T, not raw bytes
            width_,
            height_,
            metersPerCell_,
            cellsPerMeter_,
            boundary_);
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
template <class Grid, typename T>
Grid transform_grid(const Grid& grid, T initialValue, float xRef, float yRef, float thetaRef)
{
    /*
    * Need to find the new width and height of the grid after rotation. Find the radius of the grid.
    * Take the angle of the radius and rotate it by theta to find where the radius cell lies in the
    * rotated grid. These will be the maxima to determine the width and height of the new grid.
    */

    float halfWidth  = grid.getWidthInCells() / 2.0f;
    float halfHeight = grid.getHeightInCells() / 2.0f;

    float radiusAngle = atan2(halfHeight, halfWidth);
    float radius      = ceil(sqrt(halfWidth*halfWidth + halfHeight*halfHeight));

    int newHalfWidth  = ceil(radius * std::max(std::abs(cos(radiusAngle - thetaRef)), std::abs(cos(-radiusAngle - thetaRef))));
    int newHalfHeight = ceil(radius * std::max(std::abs(sin(radiusAngle - thetaRef)), std::abs(sin(-radiusAngle - thetaRef))));

#ifdef DEBUG_CHANGE_FRAME
    std::cout<<"INFO:transform_grid:Old dim:("<<halfWidth<<','<<halfHeight<<") New dim:("<<newHalfWidth<<','<<newHalfHeight<<")\n";
#endif

    Grid rotated(grid);
    rotated.setGridSizeInCells(2*newHalfWidth, 2*newHalfHeight);
    rotated.reset(initialValue);

    Point<float> originalCell;
    Point<float> rotatedCell;

    int centerX = grid.getWidthInCells() / 2;
    int centerY = grid.getHeightInCells() / 2;

    // Go through each cell in the rotated grid to ensure no holes crop up in the new image rather than only converting
    // the old lpm cell-by-cell into the new one.
    for(std::size_t y = 0; y < rotated.getHeightInCells(); ++y)
    {
        rotatedCell.y = y - newHalfHeight;

        for(std::size_t x = 0; x < rotated.getWidthInCells(); ++x)
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
    Point<float> oldCenter = grid.getGlobalCenter();

    // Use the cell dimensions/2 in order to make sure the bottom left is an integral number of cells from the center, which it needs to be
    rotated.setBottomLeft(Point<float>(oldCenter.x - newHalfWidth*rotated.metersPerCell()  - xRef,
                                             oldCenter.y - newHalfHeight*rotated.metersPerCell() - yRef));

#ifdef DEBUG_CHANGE_FRAME
    std::cout<<"INFO:transform_grid:Old BL:"<<grid.getBottomLeft()<<" New BL:"<<rotated.getBottomLeft()<<'\n';
#endif

    return rotated;
}


// Implementation of the CellGrid interface for the non-one-liner methods
template <typename T>
CellGrid<T>::CellGrid(void)
: cells_(0)
, cellBufferSize_(0)
, width_(0)
, height_(0)
, metersPerCell_(0.314159) // have a non-zero scale so global_point_to_grid_cell on a default constructed cell grid doesn't divide by 0 but not one that we might actually use
, cellsPerMeter_(1.0 / metersPerCell_)
{
    // Don't worry about setting all parameters, just those that concern general operation
}


template <typename T>
CellGrid<T>::CellGrid(double metersPerCell)
: cells_(0)
, cellBufferSize_(0)
, width_(0)
, height_(0)
, metersPerCell_(metersPerCell)
, cellsPerMeter_(1.0 / metersPerCell_)
{
}


template <typename T>
CellGrid<T>::CellGrid(std::size_t gridWidth, std::size_t gridHeight, double metersPerCell, const Point<float>& globalCenter, T initialValue)
: cells_(0)
, cellBufferSize_(0)
, width_(gridWidth)
, height_(gridHeight)
, metersPerCell_(metersPerCell)
, cellsPerMeter_(1.0 / metersPerCell_)
{
    assert(width_ > 0);
    assert(height_ > 0);

    // Calculate the boundary in two parts because truncation can lead to inaccurate results otherwise.
    // ALL calculations MUST be done relative to the establish bottomLeft corner, otherwise the implied truncations
    // from the casts to integers result in broken maps
    updateBoundary(bottomLeftFromCenter(globalCenter));

    cellBufferSize_ = width_ * height_;
    cells_ = new T[cellBufferSize_];
    reset(initialValue);
}

template <typename T>
CellGrid<T>::CellGrid(const Point<float>& origin, double width, double height, double cellsPerMeter, T initialValue)
: cells_(0)
, cellBufferSize_(0)
, width_(std::lrint(width * cellsPerMeter))
, height_(std::lrint(height * cellsPerMeter))
, metersPerCell_(1.0 / cellsPerMeter)
, cellsPerMeter_(cellsPerMeter)
{
    assert(width_ > 0);
    assert(height_ > 0);

    updateBoundary(origin);

    cellBufferSize_ = width_ * height_;
    cells_ = new T[cellBufferSize_];
    reset(initialValue);
}


template <typename T>
CellGrid<T>::CellGrid(const CellGrid<T>& gridToCopy)
: cells_(0)
, cellBufferSize_(0)
, width_(gridToCopy.width_)
, height_(gridToCopy.height_)
, metersPerCell_(gridToCopy.metersPerCell_)
, cellsPerMeter_(gridToCopy.cellsPerMeter_)
, boundary_(gridToCopy.boundary_)
{
    cellBufferSize_ = width_*height_;
    cells_ = new T[cellBufferSize_];
    copy_grid(cells_, gridToCopy.cells_, width_, height_);
}


template <typename T>
CellGrid<T>::CellGrid(CellGrid<T>&& toMove)
: cells_(toMove.cells_)
, cellBufferSize_(toMove.cellBufferSize_)
, width_(toMove.width_)
, height_(toMove.height_)
, metersPerCell_(toMove.metersPerCell_)
, cellsPerMeter_(toMove.cellsPerMeter_)
, boundary_(toMove.boundary_)
{
    toMove.cells_          = 0;
    toMove.cellBufferSize_ = 0;
    toMove.width_          = 0;
    toMove.height_         = 0;
}


template <typename T>
CellGrid<T>& CellGrid<T>::operator=(const CellGrid<T>& rhs)
{
    if((cellBufferSize_ < rhs.width_*rhs.height_) && cells_)
    {
        delete [] cells_;

        cells_ = 0;
        cellBufferSize_ = 0;
    }

    width_         = rhs.width_;
    height_        = rhs.height_;
    metersPerCell_ = rhs.metersPerCell_;
    cellsPerMeter_ = rhs.cellsPerMeter_;
    boundary_      = rhs.boundary_;

    setGridSizeInCells(width_, height_);

    copy_grid(cells_, rhs.cells_, width_, height_);

    return *this;
}


template <typename T>
CellGrid<T>& CellGrid<T>::operator=(CellGrid<T>&& rhs)
{
    std::swap(cells_, rhs.cells_);
    std::swap(cellBufferSize_, rhs.cellBufferSize_);

    width_         = rhs.width_;
    height_        = rhs.height_;
    metersPerCell_ = rhs.metersPerCell_;
    cellsPerMeter_ = rhs.cellsPerMeter_;
    boundary_      = rhs.boundary_;

    return *this;
}


template <typename T>
CellGrid<T>::~CellGrid(void)
{
    if(cells_)
    {
        delete [] cells_;
    }
}


// Methods for mutating the grid
template <typename T>
Point<float> CellGrid<T>::getGlobalCenter(void) const
{
    return Point<float>(boundary_.bottomLeft.x + (width_/2)*metersPerCell_,
                              boundary_.bottomLeft.y + (height_/2)*metersPerCell_);
}


template <typename T>
void CellGrid<T>::setGridSizeInCells(std::size_t width, std::size_t height)
{
    if(width*height > cellBufferSize_)
    {
        delete [] cells_;
        cellBufferSize_ = width*height;
        cells_ = new T[width * height];
    }

    width_  = width;
    height_ = height;

    updateBoundary(boundary_.bottomLeft);
}


// Methods for modifying position of the grid
template <typename T>
void CellGrid<T>::changeBoundary(const math::Rectangle<float>& newBoundary, T initialValue)
{
    if((newBoundary.bottomLeft == boundary_.bottomLeft) && (newBoundary.topRight == boundary_.topRight))
    {
        return;
    }

    std::size_t newWidth  = ceil((newBoundary.topRight.x - newBoundary.bottomLeft.x) * cellsPerMeter_);
    std::size_t newHeight = ceil((newBoundary.topRight.y - newBoundary.bottomLeft.y) * cellsPerMeter_);

    assert(newWidth > 0 && newHeight > 0);

    T* newGrid = nullptr;

    if((newWidth != width_) || (newHeight != height_))
    {
        newGrid = new T[newWidth * newHeight];
        std::fill(newGrid, newGrid + newWidth * newHeight, initialValue);
    }

    if(newBoundary.intersection(boundary_).area() != 0)
    {
        boundary_intersection_t coords(newBoundary, boundary_, cellsPerMeter_, std::make_pair(newWidth, newHeight), std::make_pair(width_, height_));

        int xShift = coords.gridStartCell.x;
        int yShift = coords.gridStartCell.y;

        xShift -= coords.updateStartCell.x;
        yShift -= coords.updateStartCell.y;

        adjustBottomLeft(xShift, yShift);

#ifdef DEBUG_RESHAPE
        std::cout<<"DEBUG:CellGrid::changeBoundary: coords:"<<coords.updateWidth<<'x'<<coords.updateHeight<<" old:"<<width<<'x'<<height<<" new:"<<newWidth<<'x'<<newHeight
                    <<" old bl:"<<boundary.bottomLeft<<" new bl:"<<newBottomLeft<<" shift:"<<xShift<<'x'<<yShift<<'\n';
#endif

        // If a new grid was created, just need to copy over the internal region
        if(newGrid)
        {
            copy_boundary_region(coords, newGrid, cells_, newWidth, width_);
        }
        // Otherwise, the grid needs to be carefully shifted by the correct amount
        else
        {
            shiftGrid(xShift, yShift, initialValue);
        }
    }
    else
    {
        // If there's no intersection, then reset the grid to the initial value if a new grid isn't already being
        // assigned (that grid has the cell filled in correctly already)
        if(!newGrid)
        {
            reset(initialValue);
        }
        updateBoundary(newBoundary.bottomLeft);
    }

    // If a new grid is being created, get rid of the old grid and store the new data
    if(newGrid)
    {
        if(cells_)
        {
            delete [] cells_;
        }

        cells_          = newGrid;
        cellBufferSize_ = newWidth*newHeight;
        width_          = newWidth;
        height_         = newHeight;
        updateBoundary(boundary_.bottomLeft);
    }
}


template <typename T>
void CellGrid<T>::changeGlobalCenter(const Point<float>& newGlobalCenter, T initialValue)
{
    // Changing the global center requires shifting the whole grid to reflect the new position
    Point<float> newBottomLeft = bottomLeftFromCenter(newGlobalCenter);

    if(newBottomLeft != boundary_.bottomLeft)
    {
        std::pair<int, int> cells_to_shift = calc_cells_to_shift(boundary_.bottomLeft, newBottomLeft, cellsPerMeter_);

        shiftGrid(cells_to_shift.first, cells_to_shift.second, initialValue);
        adjustBottomLeft(cells_to_shift.first, cells_to_shift.second);
    }
}


template <typename T>
void CellGrid<T>::reset(T initialValue)
{
    // Going to skip the setValue abstraction that would be more proper to use because
    // the extra checking that occurs in there is unnecessary
    for(int y = height_; --y >= 0;)
    {
        for(int x = width_; --x >= 0;)
        {
            cells_[cell_to_index(x, y, width_)] = initialValue;
        }
    }
}


// Methods for manipulating the costs in the grid
template <typename T>
T CellGrid<T>::getValue(int cellX, int cellY) const
{
    if(isCellInGrid(cellX, cellY))
    {
        return getValueNoCheck(cellX, cellY);
    }
    else
    {
        return T();
    }
}


template <typename T>
void CellGrid<T>::setValue(int cellX, int cellY, const T& value)
{
    if(isCellInGrid(cellX, cellY))
    {
        setValueNoCheck(cellX, cellY, value);
    }
}


template <typename T>
void CellGrid<T>::shiftGrid(int deltaX, int deltaY, T initialValue)
{
    /*
    * For the shift, both x and y shifts are handled at the same time. The idea here is to start
    * at the end of the shifted region and move backwards through it so that no temporaries need
    * to be created.
    *
    * First, shift the (x, y) values, then fill in the blanks with initialValue afterward.
    */

    grid_shift_params_t xShift(deltaX, width_);
    grid_shift_params_t yShift(deltaY, height_);

    shift_grid_cells    (xShift, yShift, cells_, width_);
    reset_new_grid_cells(xShift, yShift, cells_, initialValue, width_);
}


template <typename T>
Point<float> CellGrid<T>::bottomLeftFromCenter(const Point<float>& center)
{
    return Point<float>(center.x - getWidthInMeters()/2.0f, center.y - getHeightInMeters()/2.0f);
}

template <typename T>
void CellGrid<T>::adjustBottomLeft(int deltaX, int deltaY)
{
    // The new bottom left is not based on the newBottomLeft because the grid only shifts in whole cell increments
    // as a result, the center only moves the shift*cellScale() in each direction. Doing otherwise results in a
    // gradual drift in the map as the small shift errors accumulate
    Point<float> newBottomLeft;
    newBottomLeft.x = boundary_.bottomLeft.x - deltaX * metersPerCell_;
    newBottomLeft.y = boundary_.bottomLeft.y - deltaY * metersPerCell_;
    updateBoundary(newBottomLeft);
}

template <typename T>
void CellGrid<T>::updateBoundary(const Point<float>& newBottomLeft)
{
    boundary_ = math::Rectangle<float>(newBottomLeft,
                                      Point<float>(newBottomLeft.x + getWidthInMeters(), newBottomLeft.y + getHeightInMeters()));
}


extern template class CellGrid<uint8_t>;
extern template class CellGrid<int16_t>;
extern template class CellGrid<uint16_t>;

} // namespace utils
} // namespace vulcan

#endif // UTILS_CELL_GRID_H
