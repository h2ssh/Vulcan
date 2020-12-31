/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     cell_grid_utils.h
* \author   Collin Johnson
*
* Definition of following functionality for operations on CellGrids:
*
* Cell indexing:
*
*   - cell_to_index : calculate index of a given cell
*   - index_to_cell : calculate the cell for a given index
* 
* Coordinate transforms:
*   
*   - grid_point_to_global_point : find the global position of a grid cell
*   - global_point_to_grid_cell  : find the cell for a global position
*   - global_point_to_grid_point : find the grid position, not just cell, for a global position
*
* Boundary-based map updates:
*   - boundary_update_coords_t : cells within the grid and update region for copying a region into a grid
*   - copy_boundary_region : copies the boundary region into the appropriate part of the grid using the provided coordinates
*
* Grid shifting:
*   - grid_shift_params_t  : struct that calculates the indices by which a shift will occur, for use in the shift functions
*   - shift_grid_cells     : shift a grid by a specified number of cells along the x and y directions
*   - reset_new_grid_cells : set the newly blanked out cells when shifting to some default value
*
* Memory operations:
*   - allocate_grid        : allocate a new grid with the requested dimensions
*   - copy_grid            : copy a grid to another buffer
*   - deallocate_grid      : free up the memory associated with a grid
*/

#ifndef UTILS_CELL_GRID_UTILS_H
#define UTILS_CELL_GRID_UTILS_H

#include "core/point.h"
#include "math/geometry/rectangle.h"
#include "core/float_comparison.h"
#include <cassert>
#include <cstdint>
#include <cstring> // for memcpy

// #define DEBUG_BOUNDARY_COORDS

#ifdef DEBUG_BOUNDARY_COORDS
#include <iostream>
#endif

namespace vulcan
{
namespace utils
{

inline uint32_t cell_to_index(int x, int y, std::size_t width)
{
    return y*width + x;
}

inline Point<int> index_to_cell(int cell, std::size_t width)
{
    return Point<int>(cell % width, cell / width);
}


/**
* grid_point_to_global_point converts a point in the grid coordinate system to the global
* coordinate system.
*
* \param    gridPoint       Point to be converted
* \param    grid            Grid in which gridPoint exists
* \return   Point in the global coordinate frame.
*/
template <typename Grid, typename T>
Point<double> grid_point_to_global_point(const Point<T>& gridPoint, const Grid& grid)
{
    return Point<double>(grid.getBottomLeft().x + gridPoint.x*grid.metersPerCell(),
                               grid.getBottomLeft().y + gridPoint.y*grid.metersPerCell());
}

/**
* global_point_to_grid_point converts a point in the global coordinate system to a point in the grid coordinate system. The
* point can be fractions of a grid cell so no information is lost converting back and forth between global and grid points.
*
* \param    globalPoint     Point to be converted
* \param    grid            CellGrid in which globalPoint will be converted
* \return   Point in the grid coordinate frame.
*/
template <typename Grid>
Point<double> global_point_to_grid_point(const Point<double>& globalPoint, const Grid& grid)
{
    return Point<double>((globalPoint.x - grid.getBottomLeft().x) * grid.cellsPerMeter(),
                               (globalPoint.y - grid.getBottomLeft().y) * grid.cellsPerMeter());
}


/**
* global_point_to_grid_cell converts a point in the global coordinate system to a cell in the grid coordinate system.
* 
* NOTE: This function only returns the correct value when the grid cell is positive. All cells actually in the grid have
* a positive coordinate, so for all practical purposes this function works correctly.
*
* \param    globalPoint     Point to be converted
* \param    grid            CellGrid in which globalPoint will be converted
* \return   Cell in the grid coordinate frame.
*/
template <typename Grid>
Point<int> global_point_to_grid_cell(const Point<double>& globalPoint, const Grid& grid)
{
    return Point<int>(static_cast<int>((globalPoint.x - grid.getBottomLeft().x) * grid.cellsPerMeter()),
                            static_cast<int>((globalPoint.y - grid.getBottomLeft().y) * grid.cellsPerMeter()));
}


/**
* global_point_to_grid_cell_round converts a point in the global coordinate system to a cell in the grid coordinate
* system. The round version of the function helps deal with truncation problems associated with not-quite exact floating
* point math. Any values within 1e-5 of the next higher integer will be rounded to that integer.
* 
* This function is substantially slower than non-rounding version and should only be used where precision is essential.
* Furthermore, using doubles should alleviate most problems associated with floating-point issues within the grids,
* which aren't particularly large.
*
* \param    globalPoint     Point to be converted
* \param    grid            CellGrid in which globalPoint will be converted
* \return   Cell in the grid coordinate frame.
*/
template <typename Grid>
Point<int> global_point_to_grid_cell_round(const Point<double>& globalPoint, const Grid& grid)
{
    const double kErrorDelta = 1e-3;
    
    Point<double> gridPoint = global_point_to_grid_point(globalPoint, grid);
    Point<int> gridCell = gridPoint;
    
    // Truncation goes towards 0, so if above 0, then check if adding delta changes the cell 
    if((gridPoint.x > 0) && (static_cast<int>(gridPoint.x + kErrorDelta) != gridCell.x))
    {
        gridCell.x = gridPoint.x + kErrorDelta;
    }
    // If less than 0, check if subtracting delta changes the cell
    else if((gridPoint.x < 0) && (static_cast<int>(gridPoint.x - kErrorDelta) != gridCell.x))
    {
        gridCell.x = gridPoint.x - kErrorDelta;
    }
    
    // Repeat the process for the y-coordinate of the cell
    if((gridPoint.y > 0) && (static_cast<int>(gridPoint.y + kErrorDelta) != gridCell.y))
    {
        gridCell.y = gridPoint.y + kErrorDelta;
    }
    else if((gridPoint.y < 0) && (static_cast<int>(gridPoint.y - kErrorDelta) != gridCell.y))
    {
        gridCell.y = gridPoint.y - kErrorDelta;
    }
    
    return gridCell;
}


// Boundary copying operations
/**
* boundary_update_coords_t considers the boundary of two grids. The update coords are set as the intersection of
* the two regions. The corresponding cells and dimensions of the area in the grids to be updated are the values
* stored in the boundary coordinates.
*/
struct boundary_intersection_t
{
    math::Rectangle<float> boundary;

    Point<int> gridStartCell;
    Point<int> updateStartCell;

    std::size_t updateWidth;
    std::size_t updateHeight;

    /**
    * Constructor for boundary_update_coords_t.
    *
    * \param    gridBoundary            Boundary of the grid in which the update is being made
    * \param    updateBoundary          Boundary of the updated region being applied to the grid
    * \param    cellsPerMeter           Cells per meter in the grid
    * \param    gridDimensions          .first = width in cells, .second = height in cells
    * \param    updateDimensions        .first = width in cells, .second = height in cells
    */
    boundary_intersection_t(const math::Rectangle<float>& gridBoundary,
                            const math::Rectangle<float>& updateBoundary,
                            float cellsPerMeter,
                            std::pair<std::size_t, std::size_t> gridDimensions,
                            std::pair<std::size_t, std::size_t> updateDimensions)
    : updateWidth(0)
    , updateHeight(0)
    {
        boundary = gridBoundary.intersection(updateBoundary);

        if(boundary.area() > 0)
        {
            gridStartCell.x = round_in_tolerance((boundary.bottomLeft.x - gridBoundary.bottomLeft.x) * cellsPerMeter);
            gridStartCell.y = round_in_tolerance((boundary.bottomLeft.y - gridBoundary.bottomLeft.y) * cellsPerMeter);

            updateStartCell.x = round_in_tolerance((boundary.bottomLeft.x - updateBoundary.bottomLeft.x) * cellsPerMeter);
            updateStartCell.y = round_in_tolerance((boundary.bottomLeft.y - updateBoundary.bottomLeft.y) * cellsPerMeter);

            updateWidth  = ceil((boundary.topRight.x - boundary.bottomLeft.x) * cellsPerMeter);
            updateHeight = ceil((boundary.topRight.y - boundary.bottomLeft.y) * cellsPerMeter);

            assert(boundary.topRight.x > boundary.bottomLeft.x);
            assert(boundary.topRight.y > boundary.bottomLeft.y);

            // After the calculations, go through and confirm that floating point errors haven't created an untenable and memory-crashing situation
            // by adjusting the boundary dimensions as needed based on the input sizes.
            // Explicitly supplying these dimensions because other floating-point errors could result in incorrect calculations anyhow
            if(updateHeight + gridStartCell.y > gridDimensions.second)
            {
                updateHeight = gridDimensions.second - gridStartCell.y;
            }

            if(updateHeight + updateStartCell.y > updateDimensions.second)
            {
                updateHeight = updateDimensions.second - updateStartCell.y;
            }

            if(updateWidth + gridStartCell.x > gridDimensions.first)
            {
                updateWidth = gridDimensions.first - gridStartCell.x;
            }

            if(updateWidth + updateStartCell.x > updateDimensions.first)
            {
                updateWidth = updateDimensions.first - updateStartCell.x;
            }

#ifdef DEBUG_BOUNDARY_COORDS
            std::cout<<"DEBUG:boundary_coords:Intersection:"<<boundary<<" Grid:"<<gridStartCell<<" Boundary:"<<updateStartCell<<" Width:"<<updateWidth<<" Height:"<<updateHeight<<'\n'
                     <<"Dimensions:Grid:"<<gridDimensions.first<<','<<gridDimensions.second<<" Update:"<<updateDimensions.first<<','<<updateDimensions.second<<'\n';
#endif
        }
    }
};

template <typename T>
void copy_boundary_region(const boundary_intersection_t& coords, T* grid, T* update, std::size_t gridWidth, std::size_t updateWidth)
{
    if(coords.updateWidth * coords.updateHeight == 0)
    {
        return;
    }

    #ifdef DEBUG_BOUNDARY_COORDS
    std::cout<<"DEBUG:copy_boundary: boundary:"<<coords.updateWidth<<" update:"<<updateWidth<<" grid:"<<gridWidth<<" height:"<<coords.updateHeight
             <<" grid start:"<<coords.gridStartCell<<" update start:"<<coords.updateStartCell<<'\n';
    #endif

    assert(coords.updateWidth <= updateWidth);
    assert(coords.updateWidth <= gridWidth);
    assert(coords.gridStartCell.x + coords.updateWidth <= gridWidth);
    assert(coords.updateStartCell.x + coords.updateWidth <= updateWidth);

    // Shift the buffer pointers to the start of the update region
    grid   += coords.gridStartCell.y*gridWidth + coords.gridStartCell.x;
    update += coords.updateStartCell.y*updateWidth + coords.updateStartCell.x;

    // Copy the update region, then jump to the same spot on the next row for the buffers
    for(std::size_t y = 0; y < coords.updateHeight; ++y)
    {
        memcpy(grid, update, coords.updateWidth*sizeof(T));

        grid   += gridWidth;
        update += updateWidth;
    }
}

// Grid shifting operations
struct grid_shift_params_t
{
    int shift;
    int maxValue;

    int shiftIncrement;
    int shiftStart;
    int shiftEnd;

    // Params for the zero-ing out of unneeded values
    int zeroStart;
    int zeroEnd;

    grid_shift_params_t(int gridShift, int gridMaxValue)
    : shift(gridShift)
    , maxValue(gridMaxValue)
    {
        // If shifting right, then need to start at the rightmost point and move towards the left.
        if(shift > 0)
        {
            shiftIncrement = -1;
            shiftStart     = maxValue - 1;
            shiftEnd       = shift - 1;

            zeroStart = shift - 1;
            zeroEnd   = -1;
        }
        else if(shift < 0) // Shifting left, start at leftmost point and move right
        {
            shiftIncrement = 1;
            shiftStart     = 0;
            shiftEnd       = maxValue + shift;  // shift is negative, adding is subtracting

            zeroStart = maxValue + shift;
            zeroEnd   = maxValue;
        }
        else  // shift == 0
        {
            shiftIncrement = 1;
            shiftStart     = 0;
            shiftEnd       = maxValue;

            zeroStart = 0;
            zeroEnd   = 0;
        }
    }
};

// Helper functions for the shift
/**
* calc_cells_to_shift determines the number of cells to shift the grid from its old origin to a new origin. The origin
* is moved in units of cells. The translation truncates to the nearest cell. Thus, shifting cellsPerMeter - delta means
* there is no shift, that is:
*
*       shift = 0 if delta in [0, cellsPerMeter)
*
* \param    bottomLeft          Current bottom left of the grid
* \param    newBottomLeft       New bottom left coordinate for the grid
* \return   Number of cells to shift along the x (.first) and y (.second) axes.
*/
inline std::pair<int, int> calc_cells_to_shift(const Point<float>& bottomLeft,
                                               const Point<float>& newBottomLeft,
                                               double cellsPerMeter)
{
    int xShift = std::lround((bottomLeft.x - newBottomLeft.x) * cellsPerMeter);
    int yShift = std::lround((bottomLeft.y - newBottomLeft.y) * cellsPerMeter);

    return std::make_pair(xShift, yShift);
}

template <typename T>
void shift_grid_cells(const grid_shift_params_t& xShift, const grid_shift_params_t& yShift, T* gridValues, std::size_t width)
{
    for(int y = yShift.shiftStart; y != yShift.shiftEnd; y += yShift.shiftIncrement)
    {
        for(int x = xShift.shiftStart; x != xShift.shiftEnd; x += xShift.shiftIncrement)
        {
            gridValues[cell_to_index(x, y, width)] = gridValues[cell_to_index(x - xShift.shift, y - yShift.shift, width)];
        }
    }
}

template <typename T>
void reset_new_grid_cells(const grid_shift_params_t& xShift, const grid_shift_params_t& yShift, T* gridValues, T initialValue, uint16_t width)
{
    // Now fill in the blanks
    for(int y = yShift.zeroStart; y != yShift.zeroEnd; y += yShift.shiftIncrement)
    {
        for(int x = 0; x < xShift.maxValue; ++x)
        {
            gridValues[cell_to_index(x, y, width)] = initialValue;
        }
    }

    for(int y = 0; y < yShift.maxValue; ++y)
    {
        for(int x = xShift.zeroStart; x != xShift.zeroEnd; x += xShift.shiftIncrement)
        {
            gridValues[cell_to_index(x, y, width)] = initialValue;
        }
    }
}

// Helper functions for CellGrid memory management
template <typename T>
void copy_grid(T* dest, T* src, std::size_t width, std::size_t height)
{
    memcpy(dest, src, width*height*sizeof(T));
}

}
}

#endif // UTILS_CELL_GRID_UTILS_H
