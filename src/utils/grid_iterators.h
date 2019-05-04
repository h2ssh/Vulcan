/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     grid_iterators.h
* \author   Collin Johnson
* 
* Definition of iterators for Grids:
* 
*   - GridIterator : iterate through a standard, non-tiled grid
*   - TiledGridIterator : iterate through a grid with cells organized into tiles
*/

#ifndef UTILS_GRID_ITERATORS_H
#define UTILS_GRID_ITERATORS_H

#include <iterator>
#include <cassert>

namespace vulcan
{
namespace utils
{

/**
* GridIterator
*/
template <class T, class Grid>
class GridIterator : public std::iterator<std::forward_iterator_tag, T>
{
public:
    
    using iterator = GridIterator<T, Grid>;
    
    /**
    * Constructor for GridIterator.
    * 
    * Create an iterator across the entire grid.
    */
    GridIterator(Grid& grid)
    : GridIterator(0, 0, grid.getWidthInCells(), grid)
    {
    }
    
    /**
    * Constructor for GridIterator.
    * 
    * Create an iterator for a rectangular portion of the grid from x to the end of the row.
    */
    GridIterator(int cellX, int cellY, Grid& grid)
    : GridIterator(cellX, cellY, grid.getWidthInCells() - cellX, grid)
    {
    }
    
    /**
    * Constructor for GridIterator.
    * 
    * Create an iterator for a rectangular view of the grid from x to x + width.
    */
    GridIterator(int cellX, int cellY, int widthInCells, Grid& grid)
    : grid_(&grid)
    , x_(cellX)
    , y_(cellY)
    , startX_(x_)
    , endX_(startX_ + widthInCells)
    {
        assert(endX_ <= static_cast<int>(grid.getWidthInCells()));
    }
    
    // Allow copy and move construction
    GridIterator(const GridIterator& rhs) = default;
    GridIterator& operator=(const GridIterator& rhs) = default;
    GridIterator(GridIterator&& rhs) = default;
    GridIterator& operator=(GridIterator&& rhs) = default;
    
    // Cell being pointed to by the iterator
    int x(void) const { return x_; }
    int y(void) const { return y_; }
    
    // forward_iterator interface
    
    /**
    * Two GridIterators are equal if they point to the same cell.
    */
    bool operator==(const iterator& rhs) { return !operator!=(rhs); }
    bool operator!=(const iterator& rhs) { return (x_ != rhs.x_) || (y_ != rhs.y_); }
    
    /**
    * The dereference points to the underlying cell value. Both modifying and non-modifying versions exist.
    */
    const T& operator*(void) const { return (*grid_)(x_, y_); }
    T& operator*(void) { return (*grid_)(x_, y_); }
    
    /**
    * Increment goes to the next cell in the current tile or switches tiles.
    */
    iterator& operator++(void)
    {
        increment();
        return *this;
    }
    
    iterator operator++(int)
    {
        auto tmp = *this;
        increment();
        return tmp;
    }
    
private:
    
    Grid* grid_;
    int x_;
    int y_;
    int startX_;
    int endX_;
    
    void increment(void)
    {
        ++x_;
        
        // If reached the end of the row, jump up to the next row
        if(x_ == endX_)
        {
            ++y_;
            x_ = startX_;
        }
    }
};


/**
* make_grid_region_iterators creates a range for iterating through a rectangle region of a Grid.
*/
template <class T, class Grid>
std::pair<GridIterator<T, Grid>, GridIterator<T, Grid>> 
    make_grid_region_iterators(int startX, int startY, int widthInCells, int heightInCells, Grid& grid)
{
    // The iteration goes in row-major order through the cells, so one-past-the-end is the first cell of the row
    // above the desired region
    using iterator = GridIterator<T, Grid>;
    return std::make_pair(iterator(startX, startY, widthInCells, grid),
                          iterator(startX, startY + heightInCells, widthInCells, grid));
}


/**
* make_grid_iterators creates a range for iterating through an entire Grid
*/
template <class T, class Grid>
std::pair<GridIterator<T, Grid>, GridIterator<T, Grid>> 
    make_grid_iterators(Grid& grid)
{
    // The iteration goes in row-major order through the cells, so one-past-the-end is the first cell of the row
    // above the desired region
    using iterator = GridIterator<T, Grid>;
    return std::make_pair(iterator(0, 0, grid.getWidthInCells(), grid),
                          iterator(0, grid.getHeightInCells(), grid.getWidthInCells(), grid));
}


/**
* TiledGridIterator
*/
template <class T, class TiledGrid>
class TiledGridIterator : public std::iterator<std::forward_iterator_tag, T>
{
public:
    
    using iterator = TiledGridIterator<T, TiledGrid>;
    using cell_iterator = GridIterator<T, TiledGrid>;
    
    /**
    * Constructor for TiledGrid.
    * 
    * Create an iterator across the entire grid.
    */
    TiledGridIterator(TiledGrid& grid)
    : TiledGridIterator(0, 0, grid.getWidthInCells(), grid)
    {
    }
    
    /**
    * Constructor for TiledGridIterator.
    * 
    * Create an iterator for a rectangle view of the tiles from x to the end of the row.
    */
    TiledGridIterator(int tileX, int tileY, TiledGrid& grid)
    : TiledGridIterator(tileX, tileY, grid.getWidthInTiles() - tileX, grid)
    {
    }
    
    /**
    * Constructor for TiledGridIterator.
    * 
    * Create an iterator for a rectangular view of the tiles in the range [x, x + widthInTiles)
    */
    TiledGridIterator(int tileX, int tileY, int widthInTiles, TiledGrid& grid)
    : grid_(grid)
    , x_(tileX)
    , y_(tileY)
    , startX_(tileX)
    , endX_(startX_ + widthInTiles)
    {
        assert(endX_ <= static_cast<int>(grid.getWidthInTiles()));
    }
    
    // Cell being pointed to by the iterator
    int x(void) const { return x_; }
    int y(void) const { return y_; }
    
    // Iterate through cells points to by the current tile
    cell_iterator begin(void)
    { 
        return cell_iterator(x_ * grid_.getTileSize(), 
                             y_ * grid_.getTileSize(),
                             grid_.getTileSize(),
                             grid_); 
    }
    
    cell_iterator end(void)
    {  
        return cell_iterator(x_ * grid_.getTileSize(), 
                             (y_ + 1) * grid_.getTileSize(),
                             grid_.getTileSize(),
                             grid_); 
    }
    
    // forward_iterator interface
    
    /**
    * Two TiledGridIterators are equal if they point to the same cell.
    */
    bool operator==(const iterator& rhs) { return !operator!=(rhs); }
    bool operator!=(const iterator& rhs) { return (x_ != rhs.x_) || (y_ != rhs.y_); }
    
    /**
    * The dereference points to the underlying cell value for the origin of the active cell. Both modifying and 
    * non-modifying versions exist.
    */
    const T& operator*(void) const { return grid_(x_ * grid_.getTileSize(), y_* grid_.getTileSize()); }
    T& operator*(void) { return grid_(x_ * grid_.getTileSize(), y_ * grid_.getTileSize()); }
    
    /**
    * Increment goes to the next cell in the current tile or switches tiles.
    */
    iterator& operator++(void)
    {
        increment();
        return *this;
    }
    
    iterator operator++(int)
    {
        auto tmp = *this;
        increment();
        return tmp;
    }
    
private:
    
    TiledGrid& grid_;
    int x_;
    int y_;
    int startX_;
    int endX_;
    
    void increment(void)
    {
        ++x_;
        
        // If reached the end of the row, jump up to the next row
        if(x_ == endX_)
        {
            ++y_;
            x_ = startX_;
        }
    }
};


/**
* make_tiled_region_iterators creates a range for iterating through a rectangle region of a TiledGrid.
*/
template <class T, class TiledGrid>
std::pair<TiledGridIterator<T, TiledGrid>, TiledGridIterator<T, TiledGrid>> 
    make_tiled_region_iterators(int startTileX, int startTileY, int widthInTiles, int heightInTiles, TiledGrid& grid)
{
    // The iteration goes in row-major order through the tiles, so one-past-the-end is the first tile of the row
    // above the desired region
    using iterator = TiledGridIterator<T, TiledGrid>;
    return std::make_pair(iterator(startTileX, startTileY, widthInTiles, grid),
                          iterator(startTileX, startTileY + heightInTiles, widthInTiles, grid));
}
    
} // namespace utils
} // namespace vulcan

#endif // UTILS_GRID_ITERATORS_H
