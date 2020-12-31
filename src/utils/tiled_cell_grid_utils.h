/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     tiled_cell_grid.h
* \author   Collin Johnson
* 
* Utility functions for dealing with the TiledCellGrid:
* 
* 
*/

#ifndef UTILS_TILED_CELL_GRID_UTILS_H
#define UTILS_TILED_CELL_GRID_UTILS_H

#include "core/point.h"
#include "utils/cell_grid_utils.h"

namespace vulcan
{
namespace utils
{

inline int round_up_to_nearest_n(int value, const int N)
{
    return value  + (N - (value % N)) % N;
}

/**
* cell_to_tile finds the tile in which a cell belongs.
* 
* \param    cell            Cell for which to find the tile
* \param    grid            TiledCellGrid containing the cell
* \return   Tile containing the cell.
*/
template <class TiledGrid>
Point<int> cell_to_tile(Point<int> cell, const TiledGrid& grid)
{
    return Point<int>(cell.x / grid.getTileSize(), cell.y / grid.getTileSize());
}

/**
* position_to_tile finds the tile in which a global position belongs.
* 
* \param    position        Position in global coordinates
* \param    grid            TiledCellGrid containing the position
* \return   Tile containing the position.
*/
template <class TiledGrid>
Point<int> position_to_tile(Point<double> position, const TiledGrid& grid)
{
    return cell_to_tile(global_point_to_grid_cell(position, grid), grid);
}

/**
* tile_origin_cell retrieves the origin cell of a tile.
*/
template <class TiledGrid>
Point<int> tile_origin_cell(Point<int> tile, const TiledGrid& grid)
{
    return Point<int>(tile.x * grid.getTileSize(), tile.y * grid.getTileSize());
}

/**
* tile_origin_position retrieves the origin position of a tile.
*/
template <class TiledGrid>
Point<double> tile_origin_position(Point<int> tile, const TiledGrid& grid)
{
    return grid_point_to_global_point(tile_origin_cell(tile, grid), grid);
}

}
}

#endif // UTILS_TILED_CELL_GRID_UTILS_H
