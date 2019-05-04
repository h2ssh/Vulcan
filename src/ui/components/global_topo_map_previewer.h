/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_topo_map_previewer.h
* \author   Collin Johnson
*
* Declaration of GlobalTopoMapPreviewer.
*/

#ifndef UI_COMPONENTS_GLOBAL_TOPO_MAP_PREVIEWER_H
#define UI_COMPONENTS_GLOBAL_TOPO_MAP_PREVIEWER_H

#include <core/point.h>
#include <ui/common/gl_camera.h>
#include <ui/common/ui_color.h>
#include <ui/common/ui_forward_declarations.h>
#include <vector>
#include <map>

namespace vulcan
{
namespace ui
{

class TopologicalMapRenderer;

/**
* grid_cell_t defines a particular cell in the preview grid.
*/
struct grid_cell_t
{
    int row;
    int column;

    explicit grid_cell_t(int row = 0, int column = 0)
    : row(row)
    , column(column)
    {
    }

    bool operator<(const grid_cell_t& rhs) const
    {
        return (row < rhs.row) || (row == rhs.row && column < rhs.column);
    }
};

/**
* preview_cell_contents_t defines the contents of a preview cell. The contents are the id
* of the map to be rendered and the color of the border drawn around the cell.
*/
struct preview_cell_contents_t
{
    GLColor borderColor;
    int mapId;

    explicit preview_cell_contents_t(GLColor color = GLColor())
    : borderColor(color)
    , mapId(-1)
    {
    }
};

/**
* GlobalTopoMapPreviewer is a utility class that allows a number of TopologicalMaps to be displayed
* on the screen at the same time. The previewer creates a grid. Each cell in the grid can be assigned
* a map. The cell will be surrounded by a unique color to allow synchronization with the HypothesisTree.
*/
class GlobalTopoMapPreviewer
{
public:

    /**
    * Constructor for GlobalTopoMapPreviewer.
    *
    * \param    rows        Number of rows  (> 0)
    * \param    columns     Number of columns (> 0)
    */
    GlobalTopoMapPreviewer(int rows, int columns, TopologicalMapRenderer* renderer);

    /**
    * setBoundary sets the boundary, in pixels, in which the grid of maps will be rendered.
    *
    * \param    bottomLeft  Bottom left corner of the region
    * \param    width       Width of the region
    * \param    height      Height of the region
    */
    void setBoundary(const Point<int>& bottomLeft, int width, int height);

    /**
    * setGridDimensions sets the number of rows and columns in the grid.
    *
    * NOTE: Changing the grid dimensions clears the contents of all grid cells.
    *
    * \param    rows        Number of rows (> 0)
    * \param    columns     Number of columns (> 0)
    */
    void setGridDimensions(int rows, int columns);

    /**
    * setMap sets the map to be rendered in a cell.
    *
    * \param    id          Id of the map to be rendered
    * \param    cell        Cell in which the map should be rendered
    * \return   True if the cell is valid. False if it does not exist in the grid.
    */
    bool setMap(int id, grid_cell_t cell);

    /**
    * addMap adds a new map to the grid. The map will be added to the first open cell.
    *
    * \param    id          Id of the map to be added
    * \return   Grid cell to which the map was set. If no cell is open, then the grid cell will be (-1,-1) to indicate failure.
    */
    grid_cell_t addMap(int id);

    /**
    * clearCell empties the contents of a particular cell to allow new maps to be added.
    *
    * \param    cell        Cell to be cleared
    */
    void clearCell(grid_cell_t cell);

    /**
    * clearAllCells empties all cells in the grid.
    */
    void clearAllCells(void);

    /**
    * getCellUnderCursor retrieves the cell the mouse cursor is currently in. If the cursor
    * position does not fall within a cell, then the cell (-1, -1) will be returned to indicate
    * an invalid cell.
    *
    * \param    cursorPosition      Position of the mouse cursor in pixels
    */
    grid_cell_t getCellUnderCursor(const Point<int>& cursorPosition) const;

    /**
    * getContents retrieves the contents of a particular cell. If such a cell does not exist,
    * then the id of the map in the contents will be -1 to indicate empty.
    *
    * \param    cell                Cell for which to get the contents
    * \return   Contents of the cell.
    */
    preview_cell_contents_t getContents(grid_cell_t cell) const;

    /**
    * getActiveCellsContents retrieves the contents of all cells with an associated map.
    *
    * \return   Vector of contents for all cells currently assigned a map.
    */
    std::vector<preview_cell_contents_t> getActiveCellsContents(void) const;

    /**
    * render draws all the preview maps onto the screen. The source of maps needs to be provided,
    * along with the cache of local metric maps. All cells will have their border drawn. Active
    * cells will have a map drawn inside.
    *
    * NOTE: This method modifies the viewpoint, the perspective, and the modelview in dramatic ways.
    *       However, all state -- viewpoint, projection, modelview -- is returned to whatever it was
    *       before the method was called, so all should be well.
    *
    * \param    maps            Collection of all maps from which to pull the preview maps
    * \param    places          Collection of local places
    */
    void render(const hssh::TreeOfMaps& maps, const hssh::MetricMapCache& places);

private:

    void renderCell(const std::pair<grid_cell_t, preview_cell_contents_t>& cell,
                    const hssh::TreeOfMaps& maps,
                    const hssh::MetricMapCache& places);

    bool isValidCell(const grid_cell_t& cell) const
    {
        return (cell.row >= 0) && (cell.row < numRows) && (cell.column >= 0) && (cell.column < numColumns);
    }

    TopologicalMapRenderer* mapRenderer;
    GLCamera camera;

    // INVARIANT: numRows    >= height > 0
    // INVARIANT: numColumns >= width  > 0

    int numRows;
    int numColumns;
    int width;
    int height;
    Point<int> bottomLeft;

    std::map<grid_cell_t, preview_cell_contents_t> cells;
};

} // namespace ui
} // namespace hssh

#endif // UI_COMPONENTS_GLOBAL_TOPO_MAP_PREVIEWER_H
