/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     global_topo_map_previewer.cpp
* \author   Collin Johnson
*
* Definition of GlobalTopoMapPreviewer.
*/

#include <ui/components/global_topo_map_previewer.h>
#include <ui/components/graph_view_topological_map_renderer.h>
#include <ui/common/gl_shapes.h>
#include <ui/common/color_generator.h>
#include <hssh/global_topological/topological_map.h>
#include <hssh/global_topological/mapping/tree_of_maps.h>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

const int EMPTY_CELL_ID = -1;

GlobalTopoMapPreviewer::GlobalTopoMapPreviewer(int rows, int columns, TopologicalMapRenderer* renderer)
    : mapRenderer(renderer)
{
    setGridDimensions(rows, columns);
}


void GlobalTopoMapPreviewer::setBoundary(const Point<int>& bottomLeft, int width, int height)
{
    this->bottomLeft = bottomLeft;

    // Make sure one pixel is allocated to each row and column -- makes other handling easier
    this->width      = (width  > numColumns) ? width  : numColumns;
    this->height     = (height > numRows)    ? height : numRows;
}


void GlobalTopoMapPreviewer::setGridDimensions(int rows, int columns)
{
    numRows    = (rows    > 0) ? rows    : 1;
    numColumns = (columns > 0) ? columns : 1;

    // reset the boundary to ensure it satisifies the invariants now that the grid dimensions are different and the
    // invariants rely on the dimensions
    setBoundary(bottomLeft, width, height);

    cells.clear();

    std::vector<GLColor> cellColors = generate_colors(numRows * numColumns);
    auto                 colorIt    = cellColors.begin();

    for(int row = 0; row < numRows; ++row)
    {
        for(int column = 0; column < numColumns; ++column)
        {
            cells.insert(std::make_pair(grid_cell_t(row, column), preview_cell_contents_t(*colorIt)));
            ++colorIt;
        }
    }
}


bool GlobalTopoMapPreviewer::setMap(int id, grid_cell_t cell)
{
    if(!isValidCell(cell))
    {
        return false;
    }

    cells[cell].mapId = id;

    return true;
}


grid_cell_t GlobalTopoMapPreviewer::addMap(int id)
{
    for(auto cellIt = cells.begin(), cellEnd = cells.end(); cellIt != cellEnd; ++cellIt)
    {
        if(cellIt->second.mapId == EMPTY_CELL_ID)
        {
            cellIt->second.mapId = id;
            return cellIt->first;
        }
    }

    return grid_cell_t(-1, -1);
}


void GlobalTopoMapPreviewer::clearCell(grid_cell_t cell)
{
    if(isValidCell(cell))
    {
        cells[cell].mapId = EMPTY_CELL_ID;
    }
}


void GlobalTopoMapPreviewer::clearAllCells(void)
{
    for(auto cellIt = cells.begin(), cellEnd = cells.end(); cellIt != cellEnd; ++cellIt)
    {
        cellIt->second.mapId = EMPTY_CELL_ID;
    }
}


grid_cell_t GlobalTopoMapPreviewer::getCellUnderCursor(const Point<int>& cursorPosition) const
{
    grid_cell_t cell;

    int columnWidth = width / numColumns;
    int rowHeight   = height / numRows;

    cell.column = (cursorPosition.x - bottomLeft.x) / columnWidth;
    cell.row    = (cursorPosition.y - bottomLeft.y) / rowHeight;

    if((cell.column < 0) || (cell.column >= numColumns))
    {
        cell.column = -1;
    }

    if((cell.row < 0) || (cell.row >= numRows))
    {
        cell.row = -1;
    }

    return cell;
}


preview_cell_contents_t GlobalTopoMapPreviewer::getContents(grid_cell_t cell) const
{
    if(isValidCell(cell))
    {
        return cells.find(cell)->second;
    }

    return preview_cell_contents_t();
}


std::vector<preview_cell_contents_t> GlobalTopoMapPreviewer::getActiveCellsContents(void) const
{
    std::vector<preview_cell_contents_t> active;

    for(auto cellIt = cells.begin(), cellEnd = cells.end(); cellIt != cellEnd; ++cellIt)
    {
        if(cellIt->second.mapId >= 0)
        {
            active.push_back(cellIt->second);
        }
    }

    return active;
}


void GlobalTopoMapPreviewer::render(const hssh::TreeOfMaps& maps, const hssh::MetricMapCache& places)
{
    glPushAttrib(GL_VIEWPORT_BIT);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();

    for(auto cellIt = cells.begin(), cellEnd = cells.end(); cellIt != cellEnd; ++cellIt)
    {
        renderCell(*cellIt, maps, places);
    }

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glPopAttrib();
}


void GlobalTopoMapPreviewer::renderCell(const std::pair<grid_cell_t, preview_cell_contents_t>& cell,
                                        const hssh::TreeOfMaps& maps,
                                        const hssh::MetricMapCache& places)
{
    int columnWidth = width / numColumns;
    int rowHeight   = height / numRows;

    Point<int> cellBottomLeft(bottomLeft.x + cell.first.column*columnWidth,
                                    bottomLeft.y + cell.first.row*rowHeight);

    glViewport(cellBottomLeft.x, cellBottomLeft.y, columnWidth, rowHeight);
    glClear(GL_DEPTH_BUFFER_BIT);

    math::Point3D<float> focalPoint(columnWidth/2.0f, rowHeight/2.0f);

    camera.setViewRegion(columnWidth, rowHeight);
    camera.setFocalPoint(focalPoint);
    camera.setupCamera(columnWidth, rowHeight);

    auto state = maps.stateWithId(cell.second.mapId);
    if(state && state->map)
    {
        math::Rectangle<float> boundary = mapRenderer->calculateRenderedBoundary(*state, places);
        Point<float> center(boundary.bottomLeft + boundary.topRight);
        center.x /= 2.0f;
        center.y /= 2.0f;

        float widthScale  = columnWidth / boundary.width();
        float heightScale = rowHeight   / boundary.height();
        float scale       = (widthScale < heightScale) ? widthScale : heightScale;
        scale *= 0.9;

        glPushMatrix();
        glTranslatef(focalPoint.x - center.x*scale, focalPoint.y - center.y*scale, 0.0);
        glScalef(scale, scale, 1.0);

        mapRenderer->renderTopoMap(*state, places);

        glPopMatrix();

        cell.second.borderColor.set();
    }
    else
    {
        cell.second.borderColor.set(0.5);
    }

    glLineWidth(5.0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(0, 0);
    glVertex2f(columnWidth, 0);
    glVertex2f(columnWidth, rowHeight);
    glVertex2f(0, rowHeight);
    glEnd();
}

} // namespace ui
} // namespace vulcan
