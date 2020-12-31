/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     cell_grid_renderer.h
* \author   Collin Johnson
*
* Definition of CellGridAlphaRenderer.
*/

#include "ui/components/cell_grid_renderer.h"
#include "ui/common/gl_texture_helpers.h"
#include "utils/cell_grid.h"
#include <cassert>
#include <iostream>

namespace vulcan
{
namespace ui
{

CellGridAlphaRenderer::CellGridAlphaRenderer(int32_t maxValue, bool invertAlpha)
: maxValue(maxValue)
, invert(invertAlpha)
, gridWidth(0)
, gridHeight(0)
, gridTexture(0)
, initialized(false)
{
}


void CellGridAlphaRenderer::setCellGrid(const utils::CellGrid<int32_t>& grid)
{
    if((gridWidth != static_cast<int>(grid.getWidthInCells())) 
        || (gridHeight != static_cast<int>(grid.getHeightInCells())))
    {
        if(!initialized)
        {
            glGenTextures(1, &textureName);
            initialized = true;
        }

        delete [] gridTexture;

        gridWidth  = grid.getWidthInCells();
        gridHeight = grid.getHeightInCells();

        // The place exists in its own abstract coordinate system that doesn't have any
        // tie to metric space, so the center point is always just the middle of the grid
        gridCenter.x = gridWidth / 2;
        gridCenter.y = gridHeight / 2;

        initializeGridTexture();
    }

    gridWidthInMeters  = grid.getWidthInMeters();
    gridHeightInMeters = grid.getHeightInMeters();
    gridCenterInMeters = grid.getBottomLeft();

    updateGridTexture(grid);
}


void CellGridAlphaRenderer::setCellGrid(const utils::CellGrid<uint16_t>& grid)
{
    if((gridWidth != static_cast<int>(grid.getWidthInCells())) 
        || (gridHeight != static_cast<int>(grid.getHeightInCells())))
    {
        if(!initialized)
        {
            glGenTextures(1, &textureName);
            initialized = true;
        }
        
        delete [] gridTexture;
        
        gridWidth  = grid.getWidthInCells();
        gridHeight = grid.getHeightInCells();
        
        // The place exists in its own abstract coordinate system that doesn't have any
        // tie to metric space, so the center point is always just the middle of the grid
        gridCenter.x = gridWidth / 2;
        gridCenter.y = gridHeight / 2;
        
        initializeGridTexture();
    }
    
    gridWidthInMeters  = grid.getWidthInMeters();
    gridHeightInMeters = grid.getHeightInMeters();
    gridCenterInMeters = grid.getBottomLeft();
    
    updateGridTexture(grid);
}


void CellGridAlphaRenderer::renderGrid(bool useMeters)
{
    if(initialized)
    {
        enableTexture();

        float textureXMax = gridWidth  / static_cast<float>(textureWidth);
        float textureYMax = gridHeight / static_cast<float>(textureHeight);

        color.set();

        if(useMeters)
        {
            draw_one_texture_on_rectangle(gridCenterInMeters.x, gridCenterInMeters.y, gridWidthInMeters, gridHeightInMeters, textureXMax, textureYMax);
        }
        else
        {
            draw_one_texture_on_rectangle(gridCenter.x, gridCenter.y, gridWidth, gridHeight, textureXMax, textureYMax);
        }

        disableTexture();
    }
}


void CellGridAlphaRenderer::initializeGridTexture(void)
{
    textureWidth  = round_to_power_of_two(gridWidth);
    textureHeight = round_to_power_of_two(gridHeight);

    gridTexture = new uint16_t[textureWidth*textureHeight];

    initialize_texture_16(textureName, gridTexture, textureWidth, textureHeight, GL_ALPHA, GL_ALPHA);
}


void CellGridAlphaRenderer::updateGridTexture(const utils::CellGrid<int32_t>& grid)
{
    activate_texture(textureName, GL_TEXTURE0, GL_REPLACE);
    convertGridToTexture(grid);
    set_sub_texture_16(gridTexture, gridWidth, gridHeight, GL_ALPHA);
    disable_texture(GL_TEXTURE0);
}


void CellGridAlphaRenderer::updateGridTexture(const utils::CellGrid<uint16_t>& grid)
{
    activate_texture(textureName, GL_TEXTURE0, GL_REPLACE);
    convertGridToTexture(grid);
    set_sub_texture_16(gridTexture, gridWidth, gridHeight, GL_ALPHA);
    disable_texture(GL_TEXTURE0);
}


void CellGridAlphaRenderer::enableTexture(void)
{
    activate_texture(textureName, GL_TEXTURE0, GL_REPLACE);
}


void CellGridAlphaRenderer::disableTexture(void)
{
    disable_texture(GL_TEXTURE0);
}


void CellGridAlphaRenderer::convertGridToTexture(const utils::CellGrid<int32_t>& grid)
{
    assert(gridHeight == static_cast<int>(grid.getHeightInCells()));
    assert(gridWidth == static_cast<int>(grid.getWidthInCells()));

    int textureIndex = 0;

    float scale = 65535.0f / maxValue * (invert ? 1 : -1);
    int32_t cellMax = invert ? 65535 : 0;
    int32_t maxValueCell = invert ? 0 : 65535;

    for(int y = 0; y < gridHeight; ++y)
    {
        for(int x = 0; x < gridWidth; ++x)
        {
            gridTexture[textureIndex++] = (grid(x, y) < maxValue) ? (cellMax - grid(x, y)*scale) : maxValueCell;
        }
    }
}


void CellGridAlphaRenderer::convertGridToTexture(const utils::CellGrid<uint16_t>& grid)
{
    assert(gridHeight == static_cast<int>(grid.getHeightInCells()));
    assert(gridWidth == static_cast<int>(grid.getWidthInCells()));
    
    int textureIndex = 0;
    
    float scale = 65535.0f / maxValue * (invert ? 1 : -1);
    uint16_t cellMax = invert ? 65535 : 0;
    uint16_t maxValueCell = invert ? 0 : 65535;
    
    for(int y = 0; y < gridHeight; ++y)
    {
        for(int x = 0; x < gridWidth; ++x)
        {
            gridTexture[textureIndex++] = (grid(x, y) < maxValue) ? (cellMax - grid(x, y)*scale) : maxValueCell;
        }
    }
}

} // namespace ui
} // namespace vulcan
