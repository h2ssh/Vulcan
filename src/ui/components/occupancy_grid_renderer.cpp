/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "ui/components/occupancy_grid_renderer.h"
#include "hssh/metrical/occupancy_grid.h"
#include "ui/common/default_colors.h"
#include "ui/common/gl_texture_helpers.h"
#include "ui/common/ui_color.h"
#include <GL/gl.h>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>


namespace vulcan
{
namespace ui
{

const uint8_t kNumGridLayers = 2;
const uint8_t kCostGridIndex = 0;
const uint8_t kTypeGridIndex = 1;


// Helpers for doing the drawing
void draw_grid(Point<float> bottomLeft, float gridWidth, float gridHeight, float textureWidth, float textureHeight);


OccupancyGridRenderer::OccupancyGridRenderer(void) : gridWidth(0), gridHeight(0), textureNames(0), initialized(false)
{
    numTextures = kNumGridLayers;
    textures = new uint8_t*[numTextures];

    for (int i = numTextures; --i >= 0;) {
        textures[i] = 0;
    }

    // Set appealing default colors so they can be adjusted if desired, but otherwise
    setOccupiedColor(occupied_color());
    setDynamicColor(dynamic_color());
    setLimitedVisibilityColor(limited_visibility_color());
    setHazardColor(hazard_color());
    setQuasiStaticColor(quasi_static_color());
}


OccupancyGridRenderer::~OccupancyGridRenderer(void)
{
    for (int i = numTextures; --i >= 0;) {
        delete[] textures[i];
    }

    delete[] textures;
    delete[] textureNames;
}


void OccupancyGridRenderer::setOccupiedColor(const GLColor& occupied)
{
    setColor(occupiedColor, occupied);
}


void OccupancyGridRenderer::setDynamicColor(const GLColor& dynamic)
{
    setColor(dynamicColor, dynamic);
}


void OccupancyGridRenderer::setLimitedVisibilityColor(const GLColor& limited)
{
    setColor(limitedVisibilityColor, limited);
}


void OccupancyGridRenderer::setQuasiStaticColor(const GLColor& quasiStatic)
{
    setColor(quasiStaticColor, quasiStatic);
}


void OccupancyGridRenderer::setHazardColor(const GLColor& hazard)
{
    setColor(hazardColor, hazard);
}


void OccupancyGridRenderer::setGrid(const hssh::OccupancyGrid& grid)
{
    if ((gridWidth != grid.getWidthInCells()) || (gridHeight != grid.getHeightInCells())) {
        free_textures(textures, numTextures);

        gridWidth = grid.getWidthInCells();
        gridHeight = grid.getHeightInCells();

        initializeGridTextures(gridWidth, gridHeight);
    }

    updateGridTextures(grid);

    gridWidthInMeters = grid.getWidthInMeters();
    gridHeightInMeters = grid.getHeightInMeters();
    gridOrigin = grid.getBottomLeft();

    disableGridTextures();
}


void OccupancyGridRenderer::renderGrid(void)
{
    if (initialized) {
        enableGridTextures();

        float textureXMax = gridWidth / static_cast<float>(textureWidth);
        float textureYMax = gridHeight / static_cast<float>(textureHeight);

        draw_two_textures_on_rectangle(gridOrigin.x,
                                       gridOrigin.y,
                                       gridWidthInMeters,
                                       gridHeightInMeters,
                                       textureXMax,
                                       textureYMax);
        disableGridTextures();
    }
}


void OccupancyGridRenderer::setColor(TexColor& dest, const GLColor& source)
{
    dest[0] = source.red() * 255.0f;
    dest[1] = source.green() * 255.0f;
    dest[2] = source.blue() * 255.0f;
}


void OccupancyGridRenderer::initializeGridTextures(uint16_t gridWidth, uint16_t gridHeight)
{
    if (textureNames == 0) {
        textureNames = new GLuint[numTextures];
        glGenTextures(kNumGridLayers, textureNames);
    }

    textureWidth = round_to_power_of_two(gridWidth);
    textureHeight = round_to_power_of_two(gridHeight);

    textures[kCostGridIndex] = create_texture(textureWidth, textureHeight, 1);
    textures[kTypeGridIndex] = create_texture(textureWidth, textureHeight, 3);

    initialize_texture(textureNames[kCostGridIndex],
                       textures[kCostGridIndex],
                       textureWidth,
                       textureHeight,
                       GL_ALPHA,
                       GL_ALPHA);
    initialize_texture(textureNames[kTypeGridIndex],
                       textures[kTypeGridIndex],
                       textureWidth,
                       textureHeight,
                       GL_RGB,
                       GL_RGB);

    initialized = true;
}


void OccupancyGridRenderer::updateGridTextures(const hssh::OccupancyGrid& grid)
{
    assert(gridHeight == grid.getHeightInCells());
    assert(gridWidth == grid.getWidthInCells());

    convertCostGridToTextures(grid);
    convertTypeGridToTextures(grid);

    // The static grid replaces the alpha value
    activate_texture(textureNames[kCostGridIndex], GL_TEXTURE0, GL_REPLACE);
    set_sub_texture(textures[kCostGridIndex], grid.getWidthInCells(), grid.getHeightInCells(), GL_ALPHA);

    // The dynamic grid, and all other grid layers, add their colors
    activate_texture(textureNames[kTypeGridIndex], GL_TEXTURE1, GL_DECAL);
    set_sub_texture(textures[kTypeGridIndex], grid.getWidthInCells(), grid.getHeightInCells(), GL_RGB);
}


void OccupancyGridRenderer::enableGridTextures(void)
{
    // The static grid replaces the alpha value
    activate_texture(textureNames[kCostGridIndex], GL_TEXTURE0, GL_REPLACE);
    activate_texture(textureNames[kTypeGridIndex], GL_TEXTURE1, GL_DECAL);
}


void OccupancyGridRenderer::disableGridTextures(void)
{
    // Need to disable both texture units that were enabled, otherwise the rest of the rendering fails
    disable_texture(GL_TEXTURE1);
    disable_texture(GL_TEXTURE0);
}


void OccupancyGridRenderer::convertCostGridToTextures(const hssh::OccupancyGrid& map)
{
    uint8_t* costTexture = textures[kCostGridIndex];

    for (uint16_t y = 0; y < map.getHeightInCells(); ++y) {
        for (uint16_t x = 0; x < map.getWidthInCells(); ++x) {
            *costTexture++ = map.getCostNoCheck(x, y);
        }
    }
}


void OccupancyGridRenderer::convertTypeGridToTextures(const hssh::OccupancyGrid& map)
{
    uint8_t* costTexture = textures[kCostGridIndex];
    uint8_t* typeTexture = textures[kTypeGridIndex];

    for (uint16_t y = 0; y < map.getHeightInCells(); ++y) {
        for (uint16_t x = 0; x < map.getWidthInCells(); ++x) {
            uint8_t dynamic = map.getCellTypeNoCheck(x, y);

            if (dynamic & hssh::kDynamicOccGridCell) {
                *typeTexture++ = dynamicColor[0];
                *typeTexture++ = dynamicColor[1];
                *typeTexture++ = dynamicColor[2];
            } else if (dynamic & hssh::kLimitedVisibilityOccGridCell) {
                *typeTexture++ = limitedVisibilityColor[0];
                *typeTexture++ = limitedVisibilityColor[1];
                *typeTexture++ = limitedVisibilityColor[2];
            } else if (dynamic & hssh::kHazardOccGridCell) {
                *typeTexture++ = hazardColor[0];
                *typeTexture++ = hazardColor[1];
                *typeTexture++ = hazardColor[2];

                costTexture[utils::cell_to_index(x, y, map.getWidthInCells())] = 255;
            } else if (dynamic & hssh::kQuasiStaticOccGridCell) {
                *typeTexture++ = quasiStaticColor[0];
                *typeTexture++ = quasiStaticColor[1];
                *typeTexture++ = quasiStaticColor[2];
            } else {
                *typeTexture++ = occupiedColor[0];
                *typeTexture++ = occupiedColor[1];
                *typeTexture++ = occupiedColor[2];
            }
        }
    }
}

}   // namespace ui
}   // namespace vulcan
