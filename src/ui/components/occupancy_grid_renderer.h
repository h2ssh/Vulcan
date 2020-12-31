/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     occupancy_grid_renderer.h
 * \author   Collin Johnson
 *
 * Declaration of OccupancyGridRenderer.
 */

#ifndef UI_COMPONENTS_LPM_RENDERER_H
#define UI_COMPONENTS_LPM_RENDERER_H

#include "core/point.h"
#include <array>

namespace vulcan
{
namespace hssh
{
class OccupancyGrid;
}

namespace ui
{

struct GLColor;

/**
 * OccupancyGridRenderer renders an LPM using OpenGL.
 */
class OccupancyGridRenderer
{
public:
    /**
     * Constructor for OccupancyGridRenderer.
     */
    OccupancyGridRenderer(void);

    /**
     * Destructor for OccupancyGridRenderer.
     */
    ~OccupancyGridRenderer(void);

    void setOccupiedColor(const GLColor& occupied);
    void setDynamicColor(const GLColor& dynamic);
    void setLimitedVisibilityColor(const GLColor& limited);
    void setQuasiStaticColor(const GLColor& quasiStatic);
    void setHazardColor(const GLColor& hazard);

    void setGrid(const hssh::OccupancyGrid& grid);
    void renderGrid(void);

private:
    using TexColor = std::array<uint8_t, 3>;

    uint16_t gridWidth;
    uint16_t gridHeight;

    float gridWidthInMeters;
    float gridHeightInMeters;

    Point<float> gridOrigin;

    uint16_t textureWidth;
    uint16_t textureHeight;

    unsigned int* textureNames;
    uint8_t** textures;
    uint8_t numTextures;

    TexColor occupiedColor;
    TexColor dynamicColor;
    TexColor limitedVisibilityColor;
    TexColor quasiStaticColor;
    TexColor hazardColor;

    bool initialized;

    void setColor(TexColor& dest, const GLColor& source);

    void initializeGridTextures(uint16_t gridWidth, uint16_t gridHeight);
    void updateGridTextures(const hssh::OccupancyGrid& grid);
    void enableGridTextures(void);
    void disableGridTextures(void);

    void convertCostGridToTextures(const hssh::OccupancyGrid& map);
    void convertTypeGridToTextures(const hssh::OccupancyGrid& map);
    void updateDynamicCell(uint16_t x, uint16_t y, int textureIndex, uint8_t** const dynamicGrid);
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_LPM_RENDERER_H
