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
* Definition of a variety of CellGridRenderers, which provide a configurable
* interface for rendering a grid.
*/

#ifndef UI_COMPONENTS_CELL_GRID_RENDERER_H
#define UI_COMPONENTS_CELL_GRID_RENDERER_H

#include <GL/gl.h>
#include "core/point.h"
#include "ui/common/ui_color.h"

namespace vulcan
{

namespace utils
{
template <typename T>
class CellGrid;
}

namespace ui
{

/**
* CellGridAlphaRenderer renders the values in a cell grid as an alpha layer. Currently,
* the only supported grid type is int32_t. Eventually, this renderer will be converted
* to a template so all cell grid types can be rendered.
*
* The grid is rendered such that the assigned maxValue is opaque and 0 is transparent.
* The default max value is 65535.
*/
class CellGridAlphaRenderer
{
public:

    /**
    * Constructor for CellGridAlphaRenderer.
    * 
    * \param    maxValue            Maximum value for a particular coordinate (default = 2^16-1)
    * \param    invertAlpha         Make 0 = black and 1 = white (default = false)
    */
    CellGridAlphaRenderer(int32_t maxValue = 65535, bool invertAlpha = false);
    
    /**
    * setGridColor sets the background color for the grid. The grid values will change
    * the alpha of this color.
    */
    void setGridColor(const GLColor& gridColor) { color = gridColor; }

    /**
    * setCellGrid sets the cell grid to be rendered. This method should only be called when
    * the grid updates because adjusting the grid texture is an expensive operation.
    */
    void setCellGrid(const utils::CellGrid<int32_t>& grid);
    
    /**
    * setCellGrid sets the cell grid to be rendered. This method should only be called when
    * the grid updates because adjusting the grid texture is an expensive operation.
    */
    void setCellGrid(const utils::CellGrid<uint16_t>& grid);
    
    /**
    * setMaxValue sets the maximum value to be rendered.
    */
    void setMaxValue(int32_t maxValue) { this->maxValue = maxValue; }
    
    /**
    * renderGrid renders the currently set grid using GL_REPLACE for the alpha channel.
    */
    void renderGrid(bool useMeters = false);

private:

    void initializeGridTexture(void);
    void updateGridTexture(const utils::CellGrid<int32_t>& grid);
    void updateGridTexture(const utils::CellGrid<uint16_t>& grid);
    void enableTexture(void);
    void disableTexture(void);

    void convertGridToTexture(const utils::CellGrid<int32_t>& grid);
    void convertGridToTexture(const utils::CellGrid<uint16_t>& grid);
    
    GLColor color;
    int32_t maxValue;
    bool invert;

    int32_t gridWidth;
    int32_t gridHeight;
    Point<float> gridCenter;
    
    float gridWidthInMeters;
    float gridHeightInMeters;
    Point<float> gridCenterInMeters;
    
    int32_t textureWidth;
    int32_t textureHeight;

    GLuint textureName;
    uint16_t* gridTexture;
    
    bool initialized;
};

}
}

#endif // UI_COMPONENTS_CELL_GRID_RENDERER_H
