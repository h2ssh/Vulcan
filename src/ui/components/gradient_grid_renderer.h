/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gradient_grid_renderer.h
* \author   Collin Johnson
*
* Declaration of GradientGridRenderer for drawing grids representing a gradient.
*/

#ifndef UI_COMPONENTS_GRADIENT_GRID_RENDERER_H
#define UI_COMPONENTS_GRADIENT_GRID_RENDERER_H

#include <GL/gl.h>
#include "ui/common/ui_color.h"

namespace vulcan
{

namespace utils { template <typename T> class CellGrid; }

namespace ui
{

/**
* GradientGridRenderer renders gradient information for a grid. The gradient of each cell is drawn
* as a line in the direction of the gradient with a different color at each vertex, indicating
* the direction of the gradient.
*
* Currently, the gradient grid is represented as two separate grids, dx and dy, providing the vector
* along which the gradient flows.
*/
class GradientGridRenderer
{
public:

    /**
    * Constructor for GradientGridRenderer.
    */
    GradientGridRenderer(void);

    /**
    * Destructor for GradientGridRenderer.
    */
    ~GradientGridRenderer(void);

    /**
    * setRenderColors sets the rendering colors for the gradient.
    *
    * \param    tailColor           Color to use for the tail of the gradient vector
    * \param    headColor           Color to use for the head of the gradient vector
    */
    void setRenderColors(const GLColor& tailColor, const GLColor& headColor);

    /**
    * setGradientGrid creates the grid to be later drawn by the renderGradient method.
    *
    * \param    dxGrid              Grid giving dx for each vector
    * \param    dyGrid              Grid giving dy for each vector
    */
    void setGradientGrid(const utils::CellGrid<int16_t>& dxGrid, const utils::CellGrid<int16_t>& dyGrid);

    /**
    * renderGradient renders the currently set gradient grid.
    */
    void renderGradient(void);

private:

    GradientGridRenderer(const GradientGridRenderer& toCopy)         = delete;
    GradientGridRenderer& operator=(const GradientGridRenderer& rhs) = delete;

    void allocateVerticesIfNeeded(uint16_t width, uint16_t height);
    void createVertices          (const utils::CellGrid<int16_t>& dxGrid, const utils::CellGrid<int16_t>& dyGrid);
    void assignColors            (void);

    // INVARIANT: Ordering of vertices: tail head tail head etc...
    GLfloat* gradientVertices;
    GLfloat* gradientColors;

    size_t numVertices;
    size_t numTotalVertices;

    GLColor tailColor;
    GLColor headColor;
};

}
}

#endif // UI_COMPONENTS_GRADIENT_GRID_RENDERER_H
