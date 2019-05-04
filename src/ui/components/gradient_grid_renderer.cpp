/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gradient_grid_renderer.cpp
* \author   Collin Johnson
*
* Definition of GradientGridRenderer.
*/

#include <utils/cell_grid.h>
#include <ui/components/gradient_grid_renderer.h>

namespace vulcan
{
namespace ui
{

const int CELL_SPACING = 5;

GradientGridRenderer::GradientGridRenderer(void)
    : gradientVertices(0)
    , gradientColors(0)
    , numVertices(0)
    , numTotalVertices(0)
{
}


GradientGridRenderer::~GradientGridRenderer(void)
{
    delete [] gradientVertices;
    delete [] gradientColors;
}


void GradientGridRenderer::setRenderColors(const GLColor& tailColor, const GLColor& headColor)
{
    this->tailColor = tailColor;
    this->headColor = headColor;

    if(numTotalVertices)
    {
        assignColors();
    }
}


void GradientGridRenderer::setGradientGrid(const utils::CellGrid<int16_t>& dxGrid, const utils::CellGrid<int16_t>& dyGrid)
{
    allocateVerticesIfNeeded(dxGrid.getWidthInCells(), dxGrid.getHeightInCells());
    createVertices(dxGrid, dyGrid);
}


void GradientGridRenderer::renderGradient(void)
{
    glLineWidth(2.0);

    // The main bit of lidar data exists in two vector lists for the colors and vertices
    glDisableClientState(GL_EDGE_FLAG_ARRAY);  // get rid of the things we aren't using for efficiency sake
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, gradientVertices);
    glColorPointer (4, GL_FLOAT, 0, gradientColors);

    glDrawArrays(GL_LINES, 0, numVertices);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    glEnd();
}


void GradientGridRenderer::allocateVerticesIfNeeded(uint16_t width, uint16_t height)
{
    size_t verticesNeeded = width * height * 2 / (CELL_SPACING * CELL_SPACING);  // only render every 5th cell

    if(verticesNeeded > numTotalVertices)
    {
        numTotalVertices = verticesNeeded;

        delete [] gradientVertices;
        delete [] gradientColors;

        gradientVertices = new GLfloat[numTotalVertices * 2];
        gradientColors   = new GLfloat[numTotalVertices * 4];

        assignColors();
    }
}


void GradientGridRenderer::createVertices(const utils::CellGrid<int16_t>& dxGrid, const utils::CellGrid<int16_t>& dyGrid)
{
    float scale = dxGrid.metersPerCell() * 0.5f * CELL_SPACING;

    Point<uint16_t> cell;
    Point<float>    metricCell;

    float angle = 0.0f;

    int vertexIndex = 0;

    numVertices = 0;

    for(cell.y = 0; cell.y < dxGrid.getHeightInCells()-CELL_SPACING; cell.y += CELL_SPACING)
    {
        for(cell.x = 0; cell.x < dxGrid.getWidthInCells()-CELL_SPACING; cell.x += CELL_SPACING)
        {
            angle      = atan2(dyGrid.getValueNoCheck(cell.x, cell.y), dxGrid.getValueNoCheck(cell.x, cell.y));
            metricCell = utils::grid_point_to_global_point(cell, dxGrid);

            gradientVertices[vertexIndex++] = metricCell.x - scale*cos(angle);
            gradientVertices[vertexIndex++] = metricCell.y - scale*sin(angle);

            gradientVertices[vertexIndex++] = metricCell.x + scale*cos(angle);
            gradientVertices[vertexIndex++] = metricCell.y + scale*sin(angle);

            numVertices += 2;
        }
    }
}


void GradientGridRenderer::assignColors(void)
{
    int colorIndex = 0;

    // Assigning color for two vertices per loop, so only need to go through half
    for(size_t n = 0; n < numTotalVertices/2; ++n)
    {
        gradientColors[colorIndex++] = tailColor.red();
        gradientColors[colorIndex++] = tailColor.green();
        gradientColors[colorIndex++] = tailColor.blue();
        gradientColors[colorIndex++] = tailColor.alpha();

        gradientColors[colorIndex++] = headColor.red();
        gradientColors[colorIndex++] = headColor.green();
        gradientColors[colorIndex++] = headColor.blue();
        gradientColors[colorIndex++] = headColor.alpha();
    }
}

} // namespace ui
} // namespace vulcan
