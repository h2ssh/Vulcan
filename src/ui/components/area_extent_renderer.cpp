/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_extent_renderer.cpp
* \author   Collin Johnson
*
* Definition of AreaExtentRenderer.
*/

#include <ui/components/area_extent_renderer.h>
#include <ui/common/gl_shapes.h>
#include <ui/common/ui_color.h>
#include <hssh/local_topological/area_extent.h>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void AreaExtentRenderer::renderExtentCells(const hssh::AreaExtent& extent,
                                           float metersPerCell,
                                           const GLColor& color) const
{
    extentVertices_.clear();
    extentVertices_.reserve(extent.size() * 4);

    for(auto& cell : extent)
    {
        extentVertices_.push_back(cell.x);
        extentVertices_.push_back(cell.y);

        extentVertices_.push_back(cell.x + metersPerCell);
        extentVertices_.push_back(cell.y);

        extentVertices_.push_back(cell.x + metersPerCell);
        extentVertices_.push_back(cell.y + metersPerCell);

        extentVertices_.push_back(cell.x);
        extentVertices_.push_back(cell.y + metersPerCell);
    }

    color.set(0.75);

    glDisableClientState(GL_EDGE_FLAG_ARRAY);  // get rid of the things we aren't using for efficiency sake
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    glEnableClientState(GL_VERTEX_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, extentVertices_.data());
    glDrawArrays(GL_QUADS, 0, extentVertices_.size() / 2);

    glDisableClientState(GL_VERTEX_ARRAY);
    glEnd();
}


void AreaExtentRenderer::renderExtentRectangle(const hssh::AreaExtent& extent, const GLColor& color) const
{
    auto boundary = extent.rectangleBoundary(math::ReferenceFrame::GLOBAL);

    color.set(0.33);
    gl_draw_filled_rectangle(boundary);

    color.set();
    gl_draw_line_rectangle(boundary, 2.0f);
    
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    glVertex2f(extent.center().x, extent.center().y);
    glEnd();
}


void AreaExtentRenderer::renderExtentPolygon(const hssh::AreaExtent& extent, const GLColor& color) const
{
    math::Polygon<float> boundary(extent.polygonBoundary(math::ReferenceFrame::GLOBAL).vertices());

    color.set(0.33);
    gl_draw_filled_polygon(boundary.vertices());

    color.set();
    gl_draw_line_polygon(boundary.vertices(), 2.0f);
}

} // namespace ui
} // namespace vulcan
