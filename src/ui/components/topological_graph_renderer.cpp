/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     topological_graph_renderer.cpp
* \author   Collin Johnson
*
* Definition of TopologicalGraphRenderer.
*/

#include <ui/components/topological_graph_renderer.h>
#include <ui/common/gl_shapes.h>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void TopologicalGraphRenderer::setRenderColor(const GLColor& placeColor, const GLColor& pathSegmentColor, const GLColor& edgeColor)
{
    this->placeColor       = placeColor;
    this->pathSegmentColor = pathSegmentColor;
    this->edgeColor        = edgeColor;
}


void TopologicalGraphRenderer::setVertexColor(uint32_t id, const GLColor& color)
{
    vertexColors[id] = color;
}


void TopologicalGraphRenderer::setEdgeColor(uint32_t id, const GLColor& color)
{
    edgeColors[id] = color;
}


void TopologicalGraphRenderer::resetColors(void)
{
    vertexColors.clear();
    edgeColors.clear();
}


void TopologicalGraphRenderer::render(const hssh::TopologicalGraph& graph) const
{
    // Draw the edges first and then the vertices so the lines will be below the circles -- prettier this way
    renderEdges(graph.getEdges());
    renderVertices(graph.getVertices());
}


void TopologicalGraphRenderer::renderEdges(const std::vector<hssh::TopologicalEdge>& edges) const
{
    glLineWidth(2.0);
    glBegin(GL_LINES);

    for(auto edgeIt = edges.begin(), edgeEnd = edges.end(); edgeIt != edgeEnd; ++edgeIt)
    {
        auto edgeColorIt = edgeColors.find(edgeIt->getId());
        if(edgeColorIt != edgeColors.end())
        {
            edgeColorIt->second.set();
        }
        else
        {
            edgeColor.set();
        }

        glVertex2f(edgeIt->getStart().getPosition().x, edgeIt->getStart().getPosition().y);
        glVertex2f(edgeIt->getEnd().getPosition().x,   edgeIt->getEnd().getPosition().y);
    }

    glEnd();
}


void TopologicalGraphRenderer::renderVertices(const std::vector<hssh::TopologicalVertex>& vertices) const
{
    const uint16_t NUM_CIRCLE_SEGMENTS = 36;
    const float    VERTEX_RADIUS       = 1.0f;
//     const float    LINE_WIDTH          = 1.5f;

    for(auto vertexIt = vertices.begin(), vertexEnd = vertices.end(); vertexIt != vertexEnd; ++vertexIt)
    {
        auto vertexColorIt = vertexColors.find(vertexIt->getId());
        if(vertexColorIt != edgeColors.end())
        {
            vertexColorIt->second.set();
        }
        else if(vertexIt->getVertexData().isSegment)
        {
            pathSegmentColor.set();
        }
        else // if(!vertexIt->getVertexData().isSegment)
        {
            placeColor.set();
        }

        gl_draw_filled_circle(vertexIt->getPosition(), VERTEX_RADIUS, NUM_CIRCLE_SEGMENTS);
//         gl_draw_line_circle(vertexIt->getPosition(), VERTEX_RADIUS, LINE_WIDTH, NUM_CIRCLE_SEGMENTS);
    }
}

} // namespace ui
} // namespace vulcan
