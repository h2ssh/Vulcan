/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     visibility_graph_renderer.cpp
 * \author   Collin Johnson
 *
 * Declaration of VisibilityGraphRenderer.
 */

#include "ui/components/visibility_graph_renderer.h"
#include "ui/common/gl_shapes.h"
#include "utils/visibility_graph.h"
#include "utils/visibility_graph_feature.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

Point<float> global_vertex(utils::VisGraphVertex vertex, Point<float> globalOrigin, double metersPerCell)
{
    return Point<float>(globalOrigin.x + (vertex.x * metersPerCell), globalOrigin.y + (vertex.y * metersPerCell));
}


VisibilityGraphRenderer::VisibilityGraphRenderer(void)
: vertexColor_(0.8, 0.1, 0.1, 0.8)
, edgeColor_(0.0, 0.0, 0.0, 0.8)
{
    std::vector<GLColor> colors{GLColor(0, 0, 255, 200), GLColor(0, 255, 0, 200), GLColor(255, 0, 0, 200)};
    featureColor_.setColors(colors);
}


void VisibilityGraphRenderer::renderVisibilityGraph(const utils::VisibilityGraph& graph,
                                                    Point<float> globalOrigin,
                                                    double metersPerCell)
{
    // Draw the edges first so the lines go underneath the nodes, rather than on top
    drawEdges(graph, globalOrigin, metersPerCell);
    drawVertices(graph, globalOrigin, metersPerCell);
}


void VisibilityGraphRenderer::renderVisibilityGraph(const utils::VisibilityGraph& graph,
                                                    const utils::VisibilityGraphFeature& feature,
                                                    Point<float> globalOrigin,
                                                    double metersPerCell)
{
    // Draw the edges first so the lines go underneath the nodes.
    drawEdges(graph, globalOrigin, metersPerCell);
    drawVertices(feature, globalOrigin, metersPerCell);
}


void VisibilityGraphRenderer::drawVertices(const utils::VisibilityGraph& graph,
                                           Point<float> globalOrigin,
                                           double metersPerCell)
{
    vertexColor_.set();

    for (auto vertex : graph.vertices()) {
        gl_draw_filled_circle(global_vertex(vertex, globalOrigin, metersPerCell), 0.25);
    }
}


void VisibilityGraphRenderer::drawVertices(const utils::VisibilityGraphFeature& graph,
                                           Point<float> globalOrigin,
                                           double metersPerCell)
{
    for (auto vertex : graph) {
        GLColor color = featureColor_.calculateColor(vertex.second);
        color.set();

        gl_draw_filled_circle(global_vertex(vertex.first, globalOrigin, metersPerCell), 0.25);
    }
}


void VisibilityGraphRenderer::drawEdges(const utils::VisibilityGraph& graph,
                                        Point<float> globalOrigin,
                                        double metersPerCell)
{
    edgeColor_.set();

    glLineWidth(0.1f);
    glBegin(GL_LINES);

    for (auto edge : graph.edges()) {
        auto start = global_vertex(edge.first, globalOrigin, metersPerCell);
        auto end = global_vertex(edge.second, globalOrigin, metersPerCell);

        glVertex2f(start.x, start.y);
        glVertex2f(end.x, end.y);
    }

    glEnd();
}

}   // namespace ui
}   // namespace vulcan
