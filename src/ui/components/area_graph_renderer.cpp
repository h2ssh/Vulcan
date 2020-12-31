/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     area_graph_renderer.cpp
* \author   Collin Johnson
*
* Definition of AreaGraphRenderer.
*/

#include "ui/components/area_graph_renderer.h"
#include "ui/common/default_colors.h"
#include "ui/common/gl_shapes.h"
#include "hssh/local_topological/area_detection/labeling/area_graph.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void AreaGraphRenderer::setRenderColors(const GLColor& gatewayColor,
                                        const GLColor& junctionColor,
                                        const GLColor& frontierColor,
                                        const GLColor& deadEndColor)
{
    this->gatewayColor  = gatewayColor;
    this->junctionColor = junctionColor;
    this->frontierColor = frontierColor;
    this->deadEndColor  = deadEndColor;
}


void AreaGraphRenderer::renderGraph(const hssh::AreaGraph& graph)
{
    auto nodes = graph.getAllNodes();
    auto edges = graph.getAllEdges();

    for(auto& edge : edges)
    {
        drawEdge(*edge);
    }

    for(auto& node : nodes)
    {
        drawNode(*node);
    }
}


void AreaGraphRenderer::drawNode(const hssh::AreaNode& node)
{
    const float NODE_RADIUS     = 0.25f;
    const float BORDER_WIDTH    = 1.0f;
    const int   CIRCLE_SEGMENTS = 16;

    auto bottomLeft = node.getPosition();
    bottomLeft.x -= NODE_RADIUS;
    bottomLeft.y -= NODE_RADIUS;
    auto topRight = node.getPosition();
    topRight.x += NODE_RADIUS;
    topRight.y += NODE_RADIUS;

    math::Rectangle<float> boundary(bottomLeft, topRight);

    setColorFromType(node.getType(), node.getProbability(), false);
    if(node.isLoop())
    {
        gl_draw_filled_circle(node.getPosition(), NODE_RADIUS, CIRCLE_SEGMENTS);
    }
    else
    {
        gl_draw_filled_rectangle(boundary);
    }

    setColorFromType(node.getType(), node.getProbability(), true);

    if(node.isLoop())
    {
        gl_draw_line_circle(node.getPosition(), NODE_RADIUS, BORDER_WIDTH, CIRCLE_SEGMENTS);
    }
    else
    {
        gl_draw_line_rectangle(boundary, BORDER_WIDTH);
    }
}


void AreaGraphRenderer::drawEdge(const hssh::AreaEdge& edge)
{
    const float EDGE_WIDTH = 4.0f;

    auto endpoints = edge.getEndpoints();

    if(!edge.isFrontier())
    {
        glLineWidth(EDGE_WIDTH);
        glBegin(GL_LINES);
        setColorFromType(endpoints[0]->getType(), endpoints[0]->getProbability(), true);
        glVertex2f(endpoints[0]->getPosition().x, endpoints[0]->getPosition().y);

        setColorFromType(endpoints[1]->getType(), endpoints[1]->getProbability(), true);
        glVertex2f(endpoints[1]->getPosition().x, endpoints[1]->getPosition().y);

        glEnd();
    }
    else
    {
        glLineWidth(EDGE_WIDTH);
        glBegin(GL_LINES);
        frontierColor.set();
        glVertex2f(endpoints[0]->getPosition().x, endpoints[0]->getPosition().y);
        glVertex2f(endpoints[1]->getPosition().x, endpoints[1]->getPosition().y);
        glEnd();
    }
}


void AreaGraphRenderer::setColorFromType(char type, double probability, bool isBorder)
{
    float alphaScale = 0.2 + (0.8 * probability);

    if(type & hssh::AreaNode::kGateway)
    {
        if(probability > 0.5)
        {
            dynamic_color().set(alphaScale);
        }
        else
        {
            hazard_color().set(alphaScale);
        }
    }
    else if(type & hssh::AreaNode::kJunction)
    {
        junctionColor.set(alphaScale);
    }
    else if(type & hssh::AreaNode::kDeadEnd)
    {
        deadEndColor.set(alphaScale);
    }
    else if(type & hssh::AreaNode::kFrontier)
    {
        frontierColor.set(alphaScale);
    }
}

} // namespace ui
} // namespace vulcan
