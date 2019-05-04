/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     frontiers_renderer.cpp
* \author   Collin Johnson
*
* Definition of FrontiersRenderer.
*/

#include <algorithm>
#include <GL/gl.h>
#include <ui/components/frontiers_renderer.h>
#include <ui/common/gl_shapes.h>
#include <hssh/local_topological/frontier.h>

namespace vulcan
{
namespace ui
{

void draw_frontier(const hssh::Frontier& frontier);

void FrontiersRenderer::setRenderColor(const GLColor& frontierColor)
{
    this->frontierColor = frontierColor;
}


void FrontiersRenderer::render(const std::vector<hssh::Frontier>& frontiers)
{
    // Set the color here so it doesn't need to be passed into the draw_frontier function, as all colors are the same right now
    frontierColor.set();
    std::for_each(frontiers.begin(), frontiers.end(), [](const hssh::Frontier& frontier) { draw_frontier(frontier); });
}


void draw_frontier(const hssh::Frontier& frontier)
{
    const float LINE_WIDTH = 2.0f;

    glLineWidth(LINE_WIDTH);
    glBegin(GL_LINES);
    glVertex2f(frontier.boundary.a.x, frontier.boundary.a.y);
    glVertex2f(frontier.boundary.b.x, frontier.boundary.b.y);
    glEnd();

    gl_draw_small_arrow(frontier.exitPoint, length(frontier.boundary)*0.75, frontier.direction, LINE_WIDTH);
}

}
}
