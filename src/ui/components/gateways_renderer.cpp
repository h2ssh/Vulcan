/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gateways_renderer.cpp
* \author   Collin Johnson
*
* Definition of GatewaysRenderer.
*/

#include <ui/components/gateways_renderer.h>
#include <ui/common/default_colors.h>
#include <hssh/local_topological/gateway.h>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void GatewaysRenderer::setRenderColors(const GLColor& exploredColor, const GLColor& frontierColor, const GLColor& endpointColor)
{
    this->exploredColor = exploredColor;
    this->frontierColor = frontierColor;
    this->endpointColor = endpointColor;
}


void GatewaysRenderer::renderGateways(const std::vector<hssh::Gateway>& gateways, bool shouldRenderNormals)
{
    for(int n = gateways.size(); --n >= 0;)
    {
        renderGateway(gateways[n], shouldRenderNormals, &quasi_static_color());
    }
}


void GatewaysRenderer::renderGateway(const hssh::Gateway& gateway, bool shouldRenderNormals, const GLColor* color)
{
    const float BOUNDARY_WIDTH = 5.0f;
//     const float ENDPOINT_SIZE  = 6.0f;

    const GLColor* boundaryColor = color ? color : &quasi_static_color();
//     const GLColor* boundaryColor = &quasi_static_color();

    glLineWidth(BOUNDARY_WIDTH);
//     glEnable(GL_LINE_STIPPLE); // Draw boundary as a dashed line to make it pop out a bit
//     glLineStipple(3, 0xAAAA);
    glBegin(GL_LINE_STRIP);

    boundaryColor->set();

    glVertex2f(gateway.boundary().a.x, gateway.boundary().a.y);
    glVertex2f(gateway.center().x,     gateway.center().y);
    glVertex2f(gateway.boundary().b.x, gateway.boundary().b.y);

    glEnd();
// //     glDisable(GL_LINE_STIPPLE); // Draw boundary as a dashed line to make it pop out a bit

//     glPointSize(ENDPOINT_SIZE);
//     glBegin(GL_POINTS);
//     endpointColor.set();
//
//     glVertex2f(gateway.center().x, gateway.center().y);
//     glVertex2f(gateway.boundary().a.x, gateway.boundary().a.y);
//     glVertex2f(gateway.boundary().b.x, gateway.boundary().b.y);
//
//     glEnd();

    if(shouldRenderNormals)
    {
        renderGatewayNormals(gateway);
    }
}


void GatewaysRenderer::renderGatewayNormals(const hssh::Gateway& gateway)
{
    const float BOUNDARY_WIDTH = 3.0f;

    glLineWidth(BOUNDARY_WIDTH);
    glEnable(GL_LINE_STIPPLE); // Draw boundary as a dashed line to make it pop out a bit
    glLineStipple(3, 0xAAAA);
    glBegin(GL_LINES);

    frontierColor.set();

    glVertex2f(gateway.center().x, gateway.center().y);
    glVertex2f(gateway.center().x + std::cos(gateway.leftDirection()), 
               gateway.center().y + std::sin(gateway.leftDirection()));
    
    glVertex2f(gateway.center().x, gateway.center().y);
    glVertex2f(gateway.center().x + std::cos(gateway.rightDirection()), 
               gateway.center().y + std::sin(gateway.rightDirection()));

    glEnd();
    glDisable(GL_LINE_STIPPLE);
}

} // namespace ui
} // namespace vulcan
