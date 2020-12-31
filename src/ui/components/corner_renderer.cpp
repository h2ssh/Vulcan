/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     corner_renderer.cpp
 * \author   Jong Jin Park
 *
 * Definition of CornerRenderer.
 */

#include "ui/components/corner_renderer.h"
#include "ui/common/gl_shapes.h"
#include <GL/gl.h>

namespace vulcan
{

namespace ui
{

void CornerRenderer::setRenderColors(const GLColor& hoverPointColor,
                                     const GLColor& cornerPointColor,
                                     const GLColor& cornerLineColor,
                                     const GLColor& rectifiedCornerColor)
{
    this->hoverPointColor = hoverPointColor;
    this->cornerPointColor = cornerPointColor;
    this->cornerLineColor = cornerLineColor;
    this->rectifiedCornerColor = rectifiedCornerColor;
}

void CornerRenderer::setShapeWidths(float cornerLineWidth, float rectifiedCornerLineWidth, float circleRadius)
{
    this->cornerLineWidth = cornerLineWidth;
    this->rectifiedCornerLineWidth = rectifiedCornerLineWidth;
    this->circleRadius = circleRadius;
    this->hoverCircleRadius = circleRadius;
}

void CornerRenderer::renderCornerSelection(const std::vector<Point<float>>& cornerPoints,
                                           const Point<float>& hoverPoint)
{
    if (!cornerPoints.empty()) {
        renderCornerPoints(cornerPoints);
        renderLine(*cornerPoints.rbegin(), hoverPoint, cornerLineWidth, cornerLineColor);
    }

    hoverCircleRadius = hoverCircleRadius * 0.97;   // dynamically changing circle size
    if (hoverCircleRadius < 0.9 * circleRadius) {
        hoverCircleRadius = 1.3 * circleRadius;
    }

    renderFilledCircle(hoverPoint, hoverCircleRadius, hoverPointColor);
}


void CornerRenderer::renderCornerPoints(const std::vector<Point<float>>& cornerPoints)
{
    for (auto pointIt = cornerPoints.begin(), pointEnd = cornerPoints.end(); pointIt != pointEnd; ++pointIt) {
        renderFilledCircle(*pointIt, circleRadius, cornerPointColor);

        if (pointIt + 1 != pointEnd) {
            renderLine(*pointIt, *(pointIt + 1), cornerLineWidth, cornerLineColor);
        }
    }
}


void CornerRenderer::renderFilledCircle(const Point<float>& center, float radius, const GLColor& color)
{
    color.set();
    gl_draw_filled_circle(center, radius, 25);
}

void CornerRenderer::renderLine(const Line<float>& line, float lineWidth, const GLColor& color)
{
    renderLine(line.a, line.b, lineWidth, color);
}

void CornerRenderer::renderLine(const Point<float>& pointA,
                                const Point<float>& pointB,
                                float lineWidth,
                                const GLColor& color)
{
    glPushMatrix();

    glLineWidth(lineWidth);
    glBegin(GL_LINES);

    color.set();

    glVertex3f(pointA.x, pointA.y, 0);
    glVertex3f(pointB.x, pointB.y, 0);

    glEnd();

    glPopMatrix();
}


}   // namespace ui
}   // namespace vulcan