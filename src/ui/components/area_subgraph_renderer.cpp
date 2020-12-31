/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     area_subgraph_renderer.cpp
 * \author   Collin Johnson
 *
 * Implementation of AreaHypothesisRenderer.
 */

#include "ui/components/area_subgraph_renderer.h"
#include "hssh/local_topological/area_detection/labeling/debug.h"
#include "ui/common/default_colors.h"
#include "ui/common/gl_shapes.h"
#include <GL/gl.h>
#include <boost/algorithm/clamp.hpp>

namespace vulcan
{
namespace ui
{

AreaHypothesisRenderer::AreaHypothesisRenderer(void)
: unknownColor_(area_color())
, decisionPointColor_(decision_point_color())
, destinationColor_(destination_color())
, pathColor_(path_color())
{
}


void AreaHypothesisRenderer::setRenderColors(const GLColor& unknownColor,
                                             const GLColor& decisionPointColor,
                                             const GLColor& destinationColor,
                                             const GLColor& pathColor,
                                             const std::vector<GLColor>& valueColors)
{
    unknownColor_ = unknownColor;
    decisionPointColor_ = decisionPointColor;
    destinationColor_ = destinationColor;
    pathColor_ = pathColor;

    valueInterpolator_.setColors(valueColors);
}


void AreaHypothesisRenderer::renderHypothesis(const hssh::AreaHypothesis& hypothesis, float metersPerCell) const
{
    drawBoundary(hypothesis.extent(), selectColorFromType(hypothesis.getType()), metersPerCell);
}


void AreaHypothesisRenderer::renderHypothesis(const hssh::DebugHypothesis& hypothesis, float metersPerCell) const
{
    drawBoundary(hypothesis.getExtent(),
                 selectColorFromType(hypothesis.getType()),
                 metersPerCell,
                 0.2 + 0.8 * (1.0 - hypothesis.getFrontierRatio()));
    drawStatistics(hypothesis);
    drawEndpointsIfPath(hypothesis);
}


void AreaHypothesisRenderer::renderHypothesis(const hssh::DebugHypothesis& hypothesis,
                                              double normalizedValue,
                                              float metersPerCell) const
{
    drawBoundary(hypothesis.getExtent(), valueInterpolator_.calculateColor(normalizedValue), metersPerCell);
}


void AreaHypothesisRenderer::renderHypothesisDistribution(const hssh::DebugHypothesis& hypothesis,
                                                          float metersPerCell) const
{
    auto distribution = hypothesis.getDistribution();

    float red = distribution.destination;
    float green = distribution.path;
    float blue = distribution.decision;

    drawBoundary(hypothesis.getExtent(), GLColor(red, green, blue, 0.66), metersPerCell);
}


void AreaHypothesisRenderer::drawBoundary(const hssh::AreaExtent& extent,
                                          const GLColor& boundaryColor,
                                          float metersPerCell,
                                          float filledScale) const
{
    auto scaledColor =
      GLColor(boundaryColor.red(), boundaryColor.green(), boundaryColor.blue(), boundaryColor.alpha() * filledScale);
    extentRenderer_.renderExtentCells(extent, metersPerCell, scaledColor);
}


void AreaHypothesisRenderer::drawStatistics(const hssh::DebugHypothesis& hypothesis) const
{
    const float ARROW_WIDTH = 3.0f;

    auto extent = hypothesis.getExtent().rectangleBoundary(math::ReferenceFrame::GLOBAL);

    float majorLength = hypothesis.getEigRatio() * std::max(extent.width(), extent.height()) / 2.0f;
    float majorDir = hypothesis.getEigDirection();

    float minorLength = (1.0 - hypothesis.getEigRatio()) * std::max(extent.width(), extent.height()) / 2.0f;
    float minorDir = majorDir + M_PI / 2.0f;

    glColor4f(0, 0, 0, 0.75f);
    gl_draw_medium_arrow(extent.center(), majorLength, majorDir, ARROW_WIDTH);
    gl_draw_medium_arrow(extent.center(), minorLength, minorDir, ARROW_WIDTH);
}


void AreaHypothesisRenderer::drawEndpointsIfPath(const hssh::DebugHypothesis& hypothesis) const
{
    if (hypothesis.getType() != hssh::HypothesisType::kPath) {
        return;
    }

    auto endpoints = hypothesis.getPathEndpoints();

    glPointSize(5.0);
    glColor4f(0, 0, 0, 0.75f);
    glBegin(GL_POINTS);
    glVertex2f(endpoints[0].x, endpoints[0].y);
    glVertex2f(endpoints[1].x, endpoints[1].y);
    glEnd();
}


GLColor AreaHypothesisRenderer::selectColorFromType(hssh::HypothesisType type) const
{
    switch (type) {
    case hssh::HypothesisType::kDecision:
        return decisionPointColor_;

    case hssh::HypothesisType::kDest:
        return destinationColor_;

    case hssh::HypothesisType::kPath:
        return pathColor_;

    case hssh::HypothesisType::kArea:
    default:
        return unknownColor_;
    }
}

}   // namespace ui
}   // namespace vulcan
