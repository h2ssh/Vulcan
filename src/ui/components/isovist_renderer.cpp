/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     isovist_renderer.cpp
 * \author   Collin Johnson
 *
 * Implementation of IsovistRenderer.
 */

#include "ui/components/isovist_renderer.h"
#include "math/geometry/convex_hull.h"
#include "ui/common/default_colors.h"
#include "utils/histogram.h"
#include <GL/gl.h>
#include <algorithm>

namespace vulcan
{
namespace ui
{

bool is_scalar_orientation(utils::Isovist::Scalar scalar);


void IsovistRenderer::setRenderColors(const GLColor& rayColor, const std::vector<GLColor>& fieldColors)
{
    rayColor_ = rayColor;
    scalarInterpolator_.setColors(fieldColors);
}


void IsovistRenderer::renderIsovist(const utils::Isovist& isovist) const
{
    renderIsovist(isovist, rayColor_);
}


void IsovistRenderer::renderIsovist(const utils::Isovist& isovist, const GLColor& color) const
{
    auto position = isovist.position();

    std::size_t numPoints = isovist.size();
    double minDist = HUGE_VAL;
    int minIdx = 0;

    int otherSide = numPoints / 2;
    for (int n = 0, end = otherSide; n < end; ++n) {
        double dist = distance_between_points(*(isovist.begin() + n), *(isovist.begin() + n + otherSide));
        if (dist < minDist) {
            minDist = dist;
            minIdx = n;
        }
    }

    GLColor halfColor(1.0f, 0.0f, 0.0f, 0.75f);
    GLColor otherColor(129, 185, 255, 192);

    glLineWidth(1.5f);
    glBegin(GL_LINES);
    for (auto ptIt = isovist.begin(); ptIt != isovist.end(); ++ptIt) {
        int idx = std::distance(isovist.begin(), ptIt);
        if ((idx >= minIdx) && (idx < minIdx + otherSide)) {
            halfColor.set();
        } else {
            otherColor.set();
        }
        glVertex2f(position.x, position.y);
        glVertex2f(ptIt->x, ptIt->y);
    }
    glEnd();

    auto hull = math::convex_hull<float>(isovist.begin(), isovist.end());

    glLineWidth(1.5f);
    occupied_color().set(0.75);
    glBegin(GL_LINE_LOOP);
    for (auto endpoint : hull) {
        glVertex2f(endpoint.x, endpoint.y);
    }
    glEnd();

    glLineWidth(3.0f);
    color.set();
    glBegin(GL_LINE_LOOP);
    for (auto endpoint : isovist) {
        glVertex2f(endpoint.x, endpoint.y);
    }
    glEnd();
}


void IsovistRenderer::renderIsovistField(utils::IsovistField::Iter begin,
                                         utils::IsovistField::Iter end,
                                         utils::Isovist::Scalar value,
                                         double scale) const
{
    if (begin == end) {
        return;
    }

    if (is_scalar_orientation(value)) {
        renderOrientationField(begin, end, value, scale);
    } else {
        renderScalarField(begin, end, value, scale);
    }
}


void IsovistRenderer::renderIsovistDerivField(utils::IsovistField::Iter begin,
                                              utils::IsovistField::Iter end,
                                              utils::Isovist::Scalar value,
                                              double scale) const
{
    if (begin == end) {
        return;
    }

    renderDerivField(begin, end, value, scale);
}


void IsovistRenderer::renderScalarField(utils::IsovistField::Iter begin,
                                        utils::IsovistField::Iter end,
                                        utils::Isovist::Scalar value,
                                        double scale) const
{
    auto scalarComp = [value](const utils::Isovist& lhs, const utils::Isovist& rhs) {
        return lhs.scalar(value) < rhs.scalar(value);
    };

    double minValue = std::min_element(begin, end, scalarComp)->scalar(value);
    double maxValue = std::max_element(begin, end, scalarComp)->scalar(value);
    double range = maxValue - minValue;

    glBegin(GL_QUADS);
    while (begin != end) {
        drawScalar(begin->position(), (begin->scalar(value) - minValue) / range, scale);
        ++begin;
    }
    glEnd();
}


void IsovistRenderer::renderOrientationField(utils::IsovistField::Iter begin,
                                             utils::IsovistField::Iter end,
                                             utils::Isovist::Scalar scalar,
                                             double scale) const
{
    glBegin(GL_QUADS);
    while (begin != end) {
        drawOrientation(begin->position(), begin->scalar(scalar), scale);
        ++begin;
    }
    glEnd();
}


void IsovistRenderer::renderDerivField(utils::IsovistField::Iter begin,
                                       utils::IsovistField::Iter end,
                                       utils::Isovist::Scalar value,
                                       double scale) const
{
    auto scalarComp = [value](const utils::Isovist& lhs, const utils::Isovist& rhs) {
        return lhs.scalarDeriv(value) < rhs.scalarDeriv(value);
    };

    double minValue = std::min_element(begin, end, scalarComp)->scalarDeriv(value);
    double maxValue = std::max_element(begin, end, scalarComp)->scalarDeriv(value);
    double range = maxValue - minValue;

    glBegin(GL_QUADS);
    while (begin != end) {
        drawScalar(begin->position(), (begin->scalarDeriv(value) - minValue) / range, scale);
        ++begin;
    }
    glEnd();
}


void IsovistRenderer::drawScalar(const Point<double>& position, double value, double scale) const
{
    auto color = scalarInterpolator_.calculateColor(value);
    color.set();
    drawValueRect(position, scale);
}


void IsovistRenderer::drawOrientation(const Point<double>& position, double orientation, double scale) const
{
    auto color = orientationInterpolator_.calculateColor(orientation, CircularColorInterpolator::pi_over_two);
    color.set();
    drawValueRect(position, scale);
}


void IsovistRenderer::drawValueRect(const Point<double>& position, double scale) const
{
    const double kWidth = scale * 2.0;

    glVertex2f(position.x, position.y);
    glVertex2f(position.x + kWidth, position.y);
    glVertex2f(position.x + kWidth, position.y + kWidth);
    glVertex2f(position.x, position.y + kWidth);
}


bool is_scalar_orientation(utils::Isovist::Scalar scalar)
{
    return (scalar == utils::Isovist::kOrientation) || (scalar == utils::Isovist::kWeightedOrientation);
}

}   // namespace ui
}   // namespace vulcan
