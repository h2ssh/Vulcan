/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     gl_shapes.h
 * \author   Collin Johnson
 *
 * Declaration of functions for drawing various shape primitives using OpenGL.
 */

#ifndef UI_COMMON_GL_SHAPES_H
#define UI_COMMON_GL_SHAPES_H

#include "core/point.h"
#include "math/geometry/shapefwd.h"
#include "math/mathfwd.h"
#include <vector>


namespace vulcan
{
class MultivariateGaussian;
namespace ui
{

class GLColor;

void gl_draw_filled_circle(const Point<float>& center, float radius, int numSegments = 36);
void gl_draw_line_circle(const Point<float>& center, float radius, float width = 2.0f, int numSegments = 36);

void gl_draw_filled_circle(const math::Circle<float>& circle, int numSegments = 36);
void gl_draw_line_circle(const math::Circle<float>& circle, float width = 2.0f, int numSegments = 36);

void gl_draw_filled_arc(const math::Arc<float>& arc, int numSegments = 36);
void gl_draw_line_arc(const math::Arc<float>& arc, float width = 2.0f, int numSegments = 36);

void gl_draw_filled_ellipse(const Point<float>& center, float xAxisRadius, float yAxisRadius, int numSegments = 36);
void gl_draw_line_ellipse(const Point<float>& center,
                          float xAxisRadius,
                          float yAxisRadius,
                          float width = 2.0f,
                          int numSegments = 36);

void gl_draw_filled_polygon(const std::vector<Point<float>>& vertices);
void gl_draw_line_polygon(const std::vector<Point<float>>& vertices, float width = 2.0f);

void gl_draw_filled_polygon(const math::Polygon<double>& polygon);
void gl_draw_line_polygon(const math::Polygon<double>& polygon, float width = 2.0f);

void gl_draw_filled_rectangle(const math::Rectangle<float>& rect);
void gl_draw_line_rectangle(const math::Rectangle<float>& rect, float width = 2.0f);

void gl_draw_filled_triangle(const Point<float>& center, float width, float height, float orientation);
void gl_draw_line_triangle(const Point<float>& center,
                           float width,
                           float height,
                           float orientation,
                           float lineWidth = 2.0f);

void gl_draw_small_arrow(const Point<float>& start, float length, float direction, float lineWidth = 2.0f);
void gl_draw_medium_arrow(const Point<float>& start, float length, float direction, float lineWidth = 2.0f);
void gl_draw_large_arrow(const Point<float>& start, float length, float direction, float lineWidth = 2.0f);

void gl_draw_small_arrow_polygon_line(const Point<float>& start, float length, float direction, float lineWidth = 2.0f);
void gl_draw_medium_arrow_polygon_line(const Point<float>& start,
                                       float length,
                                       float direction,
                                       float lineWidth = 2.0f);
void gl_draw_large_arrow_polygon_line(const Point<float>& start, float length, float direction, float lineWidth = 2.0f);

void gl_draw_small_arrow_polygon_filled(const Point<float>& start, float length, float direction);
void gl_draw_medium_arrow_polygon_filled(const Point<float>& start, float length, float direction);
void gl_draw_large_arrow_polygon_filled(const Point<float>& start, float length, float direction);

void gl_draw_gaussian_distribution(const MultivariateGaussian& gaussian,
                                   float numSigma,
                                   const GLColor& color,
                                   float width = 2.0f);

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMMON_GL_SHAPES_H
