/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     gl_shapes.cpp
* \author   Collin Johnson
*
* Definition of a number of functions for drawing various primitive shapes using OpenGL.
*/

#include <ui/common/gl_shapes.h>
#include <ui/common/ui_color.h>
#include <math/angle_range.h>
#include <core/point.h>
#include <core/multivariate_gaussian.h>
#include <math/geometry/arc.h>
#include <math/geometry/circle.h>
#include <math/geometry/rectangle.h>
#include <math/geometry/polygon.h>
#include <boost/range/iterator_range.hpp>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

const float ARROW_ANGLE        = 150.0f;   // degrees of offset for the lines forming the point of the arrow
// The arrow ratios are how long the arrow head is vs. the length
const float SMALL_ARROW_RATIO  = 0.1f;
const float MEDIUM_ARROW_RATIO = 0.2f;
const float LARGE_ARROW_RATIO  = 0.3f;

void draw_filled_partial_ellipse(Point<float>  center,
                                 float               xAxisRadius,
                                 float               yAxisRadius,
                                 math::angle_range_t range,
                                 int                 numSegments);
void draw_line_partial_ellipse(Point<float>  center,
                               float               xAxisRadius,
                               float               yAxisRadius,
                               math::angle_range_t range,
                               float               lineWidth,
                               int                 numSegments);
void draw_arrow(const Point<float>& start, float length, float direction, float lineWidth, float ratio);
void draw_arrow_polygon(const Point<float>& start,
                        float length,
                        float direction,
                        float lineWidth,
                        float ratio,
                        bool filled);
template <typename PointIter>
void gl_draw_filled_polygon_impl(PointIter begin, PointIter end);

template <typename PointIter>
void gl_draw_line_polygon_impl(PointIter begin, PointIter end, float lineWidth);


void gl_draw_filled_circle(const Point<float>& center, float radius, int numSegments)
{
    gl_draw_filled_ellipse(center, radius, radius, numSegments);
}


void gl_draw_line_circle(const Point<float>& center, float radius, float width, int numSegments)
{
    gl_draw_line_ellipse(center, radius, radius, width, numSegments);
}


void gl_draw_filled_arc(const math::Arc<float>& arc, int numSegments)
{
    draw_filled_partial_ellipse(arc.center(), arc.radius(), arc.radius(), arc.range(), numSegments);
}


void gl_draw_line_arc(const math::Arc<float>& arc, float width, int numSegments)
{
    draw_line_partial_ellipse(arc.center(), arc.radius(), arc.radius(), arc.range(), width, numSegments);

    // Draw line segments connecting the center of the arc to the curved portion
    glLineWidth(width);
    glBegin(GL_LINES);
    glVertex2f(arc.center().x, arc.center().y);
    glVertex2f(arc.center().x + (arc.radius() * std::cos(arc.range().start)),
               arc.center().y + (arc.radius() * std::sin(arc.range().start)));
    glVertex2f(arc.center().x, arc.center().y);
    glVertex2f(arc.center().x + (arc.radius() * std::cos(arc.range().start + arc.range().extent)),
               arc.center().y + (arc.radius() * std::sin(arc.range().start + arc.range().extent)));
    glEnd();
}


void gl_draw_filled_circle(const math::Circle<float>& circle, int numSegments)
{
    gl_draw_filled_circle(circle.center(), circle.radius(), numSegments);
}


void gl_draw_line_circle(const math::Circle<float>& circle, float width, int numSegments)
{
    gl_draw_line_circle(circle.center(), circle.radius(), width, numSegments);
}


void gl_draw_filled_ellipse(const Point<float>& center, float xAxisRadius, float yAxisRadius, int numSegments)
{
    draw_filled_partial_ellipse(center, xAxisRadius, yAxisRadius, math::angle_range_t(0.0f, 2*M_PI), numSegments);
}


void gl_draw_line_ellipse(const Point<float>& center, float xAxisRadius, float yAxisRadius, float width, int numSegments)
{
    draw_line_partial_ellipse(center, xAxisRadius, yAxisRadius, math::angle_range_t(0.0f, 2*M_PI), width, numSegments);
}


void gl_draw_filled_polygon(const std::vector<Point<float>>& vertices)
{
    gl_draw_filled_polygon_impl(vertices.begin(), vertices.end());
}


void gl_draw_line_polygon(const std::vector<Point<float>>& vertices, float width)
{
    gl_draw_line_polygon_impl(vertices.begin(), vertices.end(), width);
}


void gl_draw_filled_polygon(const math::Polygon<double>& polygon)
{
    gl_draw_filled_polygon_impl(polygon.begin(), polygon.end());
}


void gl_draw_line_polygon(const math::Polygon<double>& polygon, float width)
{
    gl_draw_line_polygon_impl(polygon.begin(), polygon.end(), width);
}


void gl_draw_filled_rectangle(const math::Rectangle<float>& rect)
{
    glBegin(GL_QUADS);
    glVertex2f(rect.bottomLeft.x,  rect.bottomLeft.y);
    glVertex2f(rect.bottomRight.x, rect.bottomRight.y);
    glVertex2f(rect.topRight.x,    rect.topRight.y);
    glVertex2f(rect.topLeft.x,     rect.topLeft.y);
    glEnd();
}


void gl_draw_line_rectangle(const math::Rectangle<float>& rect, float width)
{
    glLineWidth(width);
    glBegin(GL_LINE_LOOP);
    glVertex2f(rect.bottomLeft.x,  rect.bottomLeft.y);
    glVertex2f(rect.bottomRight.x, rect.bottomRight.y);
    glVertex2f(rect.topRight.x,    rect.topRight.y);
    glVertex2f(rect.topLeft.x,     rect.topLeft.y);
    glEnd();
}


void gl_draw_filled_triangle(const Point<float>& center, float width, float height, float orientation)
{
    glPushMatrix();
    glTranslatef(center.x, center.y, 0.0f);
    glRotatef(orientation*180.0f/M_PI, 0.0f, 0.0f, 1.0f);
    glBegin(GL_TRIANGLES);
    glVertex2f(-height/2.0f, -width/2.0f);
    glVertex2f(height/2.0f, 0.0f);
    glVertex2f(-height/2.0f, width/2.0f);
    glEnd();
    glPopMatrix();
}


void gl_draw_line_triangle(const Point<float>& center, float width, float height, float orientation, float lineWidth)
{
    glLineWidth(lineWidth);
    glPushMatrix();
    glTranslatef(center.x, center.y, 0.0f);
    glRotatef(orientation*180.0f/M_PI, 0.0f, 0.0f, 1.0f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(-height/2.0f, -width/2.0f);
    glVertex2f(height/2.0f, 0.0f);
    glVertex2f(-height/2.0f, width/2.0f);
    glEnd();
    glPopMatrix();
}


void gl_draw_small_arrow(const Point<float>& start, float length, float direction, float lineWidth)
{
    draw_arrow(start, length, direction, lineWidth, SMALL_ARROW_RATIO);
}


void gl_draw_medium_arrow(const Point<float>& start, float length, float direction, float lineWidth)
{
    draw_arrow(start, length, direction, lineWidth, MEDIUM_ARROW_RATIO);
}


void gl_draw_large_arrow(const Point<float>& start, float length, float direction, float lineWidth)
{
    draw_arrow(start, length, direction, lineWidth, LARGE_ARROW_RATIO);
}


void gl_draw_small_arrow_polygon_line(const Point<float>& start, float length, float direction, float lineWidth)
{
    draw_arrow_polygon(start, length, direction, lineWidth, SMALL_ARROW_RATIO, false);
}


void gl_draw_medium_arrow_polygon_line(const Point<float>& start, float length, float direction, float lineWidth)
{
    draw_arrow_polygon(start, length, direction, lineWidth, MEDIUM_ARROW_RATIO, false);
}


void gl_draw_large_arrow_polygon_line(const Point<float>& start, float length, float direction, float lineWidth)
{
    draw_arrow_polygon(start, length, direction, lineWidth, LARGE_ARROW_RATIO, false);
}


void gl_draw_small_arrow_polygon_filled(const Point<float>& start, float length, float direction)
{
    draw_arrow_polygon(start, length, direction, 0.0f, SMALL_ARROW_RATIO, true);
}


void gl_draw_medium_arrow_polygon_filled(const Point<float>& start, float length, float direction)
{
    draw_arrow_polygon(start, length, direction, 0.0f, MEDIUM_ARROW_RATIO, true);
}


void gl_draw_large_arrow_polygon_filled(const Point<float>& start, float length, float direction)
{
    draw_arrow_polygon(start, length, direction, 0.0f, LARGE_ARROW_RATIO, true);
}


void gl_draw_gaussian_distribution(const MultivariateGaussian& gaussian, float numSigma, const GLColor& color, float width)
{
    /*
    * A Gaussian distribution will be drawn as the following:
    *
    *   - A low-alpha filled-in ellipse
    *   - Lines along the major and minor axis
    *   - A solid line bounding the ellipse
    */

    const int NUM_SEGMENTS = 36;

    Vector eigenvalues;
    Matrix eigenvectors;

    Point<float> center(gaussian.getMean()(0), gaussian.getMean()(1));
    Matrix xyCov = gaussian.getCovariance().submat(arma::span(0,1), arma::span(0,1));

    arma::eig_sym(eigenvalues, eigenvectors, xyCov);

    float theta = atan2(eigenvectors(1, 0), eigenvectors(0, 0));

    // The eigenvalues are sigma^2, so scale them to represent the desired number of sigmas for the ellipses
    eigenvalues = numSigma * arma::sqrt(eigenvalues);

    glPushMatrix();

    glTranslatef(center.x, center.y, 0.0f);
    glRotatef(theta*180.0f/M_PI, 0.0f, 0.0f, 1.0f);

    color.set(0.33f);
    gl_draw_filled_ellipse(Point<float>(0.0f, 0.0f), eigenvalues(0), eigenvalues(1), NUM_SEGMENTS);

    color.set();

    glLineWidth(width);
    glBegin(GL_LINES);
    glVertex2f(-eigenvalues(0), 0.0f);
    glVertex2f(eigenvalues(0), 0.0f);

    glVertex2f(0.0f, -eigenvalues(1));
    glVertex2f(0.0f, eigenvalues(1));
    glEnd();

    gl_draw_line_ellipse(Point<float>(0.0f, 0.0f), eigenvalues(0), eigenvalues(1), width, NUM_SEGMENTS);

    glPopMatrix();
}


void draw_filled_partial_ellipse(Point<float>  center,
                                 float               xAxisRadius,
                                 float               yAxisRadius,
                                 math::angle_range_t range,
                                 int                 numSegments)
{
    // Use the triangle fan to easily draw the circle by incrementing the next vertex by the segment interval
    float rotationStep = range.extent / numSegments;

    glBegin(GL_TRIANGLE_FAN);

    glVertex2f(center.x, center.y);

    // Need to do <= numSegments because the first vertex needs to be drawn a second time in order to close the circle
    for(int n = 0; n <= numSegments; ++n)
    {
        glVertex2f(center.x + xAxisRadius*cos(n*rotationStep + range.start),
                   center.y + yAxisRadius*sin(n*rotationStep + range.start));
    }

    glEnd();
}


void draw_line_partial_ellipse(Point<float>  center,
                               float               xAxisRadius,
                               float               yAxisRadius,
                               math::angle_range_t range,
                               float               lineWidth,
                               int                 numSegments)
{
    float rotationStep = range.extent / numSegments;

    glLineWidth(lineWidth);
    glBegin(GL_LINE_STRIP);

    // Need to do <= numSegments because the first vertex needs to be drawn a second time in order to close the circle
    for(int n = 0; n <= numSegments; ++n)
    {
        glVertex2f(center.x + xAxisRadius*cos(n*rotationStep + range.start),
                   center.y + yAxisRadius*sin(n*rotationStep + range.start));
    }

    glEnd();
}


void draw_arrow(const Point<float>& start, float length, float direction, float lineWidth, float ratio)
{
    float headLength = length * ratio;

    glLineWidth(lineWidth);
    glPushMatrix();
    glTranslatef(start.x, start.y, 0.0f);
    glRotatef(direction*180.0/M_PI, 0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);

    // Body of the arrow
    glVertex2f(0.0f, 0.0f);
    glVertex2f(length, 0.0f);

    // Left arrow
    glVertex2f(length, 0.0f);
    glVertex2f(length - headLength*std::cos(ARROW_ANGLE), headLength*std::sin(ARROW_ANGLE));

    // Right arrow
    glVertex2f(length, 0.0f);
    glVertex2f(length - headLength*std::cos(ARROW_ANGLE), -headLength*std::sin(ARROW_ANGLE));

    glEnd();
    glPopMatrix();
}


void draw_arrow_polygon(const Point<float>& start,
                        float length,
                        float direction,
                        float lineWidth,
                        float ratio,
                        bool filled)
{
    // The body of the array is 2/3 of the length
    // The height of the head is 1/3 the length
    const float headLength = length * ratio;
    const float height = headLength * 0.8f;
    const float bodyLength = length - headLength;
    const float bodyHeight = height / 2.0f;

    math::Rectangle<float> body(Point<float>(0.0f, -bodyHeight), Point<float>(bodyLength, bodyHeight));
    Point<float> triangleBottom(bodyLength, -height);
    Point<float> triangleTop(bodyLength, height);
    Point<float> triangleRight(length, 0.0f);

    glPushMatrix();
    glTranslatef(start.x, start.y, 0.0f);
    glRotatef(direction * 180.0 / M_PI, 0.0, 0.0, 1.0);

    if(filled)
    {
        gl_draw_filled_rectangle(body);
        glBegin(GL_TRIANGLES);
        glVertex2f(triangleTop.x, triangleTop.y);
        glVertex2f(triangleRight.x, triangleRight.y);
        glVertex2f(triangleBottom.x, triangleBottom.y);
        glEnd();
    }
    else
    {

        glLineWidth(lineWidth);
        glBegin(GL_LINE_LOOP);
        glVertex2f(body.bottomRight.x, body.bottomRight.y);
        glVertex2f(body.bottomLeft.x, body.bottomLeft.y);
        glVertex2f(body.topLeft.x, body.topLeft.y);
        glVertex2f(body.topRight.x, body.topRight.y);
        glVertex2f(triangleTop.x, triangleTop.y);
        glVertex2f(triangleRight.x, triangleRight.y);
        glVertex2f(triangleBottom.x, triangleBottom.y);
        glEnd();
    }

    glPopMatrix();
}


template <typename PointIter>
void gl_draw_filled_polygon_impl(PointIter begin, PointIter end)
{
    glBegin(GL_POLYGON);

    for(auto vertex : boost::make_iterator_range(begin, end))
    {
        glVertex2f(vertex.x, vertex.y);
    }

    glEnd();
}


template <typename PointIter>
void gl_draw_line_polygon_impl(PointIter begin, PointIter end, float lineWidth)
{
    glLineWidth(lineWidth);
    glBegin(GL_LINE_LOOP);

    for(auto vertex : boost::make_iterator_range(begin, end))
    {
        glVertex2f(vertex.x, vertex.y);
    }

    glEnd();
}

} // namespace ui
} // namespace vulcan
