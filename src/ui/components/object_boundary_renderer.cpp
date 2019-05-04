/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     object_boundary_renderer.cpp
* \author   Collin Johnson
*
* Definition of ObjectBoundaryRenderer and OutlineObjectBoundaryRenderer.
*/

#include <ui/components/object_boundary_renderer.h>
#include <ui/common/gl_shapes.h>
#include <ui/common/ui_color.h>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

constexpr float kLineWidth = 2.0f;

//////////////////////////////////// ObjectBoundaryRenderer ///////////////////////////////////

ObjectBoundaryRenderer::ObjectBoundaryRenderer(const GLColor& color)
: color_(color)
{
}


void ObjectBoundaryRenderer::operator()(const tracker::Rectangle& rectangle)
{
    color_.set(0.3);
    gl_draw_filled_rectangle(rectangle);
    color_.set();
    gl_draw_line_rectangle(rectangle, kLineWidth);
}


void ObjectBoundaryRenderer::operator()(const tracker::Circle& circle)
{
    color_.set(0.3);
    gl_draw_filled_circle(circle);
    color_.set();
    gl_draw_line_circle(circle, kLineWidth);
}


void ObjectBoundaryRenderer::operator()(const tracker::TwoCircles& twoCircles)
{
    color_.set(0.3);
    gl_draw_filled_circle(twoCircles[0]);
    gl_draw_filled_circle(twoCircles[1]);
    color_.set();
    gl_draw_line_circle(twoCircles[0], kLineWidth);
    gl_draw_line_circle(twoCircles[1], kLineWidth);
    
    glLineWidth(kLineWidth);
    glBegin(GL_LINES);
    glVertex2f(twoCircles[0].center().x, twoCircles[0].center().y);
    glVertex2f(twoCircles[1].center().x, twoCircles[1].center().y);
    glEnd();
}


void ObjectBoundaryRenderer::operator()(const tracker::CircleRect& circleRect)
{
    color_.set(0.3);
    gl_draw_filled_circle(circleRect.circle);
    gl_draw_filled_rectangle(circleRect.rect);
    color_.set();
    gl_draw_line_circle(circleRect.circle, kLineWidth);
    gl_draw_line_rectangle(circleRect.rect, kLineWidth);

    glLineWidth(kLineWidth);
    glBegin(GL_LINES);
    glVertex2f(circleRect.circle.center().x, circleRect.circle.center().y);
    glVertex2f(circleRect.rect.center().x, circleRect.rect.center().y);
    glEnd();
}


void ObjectBoundaryRenderer::operator()(const tracker::TwoRects& rects)
{
    color_.set(0.3);
    gl_draw_filled_rectangle(rects[0]);
    gl_draw_filled_rectangle(rects[1]);
    color_.set();
    gl_draw_line_rectangle(rects[0], kLineWidth);
    gl_draw_line_rectangle(rects[1], kLineWidth);

    glLineWidth(kLineWidth);
    glBegin(GL_LINES);
    glVertex2f(rects[0].center().x, rects[0].center().y);
    glVertex2f(rects[1].center().x, rects[1].center().y);
    glEnd();
}


//////////////////////////////////// OutlineObjectBoundaryRenderer ///////////////////////////////////
OutlineObjectBoundaryRenderer::OutlineObjectBoundaryRenderer(const GLColor& color)
: color_(color)
{
}


void OutlineObjectBoundaryRenderer::operator()(const tracker::Rectangle& rectangle)
{
    color_.set();
    gl_draw_line_rectangle(rectangle, kLineWidth);
}


void OutlineObjectBoundaryRenderer::operator()(const tracker::Circle& circle)
{
    color_.set();
    gl_draw_line_circle(circle, kLineWidth);
}


void OutlineObjectBoundaryRenderer::operator()(const tracker::TwoCircles& twoCircles)
{
    color_.set();
    gl_draw_line_circle(twoCircles[0], kLineWidth);
    gl_draw_line_circle(twoCircles[1], kLineWidth);
}


void OutlineObjectBoundaryRenderer::operator()(const tracker::CircleRect& circleRect)
{
    color_.set();
    gl_draw_line_circle(circleRect.circle, kLineWidth);
    gl_draw_line_rectangle(circleRect.rect, kLineWidth);

    glLineWidth(kLineWidth);
    glBegin(GL_LINES);
    glVertex2f(circleRect.circle.center().x, circleRect.circle.center().y);
    glVertex2f(circleRect.rect.center().x, circleRect.rect.center().y);
    glEnd();
}


void OutlineObjectBoundaryRenderer::operator()(const tracker::TwoRects& rects)
{
    color_.set();
    gl_draw_line_rectangle(rects[0], kLineWidth);
    gl_draw_line_rectangle(rects[1], kLineWidth);

    glLineWidth(kLineWidth);
    glBegin(GL_LINES);
    glVertex2f(rects[0].center().x, rects[0].center().y);
    glVertex2f(rects[1].center().x, rects[1].center().y);
    glEnd();
}

} // namespace ui
} // namespace vulcan
