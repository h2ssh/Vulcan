/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose_target_renderer.cpp
* \author   Collin Johnson and Jong Jin Park
*
* Definition of PoseTargetRenderer.
*/

#include "ui/components/pose_target_renderer.h"
#include "ui/components/robot_renderer.h"
#include "core/pose.h"
#include "ui/common/gl_shapes.h"
#include <GL/gl.h>


namespace vulcan
{
namespace ui
{

PoseTargetRenderer::PoseTargetRenderer(void)
: targetSize_(0.5)
, arrowLength_(0.75)
, targetColor_(GLColor(0.8f, 0.8f, 0.1f, 0.5f))
, arrowColor_(GLColor(0.2f, 0.2f, 0.2f, 0.5f))
, robotRenderer_(std::make_unique<RobotRenderer>())
{

}


PoseTargetRenderer::~PoseTargetRenderer(void)
{
    // For std::unique_ptr
}


void PoseTargetRenderer::setRenderColors(const GLColor& targetColor, const GLColor& arrowColor)
{
    this->targetColor_ = targetColor;
    this->arrowColor_  = arrowColor;
    robotRenderer_->setRobotColor(targetColor);
}


void PoseTargetRenderer::renderTargetCircle(const Point<float>& waypointPosition, const Point<float>& orientationPosition) const
{
    // Two pieces: draw the circle at the waypoint. draw the line/arrow pointing to the orientation
    // The pose target is going to be a bullseye!
    targetColor_.set();
    gl_draw_filled_circle(waypointPosition, targetSize_, 25);

    glColor4f(1.0f, 1.0f, 1.0f, 0.1f);
    gl_draw_filled_circle(waypointPosition, targetSize_*0.66, 25);

    targetColor_.set();
    gl_draw_filled_circle(waypointPosition, targetSize_*0.33, 25);

    arrowColor_.set();
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glVertex2f(waypointPosition.x, waypointPosition.y);
    glVertex2f(orientationPosition.x, orientationPosition.y);
    glEnd();
}


void PoseTargetRenderer::renderTargetCircle(const pose_t& target) const
{
    // Create the two points, then just render with the other method
    Point<float> waypointPosition(target.x, target.y);
    Point<float> orientationPosition(target.x + arrowLength_*cos(target.theta),
                                           target.y + arrowLength_*sin(target.theta));

    renderTargetCircle(waypointPosition, orientationPosition);
}


void PoseTargetRenderer::renderTargetTriangle(const pose_t& target, bool isFilled) const
{
    Point<float> triangleCenter(target.x, target.y);

    targetColor_.set();

    if(isFilled)
    {
        gl_draw_filled_triangle(triangleCenter, 0.5*targetSize_, targetSize_, target.theta);
    }
    else
    {
        float triangleLineWidth = 3.0f;
        gl_draw_line_triangle(triangleCenter, 0.5*targetSize_, targetSize_, target.theta, triangleLineWidth);
    }
}


void PoseTargetRenderer::renderTargetRectangle(const pose_t& target) const
{
    robotRenderer_->renderBoundary(target);

    arrowColor_.set();
    glLineWidth(3.0f);
    glBegin(GL_LINES);
    glVertex2f(target.x, target.y);
    glVertex2f(target.x + arrowLength_*cos(target.theta), target.y + arrowLength_*sin(target.theta));
    glEnd();
}


}
}
