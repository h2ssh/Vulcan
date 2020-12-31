/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     tracked_object_renderer.cpp
 * \author   Collin Johnson
 *
 * Definition of TrackedObjectRenderer.
 */

#include "ui/components/tracked_object_renderer.h"
#include "mpepc/simulator/dynamic_object_trajectory.h"
#include "tracker/dynamic_object_collection.h"
#include "ui/common/color_generator.h"
#include "ui/common/gl_shapes.h"
#include "ui/components/object_boundary_renderer.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

TrackedObjectRenderer::TrackedObjectRenderer(void)
{
}


void TrackedObjectRenderer::setRenderColor(const GLColor& objectColor)
{
    objectColor_ = objectColor;
}


void TrackedObjectRenderer::renderObjects(const tracker::DynamicObjectCollection& objects, int options)
{
    auto objectColors = generate_colors(objects.size(), 0.8, false);
    int colorIndex = 0;

    for (auto& o : objects) {
        renderObject(*o, options, objectColors[colorIndex++]);
    }
}


void TrackedObjectRenderer::renderObject(const tracker::DynamicObject& object, int options)
{
    renderObject(object, options, objectColor_);
}


void TrackedObjectRenderer::renderObjectMotion(const tracker::DynamicObject& object, float time, size_t steps)
{
    auto position = object.position();
    auto velocity = object.velocity();

    // Fade the alpha with each step to avoid making a completely incomprehensible display
    float alpha = calculateObjectAlpha(object.totalTimeSeen(), 3000);
    float alphaStep = 1.0f - 1.0f / steps;
    float timestep = time / steps;

    glPushMatrix();
    for (std::size_t n = 1; n <= steps; ++n) {
        objectColor_.set();
        glTranslatef(velocity.x * timestep, velocity.y * timestep, 0.0f);
        drawBoundary(
          object.boundary(),
          GLColor(objectColor_.red(), objectColor_.green(), objectColor_.blue(), alpha * std::pow(alphaStep, n)),
          false);
    }
    glPopMatrix();

    glBegin(GL_LINES);
    objectColor_.set(alpha);
    glVertex2f(position.x, position.y);
    objectColor_.set(alpha * std::pow(alphaStep, steps));
    glVertex2f(position.x + time * velocity.x, position.y + time * velocity.y);
    glEnd();
}

void TrackedObjectRenderer::renderEstimatedObjectTrajectory(
  const mpepc::dynamic_object_trajectory_debug_info_t& estimatedObjectTrajectory)
{
    // parameters for drawing markers and the estimated trajectory of the object
    const size_t kNumEstimatedObjectCenters = 5;
    const float kTrajectoryLineWidth = 2.0f;

    // allocate array if needed
    size_t trjLength = estimatedObjectTrajectory.states.size();
    allocatePoseArraysIfNeeded(trjLength * 2);

    // draw object markers
    size_t stepSize = trjLength / kNumEstimatedObjectCenters;
    //     double objectRadius = std::max(0.2, estimatedObjectTrajectory.radius);
    double objectRadius = 0.2;
    for (size_t n = 0; n < trjLength; n += stepSize) {
        Point<float> estimatedObjectCenter(estimatedObjectTrajectory.states[n].x,
                                           estimatedObjectTrajectory.states[n].y);

        float estimatedObjectSpeed =
          sqrt(estimatedObjectTrajectory.states[n].xVel * estimatedObjectTrajectory.states[n].xVel
               + estimatedObjectTrajectory.states[n].yVel * estimatedObjectTrajectory.states[n].yVel);

        float objectAlpha = 0.75f - (0.50f * static_cast<float>(n) / trjLength);
        glColor4f(0.5f, 0.1f, 0.6f, objectAlpha);
        gl_draw_filled_circle(estimatedObjectCenter, objectRadius);   // draw small circle

        if (estimatedObjectSpeed > 0.05)   // if the speed is non-zero, draw the heading as a line on the circle
        {
            float estimatedObjectHeading =
              atan2(estimatedObjectTrajectory.states[n].yVel, estimatedObjectTrajectory.states[n].xVel);
            GLColor black;
            black.set();
            glBegin(GL_LINES);
            glVertex2f(estimatedObjectCenter.x, estimatedObjectCenter.y);
            glVertex2f(estimatedObjectCenter.x + (objectRadius * std::cos(estimatedObjectHeading)),
                       estimatedObjectCenter.y + (objectRadius * std::sin(estimatedObjectHeading)));
            glEnd();
        }
    }

    // draw trajectory
    for (size_t n = 0; n < trjLength; ++n) {
        trajectoryPoses[2 * n] = estimatedObjectTrajectory.states[n].x;
        trajectoryPoses[2 * n + 1] = estimatedObjectTrajectory.states[n].y;
    }

    glLineWidth(kTrajectoryLineWidth);

    glEnableClientState(GL_VERTEX_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, trajectoryPoses.data());
    glDrawArrays(GL_LINE_STRIP, 0, estimatedObjectTrajectory.states.size());

    glDisableClientState(GL_VERTEX_ARRAY);
}


void TrackedObjectRenderer::allocatePoseArraysIfNeeded(std::size_t size)
{
    if (size > trajectoryPoses.size()) {
        trajectoryPoses.resize(size);
    }
}


void TrackedObjectRenderer::renderObject(const tracker::DynamicObject& object, int options, const GLColor& color)
{
    GLColor objectColor(color.red(), color.green(), color.blue(), calculateObjectAlpha(object.totalTimeSeen(), 3000));
    drawBoundary(object.boundary(), objectColor, true);

    // Draw the velocity vector
    auto position = object.position();
    auto velocity = object.velocity();

    glBegin(GL_LINES);
    glVertex2f(position.x, position.y);
    glVertex2f(position.x + 2 * velocity.x, position.y + 2 * velocity.y);
    glEnd();
}


void TrackedObjectRenderer::drawBoundary(const tracker::ObjectBoundary& boundary, const GLColor& color, bool filled)
{
    if (filled) {
        boundary.visitShape(ObjectBoundaryRenderer(color));
    } else {
        boundary.visitShape(OutlineObjectBoundaryRenderer(color));
    }
}


float TrackedObjectRenderer::calculateObjectAlpha(int32_t trackedTime, int32_t maxTrackedTime)
{
    return 0.5 + (0.9 * std::min(trackedTime, maxTrackedTime)) / maxTrackedTime;
}

}   // namespace ui
}   // namespace vulcan
