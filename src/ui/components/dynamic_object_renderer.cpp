/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     dynamic_object_renderer.cpp
* \author   Collin Johnson
*
* Definition of DynamicObjectRenderer.
*/

#include "ui/components/dynamic_object_renderer.h"
#include "ui/components/object_boundary_renderer.h"
#include "ui/common/color_interpolator.h"
#include "ui/common/gl_shapes.h"
#include "tracker/dynamic_object_collection.h"
#include "tracker/dynamic_object.h"
#include "tracker/objects/person.h"
#include "tracker/objects/rigid.h"
#include "tracker/objects/unclassified.h"
#include "tracker/objects/pivoting_object.h"
#include "tracker/objects/sliding_object.h"
#include <GL/gl.h>
#include <boost/variant/static_visitor.hpp>

namespace vulcan
{
namespace ui
{


struct GoalRenderer : boost::static_visitor<>
{
    GLColor color;
    Point<float> objectPosition;
    bool drawDistribution;

    void operator()(const Point<double>& position)
    {
        color.set(0.9);
        gl_draw_filled_circle(position, 0.125);
        color.set();
        gl_draw_line_circle(position, 0.125, 1.0);

        glLineWidth(4.0);
        color.set(0.9);
        glBegin(GL_LINES);
        glVertex2f(objectPosition.x, objectPosition.y);
        glVertex2f(position.x, position.y);
        glEnd();
    }

    void operator()(const Line<double>& boundary)
    {
        glLineWidth(4.0);
        color.set(0.9);
        glBegin(GL_TRIANGLES);
        glVertex2f(objectPosition.x, objectPosition.y);
        glVertex2f(boundary.a.x, boundary.a.y);
        glVertex2f(boundary.b.x, boundary.b.y);
        glEnd();
    }
};


DynamicObjectRenderer::DynamicObjectRenderer(void)
: uncertaintyFlags_(0)
, unclassifiedColor_(0, 0, 0, 200)
, pivotingColor_(157, 104, 12, 200)
, slidingColor_(136, 136, 136, 200)
, stridingColor_(55, 166, 70, 200)
, rigidColor_(58, 131, 205, 200)
{
}


void DynamicObjectRenderer::renderCollectionStateEstimates(const tracker::DynamicObjectCollection& collection,
                                                           uint32_t uncertaintyOptions)
{
    uncertaintyFlags_ = uncertaintyOptions;
    collection.visitAll(*this);
}


void DynamicObjectRenderer::renderObjectStateEstimate(const tracker::DynamicObject& object,
                                                      uint32_t uncertaintyOptions)
{
    uncertaintyFlags_ = uncertaintyOptions;
    object.accept(*this);
}


void DynamicObjectRenderer::renderObjectGoals(const tracker::DynamicObject& object, bool showFullDistribution)
{
    auto goals = object.goals();

    // If not showing the full distribution, just erase all not-the-best goals
    if(!showFullDistribution)
    {
        goals = tracker::ObjectGoalDistribution{goals.bestGoal()};
    }

    GoalRenderer r;
    r.drawDistribution = showFullDistribution;

    ValueColorInterpolator interpolator(GLColor(233./255, 132./255, 255./255, 0.5));

    for(auto& g : goals)
    {
        r.color = interpolator.calculateColor(g.probability());
        r.objectPosition = object.position();
        auto dest = g.destination();
        apply_visitor(r, dest);
    }
}


void DynamicObjectRenderer::visitPerson(const tracker::Person& person)
{
    drawObject(person, stridingColor_);
}


void DynamicObjectRenderer::visitPivotingObject(const tracker::PivotingObject& door)
{
    drawObject(door, pivotingColor_);

    // Draw the arc through which the door sweeps
    pivotingColor_.set(0.25);
    gl_draw_filled_arc(door.arc());
    pivotingColor_.set(0.5);
    gl_draw_line_arc(door.arc());
}


void DynamicObjectRenderer::visitRigid(const tracker::RigidObject& object)
{
    drawBoundary(object.boundary(), rigidColor_, true);
    drawRigidObjectState(object, rigidColor_);
}


void DynamicObjectRenderer::visitSlidingObject(const tracker::SlidingObject& door)
{
    drawObject(door, slidingColor_);
}


void DynamicObjectRenderer::visitUnclassified(const tracker::UnclassifiedObject& object)
{
    drawObject(object, unclassifiedColor_);
}


void DynamicObjectRenderer::drawObject(const tracker::DynamicObject& object, const GLColor& color)
{
    drawBoundary(object.boundary(), color, true);
    drawVelocity(object, color);
}


void DynamicObjectRenderer::drawBoundary(const tracker::ObjectBoundary& boundary, const GLColor& color, bool filled)
{
//     if(filled)
//     {
//         boundary.visitShape(ObjectBoundaryRenderer(color));
//     }
//     else
//     {
//         boundary.visitShape(OutlineObjectBoundaryRenderer(color));
//     }

    color.set();
    gl_draw_line_circle(boundary.circleApproximation().center(), boundary.circleApproximation().radius());
}


void DynamicObjectRenderer::drawVelocity(const tracker::DynamicObject& object, const GLColor& color)
{
    GLColor accelColor(200, 0, 0, 150);
    const float kVelocityLineWidth = 2.0f;

    auto motionState = object.motionState();

    glLineWidth(kVelocityLineWidth);
    glPushMatrix();
    glTranslatef(motionState.x, motionState.y, 0.0f);
    glBegin(GL_LINES);

    color.set();
    glVertex2f(0.0f, 0.0f);
    glVertex2f(motionState.xVel, motionState.yVel);

    if(uncertaintyFlags_ & kShowAcceleration)
    {
        accelColor.set();
        glVertex2f(0.0f, 0.0f);
        glVertex2f(motionState.xAccel, motionState.yAccel);
    }

    glEnd();
    glPopMatrix();
}


void DynamicObjectRenderer::drawRigidObjectState(const tracker::RigidObject& object, const GLColor& color)
{
    using msi = tracker::MotionStateIndex;

    // Draw uncertainty
    auto stateToDraw = (uncertaintyFlags_ & kShowFastState) ? object.fastMotionState() : object.slowMotionState();

    if(uncertaintyFlags_ & (kShowAccelerationUncertainty | kShowVelocityUncertainty | kShowPositionUncertainty))
    {
        Vector mean(2);
        mean[msi::xIndex] = stateToDraw[msi::xIndex];
        mean[msi::yIndex] = stateToDraw[msi::yIndex];
        Matrix cov;
        if(uncertaintyFlags_ & kShowPositionUncertainty)
        {
            cov = stateToDraw.getCovariance().submat(msi::xIndex, msi::xIndex, msi::yIndex, msi::yIndex);
        }
        else if(uncertaintyFlags_ & kShowVelocityUncertainty)
        {
            cov = stateToDraw.getCovariance().submat(msi::velXIndex, msi::velXIndex, msi::velYIndex, msi::velYIndex);
        }
        else if(uncertaintyFlags_ & kShowAccelerationUncertainty)
        {
            cov = stateToDraw.getCovariance().submat(msi::accelXIndex, msi::accelXIndex, msi::accelYIndex, msi::accelYIndex);
        }

        MultivariateGaussian uncertainty(mean, cov);
        gl_draw_gaussian_distribution(uncertainty, 1.0f, color);
    }

    // Draw velocity and acceleration estimates
    const float kVelocityLineWidth = 2.0f;

    glLineWidth(kVelocityLineWidth);
    glPushMatrix();
    glTranslatef(stateToDraw[msi::xIndex], stateToDraw[msi::yIndex], 0.0f);
    glBegin(GL_LINES);

    color.set();
    glVertex2f(0.0f, 0.0f);
    glVertex2f(stateToDraw[msi::velXIndex], stateToDraw[msi::velYIndex]);

    if(uncertaintyFlags_ & kShowAcceleration)
    {
        GLColor accelColor(200, 0, 0, 150);
        accelColor.set();
        glVertex2f(0.0f, 0.0f);
        glVertex2f(stateToDraw[msi::accelXIndex], stateToDraw[msi::accelYIndex]);
    }

    glEnd();
    glPopMatrix();
}

} // namespace ui
} // namespace vulcan
