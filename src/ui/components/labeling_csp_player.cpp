/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     labeling_csp_playback.cpp
* \author   Collin Johnson
*
* Definition of LabelingCSPPlayer.
*/

#include "ui/components/labeling_csp_player.h"
#include "ui/components/area_extent_renderer.h"
#include "ui/common/hssh_colors.h"
#include "ui/common/gl_shapes.h"
#include "ui/common/ui_color.h"
#include "hssh/local_topological/area_detection/labeling/csp_debug.h"
#include <GL/gl.h>
#include <cassert>

namespace vulcan
{
namespace ui
{

const GLColor& color_from_hypothesis_type(hssh::HypothesisType type);


LabelingCSPPlayer::LabelingCSPPlayer(void)
: debugInfo_(std::make_unique<hssh::CSPDebugInfo>())
, iteration_(0)
, state_(stopped)
, numFramesShown_(0)
, framesPerIteration_(1)
, extentRenderer_(std::make_unique<AreaExtentRenderer>())
{
}


LabelingCSPPlayer::~LabelingCSPPlayer(void)
{
    // For std::unique_ptr
}


void LabelingCSPPlayer::update(void)
{
    // Nothing is displayed while stopped and the state doesn't change.
    if(state_ == stopped)
    {
        return;
    }

    // Always increment the frame count. If paused longer than the frames per iteration, it will result in the iteration
    // being changed immediately, which is a logical behavior
    ++numFramesShown_;

    // If playing, need to determine if a frame transition needs to occur. Otherwise,
    if(state_ == playing)
    {
        if(numFramesShown_ > framesPerIteration_)
        {
            stepIteration(1);
            numFramesShown_ = 0;
        }
    }

    if(iteration_ >= 0 && iteration_ < static_cast<int>(debugInfo_->iterations.size()))
    {
        renderIteration(debugInfo_->iterations[iteration_]);
    }
}


void LabelingCSPPlayer::setCSPInfo(const hssh::CSPDebugInfo& info, double metersPerCell)
{
    *debugInfo_ = info;
    iteration_ = 0;
    numFramesShown_ = 0;

    if(state_ != stopped)
    {
        state_ = paused;
    }

    metersPerCell_ = metersPerCell;

    std::cout << "CSPPlayer: New CSP solution: " << info.iterations.size() << " iterations.\n";
}


int LabelingCSPPlayer::numIterations(void) const
{
    return debugInfo_->iterations.size();
}


void LabelingCSPPlayer::setFramesPerIteration(int framesPerIteration)
{
    assert(framesPerIteration > 0);
    framesPerIteration_ = framesPerIteration;
}


void LabelingCSPPlayer::play(void)
{
    state_ = playing;
}


void LabelingCSPPlayer::pause(void)
{
    state_ = paused;
}


void LabelingCSPPlayer::stop(void)
{
    state_ = stopped;
}


void LabelingCSPPlayer::stepForward(void)
{
    state_ = paused;
    stepIteration(1);
}


void LabelingCSPPlayer::stepBackward(void)
{
    state_ = paused;
    stepIteration(-1);
}


void LabelingCSPPlayer::jumpToStart(void)
{
    state_ = paused;
    stepIteration(-iteration_);
}


void LabelingCSPPlayer::jumpToEnd(void)
{
    state_ = paused;
    stepIteration(debugInfo_->iterations.size() - iteration_);
}


void LabelingCSPPlayer::jumpToIteration(int iteration)
{
    if((iteration < 0) || (iteration >= numIterations()))
    {
        return;
    }

    stepIteration(iteration - iteration_);
}


void LabelingCSPPlayer::stepIteration(int direction)
{
    numFramesShown_ = 0;
    iteration_ += direction;

    if(iteration_ >= numIterations())
    {
        iteration_ = numIterations() - 1;
    }

    // Don't use else-if because adjusting the max involves a subtraction and an empty set of info will cause
    // iteration to go below 0.
    if(iteration_ < 0)
    {
        iteration_ = 0;
    }
}


void LabelingCSPPlayer::renderIteration(const hssh::CSPIteration& iteration)
{
    assert(iteration.labels.size() == debugInfo_->extents.size());

    bool colorAllAreas = iteration.failedAreas.empty();
    GLColor defaultColor(0.0, 0.0, 0.0, 1.0);

    // Draw all the extents with the appropriate label
    for(std::size_t n = 0; n < iteration.labels.size(); ++n)
    {
        GLColor color = color_from_hypothesis_type(iteration.labels[n]);
        color.alpha((iteration.isFailing[n] || colorAllAreas) ? 0.9 : 0.2);
        extentRenderer_->renderExtentCells(debugInfo_->extents[n], metersPerCell_, color);
    }

    // All failed areas are drawn as failed areas and the assigned is drawn as assigned
    // Draw the updated area first, so the thick boundary is drawn on top of it.
    renderArea(iteration.updatedArea, assigned);

//     for(auto& area : iteration.failedAreas)
//     {
//         renderArea(area, failed);
//     }

    // Draw the endpoints for the areas with assigned path segments
    renderEndpoints(iteration);

    // Draw all active gateway boundaries
    renderGateways(iteration);
}


void LabelingCSPPlayer::renderArea(const hssh::CSPArea& area, AreaStatus status)
{
    const float kBoundaryThickness = 3.0f;

    color_from_hypothesis_type(area.oldType).set();
    gl_draw_line_rectangle(area.boundary, kBoundaryThickness);

    if(status == assigned)
    {
        // Halfway point between the left and right sides
        float splitX = area.boundary.bottomLeft.x + area.boundary.width() / 2;

        color_from_hypothesis_type(area.oldType).set(0.6);
        gl_draw_filled_rectangle(math::Rectangle<float>(area.boundary.bottomLeft,
                                                        Point<float>(splitX, area.boundary.topRight.y)));
        color_from_hypothesis_type(area.newType).set(0.6);
        gl_draw_filled_rectangle(math::Rectangle<float>(Point<float>(splitX, area.boundary.bottomLeft.y),
                                                        area.boundary.topRight));
    }
}


void LabelingCSPPlayer::renderEndpoints(const hssh::CSPIteration& iteration)
{
    glPointSize(5.0);
    glColor4f(0, 0, 0, 0.75f);
    glBegin(GL_POINTS);

    for(auto& endpoints : iteration.pathEndpoints)
    {
        glVertex2f(endpoints.first.x, endpoints.first.y);
        glVertex2f(endpoints.second.x, endpoints.second.y);
    }

    glEnd();
}


void LabelingCSPPlayer::renderGateways(const hssh::CSPIteration& iteration)
{
    glLineWidth(2.0);
    glColor4f(0, 0, 0, 0.75f);
    glBegin(GL_LINES);

    for(auto& gwy : iteration.gateways)
    {
        glVertex2f(gwy.a.x, gwy.a.y);
        glVertex2f(gwy.b.x, gwy.b.y);
    }

    glEnd();
}

} // namespace ui
} // namespace vulcan
