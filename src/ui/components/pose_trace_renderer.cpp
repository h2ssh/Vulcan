/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     pose_trace_renderer.cpp
* \author   Collin Johnson
*
* Implementation of PoseTraceRenderer.
*/

#include <ui/components/pose_trace_renderer.h>
#include <core/pose.h>
#include <utils/pose_trace.h>
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

template <class PoseIter>
std::size_t set_trace_poses(PoseIter traceBegin, PoseIter traceEnd, std::vector<float>& poseTrace);


PoseTraceRenderer::PoseTraceRenderer(void)
: traceLength(0)
, poseColor(203, 13, 196, 200)
{
}


void PoseTraceRenderer::setRenderColor(const GLColor& color)
{
    if(traceColors.size() < traceLength * 4)
    {
        traceColors.resize(traceLength * 8);  // double the size + four floats per pose to hold the colors
    }

    float* colors = traceColors.data();

    for(std::size_t i = 0; i < traceLength; ++i)
    {
        *colors++ = color.red();
        *colors++ = color.green();
        *colors++ = color.blue();
        *colors++ = color.alpha();
    }

    poseColor = color;
}


void PoseTraceRenderer::setPoseTrace(const std::deque<pose_t>& trace)
{
    traceLength = set_trace_poses(trace.begin(), trace.end(), poseTrace);
    setRenderColor(poseColor);

    intermediate.clear();
    int64_t lastTime = 0;
    for(auto& pose : trace)
    {
        if(pose.timestamp - lastTime > 1000000)
        {
            intermediate.push_back(pose);
            lastTime = pose.timestamp;
        }
    }
}


void PoseTraceRenderer::setPoseTrace(const utils::PoseTrace& trace)
{
    traceLength = set_trace_poses(trace.begin(), trace.end(), poseTrace);
    setRenderColor(poseColor);
}


void PoseTraceRenderer::renderTrace(void)
{
    if(traceLength == 0)
    {
        return;
    }

    glLineWidth(1.5);

    // The main bit of lidar data exists in two vector lists for the colors and vertices
    glDisableClientState(GL_EDGE_FLAG_ARRAY);  // get rid of the things we aren't using for efficiency sake
    glDisableClientState(GL_INDEX_ARRAY);
    glDisableClientState(GL_SECONDARY_COLOR_ARRAY);
    glDisableClientState(GL_FOG_COORDINATE_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glVertexPointer(2, GL_FLOAT, 0, poseTrace.data());
    glColorPointer(4, GL_FLOAT, 0, traceColors.data());

    glDrawArrays(GL_LINE_STRIP, 0, traceLength);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    glEnd();

    if(showTicks_)
    {
        GLColor black;
        black.set();
        glPointSize(4.0f);
        glBegin(GL_POINTS);
        for(auto& pose : intermediate)
        {
            glVertex2f(pose.x, pose.y);
        }
        glEnd();
    }
}


template <class PoseIter>
std::size_t set_trace_poses(PoseIter traceBegin, PoseIter traceEnd, std::vector<float>& poseTrace)
{
    std::size_t traceLength = std::distance(traceBegin, traceEnd);

    if(poseTrace.size() < traceLength * 2)
    {
        poseTrace.resize(traceLength * 4); // double the size + two floats per pose to hold the position
    }

    float* poses = poseTrace.data();

    for(auto& poseIt = traceBegin; poseIt != traceEnd; ++poseIt)
    {
        *poses++ = poseIt->x;
        *poses++ = poseIt->y;
    }

    return traceLength;
}

} // namespace ui
} // namespace vulcan
