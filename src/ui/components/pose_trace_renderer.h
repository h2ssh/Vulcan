/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     pose_trace_renderer.h
 * \author   Collin Johnson
 *
 * Definition of PoseTraceRenderer.
 */

#ifndef UI_COMPONENTS_POSE_TRACE_RENDERER_H
#define UI_COMPONENTS_POSE_TRACE_RENDERER_H

#include "core/pose.h"
#include "ui/common/ui_color.h"
#include <deque>
#include <vector>

namespace vulcan
{
struct pose_t;
namespace utils
{
class PoseTrace;
}
namespace ui
{

/**
 * PoseTraceRenderer renders the provided path. A path can be rendered, as long as it is
 * represented a series of poses in the global coordinate reference frame. The intended
 * purpose is for drawing the trajectory followed by the robot, but if objects around the
 * robot are being tracked, then those poses could also be drawn. That would look incredibly awesome.
 *
 * Optionally, tick marks can be drawn along the trace at one second intervals to give a perspective
 * on the velocity of the vehicle.
 */
class PoseTraceRenderer
{
public:
    PoseTraceRenderer(void);

    // Set flag whether or not to draw the one second tick marks along the trajectory
    void showVelocityTicks(bool show) { showTicks_ = show; }

    void setRenderColor(const GLColor& color);
    void setPoseTrace(const std::deque<pose_t>& trace);
    void setPoseTrace(const utils::PoseTrace& trace);
    void renderTrace(void);

private:
    std::vector<float> poseTrace;
    std::vector<float> traceColors;
    std::deque<pose_t> intermediate;

    bool showTicks_ = true;

    std::size_t traceLength;
    GLColor poseColor;
};

}   // namespace ui
}   // namespace vulcan

#endif   // UI_COMPONENTS_POSE_TRACE_RENDERER_H
