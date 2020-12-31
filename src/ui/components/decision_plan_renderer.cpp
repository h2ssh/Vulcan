/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision_plan_renderer.cpp
 * \author   Collin Johnson
 *
 * Definition of DecisionPlanRenderer.
 */

#include "ui/components/decision_plan_renderer.h"
#include <GL/gl.h>

namespace vulcan
{
namespace ui
{

void DecisionPlanRenderer::setRenderColors(const GLColor& entry, const GLColor& exit, const GLColor& target)
{
    entryColor = entry;
    exitColor = exit;
    targetColor = target;

    targetRenderer.setRenderColors(target, GLColor());
}


void DecisionPlanRenderer::render(const planner::DecisionPlan& plan)
{
}

}   // namespace ui
}   // namespace vulcan
