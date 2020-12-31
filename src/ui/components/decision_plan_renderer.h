/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_plan_renderer.h
* \author   Collin Johnson
*
* Declaration of DecisionPlanRenderer.
*/

#ifndef UI_COMPONENTS_DECISION_PLAN_RENDERER_H
#define UI_COMPONENTS_DECISION_PLAN_RENDERER_H

#include "ui/common/ui_color.h"
#include "ui/components/pose_target_renderer.h"
#include "ui/components/gateways_renderer.h"

namespace vulcan
{
namespace planner { class DecisionPlan; }

namespace ui
{

/**
* DecisionPlanRenderer renders the state contained in a DecisionPlan. The plan is drawn in two parts.
* First, the metric_pose_target_t is drawn as a target. Second, if the plan involves a place, then
* the entry and exit gateways are highlighted.
*/
class DecisionPlanRenderer
{
public:

    /**
    * setRenderColors sets the colors with which to render the plan.
    */
    void setRenderColors(const GLColor& entry, const GLColor& exit, const GLColor& target);

    /**
    * render draws the plan onto the screen.
    *
    * \param    plan        Plan to be rendered
    */
    void render(const planner::DecisionPlan& plan);

private:

    PoseTargetRenderer targetRenderer;
    GatewaysRenderer   fragmentRenderer;

    GLColor entryColor;
    GLColor exitColor;
    GLColor targetColor;
};

}
}

#endif // UI_COMPONENTS_DECISION_PLAN_RENDERER_H
