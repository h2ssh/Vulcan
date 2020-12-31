/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     planner_scripting_widget.cpp
 * \author   Collin Johnson
 *
 * Implemenation of PlannerScriptingWidget.
 */

#include "ui/debug/planner_scripting_widget.h"
#include "hssh/local_metric/lpm.h"
#include "ui/common/ui_params.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "ui/components/pose_target_renderer.h"
#include "ui/components/robot_renderer.h"
#include "utils/auto_mutex.h"

namespace vulcan
{
namespace ui
{


PlannerScriptingWidget::PlannerScriptingWidget(wxWindow* parent,
                                               wxWindowID id,
                                               const wxPoint& pos,
                                               const wxSize& size,
                                               long style,
                                               const wxString& name,
                                               const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, lpmRenderer(new OccupancyGridRenderer)
, targetRenderer(new PoseTargetRenderer)
, robotRenderer_(new RobotRenderer)
{
}


void PlannerScriptingWidget::setParams(const ui_params_t& params)
{
    hoverColor = params.scriptingParams.hoverColor;
    selectedColor = params.scriptingParams.selectedColor;
    targetColor = params.scriptingParams.finalizedColor;
    arrowColor = GLColor(0.0, 0.0, 0.0, targetColor.alpha());
}


void PlannerScriptingWidget::setLPM(std::shared_ptr<hssh::LocalPerceptualMap> map)
{
    utils::AutoMutex autoLock(lpmLock);
    lpm = map;
    haveNewLPM = true;
}


void PlannerScriptingWidget::setCurrentPose(const pose_t& pose)
{
    currentPose_ = pose;
}


void PlannerScriptingWidget::setCompletedTargets(const std::vector<mpepc::named_pose_t>& targets)
{
    this->targets = targets;
}


void PlannerScriptingWidget::setHoverTarget(const pose_t& pose)
{
    hoverTarget = pose;
    haveHover = true;
}


void PlannerScriptingWidget::setSelectedTarget(const pose_t& pose)
{
    selectedTarget = pose;
    haveSelected = true;
}


Point<int> PlannerScriptingWidget::convertWorldToGrid(const Point<float>& world) const
{
    if (lpm) {
        return utils::global_point_to_grid_cell(world, *lpm);
    }

    return Point<int>(0, 0);
}


void PlannerScriptingWidget::renderWidget(void)
{
    if (haveNewLPM) {
        utils::AutoMutex autoLock(lpmLock);
        lpmRenderer->setGrid(*lpm);
        haveNewLPM = false;
    }

    lpmRenderer->renderGrid();

    robotRenderer_->renderRobot(currentPose_);

    targetRenderer->setRenderColors(targetColor, arrowColor);
    for (auto& target : targets) {
        targetRenderer->renderTargetCircle(target.pose);
    }

    if (shouldShowHover && haveHover) {
        targetRenderer->setRenderColors(hoverColor, arrowColor);
        targetRenderer->renderTargetCircle(hoverTarget);
    }

    if (shouldShowSelected && haveSelected) {
        targetRenderer->setRenderColors(selectedColor, arrowColor);
        targetRenderer->renderTargetCircle(selectedTarget);
    }
}

}   // namespace ui
}   // namespace vulcan
