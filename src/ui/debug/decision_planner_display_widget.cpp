/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     decision_planner_display_widget.cpp
 * \author   Collin Johnson
 *
 * Definition of DecisionPlannerDisplayWidget.
 */

#include "ui/debug/decision_planner_display_widget.h"
#include "ui/common/ui_params.h"
#include "ui/components/decision_plan_renderer.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "ui/components/robot_renderer.h"
#include "utils/auto_mutex.h"

namespace vulcan
{
namespace ui
{

DecisionPlannerDisplayWidget::DecisionPlannerDisplayWidget(wxWindow* parent,
                                                           wxWindowID id,
                                                           const wxPoint& pos,
                                                           const wxSize& size,
                                                           long style,
                                                           const wxString& name,
                                                           const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, lpmRenderer(new OccupancyGridRenderer)
, planRenderer(new DecisionPlanRenderer)
, robotRenderer(new RobotRenderer)
, haveNewLPM(false)
, havePlan(false)
{
}


DecisionPlannerDisplayWidget::~DecisionPlannerDisplayWidget(void)
{
    delete robotRenderer;
    delete planRenderer;
    delete lpmRenderer;
}


void DecisionPlannerDisplayWidget::setWidgetParams(const lpm_display_params_t& lpmParams,
                                                   const local_topo_display_params_t& localTopoParams,
                                                   const decision_planner_display_params_t& plannerParams)
{
    robotRenderer->setRobotColor(lpmParams.robotColor);
    planRenderer->setRenderColors(plannerParams.placeEntryFragmentColor,
                                  plannerParams.placeExitFragmentColor,
                                  lpmParams.targetCircleColor);
}


void DecisionPlannerDisplayWidget::setLPM(const hssh::LocalPerceptualMap& lpm)
{
    utils::AutoMutex autoLock(dataLock);

    this->lpm = lpm;

    haveNewLPM = true;
}


void DecisionPlannerDisplayWidget::setPose(const pose_t& pose)
{
    utils::AutoMutex autoLock(dataLock);

    this->pose = pose;
}


void DecisionPlannerDisplayWidget::renderWidget(void)
{
    utils::AutoMutex autoLock(dataLock);

    if (haveNewLPM) {
        lpmRenderer->setGrid(lpm);
        haveNewLPM = false;
    }

    lpmRenderer->renderGrid();

    robotRenderer->renderRobot(pose);
}


Point<int> DecisionPlannerDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    return utils::global_point_to_grid_cell(world, lpm);
}

}   // namespace ui
}   // namespace vulcan
