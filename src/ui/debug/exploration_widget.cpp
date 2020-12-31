/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     exploration_widget.cpp
 * \author   Collin Johnson
 *
 * Definition of ExplorationDisplayWidget.
 */

#include "ui/debug/exploration_widget.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "mpepc/simulator/dynamic_object_trajectory.h"
#include "ui/components/exploration_map_renderer.h"
#include "ui/components/occupancy_grid_renderer.h"
#include "ui/components/pose_target_renderer.h"
#include "ui/components/robot_renderer.h"
#include "ui/components/robot_trajectory_renderer.h"
#include "ui/components/tracked_object_renderer.h"

namespace vulcan
{
namespace ui
{

ExplorationDisplayWidget::ExplorationDisplayWidget(wxWindow* parent,
                                                   wxWindowID id,
                                                   const wxPoint& pos,
                                                   const wxSize& size,
                                                   long int style,
                                                   const wxString& name,
                                                   const wxPalette& palette)
: GridBasedDisplayWidget(parent, id, pos, size, style, name, palette)
, currentId_(-1)
, targetId_(-1)
, centerOnRobot_(true)
, haveNewLpm_(false)
, robotRenderer_(new RobotRenderer)
, targetRenderer_(new PoseTargetRenderer)
, lpmRenderer_(new OccupancyGridRenderer)
, explorationRenderer_(new ExplorationMapRenderer)
, robotTrajectoryRenderer_(new RobotTrajectoryRenderer)
, objectRenderer_(new TrackedObjectRenderer)
{
    // Just draw the grid as black and white
    lpmRenderer_->setDynamicColor(GLColor(255, 255, 255, 255));
    lpmRenderer_->setOccupiedColor(GLColor(0, 0, 0, 255));
    lpmRenderer_->setHazardColor(GLColor(0, 0, 0, 255));
    lpmRenderer_->setLimitedVisibilityColor(GLColor(0, 0, 0, 255));
    lpmRenderer_->setQuasiStaticColor(GLColor(0, 0, 0, 255));
}


ExplorationDisplayWidget::~ExplorationDisplayWidget(void)
{
    // For std::unique_ptr
}


void ExplorationDisplayWidget::setPose(const pose_t& pose)
{
    pose_ = pose;
}


void ExplorationDisplayWidget::setLPM(const hssh::LocalPerceptualMap& lpm)
{
    lpm_ = lpm;
    haveNewLpm_ = true;
}


void ExplorationDisplayWidget::setExplorationMap(const planner::LocalTopoExplorationMap& map)
{
    explorationMap_ = map;
}


void ExplorationDisplayWidget::setCurrentArea(hssh::LocalArea::Id currentId)
{
    currentId_ = currentId;
}


void ExplorationDisplayWidget::setTargetArea(hssh::LocalArea::Id targetId)
{
    targetId_ = targetId;
}


void ExplorationDisplayWidget::setNavigationTask(std::shared_ptr<mpepc::NavigationTask> task)
{
    plannerTask_ = task;
}


void ExplorationDisplayWidget::setOptimalTrajectory(const mpepc::robot_trajectory_debug_info_t& trajectory)
{
    optimalTrajectory_ = trajectory;
}


void ExplorationDisplayWidget::setDynamicObjects(
  const std::vector<mpepc::dynamic_object_trajectory_debug_info_t>& objects)
{
    dynamicObjects_ = objects;
}


Point<int> ExplorationDisplayWidget::convertWorldToGrid(const Point<float>& world) const
{
    return utils::global_point_to_grid_cell(world, lpm_);
}


void ExplorationDisplayWidget::renderWidget(void)
{
    if (haveNewLpm_) {
        lpmRenderer_->setGrid(lpm_);
        haveNewLpm_ = false;
    }

    if (centerOnRobot_) {
        setCameraFocalPoint(pose_.toPoint());
    }

    lpmRenderer_->renderGrid();
    explorationRenderer_->renderLocalTopoExplorationMap(explorationMap_, currentId_, targetId_);

    if (plannerTask_) {
        targetRenderer_->renderTargetRectangle(plannerTask_->target());
    }

    robotRenderer_->renderRobot(pose_);
    robotTrajectoryRenderer_->renderTrajectory(optimalTrajectory_.poses);

    for (auto& object : dynamicObjects_) {
        objectRenderer_->renderEstimatedObjectTrajectory(object);
    }
}

}   // namespace ui
}   // namespace vulcan
