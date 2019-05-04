/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     exploration_panel.cpp
* \author   Collin Johnson
* 
* Definition of ExplorationPanel.
*/

#include <ui/debug/exploration_panel.h>
#include <ui/debug/debug_ui.h>
#include <ui/debug/exploration_widget.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/pose.h>
#include <mpepc/simulator/dynamic_object_trajectory.h>
#include <mpepc/trajectory/trajectory_planner_info.h>
#include <system/module_communicator.h>
#include <utils/auto_mutex.h>

// Put event table in separate namespace to help KDevelop with parsing
namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(ExplorationPanel, wxEvtHandler)
    EVT_CHECKBOX(ID_EXPLORATION_CENTER_ON_ROBOT_BOX, ExplorationPanel::centerOnRobotChanged)
END_EVENT_TABLE()

}
}


namespace vulcan
{
namespace ui
{

ExplorationPanel::ExplorationPanel(const ui_params_t& params, const exploration_panel_widgets_t& widgets)
: widgets_(widgets)
, numLocalTopoVisited_(0)
, numLocalTopoRemaining_(0)
{
    assert(widgets_.display);
    assert(widgets_.totalText);
    assert(widgets_.remainingText);
    assert(widgets_.visitedText);
    assert(widgets_.centerOnRobotCheckbox);

    widgets_.display->shouldCenterOnRobot(widgets_.centerOnRobotCheckbox->IsChecked());
}


void ExplorationPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.display->setRenderContext(context);
    widgets_.display->setStatusBar(statusBar);
}


void ExplorationPanel::subscribe(system::ModuleCommunicator& communicator)
{
    communicator.subscribeTo<hssh::LocalPose>(this);
    communicator.subscribeTo<planner::local_topo_exploration_status_t>(this);
    communicator.subscribeTo<hssh::LocalPerceptualMap>(this);
    communicator.subscribeTo<std::vector<mpepc::dynamic_object_trajectory_debug_info_t>>(this);
    communicator.subscribeTo<mpepc::trajectory_planner_debug_info_t>(this);
}


void ExplorationPanel::setConsumer(system::ModuleCommunicator* consumer)
{
}


void ExplorationPanel::update(void)
{
    updateExplorationStatusText();
    widgets_.display->Refresh();
}


void ExplorationPanel::saveSettings(utils::ConfigFileWriter& config)
{
}


void ExplorationPanel::loadSettings(const utils::ConfigFile& config)
{
}


void ExplorationPanel::handleData(const hssh::LocalPose& pose, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    widgets_.display->setPose(pose.pose());
}


void ExplorationPanel::handleData(const planner::local_topo_exploration_status_t& status, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);

    numLocalTopoVisited_ = status.explorationMap.sizeVisited();
    numLocalTopoRemaining_ = status.explorationMap.sizeUnvisited();

    widgets_.display->setExplorationMap(status.explorationMap);
    widgets_.display->setCurrentArea(status.currentArea);
    widgets_.display->setTargetArea(status.targetArea);
    widgets_.display->setNavigationTask(status.plannerTask);
}


void ExplorationPanel::handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    widgets_.display->setLPM(lpm);
}


void ExplorationPanel::handleData(const std::vector<mpepc::dynamic_object_trajectory_debug_info_t>& objectTrajectories,
                                  const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    widgets_.display->setDynamicObjects(objectTrajectories);
}


void ExplorationPanel::handleData(const mpepc::trajectory_planner_debug_info_t& mpepcInfo, const std::string& channel)
{
    utils::AutoMutex autoLock(dataLock_);
    widgets_.display->setOptimalTrajectory(mpepcInfo.plannedTrajectory);
}


void ExplorationPanel::updateExplorationStatusText(void)
{
    widgets_.totalText->SetLabel(wxString::Format("%i", numLocalTopoRemaining_ + numLocalTopoVisited_));
    widgets_.visitedText->SetLabel(wxString::Format("%i", numLocalTopoVisited_));
    widgets_.remainingText->SetLabel(wxString::Format("%i", numLocalTopoRemaining_));
}


void ExplorationPanel::centerOnRobotChanged(wxCommandEvent& event)
{
    widgets_.display->shouldCenterOnRobot(event.IsChecked());
}

} // namespace ui
} // namespace vulcan
