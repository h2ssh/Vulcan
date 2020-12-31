/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     metric_planner_panel.cpp
 * \author   Collin Johnson and Jong Jin Park
 *
 * Definition of MetricPlannerPanel.
 */

#include "ui/debug/metric_planner_panel.h"
#include "hssh/local_topological/local_topo_map.h"
#include "mpepc/metric_planner/messages.h"
#include "mpepc/metric_planner/params.h"
#include "mpepc/metric_planner/script/script.h"
#include "mpepc/metric_planner/script/target_set.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "mpepc/motion_controller/messages.h"
#include "robot/model/params.h"
#include "ui/common/metric_path_creator.h"
#include "ui/common/ui_params.h"
#include "ui/debug/debug_ui.h"
#include "ui/debug/metric_planner_display_widget.h"
#include "utils/config_file.h"
// #include <fstream>
// #include <iostream>

// Use a separate namespace block for event table to help KDevelop with parsing.
namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(MetricPlannerPanel, wxEvtHandler)
EVT_CHECKBOX(ID_SHOW_ROBOT_POSE_CHECKBOX, MetricPlannerPanel::showRobotPoseChecked)
EVT_CHECKBOX(ID_SHOW_DESTINATION_POSE_CHECKBOX, MetricPlannerPanel::showDestinationPoseChecked)
EVT_CHECKBOX(ID_SHOW_TRACKED_OBJECTS_CHECKBOX, MetricPlannerPanel::showTrackedObjectsChecked)
EVT_CHECKBOX(ID_SHOW_OBJECTS_MOTION_CHECKBOX, MetricPlannerPanel::showEstimatedObjectMotionsChecked)
EVT_CHECKBOX(ID_SHOW_OBJECTS_MOTION_CHECKBOX, MetricPlannerPanel::showEstimatedObjectMotionsChecked)
EVT_CHECKBOX(ID_SHOW_OPTIMAL_PATH_CHECKBOX, MetricPlannerPanel::showOptimalPathChecked)
EVT_CHECKBOX(ID_SHOW_VISIBILITY_ANALYSIS_CHECKBOX, MetricPlannerPanel::showVisibilityAnalysisChecked)
EVT_CHECKBOX(ID_SHOW_SITUATIONS_CHECKBOX, MetricPlannerPanel::showTopoSituationChecked)
EVT_BUTTON(ID_METRIC_LOAD_SCRIPT_BUTTON, MetricPlannerPanel::loadScriptPressed)
EVT_CHECKBOX(ID_SHOW_WAYPOINTS_BOX, MetricPlannerPanel::showWaypointsChecked)
EVT_CHECKBOX(ID_USE_RRT_STAR_BOX, MetricPlannerPanel::useRRTStarChecked)
EVT_CHECKBOX(ID_STOP_AT_WAYPOINTS_BOX, MetricPlannerPanel::stopAtWaypointsChecked)
EVT_CHECKBOX(ID_SHOW_GRAPH_BOX, MetricPlannerPanel::showGraphChecked)
EVT_CHECKBOX(ID_SIMPLE_FOLLOWING_BOX, MetricPlannerPanel::simpleFollowingChecked)
EVT_CHECKBOX(ID_IGNORE_OBJECT_SPEED_BOX, MetricPlannerPanel::ignoreObjectSpeedChecked)
EVT_BUTTON(ID_UPDATE_WAYPOINTS_BUTTON, MetricPlannerPanel::updateWaypointsPressed)
EVT_BUTTON(ID_SEND_WAYPOINTS_BUTTON, MetricPlannerPanel::sendWaypointsPressed)
EVT_BUTTON(ID_SKIP_WAYPOINT_BUTTON, MetricPlannerPanel::skipWaypointPressed)
EVT_BUTTON(ID_CANCEL_FOLLOWING_BUTTON, MetricPlannerPanel::cancelFollowingPressed)
EVT_BUTTON(ID_LOOP_WAYPOINTS_BUTTON, MetricPlannerPanel::loopThroughWaypointsPressed)
EVT_RADIOBOX(ID_SELECT_MAP_TYPE_RADIOBOX, MetricPlannerPanel::mapTypeSelected)
EVT_RADIOBOX(ID_SELECT_TRJ_GROUP_RADIOBOX, MetricPlannerPanel::trjGroupSelected)
EVT_CHOICE(ID_SELECT_TRJ_COST_CHOICE, MetricPlannerPanel::trjCostSelected)
EVT_RADIOBOX(ID_SELECT_TRJ_DISPLAY_MODE_RADIOBOX, MetricPlannerPanel::trjDisplayModeSelected)
EVT_TEXT(ID_CHANGE_TRJ_NUM_TEXT, MetricPlannerPanel::trjNumEntered)
EVT_BUTTON(ID_INCREASE_TRJ_NUM_BUTTON, MetricPlannerPanel::increaseTrjNumPressed)
EVT_BUTTON(ID_DECREASE_TRJ_NUM_BUTTON, MetricPlannerPanel::decreaseTrjNumPressed)
EVT_CHECKBOX(ID_SHOW_MOTION_TARGETS_CHECKBOX, MetricPlannerPanel::showMotionTargetsChecked)
EVT_CHECKBOX(ID_OVERLAY_ROBOT_POSES_CHECKBOX, MetricPlannerPanel::overlayRobotPosesChecked)
EVT_BUTTON(ID_SELECT_DESTINATION_POSE_BUTTON, MetricPlannerPanel::selectDestinationPosePressed)
EVT_BUTTON(ID_SEND_DESTINATION_POSE_BUTTON, MetricPlannerPanel::sendDestinationPosePressed)
EVT_BUTTON(ID_CANCEL_DESTINATION_POSE_BUTTON, MetricPlannerPanel::cancelDestinationPosePressed)
EVT_TEXT(ID_MOTION_TARGET_R_TEXT, MetricPlannerPanel::motionTargetREntered)
EVT_TEXT(ID_MOTION_TARGET_THETA_TEXT, MetricPlannerPanel::motionTargetThetaEntered)
EVT_TEXT(ID_MOTION_TARGET_DELTA_TEXT, MetricPlannerPanel::motionTargetDeltaEntered)
EVT_TEXT(ID_MOTION_TARGET_GAIN_TEXT, MetricPlannerPanel::motionTargetGainEntered)
EVT_TEXT(ID_MOTION_TARGET_K1_TEXT, MetricPlannerPanel::motionTargetK1Entered)
EVT_TEXT(ID_MOTION_TARGET_K2_TEXT, MetricPlannerPanel::motionTargetK2Entered)
EVT_BUTTON(ID_SELECT_MOTION_TARGET_BUTTON, MetricPlannerPanel::selectMotionTargetPressed)
EVT_BUTTON(ID_EVALUATE_MOTION_TARGET_BUTTON, MetricPlannerPanel::evaluateMotionTargetPressed)
EVT_BUTTON(ID_CLEAR_MOTION_TARGET_BUTTON, MetricPlannerPanel::clearMotionTargetPressed)
END_EVENT_TABLE()

}   // namespace ui
}   // namespace vulcan

namespace vulcan
{
namespace ui
{

MetricPlannerPanel::MetricPlannerPanel(const ui_params_t& params, const metric_planner_panel_widgets_t& widgets)
: widgets_(widgets)
, numTrjEvaluated_(0)
, numTrjToShow_(0)
, isSelectingDestinationPose_(false)
, isSelectingMotionTarget_(false)
, destinationPoseSelector_(new PoseSelector)
, motionTargetPoseSelector_(new PoseSelector)
, params_(params.metricPlannerDisplayParams)
{
    widgets.displayWidget->setWidgetParams(params.metricPlannerDisplayParams, params.lpmParams);

    widgets.trajectoryCostChoice->Clear();
    widgets.trajectoryCostChoice->Append(widgets.displayWidget->setupCostDescriptions());
    widgets.trajectoryCostChoice->SetSelection(0);
}


MetricPlannerPanel::~MetricPlannerPanel(void)
{
    // For std::unique_ptr
}


void MetricPlannerPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.displayWidget->setRenderContext(context);
    widgets_.displayWidget->setStatusBar(statusBar);
}


void MetricPlannerPanel::subscribe(system::ModuleCommunicator& producer)
{
    // robot state and command information
    producer.subscribeTo<motion_state_t>(widgets_.displayWidget);
    producer.subscribeTo<robot::commanded_joystick_t>(widgets_.displayWidget);
    producer.subscribeTo<robot::commanded_velocity_t>(widgets_.displayWidget);
    producer.subscribeTo<robot::motion_command_t>(widgets_.displayWidget);

    // current state of the robot, static environment and dynamic objects
    producer.subscribeTo<pose_t>(widgets_.displayWidget);
    producer.subscribeTo<hssh::LocalPerceptualMap>(widgets_.displayWidget);
    producer.subscribeTo<hssh::LocalTopoMap>(widgets_.displayWidget);
    producer.subscribeTo<tracker::DynamicObjectCollection>(widgets_.displayWidget);

    // static and dynamic hazards
    producer.subscribeTo<mpepc::ObstacleDistanceGrid>(widgets_.displayWidget);
    producer.subscribeTo<std::vector<mpepc::dynamic_object_trajectory_debug_info_t>>(widgets_.displayWidget);

    // information on path and trajectory planning
    producer.subscribeTo<mpepc::NavigationGrid>(widgets_.displayWidget);
    producer.subscribeTo<mpepc::CostMap>(widgets_.displayWidget);
    producer.subscribeTo<mpepc::VisibilityAnalysis>(widgets_.displayWidget);
    producer.subscribeTo<std::vector<mpepc::dynamic_object_trajectory_t>>(widgets_.displayWidget);
    //    producer.subscribeTo<mpepc::rrt_info_t>(widgets_.displayWidget);
    producer.subscribeTo<mpepc::trajectory_planner_debug_info_t>(widgets_.displayWidget);
    producer.subscribeTo<mpepc::metric_planner_status_message_t>(widgets_.displayWidget);
    producer.subscribeTo<mpepc::learned_norm_info_t>(widgets_.displayWidget);
    producer.subscribeTo<std::shared_ptr<mpepc::MetricPlannerTask>>(widgets_.displayWidget);
}


void MetricPlannerPanel::setConsumer(system::ModuleCommunicator* consumer)
{
    if (consumer) {
        this->consumer_ = consumer;
    }
}


void MetricPlannerPanel::update(void)
{
    // Updating marker locations
    if (isSelectingDestinationPose_) {
        widgets_.displayWidget->setHoverDestinationPose(destinationPoseSelector_->getHoverTarget());

        if (destinationPoseSelector_->hasSelectedTarget()) {
            widgets_.displayWidget->setDestinationPose(destinationPoseSelector_->getSelectedTarget());
        }
    }

    if (isSelectingMotionTarget_) {
        widgets_.displayWidget->setHoverMotionTargetPose(motionTargetPoseSelector_->getHoverTarget());

        if (motionTargetPoseSelector_->hasSelectedTarget()) {
            widgets_.displayWidget->setMotionTargetPose(motionTargetPoseSelector_->getSelectedTarget());
        }

        updateMotionTargetTexts();
    }

    // Update text
    size_t numTrjEvaluatedToShow = widgets_.displayWidget->getNumTrjEvaluatedToShow();
    widgets_.numTrajectoriesLabel->SetLabel(
      wxString::Format(wxT("Trajectory Count (current): %u"), static_cast<uint32_t>(numTrjEvaluatedToShow)));

    std::string evaluatedCostLabelString = widgets_.displayWidget->getEvaluatedCostLabelString();
    widgets_.evaluatedCostLabel->SetLabel(wxString::Format(wxT("%s"), evaluatedCostLabelString));

    float evaluatedCostToShow = widgets_.displayWidget->getEvaluatedCostToShow();
    widgets_.evaluatedCostText->SetLabel(wxString::Format(wxT("%7.4f"), evaluatedCostToShow));

    float motionTargetCostToShow = widgets_.displayWidget->getMotionTargetCostToShow();
    widgets_.motionTargetCostText->SetLabel(
      wxString::Format(wxT("Evaluated Expected Cost: %7.4f"), motionTargetCostToShow));

    // Refresh widget
    widgets_.displayWidget->Refresh();
}


void MetricPlannerPanel::saveSettings(utils::ConfigFileWriter& config)
{
}


void MetricPlannerPanel::loadSettings(const utils::ConfigFile& config)
{
}


void MetricPlannerPanel::showRobotPoseChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showRobotPose(event.IsChecked());
}


void MetricPlannerPanel::showDestinationPoseChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showDestinationPose(event.IsChecked());
}


void MetricPlannerPanel::showTrackedObjectsChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showTrackedObjects(event.IsChecked());
}


void MetricPlannerPanel::showEstimatedObjectMotionsChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showEstimatedObjectMotions(event.IsChecked());
}


void MetricPlannerPanel::showOptimalPathChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showOptimalPath(event.IsChecked());
}


void MetricPlannerPanel::showVisibilityAnalysisChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showVisibilityAnalysis(event.IsChecked());
}


void MetricPlannerPanel::showTopoSituationChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showTopoSituation(event.IsChecked());
}


void MetricPlannerPanel::loadScriptPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.displayWidget,
                            wxT("Select planner script file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.spt"),
                            wxFD_OPEN);

    if (loadDialog.ShowModal() == wxID_OK) {
        widgets_.scriptNameText->SetValue(loadDialog.GetFilename());
        wxString path = loadDialog.GetPath();
        scriptPath_ = std::string(path.mb_str());

        mpepc::MetricPlannerScript script(scriptPath_);

        std::vector<pose_t> loadedTargets;

        for (auto& task : script) {
            auto targets = task.getTargets();
            loadedTargets.insert(loadedTargets.end(), targets.begin(), targets.end());
        }

        widgets_.displayWidget->setDestinationPoses(loadedTargets);
    }
}


void MetricPlannerPanel::showWaypointsChecked(wxCommandEvent& event)
{
    // TODO: Fill this in
    //     widgets_.displayWidget->showRRTPath(event.IsChecked());
}


void MetricPlannerPanel::useRRTStarChecked(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SET_MODE_USE_RRT_STAR;
    plannerMessage.timestamp = utils::system_time_us();
    plannerMessage.intValue = static_cast<int>(event.IsChecked());
    consumer_->sendMessage(plannerMessage);
}


void MetricPlannerPanel::stopAtWaypointsChecked(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SET_MODE_STOP_AT_WAYPOINTS;
    plannerMessage.timestamp = utils::system_time_us();
    plannerMessage.intValue = static_cast<int>(event.IsChecked());
    consumer_->sendMessage(plannerMessage);
}


void MetricPlannerPanel::showGraphChecked(wxCommandEvent& event)
{
    // TODO: Fill this in
    //     widgets_.displayWidget->showRRT(event.IsChecked());
}


void MetricPlannerPanel::simpleFollowingChecked(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SET_MODE_SIMPLE_POSE_FOLLOWING;
    plannerMessage.timestamp = utils::system_time_us();
    plannerMessage.intValue = static_cast<int>(event.IsChecked());
    consumer_->sendMessage(plannerMessage);
}


void MetricPlannerPanel::ignoreObjectSpeedChecked(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SET_MODE_QUASI_STATIC_OBJECTS;
    plannerMessage.timestamp = utils::system_time_us();
    plannerMessage.intValue = static_cast<int>(event.IsChecked());
    consumer_->sendMessage(plannerMessage);
}


void MetricPlannerPanel::updateWaypointsPressed(wxCommandEvent& event)
{
    wxString timeString = widgets_.updatePlanTimeText->GetLineText(0);

    long time = 0;
    if (timeString.ToLong(&time)) {
        mpepc::metric_planner_command_message_t plannerMessage;
        plannerMessage.command = mpepc::UPDATE_WAYPOINTS;
        plannerMessage.timestamp = utils::system_time_us();
        plannerMessage.intValue = time;
        consumer_->sendMessage(plannerMessage);
    }
}


void MetricPlannerPanel::sendWaypointsPressed(wxCommandEvent& event)
{
    if (!scriptPath_.empty()) {
        consumer_->sendMessage(mpepc::MetricPlannerScript(scriptPath_));
    }
}


void MetricPlannerPanel::skipWaypointPressed(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SCRIPT_NEXT_TASK;
    plannerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(plannerMessage);
}


void MetricPlannerPanel::cancelFollowingPressed(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SCRIPT_STOP;
    plannerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(plannerMessage);
}


void MetricPlannerPanel::loopThroughWaypointsPressed(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SCRIPT_LOOP;
    plannerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(plannerMessage);
}


void MetricPlannerPanel::mapTypeSelected(wxCommandEvent& event)
{
    switch (event.GetSelection()) {
    case 0:
        widgets_.displayWidget->showMap(MPEPCMapType::lpm);
        break;

    case 1:
        widgets_.displayWidget->showMap(MPEPCMapType::obstacles);
        break;

    case 2:
        widgets_.displayWidget->showMap(MPEPCMapType::costs);
        break;

    case 3:
        widgets_.displayWidget->showMap(MPEPCMapType::navigation);
        break;

    case 4:
    default:
        widgets_.displayWidget->showMap(MPEPCMapType::none);
    }
}


void MetricPlannerPanel::trjGroupSelected(wxCommandEvent& event)
{
    switch (event.GetSelection()) {
    case 0:
        widgets_.displayWidget->showCurrentTrajectories();
        break;

    case 1:
        widgets_.displayWidget->showOptimalTrajectory();
        break;

    case 2:
        widgets_.displayWidget->showExplorationHistory();
        break;

    case 3:
        widgets_.displayWidget->showNoTrajectories();
        break;

    case 4:
        widgets_.displayWidget->showThreeSecHistory();
        break;

    case 5:
        widgets_.displayWidget->showFiveSecHistory();
        break;

    default:
        widgets_.displayWidget->showNoTrajectories();
        break;
    }
}


void MetricPlannerPanel::trjCostSelected(wxCommandEvent& event)
{
    widgets_.displayWidget->setTrajectoryCostToShow(event.GetSelection());
}


void MetricPlannerPanel::trjDisplayModeSelected(wxCommandEvent& event)
{
    switch (event.GetSelection()) {
    case 0:
        widgets_.displayWidget->showAllTrajectories();
        break;

    case 1:
        widgets_.displayWidget->showLastNTrajectories();
        break;

    case 2:
    default:
        widgets_.displayWidget->showSingleTrajectory();
    }
}


void MetricPlannerPanel::trjNumEntered(wxCommandEvent& event)
{
    wxString numString = widgets_.numTrajectoriesText->GetLineText(0);

    long numToShow = 0;
    if (numString.ToLong(&numToShow)) {
        numTrjToShow_ = (numToShow > 0) ? static_cast<size_t>(numToShow) : 0;
        widgets_.displayWidget->setTrajectoryNumber(numTrjToShow_);
    }
}


void MetricPlannerPanel::increaseTrjNumPressed(wxCommandEvent& event)
{
    ++numTrjToShow_;
    widgets_.numTrajectoriesText->ChangeValue(wxString::Format(wxT("%u"), static_cast<uint32_t>(numTrjToShow_)));
    widgets_.displayWidget->setTrajectoryNumber(numTrjToShow_);
}


void MetricPlannerPanel::decreaseTrjNumPressed(wxCommandEvent& event)
{
    if (numTrjToShow_ > 0) {
        --numTrjToShow_;
        widgets_.numTrajectoriesText->ChangeValue(wxString::Format(wxT("%u"), static_cast<uint32_t>(numTrjToShow_)));
        widgets_.displayWidget->setTrajectoryNumber(numTrjToShow_);
    }
}


void MetricPlannerPanel::showMotionTargetsChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->showMotionTargets(event.GetSelection());
}


void MetricPlannerPanel::overlayRobotPosesChecked(wxCommandEvent& event)
{
    widgets_.displayWidget->overlayRobotPosesOverTrajectory(event.GetSelection());
}


void MetricPlannerPanel::selectDestinationPosePressed(wxCommandEvent& event)
{
    isSelectingDestinationPose_ = true;
    isSelectingMotionTarget_ = false;

    widgets_.displayWidget->clearHoverMotionTargetPose();
    widgets_.displayWidget->clearDestinationPose();

    destinationPoseSelector_->reset();
    widgets_.displayWidget->pushMouseHandler(destinationPoseSelector_.get());
}


void MetricPlannerPanel::sendDestinationPosePressed(wxCommandEvent& event)
{
    if (isSelectingDestinationPose_) {
        isSelectingDestinationPose_ = false;
        widgets_.displayWidget->clearHoverDestinationPose();
        widgets_.displayWidget->removeMouseHandler(destinationPoseSelector_.get());
    }

    if (destinationPoseSelector_->hasSelectedTarget()) {
        // When starting a new path, reset the trace of pose followed by the robot to get there
        widgets_.displayWidget->clearTrace();

        // Send the destination pose to the planner for execution
        std::shared_ptr<mpepc::MetricPlannerTask> task(
          new mpepc::NavigationTask(destinationPoseSelector_->getSelectedTarget()));
        consumer_->sendMessage<std::shared_ptr<mpepc::MetricPlannerTask>>(task);
    }
}


void MetricPlannerPanel::cancelDestinationPosePressed(wxCommandEvent& event)
{
    if (isSelectingDestinationPose_) {
        isSelectingDestinationPose_ = false;
        widgets_.displayWidget->removeMouseHandler(destinationPoseSelector_.get());
    }

    widgets_.displayWidget->clearDestinationPose();
    widgets_.displayWidget->clearTrace();

    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::CANCEL;
    plannerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(plannerMessage);

    mpepc::motion_controller_command_message_t controllerMessage;
    controllerMessage.command = mpepc::MOTION_CONTROLLER_CANCEL;
    controllerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(controllerMessage);
}


void MetricPlannerPanel::selectMotionTargetPressed(wxCommandEvent& event)
{
    isSelectingDestinationPose_ = false;
    isSelectingMotionTarget_ = true;

    widgets_.displayWidget->clearHoverDestinationPose();
    widgets_.displayWidget->clearMotionTarget();

    motionTargetPoseSelector_->reset();
    widgets_.displayWidget->pushMouseHandler(motionTargetPoseSelector_.get());
}


void MetricPlannerPanel::evaluateMotionTargetPressed(wxCommandEvent& event)
{
    // update texts
    updateMotionTargetTexts();

    // send data to display widget, if the target is selected.
    if (isSelectingMotionTarget_) {
        isSelectingMotionTarget_ = false;
        widgets_.displayWidget->clearHoverMotionTargetPose();
        widgets_.displayWidget->removeMouseHandler(motionTargetPoseSelector_.get());
    }

    if (motionTargetPoseSelector_->hasSelectedTarget()) {
        widgets_.displayWidget->evaluateMotionTarget();
    }
}


void MetricPlannerPanel::clearMotionTargetPressed(wxCommandEvent& event)
{
    if (isSelectingMotionTarget_) {
        isSelectingMotionTarget_ = false;
        widgets_.displayWidget->removeMouseHandler(motionTargetPoseSelector_.get());
    }

    widgets_.displayWidget->clearMotionTarget();

    // update text
    updateMotionTargetTexts();
}


void MetricPlannerPanel::motionTargetREntered(wxCommandEvent& event)
{
    wxString coordString = widgets_.motionTargetRText->GetLineText(0);

    double coord;
    if (coordString.ToDouble(&coord)) {
        widgets_.displayWidget->setMotionTargetR(coord);
    }
}


void MetricPlannerPanel::motionTargetThetaEntered(wxCommandEvent& event)
{
    wxString coordString = widgets_.motionTargetThetaText->GetLineText(0);

    double coord;
    if (coordString.ToDouble(&coord)) {
        widgets_.displayWidget->setMotionTargetTheta(coord);
    }
}


void MetricPlannerPanel::motionTargetDeltaEntered(wxCommandEvent& event)
{
    wxString coordString = widgets_.motionTargetDeltaText->GetLineText(0);

    double coord;
    if (coordString.ToDouble(&coord)) {
        widgets_.displayWidget->setMotionTargetDelta(coord);
    }
}


void MetricPlannerPanel::motionTargetGainEntered(wxCommandEvent& event)
{
    wxString coordString = widgets_.motionTargetGainText->GetLineText(0);

    double coord;
    if (coordString.ToDouble(&coord)) {
        widgets_.displayWidget->setMotionTargetGain(coord);
    }
}


void MetricPlannerPanel::motionTargetK1Entered(wxCommandEvent& event)
{
    wxString coordString = widgets_.motionTargetK1Text->GetLineText(0);

    double coord;
    if (coordString.ToDouble(&coord)) {
        widgets_.displayWidget->setMotionTargetK1(coord);
    }
}


void MetricPlannerPanel::motionTargetK2Entered(wxCommandEvent& event)
{
    wxString coordString = widgets_.motionTargetK2Text->GetLineText(0);

    double coord;
    if (coordString.ToDouble(&coord)) {
        widgets_.displayWidget->setMotionTargetK2(coord);
    }
}


void MetricPlannerPanel::updateMotionTargetTexts(void)
{
    mpepc::control_law_coordinates_t controlLawCoords = widgets_.displayWidget->getControlLawCoords();
    widgets_.motionTargetRText->ChangeValue(wxString::Format(wxT("%5.2f"), controlLawCoords.r));
    widgets_.motionTargetThetaText->ChangeValue(wxString::Format(wxT("%5.2f"), controlLawCoords.theta));
    widgets_.motionTargetDeltaText->ChangeValue(wxString::Format(wxT("%5.2f"), controlLawCoords.delta));

    mpepc::motion_target_t motionTargetCoords = widgets_.displayWidget->getMotionTargetCoords();
    widgets_.motionTargetGainText->ChangeValue(wxString::Format(wxT("%5.2f"), motionTargetCoords.velocityGain));
    widgets_.motionTargetK1Text->ChangeValue(wxString::Format(wxT("%5.2f"), motionTargetCoords.k1));
    widgets_.motionTargetK2Text->ChangeValue(wxString::Format(wxT("%5.2f"), motionTargetCoords.k2));
}

}   // namespace ui
}   // namespace vulcan
