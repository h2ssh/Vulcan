/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     simulator_robot_control.cpp
 * \author   Collin Johnson and Zongtai Luo
 *
 * Definition of SimulatorRobotControl.
 */
#include <cstdio>
#include <cstdlib>

#include "mpepc/metric_planner/messages.h"
#include "mpepc/metric_planner/script/script.h"
#include "mpepc/metric_planner/task/navigation.h"
#include "mpepc/motion_controller/messages.h"
#include "planner/interface/navigation_interface.h"
#include "system/module_communicator.h"
#include "ui/simulator/simulator_display.h"
#include "ui/simulator/simulator_robot_control.h"
#include "ui/simulator/simulator_robot_display.h"
#include "ui/simulator/simulator_ui.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace ui
{

BEGIN_EVENT_TABLE(SimulatorRobotControl, wxEvtHandler)
EVT_BUTTON(ID_SIMULATOR_SELECT_DESTINATION_POSE_BUTTON, SimulatorRobotControl::selectDestinationPosePressed)
EVT_BUTTON(ID_SIMULATOR_SEND_DESITNATION_POSE_BUTTON, SimulatorRobotControl::sendDestinationPosePressed)
EVT_BUTTON(ID_SIMULATOR_CANCEL_DESTINATION_POSE_BUTTON, SimulatorRobotControl::cancelDestinationPosePressed)
// Waypoints control
EVT_BUTTON(ID_SIMULATOR_LOAD_SCRIPT_BUTTON, SimulatorRobotControl::loadScriptPressed)
EVT_BUTTON(ID_SIMULATOR_SEND_SCRIPT_BUTTON, SimulatorRobotControl::sendWaypointsPressed)
EVT_BUTTON(ID_SIMULATOR_SKIP_WAYPOINT_BUTTON, SimulatorRobotControl::skipWaypointPressed)
EVT_BUTTON(ID_SIMULATOR_CANCEL_FOLLOWING_BUTTON, SimulatorRobotControl::stopWaypointsPressed)
EVT_BUTTON(ID_SIMULATOR_LOOP_WAYPOINTS_BUTTON, SimulatorRobotControl::loopWaypointsPressed)
// Add Robot Control
EVT_BUTTON(ID_SIMULATOR_ADD_ROBOT_SELECT_POSE_BUTTON,
           SimulatorRobotControl::addRobotSelectPosePressed)   // remeber to initilize the robot receiver in simulator
                                                               // display as well as the robot object in simulator main
EVT_BUTTON(ID_SIMULATOR_ADD_ROBOT_SET_POSE_BUTTON, SimulatorRobotControl::addRobotSetPosePressed)
EVT_BUTTON(ID_SIMULATOR_LOAD_CONFIG_BUTTON, SimulatorRobotControl::loadRobotConfigPressed)
EVT_BUTTON(ID_SIMULATOR_ADD_ROBOT_BUTTON, SimulatorRobotControl::addRobotPressed)
// Robot Control
// EVT_BUTTON  (ID_SIMULATOR_PAUSE_ALL_ROBOT_BUTTON,              SimulatorRobotControl::pauseAllRobotPressed)
// EVT_BUTTON  (ID_SIMULATOR_SELECT_ROBOT_PAUSE_BUTTON,           SimulatorRobotControl::pauseSelectRobotPressed)
// Simulator Control
EVT_BUTTON(ID_START_NEC_MODULAR_BUTTON, SimulatorRobotControl::startNecModularPressed)
EVT_BUTTON(ID_SIMULATOR_CASE_ONE_START_BUTTON, SimulatorRobotControl::startSimulatorCaseOnePressed)
EVT_BUTTON(ID_SIMULATOR_CASE_TWO_START_BUTTON, SimulatorRobotControl::startSimulatorCaseTwoPressed)
EVT_BUTTON(ID_SIMULATOR_CASE_THREE_START_BUTTON, SimulatorRobotControl::startSimulatorCaseThreePressed)
END_EVENT_TABLE()

}   // namespace ui
}   // namespace vulcan


namespace vulcan
{
namespace ui
{

SimulatorRobotControl::SimulatorRobotControl(const simulator_ui_panel_widgets_t& widgets)
: haveMetricMap_(false)
, haveTopoMap_(false)
, haveLocation_(false)
, isSelectingDestinationPose_(false)
, isSelectingGlobalDestinationPose_(false)
, destinationPoseSelector_(new PoseSelector)
, widgets_(widgets)
, interface_("simulator_robot_interface")
, isPaused_(false)
, addRobotPoseCheck(false)
, addRobotConfigCheck(false)
, consumer_(nullptr)
{
    robot_receivers_.initRobotGroupReceiver();

    // utils::ConfigFile simulator_config("simulator_params.cfg");
    // robot_receiver_params_t receiver_params = load_simulator_ui_params(simulator_config);

    // widgets_.robot_display_->setDestinationPoses(loadScriptPoses(receiver_params.main_script_path));
}


SimulatorRobotControl::~SimulatorRobotControl(void)
{
    // For std::unique_ptr
    // widgets_.robot_display_->GetParent()->PopEventHandler();
}


void SimulatorRobotControl::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    widgets_.robot_display_->setRenderContext(context);
    widgets_.robot_display_->setStatusBar(statusBar);

    widgets_.robot_display_->disablePanning();
    widgets_.robot_display_->disableScrolling();
    widgets_.robot_display_->disableTilting();
    widgets_.robot_display_->pushKeyboardHandler(this);

    widgets_.ground_truth_display_->setRenderContext(context);
    widgets_.ground_truth_display_->setStatusBar(statusBar);

    widgets_.ground_truth_display_->disablePanning();
    widgets_.ground_truth_display_->disableScrolling();
    widgets_.ground_truth_display_->disableTilting();

    widgets_.ground_truth_display_->setRobotReveiver(&robot_receivers_);
}


void SimulatorRobotControl::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::LocalTopoMap>(this);
    producer.subscribeTo<hssh::LocalLocation>(this);
    producer.subscribeTo<motion_state_t>(this);
    producer.subscribeTo<hssh::LocalPerceptualMap>(this);
    producer.subscribeTo<mpepc::trajectory_planner_debug_info_t>(this);
    producer.subscribeTo<tracker::DynamicObjectCollection>(this);

    robot_receivers_.subscribe();
}


void SimulatorRobotControl::setConsumer(system::ModuleCommunicator* consumer)
{
    consumer_ = consumer;
}


void SimulatorRobotControl::update(void)
{
    assert(consumer_);

    loadNewData();

    if (isSelectingDestinationPose_) {
        widgets_.robot_display_->setHoverDestinationPose(destinationPoseSelector_->getHoverTarget());

        if (destinationPoseSelector_->hasSelectedTarget()) {
            widgets_.robot_display_->setDestinationPose(destinationPoseSelector_->getSelectedTarget());
        }
    }

    if (isSelectingGlobalDestinationPose_) {
        widgets_.ground_truth_display_->setHoverDestinationPose(destinationPoseSelector_->getHoverTarget());

        if (destinationPoseSelector_->hasSelectedTarget()) {
            widgets_.ground_truth_display_->setDestinationPose(destinationPoseSelector_->getSelectedTarget());
        }
    }

    widgets_.robot_display_->Refresh();
    widgets_.robot_display_->setMode(SimulatorRobotMode::select);
    widgets_.ground_truth_display_->Refresh();
    widgets_.ground_truth_display_->setMode(NavigationInterfaceMode::select);
}


void SimulatorRobotControl::saveSettings(utils::ConfigFileWriter& config)
{
}


void SimulatorRobotControl::loadSettings(const utils::ConfigFile& config)
{
}


void SimulatorRobotControl::handleData(const hssh::LocalTopoMap& map, const std::string& channel)
{
    topoMap_ = map;
    haveTopoMap_ = true;
}


void SimulatorRobotControl::handleData(const hssh::LocalLocation& location, const std::string& channel)
{
    location_ = location;
    haveLocation_ = true;
}


void SimulatorRobotControl::handleData(const motion_state_t& motion, const std::string& channel)
{
    pose_ = motion.pose;
}


void SimulatorRobotControl::handleData(const hssh::LocalPerceptualMap& map, const std::string& channel)
{
    map_ = map;
}


void SimulatorRobotControl::handleData(const mpepc::metric_planner_status_message_t& status, const std::string& channel)
{
    status_ = status;
}


void SimulatorRobotControl::handleData(const mpepc::trajectory_planner_debug_info_t& trajectories,
                                       const std::string& channel)
{
    trajectories_ = trajectories;
}


void SimulatorRobotControl::handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel)
{
    objects_ = objects;
}


void SimulatorRobotControl::loadNewData(void)
{
    if (topoMap_.hasData()) {
        topoMap_.swapBuffers();
        widgets_.robot_display_->setAreas(topoMap_);
        haveTopoMap_ = true;
    }

    if (location_.hasData()) {
        location_.swapBuffers();
        haveLocation_ = true;
    }

    if (map_.hasData()) {
        map_.swapBuffers();
        widgets_.robot_display_->setLPM(map_);
        widgets_.ground_truth_display_->setLPM(map_);
        haveMetricMap_ = true;
    }

    if (objects_.hasData()) {
        objects_.swapBuffers();
        widgets_.robot_display_->setObjects(objects_);
    }

    if (trajectories_.hasData()) {
        trajectories_.swapBuffers();
        widgets_.robot_display_->setTrajectories(trajectories_);
        widgets_.ground_truth_display_->setTrajectories(trajectories_);
    }

    if (pose_.hasData()) {
        pose_.swapBuffers();
        widgets_.robot_display_->setPose(pose_);
        widgets_.ground_truth_display_->setPose(pose_);
    }

    // update other robot
    robot_receivers_.update();
}

std::vector<pose_t> SimulatorRobotControl::loadScriptPoses(std::string script_path)
{
    mpepc::MetricPlannerScript script(script_path);

    std::vector<pose_t> loadedTargets;

    for (auto& task : script) {
        auto targets = task.getTargets();
        loadedTargets.insert(loadedTargets.end(), targets.begin(), targets.end());
    }

    return loadedTargets;
}


GLEventStatus SimulatorRobotControl::keyPressed(wxKeyEvent& key)
{
    GLEventStatus status = GLEventStatus::capture;

    if (key.GetKeyCode() == WXK_SPACE) {
        if (isPaused_) {
            mpepc::metric_planner_command_message_t resumeMessage;
            resumeMessage.timestamp = utils::system_time_us();
            resumeMessage.command = mpepc::RESUME;
            consumer_->sendMessage(resumeMessage);

            isPaused_ = false;
        } else {
            mpepc::metric_planner_command_message_t pauseMessage;
            pauseMessage.timestamp = utils::system_time_us();
            pauseMessage.command = mpepc::PAUSE;
            consumer_->sendMessage(pauseMessage);

            isPaused_ = true;
        }
    } else if (key.GetKeyCode() == 'O') {
        widgets_.robot_display_->toggleObjects();
    } else if (key.GetKeyCode() == 'T') {
        widgets_.robot_display_->toggleTrajectories();
    } else {
        status = GLEventStatus::passthrough;
    }

    return status;
}


void SimulatorRobotControl::selectDestinationPosePressed(wxCommandEvent& event)
{
    isSelectingDestinationPose_ = true;

    widgets_.robot_display_->clearDestinationPose();

    widgets_.robot_display_->setHaveHoverDestinationPose(true);
    destinationPoseSelector_->reset();
    widgets_.robot_display_->pushMouseHandler(destinationPoseSelector_.get());
}


void SimulatorRobotControl::sendDestinationPosePressed(wxCommandEvent& event)
{
    if (isSelectingDestinationPose_) {
        isSelectingDestinationPose_ = false;
        widgets_.robot_display_->clearHoverDestinationPose();
        widgets_.robot_display_->removeMouseHandler(destinationPoseSelector_.get());
    }

    if (destinationPoseSelector_->hasSelectedTarget()) {
        // Send the destination pose to the planner for execution
        std::shared_ptr<mpepc::MetricPlannerTask> task(
          new mpepc::NavigationTask(destinationPoseSelector_->getSelectedTarget()));
        consumer_->sendMessage<std::shared_ptr<mpepc::MetricPlannerTask>>(task);
    }
}


void SimulatorRobotControl::cancelDestinationPosePressed(wxCommandEvent& event)
{
    if (isSelectingDestinationPose_) {
        isSelectingDestinationPose_ = false;
        widgets_.robot_display_->removeMouseHandler(destinationPoseSelector_.get());
    }

    widgets_.robot_display_->clearDestinationPose();

    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::CANCEL;
    plannerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(plannerMessage);

    mpepc::motion_controller_command_message_t controllerMessage;
    controllerMessage.command = mpepc::MOTION_CONTROLLER_CANCEL;
    controllerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(controllerMessage);
}


void SimulatorRobotControl::loadScriptPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.robot_display_,
                            wxT("Select planner script file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.spt"),
                            wxFD_OPEN);

    if (loadDialog.ShowModal() == wxID_OK) {
        widgets_.scriptNameText->SetValue(loadDialog.GetFilename());
        wxString path = loadDialog.GetPath();
        scriptPath_ = std::string(path.mb_str());

        widgets_.robot_display_->setDestinationPoses(loadScriptPoses(scriptPath_));
    }
}


void SimulatorRobotControl::sendWaypointsPressed(wxCommandEvent& event)
{
    if (!scriptPath_.empty()) {
        consumer_->sendMessage(mpepc::MetricPlannerScript(scriptPath_));
    }
}


void SimulatorRobotControl::skipWaypointPressed(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SCRIPT_NEXT_TASK;
    plannerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(plannerMessage);
}


void SimulatorRobotControl::stopWaypointsPressed(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SCRIPT_STOP;
    plannerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(plannerMessage);
}


void SimulatorRobotControl::loopWaypointsPressed(wxCommandEvent& event)
{
    mpepc::metric_planner_command_message_t plannerMessage;
    plannerMessage.command = mpepc::SCRIPT_LOOP;
    plannerMessage.timestamp = utils::system_time_us();
    consumer_->sendMessage(plannerMessage);
}


void SimulatorRobotControl::addRobotSelectPosePressed(wxCommandEvent& event)
{
    isSelectingGlobalDestinationPose_ = true;

    widgets_.ground_truth_display_->clearDestinationPose();

    widgets_.ground_truth_display_->setHaveHoverDestinationPose(true);
    destinationPoseSelector_->reset();
    widgets_.ground_truth_display_->pushMouseHandler(destinationPoseSelector_.get());
}


void SimulatorRobotControl::addRobotSetPosePressed(wxCommandEvent& event)
{
    if (isSelectingGlobalDestinationPose_) {
        isSelectingGlobalDestinationPose_ = false;
        widgets_.ground_truth_display_->clearHoverDestinationPose();
        widgets_.ground_truth_display_->removeMouseHandler(destinationPoseSelector_.get());
    }

    if (destinationPoseSelector_->hasSelectedTarget()) {
        simulator_robot_group_message_.pose_ = destinationPoseSelector_->getSelectedTarget();

        std::cout << "the pose is " << simulator_robot_group_message_.pose_.x << " and "
                  << simulator_robot_group_message_.pose_.y << "\n";

        addRobotPoseCheck = true;
    }
}


void SimulatorRobotControl::loadRobotConfigPressed(wxCommandEvent& event)
{
    wxFileDialog loadDialog(widgets_.robot_display_,
                            wxT("Select robot config file..."),
                            wxT(""),
                            wxT(""),
                            wxT("*.cfg"),
                            wxFD_OPEN);

    if (loadDialog.ShowModal() == wxID_OK) {
        widgets_.robotConfigText->SetValue(loadDialog.GetFilename());
        wxString path = loadDialog.GetFilename();
        std::string config_name = std::string(path.mb_str());

        std::cout << config_name << "\n";

        simulator_robot_group_message_.robot_config_ = config_name;

        addRobotConfigCheck = true;
    }
}


void SimulatorRobotControl::addRobotPressed(wxCommandEvent& event)
{
    if (addRobotPoseCheck && addRobotConfigCheck) {
        widgets_.ground_truth_display_->clearDestinationPose();

        consumer_->sendMessage(simulator_robot_group_message_);

        utils::ConfigFile add_robot_config(simulator_robot_group_message_.robot_config_);
        std::string system_url = load_system_url(add_robot_config);

        robot_receivers_.pushBack(system_url);

        addRobotPoseCheck = false;
        addRobotConfigCheck = false;
    } else {
        std::cout << "Please select both config file and starting pose.\n";
    }
}


void SimulatorRobotControl::startNecModularPressed(wxCommandEvent& event)
{
    int s = std::system("(cd ../../ && ./start_pre_simulator)");
    printf("%d\n", s);
}


void SimulatorRobotControl::startSimulatorCaseOnePressed(wxCommandEvent& event)
{
    int s = std::system("(cd ../../ && ./start_simulator_one)");
    printf("%d\n", s);
}


void SimulatorRobotControl::startSimulatorCaseTwoPressed(wxCommandEvent& event)
{
    int s = std::system("(cd ../../ && ./start_simulator_two)");
    printf("%d\n", s);
}


void SimulatorRobotControl::startSimulatorCaseThreePressed(wxCommandEvent& event)
{
    int s = std::system("(cd ../../ && ./start_simulator_three)");
    printf("%d\n", s);
}


}   // namespace ui
}   // namespace vulcan
