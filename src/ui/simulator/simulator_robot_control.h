/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_robot_control.h
* \author   Collin Johnson Zongtai Luo
* 
* Declaration of SimulatorRobotControl.
*/

#ifndef UI_SIMULATOR_SIMULATOR_ROBOT_CONTROL_H
#define UI_SIMULATOR_SIMULATOR_ROBOT_CONTROL_H

#include <wx/wx.h>
#include "ui/common/gl_event.h"
#include "ui/common/ui_panel.h"

#include "ui/common/metric_path_creator.h" // maybe needed for compiling
#include "ui/simulator/simulator_robot_group_receiver.h"
#include "planner/interface/navigation_interface.h"
#include "hssh/local_topological/local_topo_map.h"
#include "hssh/local_topological/location.h"
#include "hssh/local_metric/lpm.h"
#include "hssh/local_metric/pose.h"
#include "mpepc/trajectory/trajectory_planner_info.h"
#include "mpepc/metric_planner/messages.h"
#include "tracker/dynamic_object_collection.h"
#include "utils/locked_double_buffer.h"
#include "simulator/robot_group.h"
// #include "simulator/simulator_robot_group_message.h"

namespace vulcan
{
namespace hssh { class LocalPose; }
namespace hssh { class LocalPerceptualMap; }
namespace hssh { class LocalTopoMap; }
namespace hssh { class LocalLocation; }
namespace mpepc { struct trajectory_planner_debug_info_t; }
namespace tracker { class DynamicObjectCollection; }
namespace ui
{

class SimulatorRobotDisplay;
class SimulatorDisplay;

// The collection for the different sizer
struct simulator_ui_panel_widgets_t
{
    SimulatorRobotDisplay* robot_display_;
    SimulatorDisplay* ground_truth_display_;

    wxTextCtrl* scriptNameText;
    wxTextCtrl* robotConfigText;
};


/**
* SimulatorRobotControl controls the NavigationInterface and SimulatorRobotDisplay, feeding the appropriate
* data into both classes. The control handles delegation of Decision and Goal interfcaes to the Goal and Decision
* controls. It decides what data is shown on the screen.
* 
* SimulatorRobotControl uses keyboard control to determine the displayed data:
* 
*   - Space : pause the robot's motion
*   - O : display dynamic objects
*   - T : display planner trajectories
*/
class SimulatorRobotControl : public UIPanel, 
                              public GLKeyboardHandler,
                              public GLMouseHandler
{
public:
    
    /**
    * Constructor for SimulatorRobotControl,
    */
    // SimulatorRobotControl(SimulatorRobotDisplay* display, const GoalInterfaceWidgets& goalWidgets);
    SimulatorRobotControl(const simulator_ui_panel_widgets_t& widgets);

    ~SimulatorRobotControl(void);
    
    // UIPanel interface
    void setup(wxGLContext* context, wxStatusBar* statusBar) override;
    void subscribe(system::ModuleCommunicator& producer) override;
    void setConsumer(system::ModuleCommunicator* consumer) override;
    void update(void) override;
    void saveSettings(utils::ConfigFileWriter& config) override;
    void loadSettings(const utils::ConfigFile& config) override;
    
    // Data handlers
    void handleData(const hssh::LocalTopoMap& map, const std::string& channel);
    void handleData(const hssh::LocalLocation& location, const std::string& channel);
    void handleData(const motion_state_t& motion, const std::string& channel);
    void handleData(const hssh::LocalPerceptualMap& map, const std::string& channel);
    void handleData(const mpepc::metric_planner_status_message_t& status, const std::string& channel);
    void handleData(const mpepc::trajectory_planner_debug_info_t& trajectories, const std::string& channel);
    void handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel);
    void handleData(const sim::simulator_robot_group_message_t& simulator_robot_group_message, const std::string& channel);
    
    // GLKeyboardHandler interface
    GLEventStatus keyPressed(wxKeyEvent& key);
    
private:
    // control utils
    std::vector<pose_t> loadScriptPoses(std::string script_path);

    // section for Destination Selection
    void selectDestinationPosePressed(wxCommandEvent& event);
    void sendDestinationPosePressed(wxCommandEvent& event);
    void cancelDestinationPosePressed(wxCommandEvent& event);

    // script waypoints control
    void loadScriptPressed(wxCommandEvent& event);
    void sendWaypointsPressed(wxCommandEvent& event);
    void skipWaypointPressed(wxCommandEvent& event);
    void stopWaypointsPressed(wxCommandEvent& event);
    void loopWaypointsPressed(wxCommandEvent& event);

    // add robot control
    void addRobotSelectPosePressed(wxCommandEvent& event);
    void addRobotSetPosePressed(wxCommandEvent& event);
    void loadRobotConfigPressed(wxCommandEvent& event);
    void addRobotPressed(wxCommandEvent& event);

    // robot control
    // void pauseAllRobotPressed(wxCommandEvent& event);
    // void pauseSelectRobotPressed(wxCommandEvent& event);

    // section for start simulator
    void startNecModularPressed(wxCommandEvent& event);
    void startSimulatorCaseOnePressed(wxCommandEvent& event);
    void startSimulatorCaseTwoPressed(wxCommandEvent& event);
    void startSimulatorCaseThreePressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
    
    // Buffer for the data coming in
    template <class T>
    using Buffer = utils::LockedDoubleBuffer<T>;

    Buffer<pose_t> pose_;
    Buffer<hssh::LocalPerceptualMap> map_;
    Buffer<hssh::LocalTopoMap> topoMap_;
    Buffer<hssh::LocalLocation> location_;
    Buffer<tracker::DynamicObjectCollection> objects_;
    Buffer<mpepc::metric_planner_status_message_t> status_;
    Buffer<mpepc::trajectory_planner_debug_info_t> trajectories_;
    
    bool haveMetricMap_;
    bool haveTopoMap_;
    bool haveLocation_;

    // variables for selecting pose
    bool isSelectingDestinationPose_;
    bool isSelectingGlobalDestinationPose_;

    std::shared_ptr<PoseSelector> destinationPoseSelector_;

    // widgets for the ui
    simulator_ui_panel_widgets_t widgets_;
    planner::NavigationInterface interface_;
    
    // mpepc pause
    bool isPaused_;

    std::string scriptPath_;

    bool addRobotPoseCheck;
    bool addRobotConfigCheck;

    // message from simulator main
    sim::simulator_robot_group_message_t simulator_robot_group_message_;

    RobotGroupReceiver robot_receivers_;

    // Module Communicator
    system::ModuleCommunicator* consumer_;
    
    // update buffer
    void loadNewData(void);
};

}
}

#endif // UI_DECISION_NAVIGATION_INTERFACE_CONTROL_H
