/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_interface_control.h
* \author   Collin Johnson
* 
* Declaration of NavigationInterfaceControl.
*/

#ifndef UI_DECISION_NAVIGATION_INTERFACE_CONTROL_H
#define UI_DECISION_NAVIGATION_INTERFACE_CONTROL_H

#include <ui/navigation/navigation_data.h>
#include <ui/common/gl_event.h>
#include <ui/common/ui_panel.h>
#include <planner/interface/navigation_interface.h>
#include <hssh/local_topological/local_topo_map.h>
#include <hssh/local_topological/location.h>
#include <hssh/local_metric/lpm.h>
#include <hssh/local_metric/pose.h>
#include <mpepc/trajectory/trajectory_planner_info.h>
#include <mpepc/metric_planner/messages.h>
#include <tracker/dynamic_object_collection.h>
#include <utils/locked_double_buffer.h>

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

class DecisionInterfaceControl;
class GoalInterfaceControl;
class GoalInterfaceWidgets;
class NavigationInterfaceDisplay;


/**
* NavigationInterfaceControl controls the NavigationInterface and NavigationInterfaceDisplay, feeding the appropriate
* data into both classes. The control handles delegation of Decision and Goal interfcaes to the Goal and Decision
* controls. It decides what data is shown on the screen.
* 
* NavigationInterfaceControl uses keyboard control to determine the displayed data:
* 
*   - Space : pause the robot's motion
*   - O : display dynamic objects
*   - T : display planner trajectories
*/
class NavigationInterfaceControl : public UIPanel, 
                                   public GLKeyboardHandler,
                                   public GLMouseHandler
{
public:
    
    /**
    * Constructor for NavigationInterfaceControl,
    */
    NavigationInterfaceControl(NavigationInterfaceDisplay* display, const GoalInterfaceWidgets& goalWidgets);

    ~NavigationInterfaceControl(void);
    
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
    
    // GLKeyboardHandler interface
    GLEventStatus keyPressed(wxKeyEvent& key);
    
    // GLMouseHandler interface
    // TODO: Determine which mouse events should be processed

private:
    
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
    
    NavigationInterfaceDisplay* display_;
    planner::NavigationInterface interface_;

    std::unique_ptr<DecisionInterfaceControl> decisionControl_;
    std::unique_ptr<GoalInterfaceControl> goalControl_;
    
    bool isPaused_;
    
    system::ModuleCommunicator* consumer_;
    
    NavigationData loadNewData(void);
};

}
}

#endif // UI_DECISION_NAVIGATION_INTERFACE_CONTROL_H
