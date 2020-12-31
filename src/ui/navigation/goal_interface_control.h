/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_interface_control.h
* \author   Collin Johnson
*
* Declaration of GoalInterfaceControl.
*/

#ifndef UI_NAVIGATION_GOAL_INTERFACE_CONTROL_H
#define UI_NAVIGATION_GOAL_INTERFACE_CONTROL_H

#include "ui/common/gl_event.h"
#include "ui/common/grid_object_selector.h"
#include "core/pose.h"
#include <wx/wx.h>
#include <wx/tglbtn.h>
#include <unordered_map>

namespace vulcan
{
namespace planner { class NavigationInterface; }
namespace system { class SystemCommunicator; }
namespace ui
{

class NavigationData;
class NavigationInterfaceDisplay;

/**
* GoalInterfaceWidgets
*/
struct GoalInterfaceWidgets
{
    wxListBox* goalsList = nullptr;
    wxToggleButton* selectGoalButton = nullptr;
    wxButton* addSelectedButton = nullptr;
    wxButton* addCurrentButton = nullptr;
    wxButton* previewButton = nullptr;
    wxButton* goButton = nullptr;
};


/**
* GoalInterfaceControl handles the interface between the user and the GoalInterface. The control maintains the state
* machine for the right panel of the NavigationInterface.
*
* The various buttons are enabled if the following conditions hold:
*
*   - Select Goal In Map : The robot has a map and a location/pose in the map
*   - Add Selected Goal : A goal has been selected in the map
*   - Add Current Location : The robot has a map and a location/pose in the map
*   - Preview : A Goal is selected in the Goal list
*   - Go : A Goal is selected in the Goal list that is not the currently deferred goal
*   - Resume : A Goal is selected in the Goal list that is the current deferred goal
*
*/
class GoalInterfaceControl : public GLMouseHandler,
                             public wxEvtHandler
{
public:

    /**
    * Constructor for DecisionInterfaceControl.
    *
    * \param    interface           Interface to use for Decision-level navigation
    * \param    display             Display to be updated with Decision-level state
    */
    GoalInterfaceControl(planner::NavigationInterface& interface,
                         NavigationInterfaceDisplay& display,
                         const GoalInterfaceWidgets& widgets);

    /**
    * update
    */
    void update(const NavigationData& data, system::SystemCommunicator& communicator);

private:

    planner::NavigationInterface& interface_;
    NavigationInterfaceDisplay& display_;
    GoalInterfaceWidgets widgets_;

    GridObjectSelector<int32_t> areaSelector_;
    int32_t currentMapId_;          // Id of topo map currently being used
    int32_t currentAreaId_;         // Id of the current area
    pose_t currentPose_;     // Current pose in the map

    // Store a mapping of area->center to allow for the creation of global pose goals associated with an area
    std::unordered_map<int32_t, pose_t> areaCenters_;

    bool haveNewGoal_;      // Flag indicating if a new goal was created, so the goalsList should be repopulated
    bool shouldSendGoalCommand_;    // Flag indicating if a new goal should be sent because the Go button was pressed


    void setSelectGoalState(const NavigationData& data);
    void setAddSelectedState(const NavigationData& data);
    void setAddCurrentState(const NavigationData& data);
    void setPreviewState(const NavigationData& data);
    void setGoResumeState(const NavigationData& data);
    void populateGoalsList(void);

    std::string selectedGoal(void);

    // Event handlers
    void selectGoalToggled(wxCommandEvent& event);
    void addSelectedPressed(wxCommandEvent& event);
    void addCurrentLocationPressed(wxCommandEvent& event);
    void previewRoutePressed(wxCommandEvent& event);
    void goPressed(wxCommandEvent& event);

    DECLARE_EVENT_TABLE()
};

}
}

#endif // UI_NAVIGATION_GOAL_INTERFACE_CONTROL_H
