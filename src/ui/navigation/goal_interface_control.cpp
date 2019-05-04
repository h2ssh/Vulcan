/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     goal_interface_control.cpp
* \author   Collin Johnson
*
* Definition of GoalInterfaceControl.
*/

#include <ui/navigation/goal_interface_control.h>
#include <ui/navigation/goal_name_dialog.h>
#include <ui/navigation/navigation_data.h>
#include <ui/navigation/navigation_interface_display.h>
#include <ui/navigation/navigation_interface.h>
#include <planner/interface/navigation_interface.h>
#include <utils/stub.h>

namespace vulcan
{
namespace ui
{

const std::string kGoalsFile("navigation_goals.txt");
const int32_t kNoArea = -1;

    
void populate_object_selector(const hssh::LocalTopoMap& map,
                              const hssh::LocalPerceptualMap& lpm,
                              GridObjectSelector<int32_t>& selector);
    

BEGIN_EVENT_TABLE(GoalInterfaceControl, wxEvtHandler)
    EVT_TOGGLEBUTTON(ID_SELECT_GOAL_BUTTON, GoalInterfaceControl::selectGoalToggled)
    EVT_BUTTON(ID_ADD_SELECTED_BUTTON, GoalInterfaceControl::addSelectedPressed)
    EVT_BUTTON(ID_ADD_CURRENT_LOCATION_BUTTON, GoalInterfaceControl::addCurrentLocationPressed)
    EVT_BUTTON(ID_PREVIEW_ROUTE_BUTTON, GoalInterfaceControl::previewRoutePressed)
    EVT_BUTTON(ID_GO_BUTTON, GoalInterfaceControl::goPressed)
END_EVENT_TABLE()


GoalInterfaceControl::GoalInterfaceControl(planner::NavigationInterface& interface,
                                           NavigationInterfaceDisplay& display,
                                           const GoalInterfaceWidgets& widgets)
: interface_(interface)
, display_(display)
, widgets_(widgets)
, areaSelector_(&display)
, currentMapId_(kNoArea)
, haveNewGoal_(false)
, shouldSendGoalCommand_(false)
{
    assert(widgets_.goalsList);
    assert(widgets_.selectGoalButton);
    assert(widgets_.addSelectedButton);
    assert(widgets_.addCurrentButton);
    assert(widgets_.previewButton);
    assert(widgets_.goButton);

    widgets_.previewButton->Show(false);
    
    interface_.loadGoals(kGoalsFile);
    populateGoalsList();
}


void GoalInterfaceControl::update(const NavigationData& data, system::SystemCommunicator& communicator)
{
    setSelectGoalState(data);
    setAddSelectedState(data);
    setAddCurrentState(data);
    setPreviewState(data);
    setGoResumeState(data);
    
    if(haveNewGoal_)
    {
        populateGoalsList();
        haveNewGoal_ = false;
    }
    
    // When the map changes, a new mapping between areas and grid cells needs to be created and the new association
    // of area id to center needs to be created
    if(data.topoMap && data.metricMap && (data.topoMap->mapId() != currentMapId_))
    {
        populate_object_selector(*data.topoMap, *data.metricMap, areaSelector_);
        currentMapId_ = data.topoMap->mapId();

        areaCenters_.clear();
        for(auto& area : *data.topoMap)
        {
            areaCenters_[area->id()] = area->center();
        }
    }

    currentAreaId_ = data.location ? data.location->areaId() : kNoArea;
    currentPose_ = data.pose;
    
    display_.setHoverArea(areaSelector_.hoverObject().get_value_or(kNoArea));
    display_.setSelectedArea(areaSelector_.selectedObject().get_value_or(kNoArea));
    
    if(auto goal = interface_.currentGoal())
    {
        display_.setGoalArea(goal->localArea().get_value_or(kNoArea));
    }
    else
    {
        display_.setGoalArea(kNoArea);
    }
    

    if(shouldSendGoalCommand_)
    {
        bool success = interface_.goToGoal(selectedGoal(), data.topoMap, communicator);

        if(!success)
        {
            std::cerr << "ERROR: GoalInterfaceControl: Unable to go to goal: " << selectedGoal() << '\n';
        }

        shouldSendGoalCommand_ = false;
    }
}


void GoalInterfaceControl::setSelectGoalState(const NavigationData& data)
{
    bool haveMap = data.metricMap || data.topoMap;
    widgets_.selectGoalButton->Enable(haveMap);
}


void GoalInterfaceControl::setAddSelectedState(const NavigationData& data)
{
    bool haveSelection = static_cast<bool>(areaSelector_.selectedObject());
    widgets_.addSelectedButton->Enable(haveSelection);
}


void GoalInterfaceControl::setAddCurrentState(const NavigationData& data)
{
    bool haveMetricLocation = data.metricMap;  // always have a pose
    bool haveTopoLocation = data.topoMap && data.location && (data.topoMap->mapId() == data.location->mapId());
    widgets_.addCurrentButton->Enable(haveMetricLocation || haveTopoLocation);
}


void GoalInterfaceControl::setPreviewState(const NavigationData& data)
{
    bool hasSelected = !selectedGoal().empty();
    widgets_.previewButton->Enable(hasSelected);
}


void GoalInterfaceControl::setGoResumeState(const NavigationData& data)
{
    auto listGoal = selectedGoal();
    auto executingGoal = interface_.currentGoal();

    widgets_.goButton->SetLabel(wxString("GO"));

    if(interface_.hasDeferredGoal())
    {
        assert(executingGoal);
        if(executingGoal->name() == listGoal)
        {
            widgets_.goButton->SetLabel(wxString("RESUME"));
        }
    }

    widgets_.goButton->Enable(!listGoal.empty());
}


void GoalInterfaceControl::populateGoalsList(void)
{
    widgets_.goalsList->Clear();

    for(auto& goal : boost::make_iterator_range(interface_.beginGoals(), interface_.endGoals()))
    {
        widgets_.goalsList->Append(wxString(goal.name()));
    }
}


std::string GoalInterfaceControl::selectedGoal(void)
{
    auto selection = widgets_.goalsList->GetSelection();

    if(selection == wxNOT_FOUND)
    {
        return std::string("");
    }
    else
    {
        return widgets_.goalsList->GetString(selection).ToStdString();
    }
}


void GoalInterfaceControl::selectGoalToggled(wxCommandEvent& event)
{
    if(widgets_.selectGoalButton->GetValue())
    {
        display_.pushMouseHandler(&areaSelector_);
        display_.showAreas(true);
        display_.setMode(NavigationInterfaceMode::select);
    }
    else
    {
        display_.removeMouseHandler(&areaSelector_);
        display_.showAreas(false);
        display_.setMode(NavigationInterfaceMode::drive);
    }
}


void GoalInterfaceControl::addSelectedPressed(wxCommandEvent& event)
{
    auto selectedArea = areaSelector_.selectedObject();

    if(selectedArea)
    {
        assert(*selectedArea != kNoArea);

        GoalNameDialog nameDialog(&display_);
        if(nameDialog.ShowModal() == wxID_OK)
        {
            interface_.addGoalLocalArea(nameDialog.goalName(), *selectedArea);

            if(areaCenters_.find(*selectedArea) != areaCenters_.end())
            {
                interface_.addGoalGlobalPose(nameDialog.goalName(), areaCenters_[*selectedArea]);
            }

            haveNewGoal_ = true;
            
            interface_.saveGoals(kGoalsFile);
        }
    }
}


void GoalInterfaceControl::addCurrentLocationPressed(wxCommandEvent& event)
{
    GoalNameDialog nameDialog(&display_);
    if(nameDialog.ShowModal() == wxID_OK)
    {
        interface_.addGoalGlobalPose(nameDialog.goalName(), currentPose_);

        if(currentAreaId_ != kNoArea)
        {
            interface_.addGoalLocalArea(nameDialog.goalName(), currentAreaId_);
        }

        haveNewGoal_ = true;
        
        interface_.saveGoals(kGoalsFile);
    }
}


void GoalInterfaceControl::previewRoutePressed(wxCommandEvent& event)
{
    PRINT_STUB("GoalInterfaceControl::previewRoutePressed");
}


void GoalInterfaceControl::goPressed(wxCommandEvent& event)
{
    shouldSendGoalCommand_ = true;
}


void populate_object_selector(const hssh::LocalTopoMap& map, 
                              const hssh::LocalPerceptualMap& lpm, 
                              GridObjectSelector<int32_t>& selector)
{
    std::map<Point<int>, int32_t> cellToArea;
    for(const auto& area : map)
    {
        const auto& extent = area->extent();
        for(auto& cell : extent)
        {
            cellToArea.insert(std::make_pair(utils::global_point_to_grid_cell_round(cell, lpm), area->id()));
        }
    }
    
    selector.setObjects(cellToArea);
    
}

} // namespace ui
} // namespace vulcan
