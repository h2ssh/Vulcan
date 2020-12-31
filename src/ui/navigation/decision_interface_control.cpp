/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     decision_interface_control.cpp
* \author   Collin Johnson
*
* Definition of DecisionInterfaceControl.
*/

#include "ui/navigation/decision_interface_control.h"
#include "ui/navigation/navigation_data.h"
#include "ui/navigation/navigation_interface_display.h"
#include "planner/interface/navigation_interface.h"
#include "hssh/local_topological/local_topo_map.h"
#include "utils/algorithm_ext.h"
#include "utils/stub.h"

namespace vulcan
{
namespace ui
{

std::size_t find_action_with_direction(planner::DecisionDirection direction,
                                       const planner::NavigationInterface& interface);


DecisionInterfaceControl::DecisionInterfaceControl(planner::NavigationInterface& interface,
                                                   NavigationInterfaceDisplay& display)
: interface_(interface)
, display_(display)
, prevArea_(-1)
, prevMap_(-1)
, hoverIndex_(0)
, selectedIndex_(0)
, didSelectArea_(false)
{

}


void DecisionInterfaceControl::update(const NavigationData& data, system::SystemCommunicator& communicator)
{
    // If a selection was made, then send it off before updating any state that may have changed
    if(didSelectArea_)
    {
        interface_.goToDecision(*(interface_.beginDecisions() + selectedIndex_), pose_, communicator);
        didSelectArea_ = false;
    }

    // Nothing to do if there's no location or no topo map or if the location doesn't belong to the current,
    // which means a transition to some new map is underway
    if(!data.topoMap || !data.location || (data.topoMap->mapId() != data.location->mapId()))
    {
        return;
    }

    // If the robot has changed areas or the map has changed, then
    if((data.location->areaId() != prevArea_) || (data.location->mapId() != prevMap_))
    {
        int numNewActions = interface_.determineDecisions(*data.topoMap, *data.location, data.pose);

        // If there's a new action, then the current action state needs to be reset
        if(numNewActions > 0)
        {
            display_.setDecisions(std::vector<planner::Decision>(interface_.beginDecisions(),
                                                                 interface_.endDecisions()));

            hoverIndex_ = selectedIndex_ = interface_.sizeDecisions();
        }
        
        if(auto currentArea = data.topoMap->areaWithId(data.location->areaId()))
        {
            display_.setGateways(currentArea->gateways());
        }
    }

    display_.setHoverAction(hoverIndex_);
    display_.setSelectedAction(selectedIndex_);

    pose_ = data.pose;
    prevArea_ = data.location->areaId();
    prevMap_ = data.location->mapId();
}


GLEventStatus DecisionInterfaceControl::keyPressed(wxKeyEvent& key)
{
    GLEventStatus status = GLEventStatus::capture;

    if(!key.ControlDown() && (key.GetKeyCode() == WXK_UP))
    {
        hoverIndex_ = find_action_with_direction(planner::DecisionDirection::forward, interface_);
    }
    else if(!key.ControlDown() && (key.GetKeyCode() == WXK_DOWN))
    {
        hoverIndex_ = find_action_with_direction(planner::DecisionDirection::backward, interface_);
    }
    else if(!key.ControlDown() && (key.GetKeyCode() == WXK_LEFT))
    {
        hoverIndex_ = find_action_with_direction(planner::DecisionDirection::left, interface_);
    }
    else if(!key.ControlDown() && (key.GetKeyCode() == WXK_RIGHT))
    {
        hoverIndex_ = find_action_with_direction(planner::DecisionDirection::right, interface_);
    }
    else if(key.ControlDown() && (key.GetKeyCode() == WXK_LEFT))
    {
        hoverIndex_ = (hoverIndex_ > 0) ? hoverIndex_ - 1 : interface_.sizeDecisions() - 1;
    }
    else if(key.ControlDown() && (key.GetKeyCode() == WXK_RIGHT))
    {
        ++hoverIndex_;

        if(hoverIndex_ >= interface_.sizeDecisions())
        {
            hoverIndex_ = 0;
        }
    }
    else if(key.GetKeyCode() == WXK_RETURN)
    {
        // If something is selected, then unselect
        if((selectedIndex_ < interface_.sizeDecisions()) && (selectedIndex_ == hoverIndex_))
        {
            selectedIndex_ = interface_.sizeDecisions();
            display_.setDecisions(std::vector<planner::Decision>(interface_.beginDecisions(),
                                                                 interface_.endDecisions()));
        }
        // Otherwise, select if there is a valid hover
        else if(hoverIndex_ < interface_.sizeDecisions())
        {
            selectedIndex_ = hoverIndex_;
            didSelectArea_ = true;
            display_.setDecisions(std::vector<planner::Decision>());
        }
    }
    else
    {
        status = GLEventStatus::passthrough;
    }

    return status;
}


std::size_t find_action_with_direction(planner::DecisionDirection direction,
                                       const planner::NavigationInterface& interface)
{
    auto actionIt = std::find_if(interface.beginDecisions(),
                                 interface.endDecisions(),
                                 [direction](const planner::Decision& action) {
        return action.direction() == direction;
    });

    return std::distance(interface.beginDecisions(), actionIt);
}

} // namespace ui
} // namespace vulcan
