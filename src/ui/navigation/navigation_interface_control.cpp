/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     navigation_interface_control.cpp
* \author   Collin Johnson
*
* Definition of NavigationInterfaceControl.
*/

#include "ui/navigation/navigation_interface_control.h"
#include "ui/navigation/decision_interface_control.h"
#include "ui/navigation/goal_interface_control.h"
#include "ui/navigation/navigation_interface_display.h"
#include "system/module_communicator.h"
#include "system/system_communicator.h"
#include "utils/timestamp.h"
#include <cassert>

namespace vulcan
{
namespace ui
{

NavigationInterfaceControl::NavigationInterfaceControl(NavigationInterfaceDisplay* display,
                                                       const GoalInterfaceWidgets& goalWidgets)
: haveMetricMap_(false)
, haveTopoMap_(false)
, haveLocation_(false)
, display_(display)
, interface_("navigation_interface")
, isPaused_(false)
, consumer_(nullptr)
{
    assert(display_);

    decisionControl_.reset(new DecisionInterfaceControl(interface_, *display_));
    goalControl_.reset(new GoalInterfaceControl(interface_, *display_, goalWidgets));

    display_->GetParent()->PushEventHandler(goalControl_.get());
}


NavigationInterfaceControl::~NavigationInterfaceControl(void)
{
    // For std::unique_ptr
    display_->GetParent()->PopEventHandler();
}


void NavigationInterfaceControl::setup(wxGLContext* context, wxStatusBar* statusBar)
{
    display_->setRenderContext(context);
    display_->pushKeyboardHandler(this);
    display_->pushKeyboardHandler(decisionControl_.get());

    display_->disablePanning();
    display_->disableScrolling();
    display_->disableTilting();
}


void NavigationInterfaceControl::subscribe(system::ModuleCommunicator& producer)
{
    producer.subscribeTo<hssh::LocalTopoMap>(this);
    producer.subscribeTo<hssh::LocalLocation>(this);
    producer.subscribeTo<motion_state_t>(this);
    producer.subscribeTo<hssh::LocalPerceptualMap>(this);
    producer.subscribeTo<mpepc::trajectory_planner_debug_info_t>(this);
    producer.subscribeTo<tracker::DynamicObjectCollection>(this);
}


void NavigationInterfaceControl::setConsumer(system::ModuleCommunicator* consumer)
{
    consumer_ = consumer;
}


void NavigationInterfaceControl::update(void)
{
    assert(consumer_);
    assert(decisionControl_);
    assert(goalControl_);

    auto data = loadNewData();

    system::SystemCommunicator sysComm(*consumer_);
    decisionControl_->update(data, sysComm);
    goalControl_->update(data, sysComm);

    display_->Refresh();
}


void NavigationInterfaceControl::saveSettings(utils::ConfigFileWriter& config)
{
}


void NavigationInterfaceControl::loadSettings(const utils::ConfigFile& config)
{
}


void NavigationInterfaceControl::handleData(const hssh::LocalTopoMap& map, const std::string& channel)
{
    topoMap_ = map;
    haveTopoMap_ = true;
}


void NavigationInterfaceControl::handleData(const hssh::LocalLocation& location, const std::string& channel)
{
    location_ = location;
    haveLocation_ = true;
}


void NavigationInterfaceControl::handleData(const motion_state_t& motion, const std::string& channel)
{
    pose_ = motion.pose;
}


void NavigationInterfaceControl::handleData(const hssh::LocalPerceptualMap& map, const std::string& channel)
{
    map_ = map;
}


void NavigationInterfaceControl::handleData(const mpepc::metric_planner_status_message_t& status,
                                            const std::string& channel)
{
    status_ = status;
}


void NavigationInterfaceControl::handleData(const mpepc::trajectory_planner_debug_info_t& trajectories,
                                            const std::string& channel)
{
    trajectories_ = trajectories;
}


void NavigationInterfaceControl::handleData(const tracker::DynamicObjectCollection& objects, const std::string& channel)
{
    objects_ = objects;
}


GLEventStatus NavigationInterfaceControl::keyPressed(wxKeyEvent& key)
{
    GLEventStatus status = GLEventStatus::capture;

    if(key.GetKeyCode() == WXK_SPACE)
    {
        if(isPaused_)
        {
            mpepc::metric_planner_command_message_t resumeMessage;
            resumeMessage.timestamp = utils::system_time_us();
            resumeMessage.command = mpepc::RESUME;
            consumer_->sendMessage(resumeMessage);

            isPaused_ = false;
        }
        else
        {
            mpepc::metric_planner_command_message_t pauseMessage;
            pauseMessage.timestamp = utils::system_time_us();
            pauseMessage.command = mpepc::PAUSE;
            consumer_->sendMessage(pauseMessage);

            isPaused_ = true;
        }
    }
    else if(key.GetKeyCode() == 'O')
    {
        display_->toggleObjects();
    }
    else if(key.GetKeyCode() == 'T')
    {
        display_->toggleTrajectories();
    }
    else
    {
        status = GLEventStatus::passthrough;
    }

    return status;
}


NavigationData NavigationInterfaceControl::loadNewData(void)
{
    NavigationData newData;

    if(topoMap_.hasData())
    {
        topoMap_.swapBuffers();
        display_->setAreas(topoMap_);
        haveTopoMap_ = true;
    }

    if(location_.hasData())
    {
        location_.swapBuffers();
        haveLocation_ = true;
    }

    if(map_.hasData())
    {
        map_.swapBuffers();
        display_->setLPM(map_);
        haveMetricMap_ = true;
    }

    if(objects_.hasData())
    {
        objects_.swapBuffers();
        display_->setObjects(objects_);
    }

    if(trajectories_.hasData())
    {
        trajectories_.swapBuffers();
        display_->setTrajectories(trajectories_);
    }

    if(pose_.hasData())
    {
        pose_.swapBuffers();
        display_->setPose(pose_);
    }

    newData.pose = pose_;
    newData.metricMap = haveMetricMap_ ? &map_.read() : nullptr;
    newData.location = haveLocation_ ? &location_.read() : nullptr;
    newData.topoMap = haveTopoMap_ ? &topoMap_.read() : nullptr;

    return newData;
}

} // namespace ui
} // namespace vulcan
