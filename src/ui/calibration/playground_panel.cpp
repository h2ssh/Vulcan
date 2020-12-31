/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     playground_panel.cpp
 * \author   Collin Johnson
 *
 * Definition of PlaygroundPanel.
 */

#include "ui/calibration/playground_panel.h"

namespace vulcan
{
namespace ui
{

PlaygroundPanel::PlaygroundPanel(const playground_panel_widgets_t& widgets,
                                 const control_law_dialog_widgets_t& controlWidgets,
                                 const calibration_ui_params_t& params)
: playgroundWidgets(widgets)
, controlLawWidgets(controlWidgets)
{
}


void PlaygroundPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
}


void PlaygroundPanel::subscribe(system::ModuleCommunicator& communicator)
{
}


void PlaygroundPanel::setConsumer(system::ModuleCommunicator* consumer)
{
}


void PlaygroundPanel::update(void)
{
}


void PlaygroundPanel::saveSettings(utils::ConfigFileWriter& config)
{
}


void PlaygroundPanel::loadSettings(const utils::ConfigFile& config)
{
}


void PlaygroundPanel::handleData(const hssh::LocalPerceptualMap& lpm, const std::string& channel)
{
}


void PlaygroundPanel::handleData(const pose_t& pose, const std::string& channel)
{
}


void PlaygroundPanel::handleData(const velocity_t& velocity, const std::string& channel)
{
}


void PlaygroundPanel::handleData(const mpepc::metric_waypoint_path_t& path, const std::string& channel)
{
}


void PlaygroundPanel::createControllerPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::loadControllerPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::saveControllerPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::setPlaygroundPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::donePlaygroundPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::clearPlaygroundPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::setTargetRegionPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::doneTargetRegionPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::clearTargetRegionPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::autoCalcVMaxPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::startSessionPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::pauseSessionPressed(wxCommandEvent& event)
{
}


void PlaygroundPanel::stopSessionPressed(wxCommandEvent& event)
{
}

}   // namespace ui
}   // namespace vulcan
