/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     system_panel.cpp
 * \author   Collin Johnson
 *
 * Definition of SystemPanel.
 */

#include "ui/debug/system_panel.h"

namespace vulcan
{
namespace ui
{

void SystemPanel::setup(wxGLContext* context, wxStatusBar* statusBar)
{
}


void SystemPanel::subscribe(system::ModuleCommunicator& producer)
{
}


void SystemPanel::setConsumer(system::ModuleCommunicator* consumer)
{
}


void SystemPanel::update(void)
{
}


void SystemPanel::saveSettings(utils::ConfigFileWriter& config)
{
}


void SystemPanel::loadSettings(const utils::ConfigFile& config)
{
}

}   // namespace ui
}   // namespace vulcan
