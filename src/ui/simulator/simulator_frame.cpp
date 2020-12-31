/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_frame.cpp
* \author   Collin Johnson and Zongtai Luo
*
* Definition of SimulatorFrame.
*/

#include "ui/simulator/simulator_frame.h"
#include "ui/simulator/simulator_robot_control.h"
#include "ui/simulator/simulator_display.h"
#include "ui/simulator/simulator_robot_display.h"

namespace vulcan
{
namespace ui
{

SimulatorFrame::SimulatorFrame(void)
: SimulatorUI(0)
{
    setupSimulatorUI();
    initialize(nullptr, 30, display, gridCellStatusBar);
    initialize(nullptr, 30, robot_display, gridCellStatusBar);

    PushEventHandler(robot_control_);
}


SimulatorFrame::~SimulatorFrame(void)
{}


void SimulatorFrame::setupSimulatorUI(void)
{
	simulator_ui_panel_widgets_t widgets;

	widgets.robot_display_ = robot_display;
	widgets.ground_truth_display_ = display;
	widgets.scriptNameText = simulatorScriptFileText;
	widgets.robotConfigText = simulatorRobotConfigText;
	
	robot_control_ = new SimulatorRobotControl(widgets);
    addPanel(robot_control_, nullptr);
}

}// ui
}// vulcan
