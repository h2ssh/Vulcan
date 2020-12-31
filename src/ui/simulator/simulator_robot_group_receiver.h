/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_robot_group_receiver.h
* \author   Zongtai Luo
*
* Decleration of RobotGroupReceiver.
*/

#ifndef UI_SIMULATOR_SIMULATOR_ROBOT_GROUP_RECEIVER_H
#define UI_SIMULATOR_SIMULATOR_ROBOT_GROUP_RECEIVER_H

#include "ui/simulator/simulator_robot_receiver.h"
#include "ui/simulator/simulator_ui_params.h"

namespace vulcan
{
namespace ui
{

class RobotGroupReceiver
{
public:

	RobotGroupReceiver(void):
	robot_num_(0)
	{}

	void initRobotGroupReceiver(void);

	void pushBack(std::string system_url);

	void subscribe(void);

	void update(void);

	void RenderRobots(const std::unique_ptr<RobotRenderer>& robotRenderer);

	~RobotGroupReceiver();
	
private:
	int8_t robot_num_;

	std::vector<RobotReceiver*> robot_receivers;

};

} // ui
} // vulcan

#endif
