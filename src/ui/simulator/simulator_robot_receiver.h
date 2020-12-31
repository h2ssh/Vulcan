/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     simulator_robot_receiver.h
* \author   Zongtai Luo
*
* Definition of RobotReceiver.
*/

#ifndef UI_SIMULATOR_SIMULATOR_ROBOT_RECEIVER_H
#define UI_SIMULATOR_SIMULATOR_ROBOT_RECEIVER_H

#include "utils/locked_double_buffer.h"
#include "ui/components/robot_renderer.h"
#include "system/module_communicator.h"
#include "core/motion_state.h"

namespace vulcan
{
namespace ui
{

class RobotReceiver
{
public:
	RobotReceiver(){}

	RobotReceiver(system::ModuleCommunicator* communicator, int id):
	communicator_(communicator)
	{}

	void subscribe(void){communicator_->subscribeTo<motion_state_t>(this);}

	void handleData(const motion_state_t& motion, const std::string& channel){pose_ = motion.pose;}

	void processIncoming(int waitMs){communicator_->processIncoming(waitMs);}

	void swapBuffers(void){pose_.swapBuffers();}

	bool hasData(void){return pose_.hasData();}

	void RenderRobot(const std::unique_ptr<RobotRenderer>& robotRenderer){robotRenderer->renderRobot(pose_.read());}

	~RobotReceiver(){ delete communicator_; }

private:
	int id;
	
	// Buffer for the data coming in
    template <class T>
    using Buffer = utils::LockedDoubleBuffer<T>;

    Buffer<pose_t> pose_;

    system::ModuleCommunicator* communicator_;
	
};

} // ui
} // vulcan

#endif
