/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** \file 	robot_group.h
*	\author Zongtai Luo
*	container for all the robot project in the simulator.
*/
#ifndef ENVIRONMENT_SIMULATOR_ROBOT_GROUP_H
#define ENVIRONMENT_SIMULATOR_ROBOT_GROUP_H

#include "simulator/simulator_params.h"
#include "simulator/robot_object.h"

namespace vulcan{
namespace sim{

struct simulator_robot_group_message_t
{
	int8_t robots_num_;
	// int8_t mpepc_num_;

	pose_t 	pose_;
	std::string  robot_config_;
	
	simulator_robot_group_message_t(void):
	robots_num_(0),
	pose_(),
	robot_config_("")
	{}

	// Serialization support
	template <class Archive>
	void serialize(Archive& ar)
	{
	    ar (robots_num_,
	    	pose_,
	    	robot_config_);
	}
};


class RobotObjectGroup
{
public:
	RobotObjectGroup(void){}

	RobotObjectGroup(simulator_params_t& simulator_params);

	void subscribe(void);

	void handleData(const simulator_robot_group_message_t& simulator_robot_group_message, const std::string& channel);

	// updates all states of robots
	void runGroupUpdates(void);

	// if targets loaded the looping shall not be called twice
	void startGroupLooping(void);

	bool getLoopingFlag(void){return isLoadedGoals;}

	void updateGroupLPMs(void);

	void pushBack(robot_params_t robot_params, pose_t pose);

	// send message to the ui
	system::ModuleCommunicator* getCommunicator(void){ return communicator_; }

	~RobotObjectGroup();

private:
	// init an robot object
	void initRobotObject(robot_params_t robot_params, hssh::LocalPerceptualMap lpm, int8_t robot_id, pose_t pose, bool hasPose = false);

	// draw robots on corresponding lpm
	void drawRobot(int8_t lpm_index, int8_t robot_index);
	void drawLine(Point<double> start, Point<double> end, int resolution, int8_t lpm_index);
	Point<double> rotation(double x, double y, double theta);

	int64_t timestamp;
	int8_t 	robots_num_;
	bool	isLoadedGoals;

	// Group robots_;
	std::vector<RobotObject*> robots_;

	// lpm for each robots with other robots drawing on the lpm;
	std::vector<hssh::LocalPerceptualMap> lpms;
	// this will also be the lpm for the new robot
	hssh::LocalPerceptualMap lpm_for_ui; 

	// message from ui
	bool isAddRobotReceived;
	simulator_robot_group_message_t simulator_robot_group_message_;

	system::ModuleCommunicator* communicator_;
};


} // sim
} // vulcan

DEFINE_SYSTEM_MESSAGE(sim::simulator_robot_group_message_t, ("SIMULATOR_ROBOT_GROUP_MESSAGE"))

#endif