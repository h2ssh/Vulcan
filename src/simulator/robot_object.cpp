/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** \file 	robot_object.cpp
*	\author Zongtai Luo
*	definition of each robot object.
*/

#include <core/velocity.h>
#include <mpepc/metric_planner/messages.h>
#include <mpepc/metric_planner/script/script.h>
#include <simulator/robot_object.h>
#include <simulator/simulator_utils.h>

namespace vulcan{
namespace sim{


RobotObject::RobotObject(robot_params_t robot_params,
                         sensors::wheel_encoders_params_t wheel_encoders_params,
                         robot::DifferentialMotorsPlant PlantModel,
						 bool isSlam,
                         system::ModuleCommunicator* communicator,
                         int8_t id):
id_(id),
timestamp_(0),
leftTicksTotal(0),
rightTicksTotal(0),
robot_params_(robot_params),
isSlam_(isSlam),
hasCommand_(false),
wheel_encoders_params_(wheel_encoders_params),
PlantModel_(PlantModel),
communicator_(communicator)
{
	std::vector<pose_t> loadedTargets = loadScript();
    motion_state_.pose = loadedTargets[0];
	former_motion_state_ = motion_state_;
}


RobotObject::RobotObject(pose_t pose,
                         robot_params_t robot_params,
                         sensors::wheel_encoders_params_t wheel_encoders_params,
                         robot::DifferentialMotorsPlant PlantModel,
                         bool isSlam,
                         system::ModuleCommunicator* communicator,
                         int8_t id):
id_(id),
timestamp_(0),
leftTicksTotal(0),
rightTicksTotal(0),
robot_params_(robot_params),
isSlam_(isSlam),
hasCommand_(false),
wheel_encoders_params_(wheel_encoders_params),
PlantModel_(PlantModel),
communicator_(communicator)
{
    motion_state_.pose = pose;
    former_motion_state_ = motion_state_;
}


void RobotObject::subscribe(void)
{
	communicator_->subscribeTo<robot::motion_command_t>(this);
}


void RobotObject::handleData(const robot::motion_command_t& motion_command, const std::string& channel)
{
	motion_command_ = motion_command;
    hasCommand_ = true;
}


void RobotObject::runUpdate(hssh::LocalPerceptualMap& lpm_for_robot, hssh::LocalPerceptualMap& lpm_for_ui)
{
    motion_state_t motion_state = motion_state_;
    motion_state_.timestamp = timestamp_;
    motion_state_ = PlantModel_.nextState(motion_state,motion_command_,0.001);

    if(timestamp_ % 20000 == 0 && !isSlam_)
    {
        communicator_->sendMessage(motion_state_);
    }

    if(timestamp_ % 25000 == 0)
    {
    	sim::flaser_scan_producer(motion_state_.pose,lpm_for_robot,timestamp_,*communicator_);
    }

    if(timestamp_ % 8000 == 0)
    {
        sim::fencoder_producer(leftTicksTotal,rightTicksTotal,motion_state_,former_motion_state_,wheel_encoders_params_,timestamp_,*communicator_);
        sim::fodometry_producer(motion_state_,former_motion_state_,timestamp_,*communicator_);
    }

    timestamp_ += 1000;
    printf("Robot id is %d and Current state of robot is in %f, %f, %f and timestamp is %d\n", id_, motion_state_.pose.x,motion_state_.pose.y,motion_state_.pose.theta, timestamp_);

    communicator_->sendMessage(lpm_for_ui);
    // clean the map everytime
    lpm_for_robot = lpm_for_ui;
    communicator_->processIncoming(1);

}


void RobotObject::startLooping(void)
{
    if(!robot_params_.script_path.empty())
    {
        communicator_->sendMessage(mpepc::MetricPlannerScript(robot_params_.script_path));

        mpepc::metric_planner_command_message_t plannerMessage;
        plannerMessage.command   = mpepc::SCRIPT_LOOP;
        plannerMessage.timestamp = utils::system_time_us();
        communicator_->sendMessage(plannerMessage);
    }
}


std::vector<pose_t> RobotObject::loadScript()
{
    mpepc::MetricPlannerScript script(robot_params_.script_path);

    std::vector<pose_t> loadedTargets;

    for(auto& task : script)
    {
        auto targets = task.getTargets();
        loadedTargets.insert(loadedTargets.end(), targets.begin(), targets.end());
    }

    return loadedTargets;
}


} // sim
} // vulcan
