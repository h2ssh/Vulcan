/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     simulator_robot_group_receiver.cpp
 * \author   Zongtai Luo
 *
 * Decleration of RobotGroupReceiver.
 */

#include "ui/simulator/simulator_robot_group_receiver.h"

namespace vulcan
{
namespace ui
{

RobotGroupReceiver::~RobotGroupReceiver(void)
{
    int i = robot_num_ - 1;
    while (i < robot_num_ - 1) {
        RobotReceiver* victim = robot_receivers[i];
        robot_receivers.erase(robot_receivers.begin() + i);
        delete victim;
        i--;
    }
}

void RobotGroupReceiver::initRobotGroupReceiver(void)
{
    // set up the other communicators change to the inner structure of robot group receiver
    utils::ConfigFile simulator_config("simulator_params.cfg");
    robot_receiver_params_t receiver_params = load_simulator_ui_params(simulator_config);

    robot_num_ = receiver_params.robot_num;
    // robot_receivers.reserve(robot_num_-1);

    int8_t i = 0;
    while (i < robot_num_ - 1) {
        system::ModuleCommunicator* communicator =
          new system::ModuleCommunicator(receiver_params.system_urls[i + 1], receiver_params.system_urls[i + 1]);
        robot_receivers.push_back(new RobotReceiver(communicator, i));
        i++;
    }
}


void RobotGroupReceiver::pushBack(std::string system_url)
{
    system::ModuleCommunicator* communicator = new system::ModuleCommunicator(system_url, system_url);
    robot_receivers.push_back(new RobotReceiver(communicator, robot_num_ - 1));
    robot_receivers[robot_num_ - 1]->subscribe();
    robot_num_++;
}


void RobotGroupReceiver::subscribe(void)
{
    int8_t i = 0;
    while (i < robot_num_ - 1) {
        robot_receivers[i]->subscribe();
        i++;
    }
}


void RobotGroupReceiver::update(void)
{
    int8_t i = 0;
    while (i < robot_num_ - 1) {
        robot_receivers[i]->processIncoming(1);
        if (robot_receivers[i]->hasData()) {
            robot_receivers[i]->swapBuffers();
        }
        i++;
    }
}


void RobotGroupReceiver::RenderRobots(const std::unique_ptr<RobotRenderer>& robotRenderer)
{
    int8_t i = 0;
    while (i < robot_num_ - 1) {
        robot_receivers[i]->RenderRobot(robotRenderer);
        i++;
    }
}


}   // namespace ui
}   // namespace vulcan