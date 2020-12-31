/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** \file 	robot_group.cpp
 *	\author Zongtai Luo
 *	container for all the robot project in the simulator.
 */

#include "simulator/robot_group.h"
#include "simulator/simulator_utils.h"
#include "utils/cell_grid_utils.h"

namespace vulcan
{
namespace sim
{

RobotObjectGroup::RobotObjectGroup(simulator_params_t& simulator_params)
{
    int8_t i = 0;
    robots_num_ = simulator_params.robots_num;

    // initilize map
    hssh::LocalPerceptualMap lpm;
    Map_init(simulator_params.map_path, lpm);

    while (i < robots_num_)   // do not change the condition
    {
        pose_t pose;
        initRobotObject(simulator_params.robot_params[i], lpm, i, pose);
        i++;
    }

    lpm_for_ui = lpm;
    // have access to the deafult Modular communicator
    communicator_ = robots_[0]->getCommunicator();
    timestamp = 0;
    isLoadedGoals = false;
    isAddRobotReceived = false;
}


RobotObjectGroup::~RobotObjectGroup()
{
    int8_t i = robots_num_ - 1;
    while (i >= 0) {
        RobotObject* victim = robots_[i];
        robots_.erase(robots_.begin() + i);
        delete victim;
        i--;
    }
}


void RobotObjectGroup::subscribe(void)
{
    int8_t i = 0;
    while (i < robots_num_) {
        robots_[i]->subscribe();
        i++;
    }

    communicator_->subscribeTo<simulator_robot_group_message_t>(this);
}


void RobotObjectGroup::handleData(const simulator_robot_group_message_t& simulator_robot_group_message,
                                  const std::string& channel)
{
    simulator_robot_group_message_ = simulator_robot_group_message;
    isAddRobotReceived = true;
}


void RobotObjectGroup::runGroupUpdates(void)
{
    int8_t i = 0;
    while (i < robots_num_) {
        this->updateGroupLPMs();
        robots_[i]->runUpdate(lpms[i], lpm_for_ui);
        i++;
    }

    if (isAddRobotReceived) {
        // std::cout<<simulator_robot_group_message_.robot_config_<<"\n";
        // utils::ConfigFile config(simulator_robot_group_message_.robot_config_);
        // utils::ConfigFile config("new_robot.cfg");
        // robot_params_t robot_config_ = load_new_robot_params(config);
        // pushBack(robot_config_,simulator_robot_group_message_.pose_);
        utils::ConfigFile config_simulator("simulator_params.cfg");
        simulator_params_t simulator_params = load_simulator_params(config_simulator, 1);
        std::cout << simulator_params.robot_params[2].system_url << "\n";
        pushBack(simulator_params.robot_params[robots_num_], simulator_robot_group_message_.pose_);
        isAddRobotReceived = false;
    }

    timestamp += 1000;
}


void RobotObjectGroup::startGroupLooping(void)
{
    int8_t i = 0;
    while (i < robots_num_) {
        robots_[i]->startLooping();
        i++;
    }
    isLoadedGoals = true;
}


void RobotObjectGroup::updateGroupLPMs(void)
{
    int8_t lpm_index = 0;

    while (lpm_index < robots_num_) {
        int8_t robot_index = 0;

        while (robot_index < robots_num_) {
            if (robot_index != lpm_index) {
                drawRobot(lpm_index, robot_index);
            }
            robot_index++;
        }
        lpm_index++;
    }
}


void RobotObjectGroup::pushBack(robot_params_t robot_params, pose_t pose)
{
    initRobotObject(robot_params, lpm_for_ui, robots_num_, pose, true);
    robots_[robots_num_]->subscribe();
    robots_[robots_num_]->startLooping();
    robots_num_++;
}


void RobotObjectGroup::initRobotObject(robot_params_t robot_params,
                                       hssh::LocalPerceptualMap lpm,
                                       int8_t robot_id,
                                       pose_t pose,
                                       bool hasPose)
{
    system::ModuleCommunicator* communicator =
      new system::ModuleCommunicator(robot_params.system_url, robot_params.system_url);
    std::cout << robot_params.system_url << "\n";
    utils::ConfigFile config(robot_params.robot_model);
    robot::differential_motors_plant_params_t modelParams(config);
    robot::DifferentialMotorsPlant plantModel(modelParams);

    utils::ConfigFile config_encoder(robot_params.encoder_model);
    sensors::wheel_encoders_params_t wheel_encoder_params = sensors::load_wheel_encoders_params(config_encoder);

    if (!hasPose) {
        robots_.push_back(
          new RobotObject(robot_params, wheel_encoder_params, plantModel, false, communicator, robot_id));
    } else if (hasPose) {
        robots_.push_back(
          new RobotObject(pose, robot_params, wheel_encoder_params, plantModel, false, communicator, robot_id));
    }

    lpms.push_back(lpm);
}


void RobotObjectGroup::drawRobot(int8_t lpm_index, int8_t robot_index)
{
    pose_t pose = robots_[robot_index]->getRobotPose();
    Point<double> point_robot(pose.x, pose.y);
    robot_params_t robot_params = robots_[robot_index]->getRobotParams();

    double object_x = robot_params.dimension.x / 2;
    double object_y = robot_params.dimension.y / 2;
    Point<double> point_lb = rotation(-object_x, -object_y, pose.theta) + point_robot;
    Point<double> point_lu = rotation(-object_x, object_y, pose.theta) + point_robot;
    Point<double> point_rb = rotation(object_x, -object_y, pose.theta) + point_robot;
    Point<double> point_ru = rotation(object_x, object_y, pose.theta) + point_robot;

    drawLine(point_lb, point_lu, 20, lpm_index);
    drawLine(point_lu, point_ru, 20, lpm_index);
    drawLine(point_ru, point_rb, 20, lpm_index);
    drawLine(point_rb, point_lb, 20, lpm_index);
}


Point<double> RobotObjectGroup::rotation(double x, double y, double theta)
{
    Point<double> point(x * std::cos(theta) - y * std::sin(theta), x * std::sin(theta) + y * std::cos(theta));
    return point;
}


void RobotObjectGroup::drawLine(Point<double> start, Point<double> end, int resolution, int8_t lpm_index)
{
    double deltax = (end.x - start.x) / resolution;
    double deltay = (end.y - start.y) / resolution;
    Point<double> gridpoint;
    int k = 0;
    while (k < resolution) {
        gridpoint = utils::global_point_to_grid_point(start, lpms[lpm_index]);
        lpms[lpm_index].setCostNoCheck(gridpoint, 255);
        start.x += deltax;
        start.y += deltay;
        k++;
    }
}


}   // namespace sim
}   // namespace vulcan