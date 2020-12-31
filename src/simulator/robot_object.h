/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/** \file 	robot_object.h
 *	\author Zongtai Luo
 *	definition of each robot object.
 */
#ifndef ENVIRONMENT_SIMULATOR_ROBOT_OBJECT_H
#define ENVIRONMENT_SIMULATOR_ROBOT_OBJECT_H

#include "core/drive_wheel.h"
#include "core/motion_state.h"
#include "hssh/local_metric/lpm.h"
#include "robot/commands.h"
#include "robot/model/differential_motors_plant.h"
#include "sensors/wheel_encoders_params.h"
#include "simulator/simulator_params.h"
#include "system/module_communicator.h"

namespace vulcan
{
namespace sim
{


// how to define different channels in the simulator ui
class RobotObject
{
public:
    RobotObject(robot_params_t robot_params,
                sensors::wheel_encoders_params_t wheel_encoders_params,
                robot::DifferentialMotorsPlant PlantModel,
                bool isSlam,
                system::ModuleCommunicator* communicator,
                int8_t id);

    RobotObject(pose_t pose,
                robot_params_t robot_params,
                sensors::wheel_encoders_params_t wheel_encoders_params,
                robot::DifferentialMotorsPlant PlantModel,
                bool isSlam,
                system::ModuleCommunicator* communicator,
                int8_t id);

    pose_t getRobotPose(void) const { return motion_state_.pose; }

    robot_params_t getRobotParams(void) const { return robot_params_; }

    void subscribe(void);

    void handleData(const robot::motion_command_t& motion_command, const std::string& channel);

    void runUpdate(hssh::LocalPerceptualMap& lpm_for_robot, hssh::LocalPerceptualMap& lpm_for_ui);

    void startLooping(void);

    system::ModuleCommunicator* getCommunicator(void) { return communicator_; }

    ~RobotObject() { delete communicator_; }

private:
    std::vector<pose_t> loadScript(void);

    int8_t id_;

    int timestamp_;
    int64_t leftTicksTotal;
    int64_t rightTicksTotal;

    robot_params_t robot_params_;

    bool isSlam_;

    // keep track of the motion state of robot
    motion_state_t motion_state_;
    motion_state_t former_motion_state_;

    bool hasCommand_;
    robot::motion_command_t motion_command_;

    // model of this robot
    sensors::wheel_encoders_params_t wheel_encoders_params_;
    robot::DifferentialMotorsPlant PlantModel_;

    system::ModuleCommunicator* communicator_;
};

}   // namespace sim
}   // namespace vulcan

#endif