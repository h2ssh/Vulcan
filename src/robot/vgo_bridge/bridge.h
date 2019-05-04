/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     bridge.h
* \author   Collin Johnson
*
* Declaration of VGoBridge.
*/

#ifndef ROBOT_VGO_BRIDGE_H
#define ROBOT_VGO_BRIDGE_H

#include <system/module_communicator.h>
#include <utils/thread.h>
#include <ros/ros.h>
#include <vgo_msgs/Bumper.h>
#include <vgo_msgs/Encoders.h>
#include <vgo_msgs/ir_msg.h>
#include <vgo_msgs/PowerButton.h>
#include <string>

namespace vulcan
{
namespace robot
{

struct commanded_velocity_t;
struct vgo_service_t;

/**
* VGoBridge converts between Vulcan messages and VGo messages.
*/
class VGoBridge
{
public:

    /**
    * Constructor for VGoBridge.
    */
    VGoBridge(void);

    // Handlers for the incoming Vulcan messages
    void handleData(const commanded_velocity_t& command, const std::string& channel);
    void handleData(const vgo_service_t&        service, const std::string& channel);

private:

    VGoBridge(const VGoBridge&  toCopy)   = delete;
    VGoBridge(const VGoBridge&& toMove)   = delete;
    void operator=(const VGoBridge&  rhs) = delete;
    void operator=(const VGoBridge&& rhs) = delete;

    void processServiceRequest(const vgo_service_t& service);

    void handleVGoEncoders(const vgo_msgs::Encoders::ConstPtr&    encoderData);
    void handleVGoBumpers (const vgo_msgs::Bumper::ConstPtr&      bumperData);
    void handleVGoIR      (const vgo_msgs::ir_msg::ConstPtr&      irData);
    void handleVGoPower   (const vgo_msgs::PowerButton::ConstPtr& powerData);

    system::ModuleCommunicator communicator;
    utils::Thread              receiverThread;

    bool haveInitializedTicks;
    int  lastLeftTicks;
    int  lastRightTicks;
    int  encoderMessageId;

    ros::NodeHandle rosHandle;
    ros::Subscriber encoderSubscriber;
    ros::Subscriber bumperSubscriber;
    ros::Subscriber irSubscriber;
    ros::Subscriber powerSubscriber;
    ros::Publisher  commandPublisher;
};

}
}

#endif // ROBOT_VGO_BRIDGE_H
