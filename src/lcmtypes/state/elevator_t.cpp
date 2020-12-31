/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/state/elevator_t.h"
#include "lcmtypes/message_helpers.h"
#include "lcmtypes/subscription_manager.h"
#include "robot/elevator.h"

static vulcan::lcm::SubscriptionManager<vulcan_lcm_elevator_t, vulcan::robot::elevator_t> subscribers;

void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_elevator_t& elevatorMessage, robot::elevator_t& elevator)
{
    elevator.timestamp = elevatorMessage.timestamp;
    elevator.state = static_cast<robot::elevator_state_t>(elevatorMessage.state);
    elevator.startTime = elevatorMessage.startTime;
    elevator.stopTime = elevatorMessage.stopTime;
    elevator.distance = elevatorMessage.distance;
    elevator.velocity = elevatorMessage.velocity;
}


void vulcan::lcm::convert_vulcan_to_lcm(const robot::elevator_t& elevator, vulcan_lcm_elevator_t& elevatorMessage)
{
    elevatorMessage.timestamp = elevator.timestamp;
    elevatorMessage.state = elevator.state;
    elevatorMessage.startTime = elevator.startTime;
    elevatorMessage.stopTime = elevator.stopTime;
    elevatorMessage.distance = elevator.distance;
    elevatorMessage.velocity = elevator.velocity;
}


void vulcan::lcm::publish_data(lcm_t* lcm, const robot::elevator_t& elevator, std::string channel)
{
    verify_channel(channel, ELEVATOR_CHANNEL, false);

    vulcan_lcm_elevator_t elevatorMessage;

    convert_vulcan_to_lcm(elevator, elevatorMessage);
    vulcan_lcm_elevator_t_publish(lcm, channel.c_str(), &elevatorMessage);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm,
                                       void (*callback)(const robot::elevator_t&, const std::string&, void*),
                                       void* userdata,
                                       std::string channel)
{
    verify_channel(channel, ELEVATOR_CHANNEL, true);

    channel_subscriber_t<robot::elevator_t> newSubscriber(channel, userdata, callback);

    if (!subscribers.isSubscribedToChannel(lcm, channel)) {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_elevator_t_subscribe(lcm,
                                        channel.c_str(),
                                        subscription_manager_callback<vulcan_lcm_elevator_t, robot::elevator_t>,
                                        &subscribers);
    } else {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
