/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/legacy/commanded_joystick_t.h"
#include "lcmtypes/message_helpers.h"
#include "lcmtypes/subscription_manager.h"
#include "robot/commands.h"

static vulcan::lcm::SubscriptionManager<vulcan_lcm_commanded_joystick_t, vulcan::robot::commanded_joystick_t>
  subscribers;


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_commanded_joystick_t& joystickMessage,
                                        robot::commanded_joystick_t& joystick)
{
    joystick.timestamp = joystickMessage.timestamp;

    joystick.forward = joystickMessage.forward;
    joystick.left = joystickMessage.left;
}


void vulcan::lcm::convert_vulcan_to_lcm(const robot::commanded_joystick_t& joystick,
                                        vulcan_lcm_commanded_joystick_t& joystickMessage)
{
    joystickMessage.timestamp = joystick.timestamp;

    joystickMessage.forward = joystick.forward;
    joystickMessage.left = joystick.left;
}


void vulcan::lcm::publish_data(lcm_t* lcm, const robot::commanded_joystick_t& joystick, std::string channel)
{
    verify_channel(channel, COMMANDED_JOYSTICK_CHANNEL, false);

    vulcan_lcm_commanded_joystick_t joystickMessage;

    convert_vulcan_to_lcm(joystick, joystickMessage);
    vulcan_lcm_commanded_joystick_t_publish(lcm, channel.c_str(), &joystickMessage);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm,
                                       void (*callback)(const robot::commanded_joystick_t&, const std::string&, void*),
                                       void* userdata,
                                       std::string channel)
{
    verify_channel(channel, COMMANDED_JOYSTICK_CHANNEL, true);

    channel_subscriber_t<vulcan::robot::commanded_joystick_t> newSubscriber(channel, userdata, callback);

    if (!subscribers.isSubscribedToChannel(lcm, channel)) {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_commanded_joystick_t_subscribe(
          lcm,
          channel.c_str(),
          subscription_manager_callback<vulcan_lcm_commanded_joystick_t, vulcan::robot::commanded_joystick_t>,
          &subscribers);
    } else {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
