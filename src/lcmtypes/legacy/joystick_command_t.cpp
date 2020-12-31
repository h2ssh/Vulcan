/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/legacy/joystick_command_t.h"
#include "robot/commands.h"
#include "lcmtypes/subscription_manager.h"
#include "lcmtypes/message_helpers.h"

static vulcan::lcm::SubscriptionManager<vulcan_lcm_joystick_command_t, vulcan::robot::joystick_command_t> subscribers;


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_joystick_command_t& joystickMessage, robot::joystick_command_t& joystick)
{
    joystick.timestamp = joystickMessage.timestamp;

    joystick.forward = joystickMessage.forward;
    joystick.left    = joystickMessage.left;
    joystick.gain    = joystickMessage.gain;
}


void vulcan::lcm::convert_vulcan_to_lcm(const robot::joystick_command_t& joystick, vulcan_lcm_joystick_command_t& joystickMessage)
{
    joystickMessage.timestamp = joystick.timestamp;

    joystickMessage.forward = joystick.forward;
    joystickMessage.left    = joystick.left;
    joystickMessage.gain    = joystick.gain;
}


void vulcan::lcm::publish_data(lcm_t* lcm, const robot::joystick_command_t& joystick, std::string channel)
{
    verify_channel(channel, JOYSTICK_COMMAND_CHANNEL, false);

    vulcan_lcm_joystick_command_t joystickMessage;

    convert_vulcan_to_lcm(joystick, joystickMessage);
    vulcan_lcm_joystick_command_t_publish(lcm, channel.c_str(), &joystickMessage);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm, void (*callback)(const robot::joystick_command_t&, const std::string&, void*), void* userdata, std::string channel)
{
    verify_channel(channel, JOYSTICK_COMMAND_CHANNEL, true);

    channel_subscriber_t<robot::joystick_command_t> newSubscriber(channel, userdata, callback);

    if(!subscribers.isSubscribedToChannel(lcm, channel))
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_joystick_command_t_subscribe(lcm, channel.c_str(),
                                                subscription_manager_callback<vulcan_lcm_joystick_command_t, robot::joystick_command_t>,
                                                &subscribers);
    }
    else
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
