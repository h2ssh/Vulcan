/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <lcmtypes/legacy/commanded_velocity_t.h>
#include <robot/commands.h>
#include <lcmtypes/subscription_manager.h>
#include <lcmtypes/message_helpers.h>

static vulcan::lcm::SubscriptionManager<vulcan_lcm_commanded_velocity_t, vulcan::robot::commanded_velocity_t> subscribers;


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_commanded_velocity_t& velocityMessage, robot::commanded_velocity_t& velocity)
{
    velocity.timestamp = velocityMessage.timestamp;

    velocity.linearVelocity  = velocityMessage.linear_velocity;
    velocity.angularVelocity = velocityMessage.angular_velocity;
}


void vulcan::lcm::convert_vulcan_to_lcm(const robot::commanded_velocity_t& velocity, vulcan_lcm_commanded_velocity_t& velocityMessage)
{
    velocityMessage.timestamp = velocity.timestamp;

    velocityMessage.linear_velocity  = velocity.linearVelocity;
    velocityMessage.angular_velocity = velocity.angularVelocity;
}


void vulcan::lcm::publish_data(lcm_t* lcm, const robot::commanded_velocity_t& velocity, std::string channel)
{
    verify_channel(channel, COMMANDED_VELOCITY_CHANNEL, false);

    vulcan_lcm_commanded_velocity_t velocityMessage;

    convert_vulcan_to_lcm(velocity, velocityMessage);
    vulcan_lcm_commanded_velocity_t_publish(lcm, channel.c_str(), &velocityMessage);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm, void (*callback)(const robot::commanded_velocity_t&, const std::string&, void*), void* userdata, std::string channel)
{
    verify_channel(channel, COMMANDED_VELOCITY_CHANNEL, true);

    channel_subscriber_t<vulcan::robot::commanded_velocity_t> newSubscriber(channel, userdata, callback);

    if(!subscribers.isSubscribedToChannel(lcm, channel))
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_commanded_velocity_t_subscribe(lcm, channel.c_str(),
                                                subscription_manager_callback<vulcan_lcm_commanded_velocity_t, vulcan::robot::commanded_velocity_t>,
                                                &subscribers);
    }
    else
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
