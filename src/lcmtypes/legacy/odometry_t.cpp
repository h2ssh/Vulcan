/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/legacy/odometry_t.h"
#include "core/odometry.h"
#include "lcmtypes/subscription_manager.h"
#include "lcmtypes/message_helpers.h"

static vulcan::lcm::SubscriptionManager<vulcan_lcm_odometry_t, vulcan::odometry_t> subscribers;


void vulcan::lcm::convert_vulcan_to_lcm(const odometry_t& odometry, vulcan_lcm_odometry_t& odometryMessage)
{
    odometryMessage.timestamp = odometry.timestamp;
    odometryMessage.id        = odometry.id;

    odometryMessage.x     = odometry.x;
    odometryMessage.y     = odometry.y;
    odometryMessage.theta = odometry.theta;
    
    odometryMessage.translation = odometry.translation;
    odometryMessage.rotation    = odometry.rotation;
}


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_odometry_t& odometryMessage, odometry_t& odometry)
{
    odometry.timestamp = odometryMessage.timestamp;
    odometry.id        = odometryMessage.id;

    odometry.x     = odometryMessage.x;
    odometry.y     = odometryMessage.y;
    odometry.theta = odometryMessage.theta;
    
    odometry.translation = odometryMessage.translation;
    odometry.rotation    = odometryMessage.rotation;
}


void vulcan::lcm::publish_data(lcm_t* lcm, const odometry_t& odometry, std::string channel)
{
    verify_channel(channel, ODOMETRY_CHANNEL, false);

    vulcan_lcm_odometry_t odometryMessage;

    convert_vulcan_to_lcm(odometry, odometryMessage);
    vulcan_lcm_odometry_t_publish(lcm, channel.c_str(), &odometryMessage);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm, void (*callback)(const odometry_t&, const std::string&, void*), void* userdata, std::string channel)
{
    verify_channel(channel, ODOMETRY_CHANNEL, true);

    channel_subscriber_t<odometry_t> newSubscriber(channel, userdata, callback);

    if(!subscribers.isSubscribedToChannel(lcm, channel))
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_odometry_t_subscribe(lcm, channel.c_str(), subscription_manager_callback<vulcan_lcm_odometry_t, odometry_t>, &subscribers);
    }
    else
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
