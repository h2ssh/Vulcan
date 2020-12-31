/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/state/pose_distribution_t.h"
#include "core/pose_distribution.h"
#include "lcmtypes/common/gaussian_distribution_t.h"
#include "lcmtypes/subscription_manager.h"
#include "lcmtypes/message_helpers.h"

static vulcan::lcm::SubscriptionManager<vulcan_lcm_pose_distribution_t, vulcan::pose_distribution_t> subscribers;


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_pose_distribution_t& poseMessage, pose_distribution_t& pose)
{
    pose.timestamp = poseMessage.timestamp;

    pose.x     = poseMessage.x;
    pose.y     = poseMessage.y;
    pose.theta = poseMessage.theta;

    convert_lcm_to_vulcan(poseMessage.uncertainty, pose.uncertainty);
}


void vulcan::lcm::convert_vulcan_to_lcm(const pose_distribution_t& pose, vulcan_lcm_pose_distribution_t& poseMessage)
{
    poseMessage.timestamp = pose.timestamp;

    poseMessage.x     = pose.x;
    poseMessage.y     = pose.y;
    poseMessage.theta = pose.theta;

    convert_vulcan_to_lcm(pose.uncertainty, poseMessage.uncertainty);
}


void vulcan::lcm::publish_data(lcm_t* lcm, const pose_distribution_t& pose, std::string channel)
{
    verify_channel(channel, POSE_DISTRIBUTION_CHANNEL, false);

    vulcan_lcm_pose_distribution_t poseMessage;

    convert_vulcan_to_lcm(pose, poseMessage);
    vulcan_lcm_pose_distribution_t_publish(lcm, channel.c_str(), &poseMessage);
    free_gaussian_message(poseMessage.uncertainty);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm, void (*callback)(const pose_distribution_t&, const std::string&, void*), void* userdata, std::string channel)
{
    verify_channel(channel, POSE_DISTRIBUTION_CHANNEL, true);

    channel_subscriber_t<pose_distribution_t> newSubscriber(channel, userdata, callback);

    if(!subscribers.isSubscribedToChannel(lcm, channel))
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_pose_distribution_t_subscribe(lcm, channel.c_str(), subscription_manager_callback<vulcan_lcm_pose_distribution_t, pose_distribution_t>, &subscribers);
    }
    else
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
