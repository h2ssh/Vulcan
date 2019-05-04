/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <lcmtypes/state/robot_pose_t.h>
#include <core/pose.h>
#include <lcmtypes/subscription_manager.h>
#include <lcmtypes/message_helpers.h>

static vulcan::lcm::SubscriptionManager<vulcan_lcm_robot_pose_t, vulcan::pose_t> subscribers;


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_robot_pose_t& poseMessage, pose_t& pose)
{
    pose.timestamp      = poseMessage.timestamp;

    pose.x     = poseMessage.x;
    pose.y     = poseMessage.y;
    pose.theta = poseMessage.theta;
}


void vulcan::lcm::convert_vulcan_to_lcm(const pose_t& pose, vulcan_lcm_robot_pose_t& poseMessage)
{
    poseMessage.timestamp       = pose.timestamp;

    poseMessage.x     = pose.x;
    poseMessage.y     = pose.y;
    poseMessage.theta = pose.theta;
}


void vulcan::lcm::publish_data(lcm_t* lcm, const pose_t& pose, std::string channel)
{
    verify_channel(channel, POSE_CHANNEL, false);

    vulcan_lcm_robot_pose_t poseMessage;

    convert_vulcan_to_lcm(pose, poseMessage);
    vulcan_lcm_robot_pose_t_publish(lcm, channel.c_str(), &poseMessage);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm, void (*callback)(const pose_t&, const std::string&, void*), void* userdata, std::string channel)
{
    verify_channel(channel, POSE_CHANNEL, true);

    channel_subscriber_t<pose_t> newSubscriber(channel, userdata, callback);

    if(!subscribers.isSubscribedToChannel(lcm, channel))
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_robot_pose_t_subscribe(lcm, channel.c_str(), subscription_manager_callback<vulcan_lcm_robot_pose_t, pose_t>, &subscribers);
    }
    else
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
