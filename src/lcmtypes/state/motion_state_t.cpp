/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/state/motion_state_t.h"
#include "core/motion_state.h"
#include "lcmtypes/state/robot_pose_t.h"
#include "lcmtypes/state/pose_distribution_t.h"
#include "lcmtypes/common/gaussian_distribution_t.h"
#include "lcmtypes/state/robot_velocity_t.h"
#include "lcmtypes/subscription_manager.h"
#include "lcmtypes/message_helpers.h"
#include <cassert>

static vulcan::lcm::SubscriptionManager<vulcan_lcm_motion_state_t, vulcan::motion_state_t> subscribers;

vulcan::drive_wheel_t convert_lcm_to_vulcan(const vulcan_lcm_drive_wheel_t& wheelMessage);
void convert_vulcan_to_lcm(const vulcan::drive_wheel_t& wheel, vulcan_lcm_drive_wheel_t& wheelMessage);

vulcan::acceleration_t convert_lcm_to_vulcan(const vulcan_lcm_robot_acceleration_t& wheelMessage);
void convert_vulcan_to_lcm(const vulcan::acceleration_t& wheel, vulcan_lcm_robot_acceleration_t& wheelMessage);


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_motion_state_t& stateMessage, motion_state_t& state)
{
    pose_t                      pose;
    pose_distribution_t         poseDistribution;
    velocity_t                  velocity;
    acceleration_t              acceleration;
    differential_drive_wheels_t wheels;

    if(stateMessage.haveDistribution)
    {
        convert_lcm_to_vulcan(stateMessage.pose_distribution, poseDistribution);
    }
    else
    {
        convert_lcm_to_vulcan(stateMessage.pose, pose);
    }

    convert_lcm_to_vulcan(stateMessage.velocity, velocity);

    if(stateMessage.haveWheels)
    {
        acceleration      = convert_lcm_to_vulcan(stateMessage.acceleration);
        wheels.leftWheel  = convert_lcm_to_vulcan(stateMessage.left_wheel);
        wheels.rightWheel = convert_lcm_to_vulcan(stateMessage.right_wheel);
    }

    if(stateMessage.haveDistribution)
    {
        if(stateMessage.haveWheels)
        {
            state = motion_state_t(poseDistribution, velocity, acceleration, wheels);
        }
        else
        {
            state = motion_state_t(poseDistribution, velocity);
        }
    }
    else
    {
        if(stateMessage.haveWheels)
        {
            state = motion_state_t(pose, velocity, acceleration, wheels);
        }
        else
        {
            state = motion_state_t(pose, velocity);
        }
    }
}


void vulcan::lcm::convert_vulcan_to_lcm(const motion_state_t& state, vulcan_lcm_motion_state_t& stateMessage)
{
    stateMessage.timestamp = state.timestamp;

    convert_vulcan_to_lcm(state.pose,                          stateMessage.pose);
    convert_vulcan_to_lcm(state.poseDistribution,              stateMessage.pose_distribution);
    convert_vulcan_to_lcm(state.velocity,                      stateMessage.velocity);
    convert_vulcan_to_lcm(state.acceleration,                  stateMessage.acceleration);
    convert_vulcan_to_lcm(state.differentialWheels.leftWheel,  stateMessage.left_wheel);
    convert_vulcan_to_lcm(state.differentialWheels.rightWheel, stateMessage.right_wheel);

    stateMessage.haveDistribution = state.haveDistribution;
    stateMessage.haveWheels       = state.haveWheels;
}


void vulcan::lcm::publish_data(lcm_t* lcm, const motion_state_t& state, std::string channel)
{
    verify_channel(channel, MOTION_STATE_CHANNEL, false);

    vulcan_lcm_motion_state_t stateMessage;

    convert_vulcan_to_lcm(state, stateMessage);
    vulcan_lcm_motion_state_t_publish(lcm, channel.c_str(), &stateMessage);

    free_gaussian_message(stateMessage.pose_distribution.uncertainty);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm, void (*callback)(const motion_state_t&, const std::string&, void*), void* userdata, std::string channel)
{
    verify_channel(channel, MOTION_STATE_CHANNEL, true);

    channel_subscriber_t<motion_state_t> newSubscriber(channel, userdata, callback);

    if(!subscribers.isSubscribedToChannel(lcm, channel))
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_motion_state_t_subscribe(lcm, channel.c_str(), subscription_manager_callback<vulcan_lcm_motion_state_t, motion_state_t>, &subscribers);
    }
    else
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}


vulcan::acceleration_t convert_lcm_to_vulcan(const vulcan_lcm_robot_acceleration_t& accelerationMessage)
{
    vulcan::acceleration_t acceleration;
    acceleration.timestamp = accelerationMessage.timestamp;
    acceleration.linear    = accelerationMessage.linear;
    acceleration.angular   = accelerationMessage.angular;
    return acceleration;
}


void convert_vulcan_to_lcm(const vulcan::acceleration_t& acceleration, vulcan_lcm_robot_acceleration_t& accelerationMessage)
{
    accelerationMessage.timestamp = acceleration.timestamp;
    accelerationMessage.linear    = acceleration.linear;
    accelerationMessage.angular   = acceleration.angular;
}


vulcan::drive_wheel_t convert_lcm_to_vulcan(const vulcan_lcm_drive_wheel_t& wheelMessage)
{
    vulcan::drive_wheel_t wheel;
    wheel.timestamp  = wheelMessage.timestamp;
    wheel.speed      = wheelMessage.speed;
    wheel.motorAccel = wheelMessage.motor_accel;
    return wheel;
}


void convert_vulcan_to_lcm(const vulcan::drive_wheel_t& wheel, vulcan_lcm_drive_wheel_t& wheelMessage)
{
    wheelMessage.timestamp   = wheel.timestamp;
    wheelMessage.speed       = wheel.speed;
    wheelMessage.motor_accel = wheel.motorAccel;
}
