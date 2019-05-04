/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <lcmtypes/legacy/encoder_data_t.h>
#include <core/odometry.h>
#include <lcmtypes/subscription_manager.h>
#include <lcmtypes/message_helpers.h>

static vulcan::lcm::SubscriptionManager<vulcan_lcm_encoder_data_t, vulcan::encoder_data_t> subscribers;


void vulcan::lcm::convert_vulcan_to_lcm(const encoder_data_t& encoders, vulcan_lcm_encoder_data_t& encodersMessage)
{
    encodersMessage.timestamp = encoders.timestamp;
    encodersMessage.id        = encoders.id;

    encodersMessage.delta_left_wheel    = encoders.deltaLeftWheel;
    encodersMessage.delta_right_wheel   = encoders.deltaRightWheel;
    encodersMessage.wheelbase           = encoders.wheelbase;
    encodersMessage.left_circumference  = encoders.leftWheelCircumference;
    encodersMessage.right_circumference = encoders.rightWheelCircumference;
    encodersMessage.left_ticks_per_rev  = encoders.leftTicksPerRevolution;
    encodersMessage.right_ticks_per_rev = encoders.rightTicksPerRevolution;

    encodersMessage.left_rpm  = encoders.leftRPM;
    encodersMessage.right_rpm = encoders.rightRPM;

    encodersMessage.left_ticks_total  = encoders.leftTicksTotal;
    encodersMessage.right_ticks_total = encoders.rightTicksTotal;

    encodersMessage.left_index_pulse_total  = encoders.leftIndexPulseTotal;
    encodersMessage.right_index_pulse_total = encoders.rightIndexPulseTotal;
}


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_encoder_data_t& encodersMessage, encoder_data_t& encoders)
{
    encoders.timestamp = encodersMessage.timestamp;
    encoders.id        = encodersMessage.id;

    encoders.deltaLeftWheel  = encodersMessage.delta_left_wheel;
    encoders.deltaRightWheel = encodersMessage.delta_right_wheel;
    encoders.wheelbase       = encodersMessage.wheelbase;

    encoders.leftRPM  = encodersMessage.left_rpm;
    encoders.rightRPM = encodersMessage.right_rpm;

    encoders.leftTicksTotal  = encodersMessage.left_ticks_total;
    encoders.rightTicksTotal = encodersMessage.right_ticks_total;

    encoders.leftIndexPulseTotal  = encodersMessage.left_index_pulse_total;
    encoders.rightIndexPulseTotal = encodersMessage.right_index_pulse_total;

    encoders.leftTicksPerRevolution = encodersMessage.left_ticks_per_rev;
    encoders.rightTicksPerRevolution = encodersMessage.right_ticks_per_rev;

    encoders.leftWheelCircumference = encodersMessage.left_circumference;
    encoders.rightWheelCircumference = encodersMessage.right_circumference;
}


void vulcan::lcm::publish_data(lcm_t* lcm, const encoder_data_t& encoders, std::string channel)
{
    verify_channel(channel, ENCODERS_CHANNEL, false);

    vulcan_lcm_encoder_data_t encodersMessage;

    convert_vulcan_to_lcm(encoders, encodersMessage);
    vulcan_lcm_encoder_data_t_publish(lcm, channel.c_str(), &encodersMessage);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm, void (*callback)(const encoder_data_t&, const std::string&, void*), void* userdata, std::string channel)
{
    verify_channel(channel, ENCODERS_CHANNEL, true);

    channel_subscriber_t<encoder_data_t> newSubscriber(channel, userdata, callback);

    if(!subscribers.isSubscribedToChannel(lcm, channel))
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_encoder_data_t_subscribe(lcm, channel.c_str(), subscription_manager_callback<vulcan_lcm_encoder_data_t, encoder_data_t>, &subscribers);
    }
    else
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
