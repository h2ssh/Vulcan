/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/legacy/imu_t.h"
#include "core/imu_data.h"
#include "lcmtypes/message_helpers.h"
#include "lcmtypes/subscription_manager.h"

static vulcan::lcm::SubscriptionManager<vulcan_lcm_imu_t, vulcan::imu_data_t> subscribers;


void vulcan::lcm::convert_vulcan_to_lcm(const imu_data_t& imuData, vulcan_lcm_imu_t& imuMessage)
{
    imuMessage.timestamp = imuData.timestamp;
    imuMessage.sequence_number = imuData.sequenceNumber;
    imuMessage.time_delta = imuData.timeDelta;
    imuMessage.gravity = imuData.gravityMagnitude;

    for (int x = 3; --x >= 0;) {
        imuMessage.accel[x] = imuData.acceleration[x];
        imuMessage.rotational_vel[x] = imuData.rotationalVelocity[x];
        imuMessage.orientation[x] = imuData.orientation[x];
    }
}


void vulcan::lcm::convert_lcm_to_vulcan(const vulcan_lcm_imu_t& imuMessage, imu_data_t& imuData)
{
    imuData.timestamp = imuMessage.timestamp;
    imuData.sequenceNumber = imuMessage.sequence_number;
    imuData.timeDelta = imuMessage.time_delta;
    imuData.gravityMagnitude = imuMessage.gravity;

    for (int x = 3; --x >= 0;) {
        imuData.acceleration[x] = imuMessage.accel[x];
        imuData.rotationalVelocity[x] = imuMessage.rotational_vel[x];
        imuData.orientation[x] = imuMessage.orientation[x];
    }
}


void vulcan::lcm::publish_data(lcm_t* lcm, const imu_data_t& imuData, std::string channel)
{
    verify_channel(channel, IMU_DATA_CHANNEL, false);

    vulcan_lcm_imu_t imuMessage;   // no dynamic allocation here, so don't need to make static

    convert_vulcan_to_lcm(imuData, imuMessage);
    vulcan_lcm_imu_t_publish(lcm, channel.c_str(), &imuMessage);
}


void vulcan::lcm::subscribe_to_message(lcm_t* lcm,
                                       void (*callback)(const imu_data_t&, const std::string&, void*),
                                       void* userdata,
                                       std::string channel)
{
    verify_channel(channel, IMU_DATA_CHANNEL, true);

    channel_subscriber_t<imu_data_t> newSubscriber(channel, userdata, callback);

    if (!subscribers.isSubscribedToChannel(lcm, channel)) {
        subscribers.addChannelSubscriber(lcm, newSubscriber);

        vulcan_lcm_imu_t_subscribe(lcm,
                                   channel.c_str(),
                                   subscription_manager_callback<vulcan_lcm_imu_t, imu_data_t>,
                                   &subscribers);
    } else {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}
