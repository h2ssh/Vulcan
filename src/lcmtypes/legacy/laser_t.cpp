/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "lcmtypes/legacy/laser_t.h"
#include "lcmtypes/subscription_manager.h"
#include "lcmtypes/message_helpers.h"
#include <vector>

namespace vulcan
{
namespace lcm
{

const std::vector<std::string> OLD_LASER_SCAN_CHANNELS = {"SENSOR_FRONT_LASER", "SENSOR_BACK_LASER"};
const std::vector<std::string> THREE_DOF_LASER_SCAN_CHANNELS = {"SENSOR_LASER_FRONT", "SENSOR_LASER_BACK"};

static SubscriptionManager<vulcan_lcm_laser_t,                polar_laser_scan_t> subscribers;
static SubscriptionManager<vulcan_lcm_laser_3dof_t,           polar_laser_scan_t> threeDofSubscribers;
static SubscriptionManager<vulcan_lcm_laser_old_t,            polar_laser_scan_t> oldSubscribers;
static SubscriptionManager<vulcan_lcm_laser_with_intensity_t, polar_laser_scan_t> intensitySubscribers;

void subscribe_to_channel     (lcm_t* lcm, void (*callback)(const polar_laser_scan_t&, const std::string&, void*), void* userdata, const std::string& channel);
void subscribe_to_old_channels(lcm_t* lcm, void (*callback)(const polar_laser_scan_t&, const std::string&, void*), void* userdata, bool front);


void convert_lcm_to_vulcan(const vulcan_lcm_laser_t& laserMessage, polar_laser_scan_t& scan)
{
    scan.laserId           = laserMessage.laser_id;
    scan.timestamp         = laserMessage.timestamp;
    scan.scanId            = laserMessage.scan_id;
    scan.startAngle        = laserMessage.start_angle;
    scan.angularResolution = laserMessage.angle_increment;
    scan.numRanges         = laserMessage.num_ranges;
    scan.maxRange          = laserMessage.max_range;
    scan.scanPeriod        = laserMessage.scan_period;

    scan.offset.x     = laserMessage.offset_x;
    scan.offset.y     = laserMessage.offset_y;
    scan.offset.z     = laserMessage.offset_z;
    scan.offset.theta = laserMessage.offset_yaw;
    scan.offset.phi   = laserMessage.offset_pitch;
    scan.offset.rho   = laserMessage.offset_roll;

    scan.ranges.resize(laserMessage.num_ranges);
    scan.intensities.resize(laserMessage.num_ranges);

    std::copy(laserMessage.ranges,      laserMessage.ranges+laserMessage.num_ranges,      scan.ranges.begin());
    std::copy(laserMessage.intensities, laserMessage.intensities+laserMessage.num_ranges, scan.intensities.begin());
}


void convert_vulcan_to_lcm(const polar_laser_scan_t& scan, vulcan_lcm_laser_t& laserMessage)
{
    if(laserMessage.num_ranges < scan.numRanges)
    {
        if(laserMessage.ranges)
        {
            delete [] laserMessage.ranges;
        }

        laserMessage.num_ranges  = scan.numRanges;
        laserMessage.ranges      = new float[laserMessage.num_ranges];
        laserMessage.intensities = new int16_t[laserMessage.num_ranges];
    }

    laserMessage.laser_id        = scan.laserId;
    laserMessage.timestamp       = scan.timestamp;
    laserMessage.scan_id         = scan.scanId;
    laserMessage.start_angle     = scan.startAngle;
    laserMessage.angle_increment = scan.angularResolution;
    laserMessage.num_ranges      = scan.numRanges;
    laserMessage.max_range       = scan.maxRange;
    laserMessage.scan_period     = scan.scanPeriod;

    laserMessage.offset_x = scan.offset.x;
    laserMessage.offset_y = scan.offset.y;
    laserMessage.offset_z = scan.offset.z;
    laserMessage.offset_yaw = scan.offset.theta;
    laserMessage.offset_pitch = scan.offset.phi;
    laserMessage.offset_roll = scan.offset.rho;

    std::copy(scan.ranges.begin(), scan.ranges.end(), laserMessage.ranges);

    if(!scan.intensities.empty())
    {
        std::copy(scan.intensities.begin(), scan.intensities.end(), laserMessage.intensities);
    }
    else
    {
        memset(laserMessage.intensities, 0, scan.numRanges*sizeof(int16_t));
    }
}


void convert_lcm_to_vulcan(const vulcan_lcm_laser_3dof_t& laserMessage, polar_laser_scan_t& scan)
{
    scan.laserId           = laserMessage.laser_id;
    scan.timestamp         = laserMessage.timestamp;
    scan.scanId            = laserMessage.scan_id;
    scan.startAngle        = laserMessage.start_angle;
    scan.angularResolution = laserMessage.angle_increment;
    scan.numRanges         = laserMessage.num_ranges;
    scan.maxRange          = laserMessage.max_range;
    scan.scanPeriod        = laserMessage.scan_period;

    scan.offset.x     = laserMessage.offset_x;
    scan.offset.y     = laserMessage.offset_y;
    scan.offset.theta = laserMessage.offset_theta;

    scan.ranges.resize(laserMessage.num_ranges);
    scan.intensities.resize(laserMessage.num_ranges);

    std::copy(laserMessage.ranges,      laserMessage.ranges+laserMessage.num_ranges,      scan.ranges.begin());
    std::copy(laserMessage.intensities, laserMessage.intensities+laserMessage.num_ranges, scan.intensities.begin());
}


void convert_lcm_to_vulcan(const vulcan_lcm_laser_old_t& laserMessage, polar_laser_scan_t& laserScan)
{
    // For the old messages, if offset.x is less than 0, the laser is on the back of the robot, so just hard-code that for the old datasets
    laserScan.laserId = (laserMessage.offset.x > 0) ? kFrontLaserId : kBackLaserId;

    laserScan.timestamp         = laserMessage.timestamp;
    laserScan.scanId            = laserMessage.scan_id;
    laserScan.startAngle        = laserMessage.start_angle;
    laserScan.angularResolution = laserMessage.angle_increment;
    laserScan.numRanges         = laserMessage.num_ranges;
    laserScan.maxRange          = laserMessage.max_range;
    laserScan.scanPeriod        = 0.019;            // this value doesn't exist in the old logs, so set it to the value for the UTM-30LX

    laserScan.offset.x     = laserMessage.offset.x;
    laserScan.offset.y     = laserMessage.offset.y;
    laserScan.offset.theta = laserMessage.offset.theta;

    laserScan.ranges.resize(laserMessage.num_ranges);
    std::copy(laserMessage.ranges, laserMessage.ranges+laserMessage.num_ranges, laserScan.ranges.begin());
}


void convert_lcm_to_vulcan(const vulcan_lcm_laser_with_intensity_t& laserMessage, polar_laser_scan_t& scan)
{
    if(laserMessage.num_intensities)
    {
        scan.intensities.resize(laserMessage.num_intensities);
        std::copy(laserMessage.intensities, laserMessage.intensities+laserMessage.num_intensities, scan.intensities.begin());
    }
    else
    {
        scan.intensities.clear();
    }

    convert_lcm_to_vulcan(laserMessage.laser, scan);
}


void publish_data(lcm_t* lcm, const polar_laser_scan_t& scan, std::string channel)
{
    verify_channel(channel, LASER_SCAN_CHANNELS, false);

    vulcan_lcm_laser_t laserMessage;
    laserMessage.num_ranges  = 0;
    laserMessage.ranges      = 0;
    laserMessage.intensities = 0;

    initialize_laser_message(laserMessage);
    convert_vulcan_to_lcm(scan, laserMessage);
    vulcan_lcm_laser_t_publish(lcm, channel.c_str(), &laserMessage);
    free_laser_message(laserMessage);
}


void subscribe_to_message(lcm_t* lcm, void (*callback)(const polar_laser_scan_t&, const std::string&, void*), void* userdata, std::string channel)
{
    if(channel.empty())
    {
        for(auto& defaultChannel : LASER_SCAN_CHANNELS)
        {
            subscribe_to_channel(lcm, callback, userdata, defaultChannel);
        }

        subscribe_to_old_channels(lcm, callback, userdata, false);
        subscribe_to_old_channels(lcm, callback, userdata, true);
    }
    else
    {
        subscribe_to_channel(lcm, callback, userdata, channel);

        if(channel == LASER_SCAN_CHANNELS[0])
        {
            subscribe_to_old_channels(lcm, callback, userdata, true);
        }
        else if(channel == LASER_SCAN_CHANNELS[1])
        {
            subscribe_to_old_channels(lcm, callback, userdata, false);
        }
    }
}


void initialize_laser_message(vulcan_lcm_laser_t& laserMessage)
{
    laserMessage.num_ranges  = 0;
    laserMessage.ranges      = 0;
    laserMessage.intensities = 0;
}


void free_laser_message(vulcan_lcm_laser_t& laserMessage)
{
    delete [] laserMessage.intensities;
    delete [] laserMessage.ranges;
}


void subscribe_to_channel(lcm_t* lcm, void (*callback)(const polar_laser_scan_t&, const std::string&, void*), void* userdata, const std::string& channel)
{
    channel_subscriber_t<polar_laser_scan_t> newSubscriber(channel, userdata, callback);

    if(!subscribers.isSubscribedToChannel(lcm, channel))
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
        vulcan_lcm_laser_t_subscribe(lcm, channel.c_str(), subscription_manager_callback<vulcan_lcm_laser_t, polar_laser_scan_t>, &subscribers);
    }
    else
    {
        subscribers.addChannelSubscriber(lcm, newSubscriber);
    }
}


void subscribe_to_old_channels(lcm_t* lcm, void (*callback)(const polar_laser_scan_t&, const std::string&, void*), void* userdata, bool front)
{
    std::string channel          = front ? OLD_LASER_SCAN_CHANNELS[0] : OLD_LASER_SCAN_CHANNELS[1];
    std::string threeDofChannel  = front ? THREE_DOF_LASER_SCAN_CHANNELS[0] : THREE_DOF_LASER_SCAN_CHANNELS[1];
    std::string intensityChannel = channel + "_INTENSITY";

    channel_subscriber_t<polar_laser_scan_t> threeDofSubscriber (threeDofChannel, userdata, callback);
    channel_subscriber_t<polar_laser_scan_t> newSubscriber      (channel, userdata, callback);
    channel_subscriber_t<polar_laser_scan_t> intensitySubscriber(intensityChannel, userdata, callback);

    if(!oldSubscribers.isSubscribedToChannel(lcm, channel))
    {
        oldSubscribers.addChannelSubscriber(lcm, newSubscriber);
        vulcan_lcm_laser_old_t_subscribe(lcm, channel.c_str(), subscription_manager_callback<vulcan_lcm_laser_old_t, polar_laser_scan_t>, &oldSubscribers);
    }
    else
    {
        oldSubscribers.addChannelSubscriber(lcm, newSubscriber);
    }

    if(!threeDofSubscribers.isSubscribedToChannel(lcm, threeDofChannel))
    {
        threeDofSubscribers.addChannelSubscriber(lcm, threeDofSubscriber);
        vulcan_lcm_laser_3dof_t_subscribe(lcm, threeDofChannel.c_str(), subscription_manager_callback<vulcan_lcm_laser_3dof_t, polar_laser_scan_t>, &threeDofSubscribers);
    }
    else
    {
        threeDofSubscribers.addChannelSubscriber(lcm, threeDofSubscriber);
    }

    if(!intensitySubscribers.isSubscribedToChannel(lcm, intensityChannel))
    {
        intensitySubscribers.addChannelSubscriber(lcm, intensitySubscriber);
        vulcan_lcm_laser_with_intensity_t_subscribe(lcm, intensityChannel.c_str(), subscription_manager_callback<vulcan_lcm_laser_with_intensity_t, polar_laser_scan_t>, &intensitySubscribers);
    }
    else
    {
        intensitySubscribers.addChannelSubscriber(lcm, intensitySubscriber);
    }
}

}
}
