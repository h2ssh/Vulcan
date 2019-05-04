/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     lpm_panel_text_updater.cpp
* \author   Collin Johnson
*
* Definition of LocalMetricPanelTextUpdater.
*/

#include <ui/debug/local_metric_panel_text_updater.h>
#include <utils/auto_mutex.h>
#include <core/imu_data.h>
#include <robot/commands.h>
#include <core/pose.h>
#include <core/velocity.h>
#include <core/motion_state.h>
#include <wx/wx.h>
#include <iomanip>
#include <ios>
#include <sstream>


namespace vulcan
{
namespace ui
{

// Set floats that are almost zero to zero for display purposes
float set_to_zero_if_needed(float value);


LocalMetricPanelTextUpdater::LocalMetricPanelTextUpdater(const local_metric_panel_text_widgets_t& widgets)
: textWidgets_(widgets)
{
    assert(textWidgets_.pose);
    assert(textWidgets_.velocity);
    assert(textWidgets_.command);
    assert(textWidgets_.imuAccel);
    assert(textWidgets_.imuVelocity);
    assert(textWidgets_.imuOrientation);
    assert(textWidgets_.leftWheel);
    assert(textWidgets_.rightWheel);
    
    poseString_           = textWidgets_.pose->GetLabel();
    velocityString_       = textWidgets_.velocity->GetLabel();
    commandString_        = textWidgets_.command->GetLabel();
    imuAccelString_       = textWidgets_.imuAccel->GetLabel();
    imuVelocityString_    = textWidgets_.imuVelocity->GetLabel();
    imuOrientationString_ = textWidgets_.imuOrientation->GetLabel();
    leftWheelString_      = textWidgets_.leftWheel->GetLabel();
    rightWheelString_     = textWidgets_.rightWheel->GetLabel();
}


void LocalMetricPanelTextUpdater::update(void)
{
    utils::AutoMutex autoLock(displayLock_);

    textWidgets_.pose->SetLabel(poseString_);
    textWidgets_.velocity->SetLabel(velocityString_);
    textWidgets_.command->SetLabel(commandString_);
    textWidgets_.imuAccel->SetLabel(imuAccelString_);
    textWidgets_.imuVelocity->SetLabel(imuVelocityString_);
    textWidgets_.imuOrientation->SetLabel(imuOrientationString_);
    textWidgets_.leftWheel->SetLabel(leftWheelString_);
    textWidgets_.rightWheel->SetLabel(rightWheelString_);
}


void LocalMetricPanelTextUpdater::handleData(const imu_data_t& imuData, const std::string& channel)
{
    utils::AutoMutex autoLock(displayLock_);

    std::ostringstream accelOut;

    accelOut << std::fixed << std::showpos << std::setprecision(2) << '(' << imuData.acceleration[IMU_X_INDEX]
                                                                   << ',' << imuData.acceleration[IMU_Y_INDEX]
                                                                   << ',' << std::setw(6) << imuData.acceleration[IMU_Z_INDEX] << ')';
    imuAccelString_ = wxString(accelOut.str().c_str(), wxConvUTF8);
    
    std::ostringstream velocityOut;
    velocityOut << std::fixed << std::showpos << std::setprecision(2) << '(' << imuData.rotationalVelocity[IMU_ROLL_INDEX] 
                                                                      << ',' << imuData.rotationalVelocity[IMU_PITCH_INDEX]
                                                                      << ',' << imuData.rotationalVelocity[IMU_YAW_INDEX] << ')';
    imuVelocityString_ = wxString(velocityOut.str().c_str(), wxConvUTF8);
    
    std::ostringstream orientationOut;
    orientationOut << std::fixed << std::showpos << std::setprecision(2) << '(' << imuData.orientation[IMU_ROLL_INDEX] 
                                                                         << ',' << imuData.orientation[IMU_PITCH_INDEX]
                                                                         << ',' << imuData.orientation[IMU_YAW_INDEX] << ')';
    imuOrientationString_ = wxString(orientationOut.str().c_str(), wxConvUTF8);
}


void LocalMetricPanelTextUpdater::handleData(const motion_state_t& motion, const std::string& channel)
{
    utils::AutoMutex autoLock(displayLock_);

    std::ostringstream poseOut;

    // Show very small values as 0
    poseOut << std::fixed << std::showpos << std::setprecision(2) << '(' << set_to_zero_if_needed(motion.pose.x)
                                                                  << ',' << set_to_zero_if_needed(motion.pose.y)
                                                                  << ',' << set_to_zero_if_needed(motion.pose.theta) << ')';
    poseString_ = wxString(poseOut.str().c_str(), wxConvUTF8);
    
    std::ostringstream velocityOut;
    velocityOut << std::fixed << std::showpos << std::setprecision(2) << '(' << set_to_zero_if_needed(motion.velocity.linear)
                                                                      << ',' << set_to_zero_if_needed(motion.velocity.angular) << ')';
    velocityString_ = wxString(velocityOut.str().c_str(), wxConvUTF8);
    
    std::ostringstream leftOut;
    leftOut << std::fixed << std::showpos << std::setprecision(2) << '(' << set_to_zero_if_needed(motion.differentialWheels.leftWheel.speed)
                                                                  << ',' << set_to_zero_if_needed(motion.differentialWheels.leftWheel.motorAccel) << ')';
    leftWheelString_ = wxString(leftOut.str().c_str(), wxConvUTF8);
    
    std::ostringstream rightOut;
    rightOut << std::fixed << std::showpos << std::setprecision(2) << '(' << set_to_zero_if_needed(motion.differentialWheels.rightWheel.speed)
                                                                   << ',' << set_to_zero_if_needed(motion.differentialWheels.rightWheel.motorAccel) << ')';
    rightWheelString_ = wxString(rightOut.str().c_str(), wxConvUTF8);
}


void LocalMetricPanelTextUpdater::handleData(const robot::commanded_velocity_t& command, const std::string& channel)
{
    utils::AutoMutex autoLock(displayLock_);

    std::ostringstream out;

    out << std::fixed << std::showpos << std::setprecision(2) << '(' << command.linearVelocity << ',' << command.angularVelocity << ')';

    commandString_ = wxString(out.str().c_str(), wxConvUTF8);
}


// Set floats that are almost zero to zero for display purposes
float set_to_zero_if_needed(float value)
{
    if(fabs(value) < 0.0001)
    {
        value = 0.0f;
    }

    return value;
}

} // namespace ui
} // namespace vulcan
