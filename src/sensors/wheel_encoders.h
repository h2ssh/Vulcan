/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     wheel_encoders.h
 * \author   Collin Johnson
 *
 * Declaration of WheelEncoders abstract base class and create_wheel_encoders factory.
 */

#ifndef SENSORS_WHEEL_ENCODERS_H
#define SENSORS_WHEEL_ENCODERS_H

#include "core/odometry.h"
#include "sensors/odometry_estimator.h"
#include "sensors/wheel_encoders_params.h"
#include <memory>
#include <string>

namespace vulcan
{
namespace sensors
{

const std::string WHEEL_ENCODERS_TYPE("wheel_encoders");

class WheelEncoders;

/**
 * create_wheel_encoders creates an instance of wheel encoders based on the specified type of
 * encoder board.
 *
 * NOTE: If type is invalid, expect fireworks.
 *
 * \param    type            Type of board being used -- driver instance to instantiate
 * \param    params          Parameters for the encoders
 * \return   Desired implementation of WheelEncoders.
 */
std::unique_ptr<WheelEncoders> create_wheel_encoders(const std::string& type, const wheel_encoders_params_t& params);


/**
 * WheelEncoders is an abstract base class for reading wheel encoder data in the form of an
 * odometry_t struct. WheelEncoders abstracts away the ticks and reports the odometry in meters.
 *
 * Subclasses of WheelEncoders are responsible for getting the wheel tick counts, but the calculation
 * of the dead reckoning pose is done by the WheelEncoders base class.
 *
 * The configuration parameters for WheelEncoders are:
 *
 *   [WheelEncodersParameters]
 *   encoder_type                = type of encoder board being used, like the phidget encoders
 *   left_wheel_circumference_m  = distance of one wheel rotation
 *   right_wheel_circumference_m = distance of one wheel rotation, allow for measured wheel differences
 *   left_ticks_per_revolution   = number of ticks in one complete wheel rotation
 *   right_ticks_per_revolution  = number of ticks in one complete wheel rotation, allow for differences in hardware
 *   wheelbase_m                 = distance between the contact point of the two wheels, along the axis of rotation
 */
class WheelEncoders : public OdometryEstimator
{
public:
    /**
     * Constructor for WheelEncoders.
     *
     * \param    params          Parameters for the general odometry
     */
    WheelEncoders(const wheel_encoders_params_t& params);

    virtual ~WheelEncoders(void) { }

    /**
     * getEncoders gets an updated count on the cumulative numbers of ticks measured by
     * the wheel encoders.
     *
     * \return   The measured encoder data.
     */
    virtual encoder_data_t getEncoders(void) = 0;

    /**
     * resetTicks resets the tick counts for the encoder.
     */
    virtual void resetEncoders(void) = 0;

    // OdometryEstimator interface
    virtual void initialize(system::ModuleCommunicator& communicator);
    virtual odometry_t update(void);
    virtual void send(system::ModuleCommunicator& transmitter);

private:
    void calculateWheelMotion(void);         // update the deltaLeft and deltaRight in cumulative
    void calculateDeadReckoningPose(void);   // use deltaLeft and deltaRight to estimate the actual robot motion

    odometry_t cumulative;
    encoder_data_t currentEncoders;
    encoder_data_t previousEncoders;

    int64_t lastOutputTime;
    bool initialized;

    wheel_encoders_params_t params;
};

}   // namespace sensors
}   // namespace vulcan

#endif   // SENSORS_WHEEL_ENCODERS_H
