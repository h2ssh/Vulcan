/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_IMU_H
#define SENSORS_IMU_H

#include "core/imu_data.h"

namespace vulcan
{
namespace sensors
{

/**
 * IMU is an abstract base class defining the interface to be used for
 * communicating with any sort of three-axis IMU, like the Microstrain 3DM-GX series.
 *
 * The interface for communication is as follows:
 *
 *   startSensor() -- do any initialization needed to open a data stream to the
 *                    sensor
 *   getIMUData()  -- obtain the most recent piece of IMU data provided
 *                    by the sensor
 */
class IMU
{
public:
    virtual ~IMU(void) { }

    /**
     * startIMU must perform any initialization necessary for IMU data to be
     * sent to the computer.
     *
     * \return   True if the sensor starts successfully. False otherwise.
     */
    virtual bool startIMU(void) = 0;

    /**
     * estimateGravityMagnitude estimates the magnitude of the acceleration that
     * is measured due to gravity. This method MUST only be run while the robot is
     * idle, otherwise the estimate will be gravely wrong and any attempts at gravity
     * compensation will result in calamity.
     *
     * \param    numSamples          Number of samples to capture for the estimate
     */
    virtual void estimateGravityMagnitude(uint16_t numSamples) = 0;

    /**
     * getIMUData provides the caller with the most recent IMU data available.
     * Two successive calls to getIMUData() must return different readings.
     * The call to getIMUData() can block if that is needed to ensure fresh
     * data is returned.
     *
     * \param    data        imu_data_t instance filled with the most recent IMU data
     * \return   True if fresh data was able to be read. False otherwise.
     */
    virtual bool getIMUData(imu_data_t& data) = 0;
};

}   // namespace sensors
}   // namespace vulcan

#endif   // SENSORS_IMU_H
