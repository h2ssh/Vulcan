/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     laser_rangefinder.h
 * \author   Collin Johnson
 *
 * Declaration of the LaserRangefinder interface.
 */

#ifndef SENSORS_LASER_RANGEFINDER_H
#define SENSORS_LASER_RANGEFINDER_H

#include "core/laser_scan.h"
#include <string>

namespace vulcan
{
namespace sensors
{

/**
 * LaserRangefinder is an abstract base class defining the interface that is used
 * for acquiring laser scan data from a laser rangefinder.
 */
class LaserRangefinder
{
public:
    virtual ~LaserRangefinder(void) { }

    /**
     * getSerialNumber retrieves the serial number of the laser.
     *
     * \return   Serial number of the laser rangefinder as a string.
     */
    virtual std::string getSerialNumber(void) const = 0;

    /**
     * calculateLatency calculates the latency between when the scan was taken and when it was received by the program.
     * The latency will be used internally to adjust the timestamp of the collected laser data to make it more accurate.
     *
     * \return   The latency calculated for the laser scans.
     */
    virtual int64_t calculateLatency(void) = 0;

    /**
     * startRangefinder must perform any initialization necessary for opening a data
     * stream from the rangefinder to the computer. startSensor() will be called
     * before any calls to getLaserScan().
     *
     * \return   True if the sensor is ready for data acquisition. False otherwise.
     */
    virtual bool startRangefinder(void) = 0;

    /**
     * stopRangefinder issues the commands necessary to shutdown the rangefinder
     * in a sensible manner.
     */
    virtual void stopRangefinder(void) = 0;

    /**
     * getLaserScan provides the caller with the most recent scan data available.
     * Two consecutive calls to getLaserScan() must return two different scans. The
     * call can block if a new scan is not immediately available.
     *
     * \param    scan        laser_scan_t instance to be filled with new data (output)
     * \return   True if a new scan was read. False otherwise.
     */
    virtual bool getLaserScan(polar_laser_scan_t& scan) = 0;
};


}   // namespace sensors
}   // namespace vulcan

#endif   // SENSORS_LASER_RANGEFINDER_H
