/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_HOKUYO_URG_LASER_H
#define SENSORS_HOKUYO_URG_LASER_H

#include <string>
#include <urg/UrgDevice.h>
#include <urg/RangeSensorParameter.h>
#include <sensors/laser_rangefinder.h>
#include <core/laser_scan.h>
#include <utils/sensor_time.h>

namespace vulcan
{
namespace sensors
{

/**
* HokuyoURGLaser is a driver for communicating with the series of Hokuyo laser rangefinders
* that use the URG library and SCIP protocol for communications. The two lasers used in the
* IRL lab, and consequently on Vulcan itself, are the UTM-30LX and the URG-04LX.
*
* The lasers communicate via a USB-to-Serial port that needs to be provided in the constructor.
* There are currently no settings that need to be modified. The class is instantiated and started
* and then beautiful laser data can begin being extracted via the getLaserScan() method.
*
* At the moment, HokuyoURGLaser does not run on its own thread. As such, a program using this class
* will need to process the data fast enough to keep pace with the data being sent by the sensor.
* The fastest laser that will be controlled by this class runs at 40Hz.
*/
class HokuyoURGLaser : public LaserRangefinder
{
public:

    /**
    * Constructor for HokuyoURGLaser.
    *
    * \param    port                Port on which the laser is connected (probably /dev/ttyACM** or /dev/ttyUSB**)
    * \param    id                  Id for the laser -- should be unique for the system
    * \param    withIntensity       Flag indicating whether or not the intensity data should be captured              (optional, default = true)
    * \param    verbose             Flag indicating if the laser should print information about itself as it connects (optional, default = true)
    */
    HokuyoURGLaser(const std::string& port,
                   int                id,
                   bool               withIntensity = true,
                   bool               verbose       = true);

    /**
    * Destructor for HokuyoURGLaser.
    */
    ~HokuyoURGLaser(void);

    // LaserRangefinder interface
    virtual std::string getSerialNumber(void) const;
    virtual int64_t     calculateLatency(void);
    virtual bool        startRangefinder(void);
    virtual void        stopRangefinder(void);
    virtual bool        getLaserScan(polar_laser_scan_t& scan);

    /**
    * isScanning checks to see if the laser is actively scanning the environment.
    *
    * \return   True if laser has been started. False otherwise.
    */
    bool isScanning();

private:

    mutable qrk::UrgDevice    laserController;          ///< Handles the low-level communications with the laser
    qrk::RangeSensorParameter laserParams;              ///< Parameters of the particular laser to which instance is connected
    qrk::RangeCaptureMode     captureMode;              ///< Capture mode being used for this instance of UrgCtrl

    std::vector<long> rawScanData;              ///< Data returned by the capture program
    std::vector<long> rawIntensityData;         ///< Data about the intensity of each ray
    int               id;                       ///< Id of the laser
    float             scanPeriod;               ///< Seconds per scan -- from first ray to last ray
    int64_t           laserLatencyUs;           ///< Latency between when the data was captured and when it was received
    long              lastLaserTimestamp;       ///< Timestamp of the most recent laser measurement
    
    utils::SensorTime time_;
};

}
}

#endif // SENSORS_HOKUYO_URG_LASER_H
