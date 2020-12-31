/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef SENSORS_MICROSTRAIN_3DMGX2_H
#define SENSORS_MICROSTRAIN_3DMGX2_H

#include <stdint.h>
#include <string>
#include "utils/thread.h"
#include "utils/mutex.h"
#include "utils/condition_variable.h"
#include "utils/sensor_time.h"
#include "utils/serial.h"
#include "sensors/imu.h"
#include "core/imu_data.h"

namespace vulcan
{
namespace sensors
{

/**
* microstrain_3DMGX2_temperature_t holds the measured temperatures from the 3DM-GX2.
* The temperatures that are measured the accelerometer and gyroscopes on each axis.
*/
struct microstrain_3DMGX2_temperature_t
{
    float accelerometer;
    float gyroXAxis;
    float gyroYAxis;
    float gyroZAxis;
};

/**
* Microstrain3DMGX2 is a driver for communicating with the Microstrain 3DM-GX2 IMU.
*/
class Microstrain3DMGX2 : public IMU,
                          public utils::Threadable
{
public:

    /**
    * Constructor for Microstrain3DMGX2.
    *
    * \param    device          Port on which to open the IMu
    * \param    ticksPerSecond  Number of IMU timer ticks per second (optional, default =
    *   number of ticks per second for a GX2. If using a GX3, then specify 16000000)
    */
    Microstrain3DMGX2(const std::string& device,
                      int64_t ticksPerSecond = 19660800);

    /** Destructor. */
    ~Microstrain3DMGX2(void);

    /**
    * startIMU performs the initialization necessary to begin reading data from
    * the IMU.
    */
    virtual bool startIMU(void);

    /**
    * estimateGravityMagnitude estimates the magnitude of the acceleration that
    * is measured due to gravity. This method MUST only be run while the robot is
    * idle.
    */
    virtual void estimateGravityMagnitude(uint16_t numSamples);

    /**
    * getIMUData reads the most recent set of IMU data sent across the serial line. This
    * call blocks until the data is received.
    */
    virtual bool getIMUData(imu_data_t& data);

    /**
    * getVersion gets the version number information for the Microstrain3DMGX2 being used.
    *
    * \return   The version information for the Microstrain3DMGX2. 0 on error.
    */
    uint32_t getVersion(void);

    /**
    * getSerialNumber gets the serial number of the Microstrain3DMGX2.
    *
    * \return   16-bit integer that is the serial number of the Microstrain3DMGX2.  0 represents an error.
    */
    std::string getSerialNumber(void);

    /**
    * getTemperature finds the current operating temperature of the Microstrain3DMGX2.
    *
    * \return   Operating temperature of the Microstrain3DMGX2 in degrees Celcius.
    */
    microstrain_3DMGX2_temperature_t getTemperature(void);

    /**
    * captureBias captures the current gyro-bias for the Microstrain3DMGX2. Capturing the bias for the Microstrain3DMGX2 makes good things happen whenever the
    * robot has decided to enter the magical, spin-in-place like Thelonius Monk thing. At the same time, the yaw reading from the
    * Microstrain3DMGX2 will jump to the 'correct' value which could cause problems with odometry. At the least, the bias should be captured when
    * the robot is booting up. It should only be called when the robot is stationary though.
    *
    * \return   True if the bias was successfully captured, false otherwise.
    */
    bool captureBias(uint16_t sampleTimeMs);

private:

    // Methods for processing IMU data
    bool        initialize(void);
    bool        establishSerialConnection(void);
    void        startContinuousMode(char command = '\xCC');
    void        stopContinuousMode(void);
    std::string readDeviceIdentifierString(char command);

    // Methods for the processing loop
    int        run(void);  // Threadable interface
    size_t     readAvailableData(void);
    int        activePacketStartInBuffer(void);
    imu_data_t parseActivePacket(int startByte);

    imu_data_t   currentData;

    unsigned int firmwareVersion;               ///< Version info for the IMU
    std::string  modelNumber;
    std::string  serialNumber;
    std::string  modelName;

    double secondsPerTick_;
    uint32_t previousTimer;
    int64_t totalSensorTime_;
    utils::SensorTime time_;

    float        gravityMagnitude;
    bool         continuousModeActive;          ///< Sensor is currently in continuous mode

    utils::SerialConnection serial;           ///< Serial connection to be used in speaking with the IMU
    std::string               dataBuffer;       ///< Buffer of IMU data that will eventually become a complete packet

    char         activeCommand;
    unsigned int activeResponseLength;

    utils::Thread            readThread;
    utils::Mutex             dataLock;
    utils::ConditionVariable dataTrigger;
};

}
}

#endif  // SENSORS_MICROSTRAIN_3DMGX2_H
