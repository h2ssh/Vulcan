/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     microstrain_3dmgx3.h
 * \author   Collin Johnson
 *
 * Declaration of Microstrain3DMGX3 driver for interfacing with the IMU
 * of the same name.
 */

#ifndef SENSORS_MICROSTRAIN_3DMGX3_H
#define SENSORS_MICROSTRAIN_3DMGX3_H

#include "sensors/imu.h"
#include "utils/condition_variable.h"
#include "utils/mutex.h"
#include "utils/sensor_time.h"
#include "utils/serial.h"
#include "utils/thread.h"
#include <vector>

namespace vulcan
{
namespace sensors
{

/**
 * Microstrain3DMGX3 is a driver for the Microstrain 3DM-GX3-35. The 3DM-GX3-35 is a combination of a
 * 3DM-GX3-25 and a GPS. The packet protocl is MIP, which allows for creating custom data packets rather
 * than the fixed types of data provided by the earlier protocol.
 */
class Microstrain3DMGX3
: public IMU
, public utils::Threadable
{
public:
    /**
     * Constructor for Microstrain3DMGX3.
     *
     * \param    port            Port on which to connect to the IMU
     */
    Microstrain3DMGX3(const std::string& port);

    virtual ~Microstrain3DMGX3(void);

    // IMU interface
    virtual bool startIMU(void);
    virtual void estimateGravityMagnitude(uint16_t numSamples);
    virtual bool getIMUData(imu_data_t& data);

private:
    enum imu_mode_t
    {
        DEVICE_INFO,
        WAIT_FOR_ACK,
        CONTINUOUS_DATA
    };

    struct ack_info_t
    {
        char set;
        std::vector<char> descriptors;
        std::vector<bool> acked;
    };

    // Setting up the 3DM-GX3 for continuous mode
    void initialize(void);
    void stopData(void);
    void readDeviceInfo(void);
    void calibrate(void);
    void setMessageFormat(void);

    // Threadable interface
    virtual int run(void);

    void startContinuousMode(void);

    void readFullPacket(void);
    void readAvailableData(void);
    bool bufferHasValidPacket(void);
    void processPacket();
    void erasePacketFromDataBuffer(void);

    void processDeviceInfo(char set, const char* packet, int length);

    void processAck(char set, const char* payload, int length);
    void verifyAck(char set, char descriptor, char status);

    void processContinuousData(char set, const char* payload, int length);
    void processAHRSData(const char* payload, int length);
    void processGPSData(const char* payload, int length);

    imu_mode_t mode;
    ack_info_t ack;

    imu_data_t imuData;
    uint32_t previousTimer;
    int64_t totalSensorTime;
    utils::SensorTime time;

    std::string dataBuffer;

    int packetStartIndex;
    int packetLength;

    utils::SerialConnection serial;

    utils::Thread readThread;
    utils::Mutex dataLock;
    utils::ConditionVariable dataTrigger;
};

}   // namespace sensors
}   // namespace vulcan

#endif   // SENSORS_MICROSTRAIN_3DMGX3_H
