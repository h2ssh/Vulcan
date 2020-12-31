/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
 * \file     microstrain_3dmgx3.cpp
 * \author   Collin Johnson
 *
 * Definition of Microstrain3DMGX3.
 */

#include "sensors/microstrain_3dmgx3.h"
#include "utils/auto_mutex.h"
#include "utils/byte_conversion.h"
#include "utils/timestamp.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

//#define DEBUG_PACKET
//#define DEBUG_CHECKSUM

namespace vulcan
{
namespace sensors
{

const char SYNC_BYTE_1 = '\x75';
const char SYNC_BYTE_2 = '\x65';

const char COMMAND_DESCRIPTOR_SET = '\x0C';
const char SCALED_ACCEL_DESCRIPTOR = '\x04';
const char SCALED_GYRO_DESCRIPTOR = '\x05';
const char EULER_DESCRIPTOR = '\x0C';
const char TIME_DESCRIPTOR = '\x0E';

const double GRAVITY_CONSTANT_3DM_GX3 = 9.80665;

bool valid_checksum(const char* data, int packetLength);
uint16_t calculate_checksum(const char* data, int packetLength);
void parse_scaled_accelerometer(const char* data, imu_data_t& imu);
void parse_scaled_gyro(const char* data, imu_data_t& imu);
void parse_euler_angles(const char* data, imu_data_t& imu);
uint32_t parse_internal_timestamp(const char* data, uint32_t previousTimer, imu_data_t& imu);


Microstrain3DMGX3::Microstrain3DMGX3(const std::string& port)
: previousTimer(0)
, totalSensorTime(0)
, serial(port, utils::BAUD_115200)
{
    initialize();
}


Microstrain3DMGX3::~Microstrain3DMGX3(void)
{
}


bool Microstrain3DMGX3::startIMU(void)
{
    if (!readThread.isRunning()) {
        readThread.attachTask(this);
        readThread.start();
    }

    return true;
}


void Microstrain3DMGX3::estimateGravityMagnitude(uint16_t numSamples)
{
    imu_data_t data;

    float accelSum[3] = {0.0f, 0.0f, 0.0f};

    for (uint16_t samplesTaken = 0; samplesTaken < numSamples; ++samplesTaken) {
        getIMUData(data);

        accelSum[0] += data.acceleration[0];
        accelSum[1] += data.acceleration[1];
        accelSum[2] += data.acceleration[2];
    }

    accelSum[0] /= numSamples;
    accelSum[1] /= numSamples;
    accelSum[2] /= numSamples;

    imuData.gravityMagnitude = sqrt(pow(accelSum[0], 2) + pow(accelSum[1], 2) + pow(accelSum[2], 2));

    std::cout << "Microstrain 3DM-GX3: Gravity magnitude:" << imuData.gravityMagnitude << '\n';
}


bool Microstrain3DMGX3::getIMUData(imu_data_t& data)
{
    dataTrigger.wait();

    utils::AutoMutex autoLock(dataLock);

    data = imuData;

    dataTrigger.setPredicate(false);

    return true;
}


void Microstrain3DMGX3::initialize(void)
{
    stopData();
    readDeviceInfo();
    calibrate();
    setMessageFormat();
}


void Microstrain3DMGX3::stopData(void)
{
    // Using the message pulled from the 3DM-GX-35-Data-Commuications-Protocol.pdf
    const char* stopMessage = "\x75\x65\x0C\x0A\x05\x11\x01\x01\x00\x05\x11\x01\x02\x00\x21\xC3";

    mode = WAIT_FOR_ACK;

    ack.set = '\x0C';
    ack.descriptors.clear();
    ack.descriptors.push_back('\x11');
    ack.descriptors.push_back('\x11');
    ack.acked.clear();
    ack.acked.push_back(false);
    ack.acked.push_back(false);

    serial.write(stopMessage, 16);

    readFullPacket();
    processPacket();
}


void Microstrain3DMGX3::readDeviceInfo(void)
{
    // Ignore for now
}


void Microstrain3DMGX3::calibrate(void)
{
    // Ignore for now
}


void Microstrain3DMGX3::setMessageFormat(void)
{
    const char DECIMATE_TO_100HZ = '\x0A';

    mode = WAIT_FOR_ACK;
    ack.set = '\x0C';
    ack.descriptors.clear();
    ack.descriptors.push_back('\x08');
    ack.acked.clear();
    ack.acked.push_back(false);

    char formatMessage[22];
    formatMessage[0] = SYNC_BYTE_1;
    formatMessage[1] = SYNC_BYTE_2;
    formatMessage[2] = COMMAND_DESCRIPTOR_SET;
    formatMessage[3] = '\x10';

    // AHRS Message Format (0x0C, 0x08)
    formatMessage[4] = '\x10';
    formatMessage[5] = '\x08';
    formatMessage[6] = '\x01';
    formatMessage[7] = '\x04';

    formatMessage[8] = SCALED_ACCEL_DESCRIPTOR;
    formatMessage[9] = '\x00';
    formatMessage[10] = DECIMATE_TO_100HZ;

    formatMessage[11] = SCALED_GYRO_DESCRIPTOR;
    formatMessage[12] = '\x00';
    formatMessage[13] = DECIMATE_TO_100HZ;

    formatMessage[14] = EULER_DESCRIPTOR;
    formatMessage[15] = '\x00';
    formatMessage[16] = DECIMATE_TO_100HZ;

    formatMessage[17] = TIME_DESCRIPTOR;
    formatMessage[18] = 0X00;
    formatMessage[19] = DECIMATE_TO_100HZ;

    uint16_t checksum = calculate_checksum(formatMessage, 20);

    formatMessage[20] = (checksum & 0xFF00) >> 8;
    formatMessage[21] = (checksum & 0x00FF);

    serial.write(formatMessage, 22);

    readFullPacket();
    processPacket();
}


int Microstrain3DMGX3::run(void)
{
    startContinuousMode();

    while (true) {
        readFullPacket();

        dataLock.lock();
        processPacket();
        dataLock.unlock();

        erasePacketFromDataBuffer();

        dataTrigger.setPredicate(true);
        dataTrigger.broadcast();
    }

    return 0;
}


void Microstrain3DMGX3::startContinuousMode(void)
{
    const char* continuousMessage = "\x75\x65\x0C\x05\x05\x11\x01\x01\x01\x04\x1A";

    mode = WAIT_FOR_ACK;

    ack.set = '\x0C';
    ack.descriptors.clear();
    ack.descriptors.push_back('\x11');
    ack.acked.clear();
    ack.acked.push_back(false);

    serial.write(continuousMessage, 11);

    readFullPacket();
    processPacket();

    mode = CONTINUOUS_DATA;
}


void Microstrain3DMGX3::readFullPacket(void)
{
    while (!bufferHasValidPacket()) {
        readAvailableData();

#ifdef DEBUG_PACKET
        std::cout << "No packet found. Buffer length:" << dataBuffer.length() << '\n';
#endif
    }
}


void Microstrain3DMGX3::readAvailableData(void)
{
    char* availableData = 0;
    int readLength = 0;

    availableData = serial.read(availableData, readLength);

    dataBuffer.append(availableData, readLength);

    delete[] availableData;
}


bool Microstrain3DMGX3::bufferHasValidPacket(void)
{
    const size_t MIN_PACKET_LENGTH = 6;   // header - 4 bytes, checksum - 2 bytes
    const int PAYLOAD_LENGTH_OFFSET = 3;

    if (dataBuffer.length() < MIN_PACKET_LENGTH) {
        return false;
    }

    int maxPacketStartIndex = dataBuffer.length() - MIN_PACKET_LENGTH;

    for (int n = 0; n < maxPacketStartIndex; ++n) {
        size_t payloadLength = dataBuffer[n + PAYLOAD_LENGTH_OFFSET];
        if ((dataBuffer[n] == SYNC_BYTE_1) && (dataBuffer[n + 1] == SYNC_BYTE_2)
            && (dataBuffer.length() - n >= payloadLength + MIN_PACKET_LENGTH)
            && valid_checksum(dataBuffer.c_str() + n, payloadLength + MIN_PACKET_LENGTH)) {
            packetStartIndex = n;
            packetLength = payloadLength + MIN_PACKET_LENGTH;
            return true;
        }
    }

    return false;
}


void Microstrain3DMGX3::processPacket(void)
{
    const int SET_INDEX = 2;
    const int PACKET_HEADER_LENGTH = 4;
    const int PACKET_CHECKSUM_LENGTH = 2;

    const int FIELD_LENGTH_INDEX = 0;

    const char* packet = dataBuffer.c_str() + packetStartIndex;
    int length = packetLength;

    char set = packet[SET_INDEX];
    const char* payload = packet + PACKET_HEADER_LENGTH;

    length -= PACKET_HEADER_LENGTH + PACKET_CHECKSUM_LENGTH;

    while (length > 0) {
        int payloadLength = payload[FIELD_LENGTH_INDEX];

        assert(payloadLength <= length);

        switch (mode) {
        case DEVICE_INFO:
            processDeviceInfo(set, payload, payloadLength);
            break;

        case WAIT_FOR_ACK:
            processAck(set, payload, payloadLength);
            break;

        case CONTINUOUS_DATA:
            processContinuousData(set, payload, payloadLength);
            break;
        }

        length -= payloadLength;
        payload += payloadLength;
    }

    erasePacketFromDataBuffer();
}


void Microstrain3DMGX3::erasePacketFromDataBuffer(void)
{
    dataBuffer.erase(0, packetStartIndex + packetLength);
}


void Microstrain3DMGX3::processDeviceInfo(char set, const char* packet, int length)
{
}


void Microstrain3DMGX3::processAck(char set, const char* payload, int length)
{
    const char ACK_KEY = '\xF1';

    const int ACK_DESCRIPTOR_OFFSET = 1;
    const int COMMAND_DESCRIPTOR_OFFSET = 2;
    const int ERROR_CODE_OFFSET = 3;

    assert(length >= ERROR_CODE_OFFSET);

    if (payload[ACK_DESCRIPTOR_OFFSET] == ACK_KEY) {
        verifyAck(set, payload[COMMAND_DESCRIPTOR_OFFSET], payload[ERROR_CODE_OFFSET]);
    } else {
        std::cerr << "ERROR:Microstrain3DMGX3: Was expecting ACK " << (int)ACK_KEY << " but got "
                  << (int)payload[ACK_DESCRIPTOR_OFFSET] << " instead.\n";
    }
}


void Microstrain3DMGX3::verifyAck(char set, char descriptor, char status)
{
    if (ack.set == set) {
        for (size_t n = 0; n < ack.descriptors.size(); ++n) {
            if ((ack.descriptors[n] == descriptor) && !ack.acked[n]) {
                ack.acked[n] = true;
            }
        }
    }
}


void Microstrain3DMGX3::processContinuousData(char set, const char* payload, int length)
{
    const char AHRS_DATA = '\x80';
    const char GPS_DATA = '\x81';

    switch (set) {
    case AHRS_DATA:
        processAHRSData(payload, length);
        break;

    case GPS_DATA:
        processGPSData(payload, length);
        break;
    }
}


void Microstrain3DMGX3::processAHRSData(const char* payload, int length)
{
    const int DESCRIPTOR_OFFSET = 1;
    const int HEADER_LENGTH = 2;

    const char* data = payload + HEADER_LENGTH;

    switch (payload[DESCRIPTOR_OFFSET]) {
    case SCALED_ACCEL_DESCRIPTOR:
        parse_scaled_accelerometer(data, imuData);
        break;

    case SCALED_GYRO_DESCRIPTOR:
        parse_scaled_gyro(data, imuData);
        break;

    case EULER_DESCRIPTOR:
        parse_euler_angles(data, imuData);
        break;

    case TIME_DESCRIPTOR:
        previousTimer = parse_internal_timestamp(data, previousTimer, imuData);
        totalSensorTime += imuData.timeDelta;
        imuData.timestamp = time.timestamp(totalSensorTime);
        break;
    }
}


void Microstrain3DMGX3::processGPSData(const char* payload, int length)
{
    std::cerr << "ERROR:Microstrain3DMGX3: Shouldn't be getting GPS data!\n";
}


bool valid_checksum(const char* data, int packetLength)
{
    uint16_t checksum = calculate_checksum(data, packetLength - 2);

#ifdef DEBUG_CHECKSUM
    bool valid = ((checksum & 0xFF00) >> 8) == (uint8_t)data[packetLength - 2]
      && ((checksum & 0x00FF) == (uint8_t)data[packetLength - 1]);

    std::cout << "DEBUG:Microstrain3DMGX3:Checksum: Calc:" << ((checksum & 0xFF00) >> 8) << ' ' << (checksum & 0x00FF)
              << " Received:" << (int)(uint8_t)data[packetLength - 2] << ' ' << (int)(uint8_t)data[packetLength - 1]
              << " Valid:" << valid << '\n';
#endif

    return ((checksum & 0xFF00) >> 8) == (uint8_t)data[packetLength - 2]
      && ((checksum & 0x00FF) == (uint8_t)data[packetLength - 1]);
}


uint16_t calculate_checksum(const char* data, int packetLength)
{
    uint8_t byte1 = 0;
    uint8_t byte2 = 0;

    for (int n = 0; n < packetLength; ++n) {
        byte1 += data[n];
        byte2 += byte1;
    }

    uint16_t checksum = (static_cast<uint16_t>(byte1) << 8) + static_cast<uint16_t>(byte2);

    return checksum;
}


void parse_scaled_accelerometer(const char* data, imu_data_t& imu)
{
    /*
     * scaled accelerometer data:
     *
     * 0-3:  x accel
     * 4-7:  y accel
     * 8-11: z accel
     *
     * Units: g
     */

    const int X_ACCEL_OFFSET = 0;
    const int Y_ACCEL_OFFSET = 4;
    const int Z_ACCEL_OFFSET = 8;

    imu.acceleration[IMU_X_INDEX] = utils::float_from_bytes(data + X_ACCEL_OFFSET) * GRAVITY_CONSTANT_3DM_GX3;
    imu.acceleration[IMU_Y_INDEX] = utils::float_from_bytes(data + Y_ACCEL_OFFSET) * GRAVITY_CONSTANT_3DM_GX3;
    imu.acceleration[IMU_Z_INDEX] = utils::float_from_bytes(data + Z_ACCEL_OFFSET) * GRAVITY_CONSTANT_3DM_GX3;
}


void parse_scaled_gyro(const char* data, imu_data_t& imu)
{
    /*
     * scale gyro data:
     *
     * 0-3:  roll rate
     * 4-7:  pitch rate
     * 8-11: yaw rate
     */

    const int ROLL_OFFSET = 0;
    const int PITCH_OFFSET = 4;
    const int YAW_OFFSET = 8;

    imu.rotationalVelocity[IMU_ROLL_INDEX] = utils::float_from_bytes(data + ROLL_OFFSET);
    imu.rotationalVelocity[IMU_PITCH_INDEX] = utils::float_from_bytes(data + PITCH_OFFSET);
    imu.rotationalVelocity[IMU_YAW_INDEX] = utils::float_from_bytes(data + YAW_OFFSET);
}


void parse_euler_angles(const char* data, imu_data_t& imu)
{
    /*
     * Euler angles data:
     *
     * 0-3:  roll
     * 4-7:  pitch
     * 8-11: yaw
     */

    const int ROLL_OFFSET = 0;
    const int PITCH_OFFSET = 4;
    const int YAW_OFFSET = 8;

    imu.orientation[IMU_ROLL_INDEX] = utils::float_from_bytes(data + ROLL_OFFSET);
    imu.orientation[IMU_PITCH_INDEX] = utils::float_from_bytes(data + PITCH_OFFSET);
    imu.orientation[IMU_YAW_INDEX] = utils::float_from_bytes(data + YAW_OFFSET);
}


uint32_t parse_internal_timestamp(const char* data, uint32_t previousTimer, imu_data_t& imu)
{
    /*
     * Internal timestamp data:
     *
     * 0-4: timestamp
     *
     * Units: 16 microsecond ticks
     */

    const int kTickDurationUs = 16;

    uint32_t timer = utils::char_to_uint32_t(data[0], data[1], data[2], data[3]);   // value in timer ticks
    uint32_t deltaTime = timer - previousTimer;

    if (timer < previousTimer)   // check to see if roll over happened, if so, then figure out what the offset is
    {
        deltaTime = timer + (std::numeric_limits<uint32_t>::max() - previousTimer);
    }

    imu.timeDelta = deltaTime * kTickDurationUs;   // convert to microseconds

    return timer;
}

}   // namespace sensors
}   // namespace vulcan
