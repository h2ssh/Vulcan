/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include "sensors/microstrain_3dmgx2.h"
#include "utils/byte_conversion.h"
#include "utils/timestamp.h"
#include <cassert>
#include <cmath>
#include <cstring>   // for memcpy
#include <iostream>
#include <limits>
#include <unistd.h>   // for usleep

namespace vulcan
{
namespace sensors
{

const float GRAVITY_M_S2 = 9.80665f;   // this value is within 0.00037 of what Ann Arbor's gravity vector might be
                                       // that's probably close enough to not worry about the difference
const uint16_t BIAS_CAPTURE_TIME_MS = 10000;


// Helpers for readings a full packet of data
size_t read_full_imu_packet(utils::SerialConnection& serial, char* packet, size_t packetLength);
bool valid_packet(const char* packet, size_t packetLength, unsigned int checksumPos);

// Helpers for parsing the useful imu data
void parse_accelerations_from_cc_packet(const char* imuResponse, imu_data_t& imuData);
void parse_angular_velocity_from_cc_packet(const char* imuResponse, imu_data_t& imuData);
void parse_euler_angles_from_cc_packet(const char* imuResponse, imu_data_t& imuData);
// Returns the Timer value in imuResponse to set as the new value for previousTimer
uint32_t calculate_time_delta_from_cc_packet(uint32_t previousTimer,
                                             const char* imuResponse,
                                             double secondsPerTick,
                                             imu_data_t& imuData);

std::string parse_device_identifier_string(const char* imuResponse);

// Helper functions for converting the data received from the Microstrain3DMGX2 into usable numbers


// Helpers for converting raw data to useful numbers
float imu_gyro_temperature_to_celcius(int rawTemperature);


Microstrain3DMGX2::Microstrain3DMGX2(const std::string& device, int64_t ticksPerSecond)
: firmwareVersion(0)
, serialNumber("")
, secondsPerTick_(1.0 / ticksPerSecond)
, previousTimer(0)
, totalSensorTime_(0)
, continuousModeActive(false)
, serial(device, utils::BAUD_115200)
{
    initialize();
}


Microstrain3DMGX2::~Microstrain3DMGX2(void)
{
    serial.close();
}


bool Microstrain3DMGX2::startIMU(void)
{
    // When starting, first run a bias capture to get rid of lingering errors
    std::cout << "Microstraing 3DM-GX2: Capturing gyro bias...Please wait " << BIAS_CAPTURE_TIME_MS << "ms"
              << std::endl;
    captureBias(BIAS_CAPTURE_TIME_MS);
    std::cout << "Microstrain 3DM-GX2: Finished capturing gyro bias\n";

    return true;
}


void Microstrain3DMGX2::estimateGravityMagnitude(uint16_t numSamples)
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

    gravityMagnitude = sqrt(pow(accelSum[0], 2) + pow(accelSum[1], 2) + pow(accelSum[2], 2));

    std::cout << "Microstrain 3DM-GX2: Gravity magnitude:" << gravityMagnitude << '\n';
}


bool Microstrain3DMGX2::getIMUData(imu_data_t& data)
{
    // Ensure continuous mode is running, otherwise data is not being read
    if (!continuousModeActive) {
        activeCommand = '\xCC';
        activeResponseLength = 79;

        startContinuousMode(activeCommand);
    }

    // Make sure the read thread is up and running once continuous mode has started
    if (!readThread.isRunning()) {
        readThread.attachTask(this);
        readThread.start();
    }

    // Wait until a new packet is available
    dataTrigger.wait();
    dataLock.lock();
    data = currentData;
    dataTrigger.setPredicate(
      false);   // set predicate as false before unlocking to ensure next arriving data sets it back to true
    dataLock.unlock();

    return true;
}


unsigned int Microstrain3DMGX2::getVersion(void)
{
    // 0xF0 is the firmware command

    const int RESPONSE_LENGTH = 7;
    const char COMMAND = '\xE9';
    int numRead = RESPONSE_LENGTH;
    char response[RESPONSE_LENGTH];

    if (firmwareVersion == 0) {
        serial.write(&COMMAND, 1);
        numRead = read_full_imu_packet(serial, response, RESPONSE_LENGTH);

        if (!valid_packet(response, numRead, RESPONSE_LENGTH - 2)) {
            return 0;
        }

        firmwareVersion = utils::char_to_uint32_t(response[1], response[2], response[3], response[4]);
    }

    return firmwareVersion;
}


std::string Microstrain3DMGX2::getSerialNumber(void)
{
    const char SERIAL_NUMBER_COMMAND = '\x01';

    if (serialNumber.empty()) {
        serialNumber = readDeviceIdentifierString(SERIAL_NUMBER_COMMAND);
    }

    return serialNumber;
}


microstrain_3DMGX2_temperature_t Microstrain3DMGX2::getTemperature(void)
{
    /*
     * Command format:
     *
     * Command byte: 0xD1
     *
     * Response: 15 bytes
     *           [0]     = 0xD1
     *           [1-2]   = TempAccel
     *           [3-4]   = TempGyroX
     *           [5-6]   = TempGyroY
     *           [7-8]   = TempGyroZ
     *           [9-12]  = Timer
     *           [13-14] = Checksum
     */

    const char COMMAND = '\xD1';
    const int RESPONSE_LENGTH = 15;
    int numRead = RESPONSE_LENGTH;
    char response[RESPONSE_LENGTH];

    serial.write(&COMMAND, 1);
    numRead = read_full_imu_packet(serial, response, RESPONSE_LENGTH);

    microstrain_3DMGX2_temperature_t imuTemperatures;

    if (valid_packet(response, numRead, RESPONSE_LENGTH - 2)) {
        imuTemperatures.accelerometer = ((utils::char_to_signed(response[1], response[2]) * 3.3 / 4096) - 0.5) * 100;
        imuTemperatures.gyroXAxis = imu_gyro_temperature_to_celcius(utils::char_to_signed(response[3], response[4]));
        imuTemperatures.gyroYAxis = imu_gyro_temperature_to_celcius(utils::char_to_signed(response[5], response[6]));
        imuTemperatures.gyroZAxis = imu_gyro_temperature_to_celcius(utils::char_to_signed(response[7], response[8]));
    }

    return imuTemperatures;
}


bool Microstrain3DMGX2::captureBias(uint16_t sampleTimeMs)
{
    /*
     * Command format:
     *
     * Command byte: [0] = 0xCD
     * Command data: 4 bytes
     *               [1] = 0xC1
     *               [2] = 0x29
     *               [3] = MSB sampleTimeMs
     *               [4] = LSB sampleTimeMs
     *
     * Response: 19 bytes
     *               [0]     = 0xCD
     *               [1-4]   = BiasX
     *               [5-8]   = BiasY
     *               [9-12]  = BiasZ
     *               [13-16] = Timer
     *               [17-18] = Checksum
     */

    const int COMMAND_LENGTH = 5;
    char command[COMMAND_LENGTH];

    const int RESPONSE_LENGTH = 19;
    char response[RESPONSE_LENGTH];

    int numRead = RESPONSE_LENGTH;

    // If currently in continuous mode, halt it, send the command, then start it back up again
    bool inContinuousMode = continuousModeActive;

    if (inContinuousMode) {
        stopContinuousMode();
    }

    // Create the command
    command[0] = '\xCD';
    command[1] = '\xC1';
    command[2] = '\x29';
    utils::uint16_to_char(sampleTimeMs, command[3], command[4]);

    // Send it off and wait for a response
    serial.write(command, COMMAND_LENGTH);

    // Sleep the specified time before returning
    // Blocking function makes more sense than returning control to program when
    // the IMU isn't actually ready for data transmission
    usleep(sampleTimeMs * 1000);

    numRead = read_full_imu_packet(serial, response, RESPONSE_LENGTH);

    if (inContinuousMode) {
        startContinuousMode();
    }

    // Don't actually care what the bias values are at the moment, though knowing the info could be useful
    // at some point in time.
    if (valid_packet(response, numRead, RESPONSE_LENGTH - 2)) {
        return true;
    } else {
        return false;
    }
}


bool Microstrain3DMGX2::initialize(void)
{
    // First, see that a connection to the Microstrain3DMGX2 can be opened
    if (!establishSerialConnection()) {
        std::cerr << "ERROR: Microstrain3DMGX2 initialize: Failed to open the serial port for communicating with the "
                     "Microstrain3DMGX2"
                  << std::endl;

        return false;
    } else {
        serial.flush();   // make sure there isn't data lingering in the serial line

        stopContinuousMode();

        std::cout << "Microstrain3DMGX2 initialized: "
                  << "\nFirmware:      " << getVersion() << "\nSerial Number: " << getSerialNumber()
                  << "\nTemperature:   " << getTemperature().accelerometer << " degrees Celcius" << std::endl;
    }

    return true;
}


bool Microstrain3DMGX2::establishSerialConnection(void)
{
    const int MAX_CONNECT_ATTEMPTS = 15;

    bool connected = serial.connect();

    for (int x = MAX_CONNECT_ATTEMPTS; (--x >= 0) && !connected;) {
        connected = serial.connect();

        if (!connected) {
            std::cout << "Failed to connect to serial device" << std::endl;
        }
        usleep(100000);   // wait a bit before attempting to connect again as some other device may be in the process of
                          // releasing control of the port
    }

    return connected;
}


void Microstrain3DMGX2::startContinuousMode(char command)
{
    // Send the command for continous mode and brace for impact!
    const int COMMAND_LENGTH = 4;
    //     const int  RESPONSE_LENGTH = 7;

    char commandString[COMMAND_LENGTH];
    //     char response[RESPONSE_LENGTH];
    //
    //     int numRead = RESPONSE_LENGTH;

    commandString[0] = '\xC4';
    commandString[1] = '\xC1';
    commandString[2] = '\x29';
    commandString[3] = command;

    serial.write(commandString, COMMAND_LENGTH);
    serial.flush();

    continuousModeActive = true;
}


void Microstrain3DMGX2::stopContinuousMode(void)
{
    // To break continous mode, send the continuous mode message with a command of 0
    startContinuousMode(0);

    continuousModeActive = false;
}


std::string Microstrain3DMGX2::readDeviceIdentifierString(char command)
{
    /*
     * Command format:
     *
     * Command byte: 0xEA
     * Command data: 1 byte
     *               [0] = command
     *
     * Response: 20 bytes
     *           [0]     = 0xEA
     *           [1]     = command
     *           [2-17]  = identifier string
     *           [18-19] = checksum
     */

    const int COMMAND_LENGTH = 2;
    const int RESPONSE_LENGTH = 20;

    char commandString[COMMAND_LENGTH];
    char response[RESPONSE_LENGTH];

    int numRead = RESPONSE_LENGTH;

    commandString[0] = '\xEA';
    commandString[1] = command;

    serial.write(commandString, COMMAND_LENGTH);
    numRead = read_full_imu_packet(serial, response, RESPONSE_LENGTH);

    if (!valid_packet(response, numRead, RESPONSE_LENGTH - 2)) {
        return "";
    }

    return parse_device_identifier_string(response);
}


int Microstrain3DMGX2::run(void)
{
    int packetStartByte = -1;

    while (true) {
        readAvailableData();

        packetStartByte = activePacketStartInBuffer();

        // If a valid packet has arrived, then parse it and set the trigger for valid data having arrived
        if (packetStartByte >= 0) {
            imu_data_t packetData = parseActivePacket(packetStartByte);

            // Erase the data read from the current packet
            dataBuffer.erase(0, packetStartByte + activeResponseLength);

            dataLock.lock();
            currentData = packetData;
            dataLock.unlock();

            dataTrigger.setPredicate(true);
            dataTrigger.broadcast();
        }

        usleep(1000);
    }

    return 0;
}


size_t Microstrain3DMGX2::readAvailableData(void)
{
    // The hacked interface for my SerialConnection will read all data if provided an empty buffer with 0 length
    char* availableData = 0;
    int readLength = 0;

    availableData = serial.read(availableData, readLength);

    if (readLength > 0) {
        dataBuffer.append(availableData, readLength);
        delete[] availableData;
        return readLength;
    } else {
        return 0;
    }
}


int Microstrain3DMGX2::activePacketStartInBuffer(void)
{
    // Go through the data buffer and consider all sequential packets of activeResponseLength
    // If the checksum is satisfied, as determined by valid_packet(), then a packet of activeCommand type
    // has been received and begins at the calculated start byte
    int bufferLength = dataBuffer.length();
    int lastValidStartByte = bufferLength - activeResponseLength;

    assert(lastValidStartByte < bufferLength);

    for (int startByte = 0; startByte <= lastValidStartByte; ++startByte) {
        if (dataBuffer[startByte] == activeCommand) {
            const char* possiblePacket = dataBuffer.c_str() + startByte;

            if (valid_packet(possiblePacket, activeResponseLength, activeResponseLength - 2)) {
                return startByte;
            }
        }
    }

    return -1;
}


imu_data_t Microstrain3DMGX2::parseActivePacket(int startByte)
{
    /*
     * The IMU is run in continuous mode (or it will be entered here) so a read operation
     * needs to happen and then parsing of the packet, which is in the described format:
     *
     * Command byte: 0xCC
     * Response: 79 bytes
     *           [0] = 0xCC
     *           [1-4] = accelX
     *           [5-8] = accelY
     *           [9-12] = accelZ
     *           [13-16] = angVelX
     *           [17-20] = angVelY
     *           [21-24] = angVelZ
     *           [25-28] = magX
     *           [29-32] = magY
     *           [33-36] = magZ
     *           [37-40] = m_11
     *           [41-44] = m_12
     *           [45-48] = m_13
     *           [49-52] = m_21
     *           [53-56] = m_22
     *           [57-60] = m_23
     *           [61-64] = m_31
     *           [65-68] = m_32
     *           [69-72] = m_33
     *           [73-76] = Timer
     *           [77-78] = Checksum
     */


    imu_data_t data;

    const char* packet = dataBuffer.c_str() + startByte;

    parse_accelerations_from_cc_packet(packet, data);
    parse_angular_velocity_from_cc_packet(packet, data);
    parse_euler_angles_from_cc_packet(packet, data);

    previousTimer = calculate_time_delta_from_cc_packet(previousTimer, packet, secondsPerTick_, data);
    totalSensorTime_ += data.timeDelta;

    data.timestamp = time_.timestamp(totalSensorTime_);
    data.gravityMagnitude = gravityMagnitude;

    return data;
}


// Helpers for readings a full packet of data
size_t read_full_imu_packet(utils::SerialConnection& serial, char* packet, size_t packetLength)
{
    size_t totalBytesRead = 0;
    int bytesRead = 0;

    while (totalBytesRead < packetLength) {
        bytesRead = (packetLength - totalBytesRead);
        serial.read(packet + totalBytesRead, bytesRead);

        totalBytesRead += bytesRead;
    }

    return totalBytesRead;
}


// Helpers for parsing the useful imu data
void parse_accelerations_from_cc_packet(const char* imuResponse, imu_data_t& imuData)
{
    /*
     * [1-4] = accelX
     * [5-8] = accelY
     * [9-12] = accelZ
     */

    imuData.acceleration[IMU_X_INDEX] = utils::float_from_bytes(imuResponse + 1) * GRAVITY_M_S2;
    imuData.acceleration[IMU_Y_INDEX] = utils::float_from_bytes(imuResponse + 5) * GRAVITY_M_S2;
    imuData.acceleration[IMU_Z_INDEX] = utils::float_from_bytes(imuResponse + 9) * GRAVITY_M_S2;
}


void parse_angular_velocity_from_cc_packet(const char* imuResponse, imu_data_t& imuData)
{
    /*
     * [13-16] = angVelX = deltaRoll
     * [17-20] = angVelY = deltaPitch
     * [21-24] = angVelZ = deltaYaw
     */

    imuData.rotationalVelocity[IMU_ROLL_INDEX] = utils::float_from_bytes(imuResponse + 13);
    imuData.rotationalVelocity[IMU_PITCH_INDEX] = utils::float_from_bytes(imuResponse + 17);
    imuData.rotationalVelocity[IMU_YAW_INDEX] = utils::float_from_bytes(imuResponse + 21);
}


void parse_euler_angles_from_cc_packet(const char* imuResponse, imu_data_t& imuData)
{
    /*
     * [37-40] = m_11
     * [41-44] = m_12
     * [45-48] = m_13
     * [49-52] = m_21
     * [53-56] = m_22
     * [57-60] = m_23
     * [61-64] = m_31
     * [65-68] = m_32
     * [69-72] = m_33
     *
     * Euler angle conversions:
     *
     * pitch = asin(-m_13)
     * roll  = atan(m_23 / m_33)
     * yaw   = atan(m_12 / m_11)
     */

    float m11 = utils::float_from_bytes(imuResponse + 37);
    float m12 = utils::float_from_bytes(imuResponse + 41);
    float m13 = utils::float_from_bytes(imuResponse + 45);
    float m23 = utils::float_from_bytes(imuResponse + 57);
    float m33 = utils::float_from_bytes(imuResponse + 69);

    imuData.orientation[IMU_PITCH_INDEX] = asin(-m13);
    imuData.orientation[IMU_ROLL_INDEX] = atan2(m23, m33);
    imuData.orientation[IMU_YAW_INDEX] = atan2(m12, m11);
}


uint32_t calculate_time_delta_from_cc_packet(uint32_t previousTimer,
                                             const char* imuResponse,
                                             double secondsPerTick,
                                             imu_data_t& imuData)
{
    /*
     * [73-76] = Timer
     *
     * From the documentation:
     *   Timer / 19660800.0 = time in seconds for GX2
     *   16us ticks for GX3
     */

    uint32_t timer = utils::char_to_uint32_t(imuResponse[73], imuResponse[74], imuResponse[75], imuResponse[76]);
    uint32_t deltaTime = timer - previousTimer;

    if (timer < previousTimer)   // check to see if roll over happened, if so, then figure out what the offset is
    {
        deltaTime = timer + (std::numeric_limits<uint32_t>::max() - previousTimer);
    }

    imuData.timeDelta = utils::sec_to_usec(deltaTime * secondsPerTick);   // go to seconds then to microseconds

    return timer;
}


std::string parse_device_identifier_string(const char* imuResponse)
{
    /*
     * [2-17]  = identifier string
     */

    return std::string(imuResponse + 2, 16);
}

// Helpers for converting raw data to useful numbers
float imu_gyro_temperature_to_celcius(int rawTemperature)
{
    /*
     * TODO: Figure out which types of gyros are on our 3DM-GX2 so appropriate formula can be used for converting
     *       the temps. Two possibilities:
     *
     *       temp_c = (raw_temp * 3.3 / 4096 * 1/0.0084) + 273
     *       temp_c = (raw_temp * 3.3 / 4096 - 2.5) * 1/0.009 + 25
     */

    return (rawTemperature * (3.3 / 4096) * (1 / 0.0084)) + 273;
}


bool valid_packet(const char* packet, size_t packetLength, unsigned int checksumPos)
{
    /*
     * The checksum for Microstrain3DMGX2 data uses the following formula:
     *
     * 1) Sum of all bytes except the checksum. Rollover from 65535 to 0.
     */

    // Packet isn't long enough if this is the case
    if (packetLength < checksumPos + 1) {
        std::cerr << "ERROR: Microstrain3DMGX2: Checksum - Packet too short. Expected: " << (checksumPos + 1)
                  << " Actual: " << packetLength << std::endl;
        return false;
    }

    uint16_t sum = 0;

    // Index starts at 1, because datasheet says to ignore the header
    for (size_t s = 0; s < checksumPos; ++s) {
        sum += static_cast<unsigned char>(packet[s]);
    }

    uint16_t checksum = utils::char_to_unsigned(packet[checksumPos], packet[checksumPos + 1]);

    return checksum == sum;
}

}   // namespace sensors
}   // namespace vulcan
