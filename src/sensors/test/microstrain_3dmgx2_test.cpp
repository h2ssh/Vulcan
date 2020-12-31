/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <iostream>
#include "utils/timestamp.h"
#include "sensors/microstrain_3dmgx2.h"


using namespace vulcan;
using vulcan::utils::system_time_us;
using namespace vulcan::sensors;


// Helpers for the test
void display_imu_data(const imu_data_t& imuData);


/**
* microstrain_3dmgx2_test is a simple program to test the driver for the Microstrain 3DM-GX2
* IMU. It retrieves information about the IMU and then grabs IMU data until being closed.
*
* Command-line: microstrain_3dmgx2_test < IMU port >
*
*/
int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout<<"Expected command-line: microstrain_3dmgx2_test <IMU port>"<<std::endl;
        return -1;
    }

    int64_t startTime   = 0;
    int64_t endTime     = 0;
    int64_t deltaTime   = 0;
    int     numReadings = 0;

    imu_data_t imuData;

    double sumAccel = 0;

    Microstrain3DMGX2 imu(argv[1]);

    startTime = system_time_us();

    if(imu.captureBias(1000))
    {
        std::cout<<"Successly captured gyro bias"<<std::endl;
    }
    else
    {
        std::cout<<"Failed to capture gyro bias"<<std::endl;
    }

    endTime = system_time_us();

    std::cout<<"Time in captureBias() function: "<<(endTime - startTime)<<std::endl;

    while(1)
    {
        startTime = system_time_us();

        imu.getIMUData(imuData);

        endTime   = system_time_us();

        deltaTime += endTime - startTime;

        ++numReadings;

        sumAccel += imuData.acceleration[IMU_X_INDEX];

        if(deltaTime > vulcan::utils::sec_to_usec(1))
        {
            display_imu_data(imuData);

            std::cout<<"Output rate: "<<(static_cast<float>(numReadings)*vulcan::utils::usec_to_sec(deltaTime))<<" Hz"<<" Delta: "<<deltaTime<<std::endl;

            std::cout<<"AccelX bias: "<<(sumAccel/numReadings)<<std::endl;

            deltaTime   = 0;
            numReadings = 0;
        }
    }

    return 0;
}


void display_imu_data(const imu_data_t& imuData)
{
    std::cout<<"IMU: Acceleration: ("<<imuData.acceleration[IMU_X_INDEX]<<", "<<imuData.acceleration[IMU_Y_INDEX]<<", "<<imuData.acceleration[IMU_Z_INDEX]<<")\n"
             <<"     Rotation vel: ("<<imuData.rotationalVelocity[IMU_X_INDEX]<<", "<<imuData.rotationalVelocity[IMU_Y_INDEX]<<", "<<imuData.rotationalVelocity[IMU_Z_INDEX]<<")\n"
             <<"     Orientation:  ("<<imuData.orientation[IMU_ROLL_INDEX]<<", "<<imuData.orientation[IMU_PITCH_INDEX]<<", "<<imuData.orientation[IMU_YAW_INDEX]<<")"<<std::endl;
}
