/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <core/odometry.h>
#include <system/module_communicator.h>
#include <utils/fixed_duration_buffer.h>
#include <utils/timestamp.h>
#include <fstream>

using namespace vulcan;


class OdometryLogger
{
public:
    
    OdometryLogger(std::ofstream& log)
    : log_(log)
    , haveEncoders_(false)
    , encoderHistory_(50000)
    {
    }
    
    void handleData(const encoder_data_t& encoders, const std::string& channel)
    {
        encoderHistory_.push(encoders);
        
        if(haveEncoders_)
        {
            double rightWheelTotal = 0.0;
            double leftWheelTotal = 0.0;
            
            std::for_each(encoderHistory_.begin(), encoderHistory_.end(), [&](const encoder_data_t& e) {
                rightWheelTotal += e.deltaRightWheel;
                leftWheelTotal += e.deltaLeftWheel;
            });
            
            double historyDuration = utils::usec_to_sec(encoderHistory_.storedDuration());
            double smoothedRightWheel = rightWheelTotal / historyDuration;
            double smoothedLeftWheel = leftWheelTotal / historyDuration;
            
            
            double deltaTime = utils::usec_to_sec(encoders.timestamp - prevEncoders_.timestamp);
            double elapsedTimeMs = (encoders.timestamp - startTime_) / 1000.0;
            
            if(deltaTime > 0.0)
            {
                log_ << elapsedTimeMs << ' ' << encoders.deltaLeftWheel << ' ' << encoders.deltaRightWheel 
                    << ' ' << (encoders.deltaLeftWheel / deltaTime) << ' ' << (encoders.deltaRightWheel / deltaTime) 
                    << ' ' << smoothedLeftWheel << ' ' << smoothedRightWheel 
                    << std::endl;
            }
        }
        else
        {
            startTime_ = encoders.timestamp;
        }
        
        prevEncoders_ = encoders;
        haveEncoders_ = true;
    }
    
private:
    
    std::ofstream& log_;
    encoder_data_t prevEncoders_;
    int64_t startTime_;
    bool haveEncoders_;
    
    utils::FixedDurationBuffer<encoder_data_t> encoderHistory_;
};


int main(int argc, char** argv)
{
    if(argc < 2)
    {
        std::cout << "Expected arguments: odometry_logger 'log_filename'\n";
        return -1;
    }
    
    std::ofstream logFile(argv[1]);
    
    if(!logFile.is_open())
    {
        std::cerr << "ERROR: Failed to open log file:" << argv[1] << '\n';
        return -1;
    }
    
    OdometryLogger logger(logFile);
    
    system::ModuleCommunicator communicator;
    communicator.subscribeTo<encoder_data_t>(&logger);
    
    while(true)
    {
        communicator.processIncoming();
    }
    
    return 0;
}
