/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


/**
* \file     sensor_time.cpp
* \author   Collin Johnson
* 
* Definition of SensorTime.
*/

#include "utils/sensor_time.h"
#include "utils/timestamp.h"
#include <algorithm>
#include <iostream>
#include <limits>

// #define DEBUG_TIME

namespace vulcan
{
namespace utils
{
    
inline int64_t drift_estimate(int64_t delta, double slowRate, double fastRate)
{
    // Equation 11
    return std::max((delta * slowRate) / (1.0 + slowRate), (delta * fastRate) / (1.0 - fastRate));
}

inline int64_t clock_offset_estimate(int64_t sensorTime, int64_t systemTime, int64_t driftEstimate)
{
    return sensorTime - systemTime - driftEstimate;
}
    

SensorTime::SensorTime(double slowDriftRate, double fastDriftRate)
: slowRate_(slowDriftRate)
, fastRate_(fastDriftRate)
, sensorTimeForOffset_(0)
, systemTimeForOffset_(0)
, previousSensorTime_(0)
, previousSystemTime_(0)
, totalSensorTimeElapsed_(0)
, totalSystemTimeElapsed_(0)
, numUpdates_(0)
{
}


int64_t SensorTime::timestamp(int64_t sensorTime)
{
    int64_t systemTime = utils::system_time_us();

    // Calculate the current estimate of the offset and the offset from the previous estimate
    int64_t currentOffsetEstimate = clock_offset_estimate(sensorTime, 
                                                          systemTime, 
                                                          drift_estimate(0, slowRate_, fastRate_));
    int64_t previousOffsetEstimate = clock_offset_estimate(sensorTimeForOffset_,
                                                           systemTimeForOffset_,
                                                           drift_estimate(sensorTime - sensorTimeForOffset_,
                                                                          slowRate_,
                                                                          fastRate_));

    int64_t correctedTime = systemTime;

    // If this measurement increases the estimated offset, then use this new estimate
    // Also save the results for the first time through the algorithm
    if(currentOffsetEstimate > previousOffsetEstimate || (sensorTimeForOffset_ == 0))
    {
        sensorTimeForOffset_ = sensorTime;
        systemTimeForOffset_ = systemTime;
        
        correctedTime = sensorTime - currentOffsetEstimate;
        
#ifdef DEBUG_TIME
        std::cout << "DEBUG:SensorTime: Changed offset estimate: Old:" << previousOffsetEstimate << "New:"
            << currentOffsetEstimate << '\n';
#endif
    }
    // Otherwise, the previous best guess holds out
    else
    {
        correctedTime = sensorTime - previousOffsetEstimate;
        
#ifdef DEBUG_TIME
        std::cout << "DEBUG:SensorTime: Using previous offset estimate:" << previousOffsetEstimate << '\n';
#endif
    }
    
#ifdef DEBUG_TIME
    std::cout << "DEBUG:SensorTime: Timestamp:" << correctedTime << " System time:" << systemTime << " Delta:"
        << (systemTime - correctedTime) << '\n';
#endif

    if(previousSensorTime_)
    {
        totalSensorTimeElapsed_ += sensorTime - previousSensorTime_;
        totalSystemTimeElapsed_ += systemTime - previousSystemTime_;
        ++numUpdates_;
    }

    previousSensorTime_ = sensorTime;
    previousSystemTime_ = systemTime;

    if((totalSystemTimeElapsed_ > 0) && (numUpdates_ % 100 == 0))
    {
        std::cout << "INFO:SensorTime: Drift estimate:" << static_cast<long double>(totalSensorTimeElapsed_) / totalSystemTimeElapsed_
            << '\n';
    }
    
    return correctedTime;
}

} // namespace utils
} // namespace vulcan
