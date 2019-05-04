/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#ifndef UTILS_TIMESTAMP_H
#define UTILS_TIMESTAMP_H

#include <stdint.h>

namespace vulcan
{
namespace utils
{
    
/**
* system_time_us gets the system time of the machine in microseconds.
*
* The time is pulled from the monotonic system clock, which indicates
* the time elapsed since the machine was turned on, rather than time 
* since the epoch. If you need the real-world time, then get it
* yourself damnit!
*/
int64_t system_time_us(void);

/**
* sec_to_usec converts times in seconds to times in microseconds.
* 
* \param    seconds         Time in seconds
* \return   Time in microseconds represented as a 64-bit signed integer.
*/
template <class T>
constexpr int64_t sec_to_usec(T seconds)
{
    return static_cast<int64_t>(seconds * 1000000ll);
}


/**
* usec_to_sec converts times in microseconds to times in seconds.
*
* \param    usec        Time in microseconds
* \return   Time in seconds represented as a double.
*/
inline constexpr double usec_to_sec(int64_t usec)
{
    return usec / 1000000.0;
}

} // namespace system
} // namespace vulcan

#endif // UTILS_TIMESTAMP_H
