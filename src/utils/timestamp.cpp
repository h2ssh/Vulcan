/* Copyright (C) 2010-2019, The Regents of The University of Michigan.
 All rights reserved.

 This software was developed as part of the The Vulcan project in the Intelligent Robotics Lab
 under the direction of Benjamin Kuipers, kuipers@umich.edu. Use of this code is governed by an
 MIT-style License that can be found at "https://github.com/h2ssh/Vulcan".
*/


#include <time.h>
#include <utils/timestamp.h>


/**
* system_time_us gets the system time of the machine in microseconds.
*/
int64_t vulcan::utils::system_time_us(void)
{
    struct  timespec currentTime;
    
    clock_gettime(CLOCK_REALTIME, &currentTime);
    
    return (currentTime.tv_sec*1000000ll) + (currentTime.tv_nsec/1000ll);
}
